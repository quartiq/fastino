from migen import *

# CIC interpolation filter implementation
#
# * Designed with an eye on efficient use of resources, especially on ice40
# * multi-channel time-domain-multuplexing, pipelined
# * Using a wide BRAM for channel stage data storage
# * one output sample per clock cycle
#    (minimum output sample period per channel: number of channels)


class CIC(Module):
    def __init__(self, width=16, rate_width=16, order=3, channels=4):
        """
        width: input and output data width in bits including sign bit
        rate_width: rate ratio register width
        order: polynomial order (3 for cubic interpolation),
            CIC terminology order is `order + 1`
        channels: number of channels, must be power of two (currently,
            for easy wrapping)
        """
        ## Inputs
        # Current input sample for the given channel
        self.x = Signal((width, True), reset_less=True)
        # Input sample valid
        self.stb = Signal()
        # Clear combs and integrators to establish new rate for current channel.
        # This re-settles the filter for the duration of `order` input samples.
        # While settling the output is marked invalid.
        self.reset = Signal()
        # Rate ratio is `r_output/r_input = rate + 1` for current channel
        # Only change `rate` when applying `reset` as well.
        self.rate = Signal(rate_width)
        # Output right shift to compensate filter gain for current channel
        # The overall filter gain is `(rate + 1) << order`
        # gain_shift should be `ceil(order * log2(rate + 1))`
        # Thus to ensure overall gain of 1 choose rates that are powers
        # of two.
        self.shift = Signal(max=order*rate_width + 1)

        ## Outputs
        # current input channel index
        self.xi = Signal(max=channels)
        # input sample acknowledged
        self.ack = Signal()
        # output sample for given output channel
        self.y = Signal((width, True), reset_less=True)
        # output channel index
        self.yi = Signal(max=channels)
        # output is valid
        self.valid = Signal()

        self.latency = 2*order
        ###

        assert channels > 2
        assert log2_int(channels)

        # global initialization sequencer
        clear = Signal(reset=1)
        # channel counter
        channel = Signal(max=channels)
        self.sync += [
            channel.eq(channel + 1),
            If(channel == channels - 1,
                channel.eq(0),
                clear.eq(0),
            ),
        ]
        self.comb += [
            self.xi.eq(channel),
            # pipeline latency, TODO: modulo
            self.yi.eq(channel - self.latency),
        ]

        # Memory and ports to work on. This is a pipelined CIC.
        layout = [
            # output sample counter per input sample
            ("i_rate", rate_width),
            # settle input sample counter
            ("i_settle", log2_int(order + 1, need_pow2=False)),
            # input memory to ensure constancy over one output sample
            # and to support settling
            ("x", (width, True)),
            # shift amount
            ("shift", len(self.shift)),
            # rate
            ("rate", rate_width),
        ]
        # See Hogenauer 1981 for bitgrowth
        # Comb stages
        for n in range(order):
            layout.append(
                ("c{}".format(n), (width + n, True)))
        # Integrator stages
        for n in range(order):
            layout.append(
                ("i{}".format(n),
                (width + order + (rate_width - 1)*(n + 1), True)))
        read = Record(layout)
        write = Record(layout, reset_less=True)
        we = Signal(len(layout), reset_less=True, reset=(1 << len(layout)) - 1)

        mem = Memory(len(read), channels)
        mem_r = mem.get_port()
        mem_w = mem.get_port(write_capable=True, we_granularity=1)
        self.specials += mem, mem_r, mem_w
        self.comb += [
            read.raw_bits().eq(mem_r.dat_r),
            mem_w.dat_w.eq(write.raw_bits()),
            mem_r.adr.eq(mem_w.adr + 2),  # TODO: modulo if not power of two
            mem_w.adr.eq(channel),
            mem_w.we.eq(Cat(
                Replicate(we[i], len(sig))
                for i, (sig, _) in enumerate(write.iter_flat()))),
        ]

        # state flags
        settled = Signal()
        rate_done = Signal()
        cfg_update = Signal()
        self.comb += [
            rate_done.eq(read.i_rate == 0),
            settled.eq(read.i_settle == 0),
            cfg_update.eq(self.reset | clear),
            self.ack.eq(rate_done & settled),
        ]

        self.sync += [
            # rate counter updates
            write.i_rate.eq(read.i_rate - 1),
            If(rate_done,
                write.i_rate.eq(read.rate),
            ),
            If(cfg_update,
                write.i_rate.eq(self.rate),
            ),
            # we[0] is 1

            # settle counter updates
            write.i_settle.eq(read.i_settle - 1),
            If(cfg_update,
                write.i_settle.eq(order),
            ),
            we[1].eq(cfg_update | (~settled & rate_done)),

            # x memory updates
            write.x.eq(self.x),
            we[2].eq(self.stb & self.ack),

            write.shift.eq(self.shift),
            we[3].eq(cfg_update),

            write.rate.eq(self.rate),
            we[4].eq(cfg_update),

            # write-enable ripples through combs
            we[5:5 + order].eq(Cat(rate_done | cfg_update, we[5:])),
            # integrators always write (we is 1)
        ]

        # shift registers to ripple along the comb and integrator stages
        shift_sr = [Signal(max=order*rate_width + 1, reset_less=True)
            for _ in range(self.latency)]
        valid_sr = Signal(self.latency)
        rst = Signal(self.latency)
        rst_sr = Signal(self.latency - 1, reset_less=True)
        self.sync += [
            # no need to mux cfg_update as that sample will be marked
            # invalid anyway
            Cat(shift_sr).eq(Cat(read.shift, shift_sr)),
            valid_sr.eq(Cat(settled & ~cfg_update, valid_sr)),
            rst_sr.eq(rst),
        ]
        self.comb += [
            rst.eq(Cat(cfg_update, rst_sr)),
        ]

        # Data path
        # For the integrators for a given channel, read is 2 cycles ahead of
        # write. The cycles are:
        #   0: read addr
        #   1: integ_r and delta
        #   2: integ_w write-back and next delta
        # This fixes the address offset also for the comb stages.
        # For the combs there would usually only be one cycle:
        #   0: read addr
        #   1: comb_r, old z, and comb_w write-back
        #   2: new z
        # Thus add one delay register at the write port to match the integrator
        # read-write pointer spacing:
        #   0: read addr
        #   1: comb_r, old z, and z1 store
        #   2: new z and comb_w write-back

        z = read.x
        # comb stages
        for n in range(order):
            cr = getattr(read, "c{}".format(n))
            cw = getattr(write, "c{}".format(n))
            z1 = Signal((len(cw) + 1, True), reset_less=True)
            self.sync += [
                cw.eq(z),
                z1.eq(z - cr),
                If(rst[n],
                    cw.eq(0),
                ),
            ]
            z = z1
        # integrator stages
        for n in range(order):
            ir = getattr(read, "i{}".format(n))
            iw = getattr(write, "i{}".format(n))
            self.sync += [
                iw.eq(ir + z),
                If(rst[order + n],
                    iw.eq(0),
                ),
            ]
            z = iw
        # output stage
        self.comb += [
            self.y.eq(z >> shift_sr[-1]),
            self.valid.eq(valid_sr[-1]),
        ]
