from migen import *

# CIC interpolation filter implementation
#
# * Designed with an eye on efficient use of resources, especially on ice40
# * multi-channel time-domain-multuplexing, pipelined
# * Using a wide BRAM for channel stage data storage
# * one output sample per clock cycle
#    (minimum output sample period per channel: number of channels)

# TODO
# [x] proper reset sequencing
# [x] check ice40 ebr mapping/mask
# [x] resource usage (clock domain (spi 2x16ch or word 3x11ch))
# [ ] Fastino integration
# [x] per channel interpolation rates
# [x] bypass for rate change and fast bursting
# ([ ] pipe draining: not possible/not required)
# ([ ] non-power-of-two number of channels: not necessary)
# ([ ] rate change/reset sequencing with output integrator
#     forced update: not necessary)
# ([ ] possibly n-by-m channels (iter-by-parallel) and a single BRAM:
#     not necessary)


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
        # Current input sample for the given channel
        self.x = Signal((width, True), reset_less=True)
        # Input sample valid
        self.stb = Signal()
        # Clear combs and integrators to establish new rate for current channel.
        # This settles and bypasses the filter for `order` input samples
        # using the previous input sample. In bypass while settling
        # the gain is exactly 1.
        # This module needs a reset on all channels after a clock domain reset.
        self.reset = Signal()
        # Rate ratio is `r_output/r_input = rate + 1`
        # Only change `rate` when applying `reset` as well.
        self.rate = Signal(rate_width)
        # Output right shift to compensate filter gain
        # The overall filter gain is `(rate + 1) << order`
        # gain_shift should be `ceil(order * log2(rate + 1))`
        # Thus to ensure overall gain of 1 choose rates that are powers
        # of two.
        self.shift = Signal(max=order*rate_width + 1)

        # current channel
        self.xi = Signal(max=channels)
        # input sample acknowledged
        self.ack = Signal()

        # output sample for given output channel
        self.y = Signal((width, True), reset_less=True)
        # output channel
        self.yi = Signal(max=channels)

        ###

        # channel counter
        channel = Signal(max=channels)
        self.sync += [
            channel.eq(channel + 1),
            If(channel == channels - 1,
                channel.eq(0),
            ),
        ]
        self.comb += [
            self.xi.eq(channel),
            # pipeline latency, # TODO: modulo
            self.yi.eq(channel - 2*order - 1),
        ]

        # Memory and ports to work on. This is a pipelined CIC.
        layout = [
            # output sample counter per input sample
            ("i_rate", rate_width),
            # bypass input sample counter
            ("i_bypass", log2_int(order, need_pow2=False)),
            # input memory to ensure constancy over one output sample
            # and to support bypass
            ("x", (width, True)),
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
        we = Signal(len(layout), reset_less=True)

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
        do_bypass = Signal()
        rate_done = Signal()
        self.comb += [
            rate_done.eq(read.i_rate == 0),
            do_bypass.eq(read.i_bypass != 0),
            self.ack.eq(rate_done & ~(do_bypass | self.reset)),
        ]

        self.sync += [
            # rate counter updates
            write.i_rate.eq(read.i_rate - 1),
            If(rate_done | self.reset,
                write.i_rate.eq(self.rate),
            ),
            we[0].eq(1),

            # bypass counter updates
            write.i_bypass.eq(read.i_bypass - 1),
            If(self.reset,
                write.i_bypass.eq(order),
            ),
            we[1].eq(self.reset | (do_bypass & rate_done)),

            # x memory updates
            write.x.eq(self.x),
            we[2].eq(self.stb & self.ack),

            # write-enable ripples through combs
            we[3:3 + order].eq(Cat(rate_done | self.reset, we[3:])),
            # integrators always write
            we[3 + order:].eq(Replicate(1, order)),
        ]

        # shift registers to ripple along the comb and integrator stages
        shift_sr = [Signal(max=order*rate_width + 1, reset_less=True)
            for _ in range(2*order)]
        bypass_sr = Signal(2*order, reset_less=True)
        x_sr = [Signal((width, True), reset_less=True) for _ in range(2*order)]
        rst = Signal(2*order)
        rst_sr = Signal(2*order - 1, reset_less=True)
        self.sync += [
            Cat(shift_sr).eq(Cat(self.shift, shift_sr)),
            bypass_sr.eq(Cat(do_bypass | self.reset, bypass_sr)),
            Cat(x_sr).eq(Cat(read.x, x_sr)),
            rst_sr.eq(rst),
        ]
        self.comb += [
            rst.eq(Cat(self.reset, rst_sr)),
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
        self.sync += [
            self.y.eq(z >> shift_sr[-1]),
            If(bypass_sr[-1],
               self.y.eq(x_sr[-1]),
            )
        ]