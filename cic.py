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
# [ ] non-power-of-two number of channels
# [ ] pipe draining (when channel number != cycle length)
# [ ] check ice40 ebr mapping/mask
# [ ] Fastino integration resource usage (clock domain (spi 2x16ch or word 3x11ch))
# [ ] per channel interpolation rates
# ([ ] possibly rate change/reset sequencing with output hold and settling)
# ([ ] possibly n-by-m channels (iter-by-parallel) and a single BRAM)

class CIC(Module):
    def __init__(self, width=16, rate_width=9, order=3, channels=4):
        """
        width: input and output data width in bits including sign bit
        rate_width: rate ratio register width
        order: polynomial order (3 for cubic interpolation),
            CIC terminology order is `order + 1`
        channels: number of channels
        """
        # rate change, rate ratio is `r_output/r_input = rate + 1`
        self.rate = Signal(rate_width)
        # output right shift to account for filter gain
        # should be ceil(order*log2(rate))
        self.gain_shift = Signal(max=order*rate_width + 1)
        # clear combs and integrators to establish new rate
        self.rate_stb = Signal()

        # current input sample for the given channel, must only change
        # after `x_ack` is asserted
        self.x = Signal((width, True), reset_less=True)
        # current channel
        self.xi = Signal(max=channels)
        # rate cycle complete
        self.x_ack = Signal()
        
        # output sample for given output channel
        self.y = Signal((width, True), reset_less=True)
        # output channel
        self.yi = Signal(max=channels)
        # output sample valid
        self.y_stb = Signal()

        ###
        
        channel = Signal(max=channels)
        rate_cnt = Signal(rate_width)
        stb = Signal(order)
        rst = Signal(2*order + 1)

        self.sync += [
            channel.eq(channel + 1),
            stb[1:].eq(stb),
            rst[1:].eq(rst),
            If(channel == channels - 1,
                channel.eq(0),
                rate_cnt.eq(rate_cnt - 1),
                stb[0].eq(0),
                If(rate_cnt == 0,
                    rate_cnt.eq(self.rate),
                    stb[0].eq(1),
                    rst[0].eq(0),
                ),
            ),
            If(self.rate_stb,
                channel.eq(0),
                rate_cnt.eq(0),
                stb[0].eq(1),
                rst[0].eq(1),
            )
        ]

        # See Hogenauer1981 for register growth
        comb = [width + n for n in range(order)]
        integ = [width + order + (rate_width - 1)*(n + 1) for n in range(order)]

        comb_r = [Signal((w, True), reset_less=True) for w in comb]
        integ_r = [Signal((w, True), reset_less=True) for w in integ]
        comb_w = [Signal((w, True), reset_less=True) for w in comb]
        integ_w = [Signal((w, True), reset_less=True) for w in integ]

        mem = Memory(sum(comb + integ), channels)
        mem_r = mem.get_port()
        mem_w = mem.get_port(write_capable=True, we_granularity=1)
        self.specials += mem, mem_r, mem_w
        
        # for the integrators for a given channel, read is 2 cycles ahead of write:
        #   0: read addr; 1: integ_r and old z, 2; new z and comb_w write-back
        # for the combs there would only be one cycle:
        #   0: read addr; 1: comb_r, old z, and comb_w write-back; 2: new z
        # add one delay register at the read port to match the integrator
        # read-write pointer spacing:
        #   0: read addr; 1: mem dat_r; 2: comb_r, old z, and comb_w write-back; 3: new z
        # alternatively try:
        #   0: read addr; 1: comb_r, old z, and comb_w1; 3: new z and comb_w write-back
        self.sync += [
            Cat(comb_r).eq(mem_r.dat_r[:sum(comb)]),  
        ]
        self.comb += [
            Cat(integ_r).eq(mem_r.dat_r[sum(comb):]),
            mem_r.adr.eq(channel + 2),
            mem_w.dat_w.eq(Cat(comb_w, integ_w)),
            mem_w.adr.eq(channel),
            mem_w.we.eq(Cat([Replicate(stb[n], w) for n, w in enumerate(comb)],
                            Replicate(1, sum(integ)))),
            self.xi.eq(channel),
            self.x_ack.eq(stb[0]),
            self.yi.eq(channel - 2*order),  # 2*order pipeline latency
            self.y_stb.eq(1),
        ]

        z = self.x
        for i, (cr, cw) in enumerate(zip(comb_r, comb_w)):
            self.comb += cw.eq(z)
            z = Signal((len(cw) + 1, True), reset_less=True)
            self.sync += [
                z.eq(cw - cr),
                If(rst[i],
                    z.eq(0),
                ),
            ]
        for i, (ir, iw) in enumerate(zip(integ_r, integ_w)):
            self.sync += [
                iw.eq(ir + z),
                If(rst[order + 1 + i],
                    iw.eq(0),
                ),
            ]
            z = iw
        self.comb += self.y.eq(z >> self.gain_shift)
