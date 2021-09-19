from migen import *
from migen.genlib.cdc import AsyncResetSynchronizer

from slipsr import SlipSR


class Link(Module):
    """7:1 multi-lane Deserializer.

    3:4 clock duty cycle.

    Creates PLLs and clock domains `sr` for serial DDR sampling and shifting and
    `word` for the `word` clock.

    The word is bit-slipped until the clock matches the desired pattern.
    The phase of the sample clock is automatically and continuously
    adjusted until the falling edge samples
    are equally likely to hit earlier/later rising edge samples.

    Delay taps are 150ps nominal. This is in fact small compared to PLL jitter (~500ps).

    t_clk=4ns, 250 MHz bit clock, 500 Mb/s DDR sampling for edge alignment
    """
    def __init__(self, clk, data, platform, t_clk=4.):
        self.n_div = 7
        n_lanes = len(data)
        # output word
        self.word = Signal(n_lanes*self.n_div)
        # clock alignment error counter
        self.align_err = Signal(8)
        # currently tuned sampling delay
        self.delay = Signal(4, reset=0x8)
        # clock aligned, word available
        self.stb = Signal()

        # link clocking
        cd_link = ClockDomain("link", reset_less=True)  # t_clk*7, 3:4 duty
        platform.add_period_constraint(cd_link.clk, t_clk*self.n_div)
        self.clock_domains += cd_link

        cd_word = ClockDomain("word")  # t_clk*7, 1:1 duty
        platform.add_period_constraint(cd_word.clk, t_clk*self.n_div)
        self.clock_domains += cd_word

        cd_sr = ClockDomain("sr", reset_less=True)  # t_clk
        self.clock_domains += cd_sr
        divr = 1 - 1
        divf = 1 - 1
        divq = 2
        t_out = t_clk*(divr + 1)/(divf + 1)
        assert t_out == t_clk
        platform.add_period_constraint(cd_sr.clk, t_out)
        t_vco = t_out/2**divq
        assert .533 <= 1/t_vco <= 1.066, 1/t_vco

        # input clock PLL
        locked = Signal()

        self.delay_relative = Signal(4)
        self.specials += [
            Instance(
                "SB_PLL40_2F_CORE",
                p_FEEDBACK_PATH="PHASE_AND_DELAY",   # out = in/r*f, vco = out*2**q
                p_DIVR=divr,  # input
                p_DIVF=divf,  # feedback
                p_DIVQ=divq,  # vco
                p_FILTER_RANGE=3,
                p_SHIFTREG_DIV_MODE=int(self.n_div == 7),  # div-by-7
                p_DELAY_ADJUSTMENT_MODE_FEEDBACK="DYNAMIC",
                p_FDA_FEEDBACK=0xf,
                p_DELAY_ADJUSTMENT_MODE_RELATIVE="DYNAMIC",
                p_FDA_RELATIVE=0xf,
                p_PLLOUT_SELECT_PORTA="SHIFTREG_0deg",
                p_PLLOUT_SELECT_PORTB="GENCLK",
                p_ENABLE_ICEGATE_PORTA=0,
                p_ENABLE_ICEGATE_PORTB=0,
                i_BYPASS=0,
                i_RESETB=1,
                i_DYNAMICDELAY=Cat(self.delay, self.delay_relative),
                i_REFERENCECLK=ClockSignal("link"),
                i_LATCHINPUTVALUE=0,
                o_LOCK=locked,
                o_PLLOUTGLOBALA=cd_word.clk,
                o_PLLOUTGLOBALB=cd_sr.clk,
                # o_PLLOUTCOREA=,
                # o_PLLOUTCOREB=,
            ),
            AsyncResetSynchronizer(cd_word, ~locked),
        ]

        # input PIO registers
        # 2 bits per lane, data lanes plus one clock lane
        self.submodules.sr = SlipSR(n_lanes=2*(n_lanes + 1), n_div=self.n_div)

        helper = Signal(n_lanes + 1)
        self.specials += [
            # buffer it to the `link` domain and DDR-sample it with the `sr` clock
            Instance(
                "SB_GB_IO",
                p_PIN_TYPE=0b000000,  # no output, i registered
                p_IO_STANDARD="SB_LVDS_INPUT",
                i_INPUT_CLK=cd_sr.clk,
                o_D_IN_0=self.sr.data[0],  # rising
                o_D_IN_1=helper[0],  # falling
                o_GLOBAL_BUFFER_OUTPUT=cd_link.clk,
                i_PACKAGE_PIN=clk,
            ),
            # help relax timings a bit to meet the t_clk/2 delay from
            # negedge sr to posedge sr: easier met in fabric than right after
            # PIO
            Instance(
                "SB_DFFN",
                i_D=helper[0],
                i_C=cd_sr.clk,
                o_Q=self.sr.data[n_lanes + 1],
            ),
        ] + [
            [Instance(
                "SB_IO",
                p_PIN_TYPE=0b000000,
                p_IO_STANDARD="SB_LVDS_INPUT",
                i_INPUT_CLK=cd_sr.clk,
                o_D_IN_0=self.sr.data[i + 1],  # rising
                o_D_IN_1=helper[i + 1],  # falling
                i_PACKAGE_PIN=data[i],
            ),
            # relax timing closure on falling edge samples,
            # same as clk lane above
            Instance(
                "SB_DFFN",
                i_D=helper[i + 1],
                i_C=cd_sr.clk,
                o_Q=self.sr.data[n_lanes + i + 2],
            )]
            for i in range(n_lanes)
        ]

        # clock aligned
        slip_good = Signal()
        mean_edges = 1 << 6
        sigma_edges = 6*mean_edges**.5
        assert mean_edges > sigma_edges  # need SNR
        # number of edges seen, one word headroom
        edges = Signal(max=2*mean_edges + len(self.sr.word))
        # number of edge samples matching the later non-edge value
        # i.e. number of edge samples biased to late sampling
        edges_late = Signal.like(edges)
        # timer to block slips and delay adjustments while others are
        # pending/pll settling
        settle = Signal(max=64, reset=63)
        settle_done = Signal()

        n = len(self.sr.data)
        lanes = [Signal(self.n_div) for i in range(n)]

        self.comb += [
            # transpose link word for delay checking
            Cat(lanes).eq(Cat(self.sr.word[i::n] for i in range(n))),
            self.word.eq(Cat(self.sr.word[i*n + 1:i*n + 1 + n_lanes]
                for i in range(self.n_div))),
            settle_done.eq(settle == 0),
            self.stb.eq(slip_good),
            # be robust, just look for rising clock edge between 1 and 2
            slip_good.eq(lanes[0][1:3] == 0b01),
        ]
        self.sync.word += [
            # delay adjustment:
            # (indices are bit indices, youngest first,
            # reversed from clock cycle numbers)
            # With the timing helper DFFNs on the edge samples (see
            # above):
            # edge sample i is half a cycle older than ref sample i
            # and half a cycle younger than ref sample i + 1
            edges.eq(edges + sum(ref[i + 1] ^ ref[i]
                for ref in lanes[:n_lanes + 1]
                for i in range(self.n_div - 1))),
            edges_late.eq(edges_late + sum(
                (ref[i + 1] ^ ref[i]) & ~(edge[i] ^ ref[i])
                for ref, edge in zip(lanes[:n_lanes + 1], lanes[n_lanes + 1:])
                for i in range(self.n_div - 1))),

            self.sr.slip_req.eq(0),
            If(~settle_done,
                settle.eq(settle - 1),
            ).Else(
                # slip adjustment and delay adjustment can happen in parallel
                # share a hold-off timer for convenience
                If(edges >= 2*mean_edges,
                    edges.eq(0),
                    edges_late.eq(0),
                    # many late samples
                    If((edges_late >= int(mean_edges + sigma_edges)) &
                        # saturate delay
                        (self.delay != 0xf),
                        # if the edge sample matches the sample after the edge
                        # then sampling is late:
                        # increment the feedback delay for earlier sampling
                        self.delay.eq(self.delay + 1),
                        settle.eq(settle.reset),
                    ),
                    # few late samples
                    If((edges_late < int(mean_edges - sigma_edges)) &
                        (self.delay != 0),
                        self.delay.eq(self.delay - 1),
                        settle.eq(settle.reset),
                    ),
                ),
                If(~slip_good,
                    self.sr.slip_req.eq(1),
                    self.align_err.eq(self.align_err + 1),
                    settle.eq(settle.reset),
                )
            )
        ]
