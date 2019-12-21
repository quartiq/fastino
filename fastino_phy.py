from math import gcd

from migen import *
from migen.genlib.cdc import MultiReg, AsyncResetSynchronizer, BlindTransfer
from misoc.cores.liteeth_mini.mac.crc import LiteEthMACCRCEngine


class SlipSR(Module):
    """Shift register deserializer.

    Takes in a couple of fast lanes of serial `data` to be shifted in
    on the `sr` clock domain and outputs the entire `word` in the `word`
    domain.

    `slip_request` (level sensitive, from the `word` clock domain)
    performs a bit-slip operation on all lanes by one bit. The first
    slipped word appears between 2 and 3 word clock cycles later.
    """
    def __init__(self, n_lanes, n_div):
        self.data = Signal(n_lanes)
        self.slip_req = Signal()
        self.word = Signal(n_div*n_lanes, reset_less=True)

        # continuously shifting input register
        buf = Signal.like(self.word)
        # slipped snapshot of buf
        slipped = Signal.like(buf)
        # delayed word clock
        clk0 = Signal()
        # work alignment marker, rotating shift register in the sr domain
        # with a single high bit
        i = Signal(n_div, reset=1)
        # no slip active
        slip_n = Signal()
        self.sync.sr += [
            # shift in new data
            buf.eq(Cat(self.data, buf)),
            clk0.eq(ClockSignal("word")),
            slip_n.eq(~(self.slip_req & ClockSignal("word") & ~clk0)),
            # shift through alignment marker if not slipping
            If(slip_n,
                i.eq(Cat(i[-1], i)),
            ),
            # if alignment marker on top, coppy into slipped
            If(i[-1],
                slipped.eq(buf),
            ),
        ]
        # in word domain, coppy into output
        self.sync.word += [
            self.word.eq(slipped),
        ]


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
        self.align_err = Signal(8, reset_less=True)
        # currently tuned sampling delay
        self.delay = Signal(8, reset=0x80, reset_less=True)
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

        self.delay_relative = Signal(4, reset_less=True)
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
                i_DYNAMICDELAY=Cat(self.delay[-4:], self.delay_relative[-4:]),
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
        # signed number of misaligned edge samples, see below for details
        delay_delta = Signal((7, True))
        # timer to block slips and delay adjustments while others are
        # pending/pll settling
        settle = Signal(max=64, reset=63, reset_less=True)
        settle_done = Signal()

        n = len(self.sr.data)
        lanes = [Signal(self.n_div) for i in range(n)]

        self.comb += [
            # transpose link word for delay checking
            # EEM LVDS pairs are inverted on Banker, TODO: check
            Cat(lanes).eq(~Cat(self.sr.word[i::n] for i in range(n))),
            # extract data lane samples, TODO: check
            self.word.eq(~Cat(self.sr.word[i*n + 1:i*n + 1 + n_lanes]
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
            delay_delta.eq(sum(
                # if the two reference samples around the edge sample are the
                # same, then don't adjust
                Mux(ref[i + 1] == ref[i], 0,
                    # if the edge sample matches the sample after the edge
                    # then sampling is late:
                    # increment the feedback delay for earlier sampling
                    # (use overflowing addition instead of -1 to evade some
                    # Mux() signedness issue)
                    Mux(edge[i] == ref[i], 1, (1 << len(delay_delta)) - 1))
                for ref, edge in zip(lanes[:n_lanes + 1], lanes[n_lanes + 1:])
                for i in range(self.n_div - 1)
            )),

            self.sr.slip_req.eq(0),
            If(~settle_done,
                settle.eq(settle - 1),
            ).Else(
                # slip adjustment and delay adjustment can happen in parallel
                # share a hold-off timer for convenience
                If(~slip_good,
                    self.sr.slip_req.eq(1),
                    self.align_err.eq(self.align_err + 1),
                    settle.eq(settle.reset),
                ),
                If(delay_delta >= 10,  # threshold for misaligned edges
                    If(self.delay != 0xff,  # saturate delay
                        self.delay.eq(self.delay + 1),
                        # only block if the top bits change
                        If(self.delay[:4] == 0xf,
                            settle.eq(settle.reset),
                        ),
                    )
                ).Elif(delay_delta <= -10,
                    If(self.delay != 0,
                        self.delay.eq(self.delay - 1),
                        If(self.delay[:4] == 0x0,
                            settle.eq(settle.reset),
                        ),
                    )
                ),
            ),
        ]


class Frame(Module):
    """Frame alignment, checksum"""
    def __init__(self, n_frame=14):
        n = 7*6
        # input word from link
        self.word = Signal(n)
        checksum = Signal(12, reset_less=True)
        self.submodules.crc = LiteEthMACCRCEngine(
            data_width=n, width=len(checksum), polynom=0x80f)  # crc-12 telco
        # frame body
        self.body = Signal(n_frame*n - n_frame//2 - 1 - len(checksum))
        self.stb = Signal()
        self.crc_err = Signal(8, reset_less=True)

        # frame: n_frame = 14
        # 0: body(n_word)
        # ...
        # n_frame//2 - 4: body(n_word)
        # ...
        # n_frame//2 - 3: marker(1), body(n_word - 1)
        # ...
        # n_frame - 2: marker(1), body(n_word - 1)
        # n_frame - 1: crc(n_crc = 12), body(n_word - 12)
        # n_body = n_frame*n_word - n_frame//2 - 1 - n_crc

        eof = Signal()
        frame = Signal(n*n_frame, reset_less=True)
        crc_good = Signal()

        body_ = Cat(
            # checksum in the most recent word
            frame[len(checksum):n],
            # skip eof marker bits in the next n_frame//2 + 1 words
            [frame[1 + i*n:(i + 1)*n] for i in range(1, 2 + n_frame//2)],
            # complete words
            frame[(2 + n_frame//2)*n:],
        )
        assert len(body_) == len(self.body)

        self.comb += [
            self.crc.last.eq(checksum),
            # LiteEthMACCRCEngine takes LSB first
            self.crc.data[::-1].eq(self.word),
            self.body.eq(body_),
            # CRC([x, CRC(x)]) == 0
            crc_good.eq(self.crc.next == 0),
        ]
        self.sync += [
            eof.eq(Cat(self.word[0], [frame[i*n] for i in range(n_frame//2)]) == 1),
            self.stb.eq(eof & crc_good),
            frame.eq(Cat(self.word, frame)),
            checksum.eq(self.crc.next),
            If(eof,
                checksum.eq(0),
                If(~crc_good,
                    self.crc_err.eq(self.crc_err + 1),
                ),
            ),
        ]


class MultiSPI(Module):
    """Multi-bus SPI streamer"""
    def __init__(self, platform, n_channels=32, n_bits=16):
        # SPI cycles=n_bits + 1 cycle for CS deasserted
        n = n_channels*(n_bits + 1)
        self.data = Signal(n)
        self.stb = Signal()

        spi = [platform.request("dac", i) for i in range(32)]

        self.busy = Signal()
        # bit counter
        i = Signal(max=n_bits)
        # SPI data shift registers
        sr = [Signal(n_bits, reset_less=True) for i in range(n_channels)]
        # SPI bus enable bits
        enable = Signal(n_channels, reset_less=True)
        assert len(Cat(sr, enable)) == n

        self.sync.spi += [
            [sri[1:].eq(sri) for sri in sr],  # MSB first
            If(~self.busy & self.stb,
                self.busy.eq(1),
                Cat(enable, sr).eq(self.data)
            ),
            If(self.busy,
                i.eq(i + 1),
            ),
            If(i == n_bits - 1,
                self.busy.eq(0),
            ),
        ]
        ce = Signal(1)
        self.comb += ce.eq(enable != 0)
        for i in range(n_channels):
            self.specials += [
                Instance(
                    "SB_IO",
                    p_PIN_TYPE=C(0b010000, 6),  # output registered DDR
                    p_IO_STANDARD="SB_LVCMOS",
                    i_OUTPUT_CLK=ClockSignal("spi"),
                    #i_CLOCK_ENABLE=ce,
                    o_PACKAGE_PIN=spi[i].clk,
                    i_D_OUT_0=self.busy & enable[i],
                    i_D_OUT_1=0),
                Instance(
                    "SB_IO",
                    p_PIN_TYPE=C(0b010100, 6),  # output registered
                    p_IO_STANDARD="SB_LVCMOS",
                    i_OUTPUT_CLK=ClockSignal("spi"),
                    #i_CLOCK_ENABLE=ce,
                    o_PACKAGE_PIN=spi[i].sdi,
                    i_D_OUT_0=self.busy & sr[i][-1] & enable[i]),
                Instance(
                    "SB_IO",
                    p_PIN_TYPE=C(0b011100, 6),  # output registered inverted
                    p_IO_STANDARD="SB_LVCMOS",
                    i_OUTPUT_CLK=ClockSignal("spi"),
                    #i_CLOCK_ENABLE=ce,
                    o_PACKAGE_PIN=spi[i].nss,
                    i_D_OUT_0=self.busy & enable[i]),
                Instance(
                    "SB_IO",
                    p_PIN_TYPE=C(0b011100, 6),  # output registered inverted
                    p_IO_STANDARD="SB_LVCMOS",
                    i_OUTPUT_CLK=ClockSignal("spi"),
                    #i_CLOCK_ENABLE=ce,
                    o_PACKAGE_PIN=spi[i].ldacn,
                    i_D_OUT_0=enable[i]),
            ]


class Fastino(Module):
    def __init__(self, platform):
        n_bits = 16
        n_frame = 14
        t_clk = 4.

        self.submodules.link = Link(
            clk=platform.request("eem0_p", 0),
            data=[platform.request("eem0_p", i + 1) for i in range(6)],
            platform=platform,
            t_clk=t_clk)

        cd_sys = ClockDomain("sys")
        self.clock_domains += cd_sys
        self.comb += [
            cd_sys.rst.eq(ResetSignal("word")),
            cd_sys.clk.eq(ClockSignal("word")),
        ]
        # platform.add_period_constraint(cd_sys.clk, t_clk*link.n_div)

        self.submodules.frame = Frame()
        self.comb += self.frame.word.eq(self.link.word)

        adr = Signal(4)
        cfg = Record([
            ("rst", 1),
            ("bypass", 1),
            ("latchinputvalue", 1),
            ("reserved", 17),
        ])
        locked = Signal()

        # slow MISO lane, TBD
        sdo = Signal()
        self.specials += [
            Instance(
                "SB_IO",
                p_PIN_TYPE=C(0b010100, 6),  # output registered
                p_IO_STANDARD="SB_LVCMOS",
                i_OUTPUT_CLK=ClockSignal("word"),
                o_PACKAGE_PIN=platform.request("eem0_p", 7),
                i_D_OUT_0=sdo),  # falling SCK
            Instance(
                "SB_IO",
                p_PIN_TYPE=C(0b011100, 6),  # output registered inverted
                p_IO_STANDARD="SB_LVCMOS",
                i_OUTPUT_CLK=ClockSignal("word"),
                o_PACKAGE_PIN=platform.request("eem0_n", 7),
                i_D_OUT_0=sdo),  # falling SCK
        ]
        # n_frame//2 bits per frame to simplify Kasli DDR PHY design
        sr = Signal(n_frame//2, reset_less=True)
        # status register
        status = Signal((1 << len(adr))*n_frame//2)
        status_ = Cat(C(0xfa, 8), locked,
                      self.link.delay[-4:], self.link.delay_relative[-4:],
                      self.link.align_err, self.frame.crc_err, cfg.raw_bits())
        assert len(status_) <= len(status)

        self.comb += [
            status.eq(status_),
            sdo.eq(sr[-1]),  # MSB first
            adr.eq(self.frame.body[len(cfg):]),
        ]
        self.sync += [
            sr[1:].eq(sr),
            If(self.frame.stb,
                cfg.eq(self.frame.body),
                # grab data from status register according to address
                sr.eq(Array([status[i*n_frame//2:(i + 1)*n_frame//2]
                    for i in range(1 << len(adr))])[adr]),
            )
        ]

        # set up spi clock to 7*t_clk*3/4 = 21 ns
        cd_spi = ClockDomain("spi")
        self.clock_domains += cd_spi
        divr = 3 - 1
        divf = 4 - 1
        divq = 4
        t_out = t_clk*self.link.n_div*(divr + 1)/(divf + 1)  # *2**divq
        assert t_out >= 20., t_out
        assert t_out < t_clk*self.link.n_div
        platform.add_period_constraint(cd_spi.clk, t_out)
        # calculate the minimum delay between the 28 ns word clock
        # and the 21 ns SPI clock: 7ns, if this is large enough we don't need a
        # synchronizer (see below)
        print("min sys-spi clock delay", min((i*t_out) % (t_clk*self.link.n_div)
            for i in range(1, (divf + 1)//gcd(divr + 1, divf + 1))))
        t_idle = n_frame*t_clk*self.link.n_div - (n_bits + 1)*t_out
        assert t_idle >= 0, t_idle
        t_vco = t_out/2**divq
        assert .533 <= 1/t_vco <= 1.066, 1/t_vco

        self.specials += [
            Instance(
                "SB_PLL40_CORE",
                p_FEEDBACK_PATH="DELAY",
                p_DIVR=divr,  # input
                p_DIVF=divf,  # feedback
                p_DIVQ=divq,  # vco
                p_FILTER_RANGE=1,
                p_PLLOUT_SELECT="GENCLK",
                p_ENABLE_ICEGATE=1,
                p_DELAY_ADJUSTMENT_MODE_FEEDBACK="DYNAMIC",
                p_FDA_FEEDBACK=0xf,
                p_DELAY_ADJUSTMENT_MODE_RELATIVE="DYNAMIC",
                p_FDA_RELATIVE=0xf,
                i_BYPASS=cfg.bypass,
                i_RESETB=~(cfg.rst | ResetSignal("word")),
                i_DYNAMICDELAY=Cat(self.link.delay[-4:], self.link.delay_relative[-4:]),
                i_REFERENCECLK=ClockSignal("link"),
                i_LATCHINPUTVALUE=cfg.latchinputvalue,
                o_LOCK=locked,
                o_PLLOUTGLOBAL=cd_spi.clk,
                # o_PLLOUTCORE=,
            ),
            AsyncResetSynchronizer(cd_spi, ~locked),
        ]

        self.submodules.spi = MultiSPI(platform)

        assert len(cfg) + len(adr) + len(self.spi.data) == len(self.frame.body)

        # no cdc, assume timing is comensurate
        # max data delay < min sys-spi clock delay
        self.comb += [
            self.spi.stb.eq(self.frame.stb),
            self.spi.data.eq(self.frame.body[-len(self.spi.data):])
        ]

        self.comb += [
            platform.request("dac_clr_n").eq(1),
            Cat(platform.request("test_point", i) for i in range(5)).eq(Cat(
                ClockSignal("link"),
                ClockSignal("word"),
                ResetSignal("word"),
                self.link.sr.slip_req,
                self.frame.stb,
            )),
            Cat(platform.request("user_led", i) for i in range(9)).eq(Cat(
                ~ResetSignal("word"),
                self.link.stb,
                self.spi.busy,
                self.frame.crc_err[0],
                self.link.delay[-4:],
                ~ResetSignal(),  # RED
            )),
        ]


if __name__ == "__main__":
    from fastino import Platform
    platform = Platform()
    fastino = Fastino(platform)
    platform.build(fastino, build_name="fastino")
