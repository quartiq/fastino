from math import gcd

from migen import *
from migen.genlib.cdc import MultiReg, AsyncResetSynchronizer, BlindTransfer
from misoc.cores.liteeth_mini.mac.crc import LiteEthMACCRCEngine


class SlipSR(Module):
    def __init__(self, n_lanes=6, n_div=7):
        self.data = Signal(n_lanes)
        self.slip_req = Signal()
        self.word = Signal(n_div*n_lanes, reset_less=True)
        # n_word = n_lanes*n_div = 42

        buf = Signal.like(self.word)
        slipped = Signal.like(buf)
        clk0 = Signal()
        i = Signal(n_div, reset=1)
        slip_n = Signal()
        self.sync.sr += [
            buf.eq(Cat(self.data, buf)),
            clk0.eq(ClockSignal("word")),
            slip_n.eq(~(self.slip_req & ClockSignal("word") & ~clk0)),
            If(slip_n,
                i.eq(Cat(i[-1], i)),
            ),
            If(i[-1],
                slipped.eq(buf),
            ),
        ]
        self.sync.word += [
            self.word.eq(slipped),
        ]


class Link(Module):
    def __init__(self, clk, data, platform, t_clk=4.):
        self.n_div = 7
        n_lanes = len(data)

        # link clocking
        cd_link = ClockDomain("link", reset_less=True)
        platform.add_period_constraint(cd_link.clk, t_clk*self.n_div)
        self.clock_domains += cd_link

        cd_word = ClockDomain("word")
        platform.add_period_constraint(cd_word.clk, t_clk*self.n_div)
        self.clock_domains += cd_word

        cd_sr = ClockDomain("sr", reset_less=True)
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
        self.delay = Signal(8, reset=0x80, reset_less=True)
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
        self.submodules.sr = SlipSR(n_lanes=2*(n_lanes + 1), n_div=self.n_div)

        helper = Signal(n_lanes + 1)
        self.specials += [
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
            Instance(
                "SB_DFFN",
                i_D=helper[i + 1],
                i_C=cd_sr.clk,
                o_Q=self.sr.data[n_lanes + i + 2],
            )]
            for i in range(n_lanes)
        ]

        self.payload = Signal(n_lanes*self.n_div)
        slip_good = Signal()
        delay_delta = Signal((7, True))
        settle = Signal(max=64, reset=63, reset_less=True)  # delay settling timer
        settle_done = Signal()
        self.align_err = Signal(8, reset_less=True)
        self.stb = Signal()

        n = len(self.sr.data)
        lanes = [Signal(self.n_div) for i in range(n)]

        self.comb += [
            Cat(lanes).eq(Cat(self.sr.word[i::n] for i in range(n))),
            # EEM inversion
            self.payload.eq(~Cat(self.sr.word[i*n + 1:i*n + 1 + n_lanes]
                for i in range(self.n_div))),
            settle_done.eq(settle == 0),
            self.stb.eq(slip_good),
            slip_good.eq(~lanes[0][1:3] == 0b01),
        ]
        self.sync.word += [
            # this is with the timing helper DFFNs on the edge samples:
            # edge sample i is half a cycle older than ref sample i
            # and half a cycle younger than ref sample i + 1
            delay_delta.eq(sum(
                # if the two reference samples around the edge sample are the
                # same, then don't adjust
                Mux(ref[i + 1] == ref[i], 0,
                    # if the edge sample matches the sample after the edge
                    # then sampling is late:
                    # increment the feedback delay for earlier sampling
                    Mux(edge[i] == ref[i], 1, (1 << len(delay_delta)) - 1))
                for ref, edge in zip(lanes[:n_lanes + 1], lanes[n_lanes + 1:])
                for i in range(self.n_div - 1)
            )),

            self.sr.slip_req.eq(0),
            If(~settle_done,
                settle.eq(settle - 1),
            ).Else(
                If(~slip_good,
                    self.sr.slip_req.eq(1),
                    self.align_err.eq(self.align_err + 1),
                    settle.eq(settle.reset),
                ),
                If(delay_delta >= 10,
                    If(self.delay != 0xff,
                        self.delay.eq(self.delay + 1),
                        If(self.delay[:4] == 0xf,
                            settle.eq(settle.reset),
                        ),
                    )
                ),
                If(delay_delta <= -10,
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
    def __init__(self, n_frame=14):
        n = 7*6
        self.payload = Signal(n)
        # TODO: invalidate marker, checksum on ~link.stb
        self.submodules.crc = LiteEthMACCRCEngine(
            data_width=n, width=12, polynom=0x80f)  # crc-12 telco
        self.checksum = Signal(len(self.crc.last), reset_less=True)
        self.body = Signal(n_frame*n - n_frame//2 - 1 - len(self.checksum))
        self.stb = Signal()
        self.crc_err = Signal(8, reset_less=True)
        self.crc_good = Signal()

        eof = Signal()
        frame = Signal(n*n_frame, reset_less=True)

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

        body_ = Cat(
            # checksum
            frame[len(self.checksum):n],
            # skip eof marker bits
            [frame[1 + i*n:(i + 1)*n] for i in range(1, 2 + n_frame//2)],
            # complete words
            frame[(2 + n_frame//2)*n:],
        )
        assert len(body_) == len(self.body)

        self.comb += [
            self.crc.last.eq(self.checksum),
            self.crc.data[::-1].eq(self.payload),
            self.body.eq(body_),
            self.crc_good.eq(self.crc.next == 0),
        ]
        self.sync += [
            eof.eq(Cat(self.payload[0], [frame[i*n] for i in range(n_frame//2)]) == 1),
            self.stb.eq(eof & self.crc_good),
            frame.eq(Cat(self.payload, frame)),
            self.checksum.eq(self.crc.next),
            If(eof,
                self.checksum.eq(0),
            ),
            If(eof & ~self.crc_good,
                self.crc_err.eq(self.crc_err + 1),
            ),
        ]


class MultiSPI(Module):
    def __init__(self, platform, n_channels=32, n_bits=16):
        n = n_channels*(n_bits + 1)
        self.data = Signal(n)
        self.stb = Signal()

        vhdci = [platform.request("vhdci", i) for i in range(2)]
        idc = [platform.request("idc", i) for i in range(0)]
        mosi = vhdci[0].io
        sck = vhdci[1].io
        cs = []
        ldac = []
        for _ in idc:
            cs.extend(_.io[i] for i in range(8))
            ldac.extend(_.io[i + 4] for i in range(4))
        self.comb += [
            [_.dir.eq(0b1111) for _ in vhdci],
            [_.dir.eq(1) for _ in idc],
            platform.request("drv_oe_n").eq(0),
        ]

        self.busy = Signal()
        i = Signal(max=n_bits)
        sr = [Signal(n_bits, reset_less=True) for i in range(n_channels)]
        mask = Signal(n_channels, reset_less=True)
        assert len(Cat(sr, mask)) == n

        self.sync.spi += [
            [sri[1:].eq(sri) for sri in sr],  # MSB first
            If(~self.busy & self.stb,
                self.busy.eq(1),
                Cat(mask, sr).eq(self.data)
            ),
            If(self.busy,
                i.eq(i + 1),
            ),
            If(i == n_bits - 1,
                self.busy.eq(0),
            ),
        ]
        ce = Signal(1)
        self.comb += ce.eq(mask != 0)
        for i in range(n_channels):
            self.specials += [
                Instance(
                    "SB_IO",
                    p_PIN_TYPE=C(0b010100, 6),  # output registered
                    p_IO_STANDARD="SB_LVCMOS",
                    i_OUTPUT_CLK=ClockSignal("spi"),
                    #i_CLOCK_ENABLE=ce,
                    o_PACKAGE_PIN=mosi[i],
                    i_D_OUT_0=self.busy & sr[i][-1] & mask[i]),
                #Instance(
                #    "SB_IO",
                #    p_PIN_TYPE=C(0b011100, 6),  # output registered inverted
                #    p_IO_STANDARD="SB_LVCMOS",
                #    i_OUTPUT_CLK=ClockSignal("spi"),
                #    #i_CLOCK_ENABLE=ce,
                #    o_PACKAGE_PIN=cs[i],
                #    i_D_OUT_0=self.busy & mask[i]),
                #Instance(
                #    "SB_IO",
                #    p_PIN_TYPE=C(0b011100, 6),  # output registered inverted
                #    p_IO_STANDARD="SB_LVCMOS",
                #    i_OUTPUT_CLK=ClockSignal("spi"),
                #    #i_CLOCK_ENABLE=ce,
                #    o_PACKAGE_PIN=ldac[i],
                #    i_D_OUT_0=mask[i]),
                Instance(
                    "SB_IO",
                    p_PIN_TYPE=C(0b010000, 6),  # output registered DDR
                    p_IO_STANDARD="SB_LVCMOS",
                    i_OUTPUT_CLK=ClockSignal("spi"),
                    #i_CLOCK_ENABLE=ce,
                    o_PACKAGE_PIN=sck[i],
                    i_D_OUT_0=self.busy & mask[i],
                    i_D_OUT_1=0),
            ]


class Fastino(Module):
    def __init__(self, platform):
        n_bits = 16
        n_frame = 14
        t_clk = 4.

        self.submodules.link = Link(
            clk=platform.request("eem2_n", 0),
            data=[platform.request("eem2_n", i + 1) for i in range(6)],
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
        self.comb += [
            self.frame.payload.eq(self.link.payload),
        ]

        adr = Signal(4)
        cfg = Record([
            ("rst", 1),
            ("bypass", 1),
            ("latchinputvalue", 1),
            ("reserved", 17),
        ])
        locked = Signal()

        sdo = Signal()
        self.specials += [
            Instance(
                "SB_IO",
                p_PIN_TYPE=C(0b010100, 6),  # output registered
                p_IO_STANDARD="SB_LVCMOS",
                i_OUTPUT_CLK=ClockSignal("word"),
                o_PACKAGE_PIN=platform.request("eem2_p", 7),
                i_D_OUT_0=sdo),  # falling SCK
            Instance(
                "SB_IO",
                p_PIN_TYPE=C(0b011100, 6),  # output registered inverted
                p_IO_STANDARD="SB_LVCMOS",
                i_OUTPUT_CLK=ClockSignal("word"),
                o_PACKAGE_PIN=platform.request("eem2_n", 7),
                i_D_OUT_0=sdo),  # falling SCK
        ]
        sr = Signal(n_frame//2, reset_less=True)
        status = Signal((1 << len(adr))*n_frame//2)
        status_ = Cat(C(0xfa, 8), locked,
                      self.link.delay[-4:], self.link.delay_relative[-4:],
                      self.link.align_err, self.frame.crc_err, cfg.raw_bits())
        assert len(status_) <= len(status)

        self.comb += [
            status.eq(status_),
            sdo.eq(sr[-1]),
            adr.eq(self.frame.body[len(cfg):]),
        ]
        self.sync += [
            sr[1:].eq(sr),
            If(self.frame.stb,
                cfg.eq(self.frame.body),
                sr.eq(Array([status[i*n_frame//2:(i + 1)*n_frame//2]
                    for i in range(1 << len(adr))])[adr]),
            )
        ]

        cd_spi = ClockDomain("spi")
        self.clock_domains += cd_spi
        divr = 3 - 1
        divf = 4 - 1
        divq = 4
        t_out = t_clk*self.link.n_div*(divr + 1)/(divf + 1)  # *2**divq
        assert t_out >= 20., t_out
        assert t_out < t_clk*self.link.n_div
        platform.add_period_constraint(cd_spi.clk, t_out)
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

        if False:
            xfer = BlindTransfer("sys", "spi", len(self.spi.data))
            assert len(self.spi.data) <= len(self.frame.body)
            self.submodules += xfer
            self.comb += [
                xfer.i.eq(self.frame.stb),
                xfer.data_i.eq(self.frame.body[-len(xfer.data_i):]),
                self.spi.stb.eq(xfer.o),
                self.spi.data.eq(xfer.data_o),
            ]
        else:
            # no cdc, assume timing is comensurate
            # max data delay <= min clock delay
            self.comb += [
                self.spi.stb.eq(self.frame.stb),
                self.spi.data.eq(self.frame.body[-len(self.spi.data):])
            ]


        self.comb += [
            platform.request("user_led").eq(~ResetSignal()),
            platform.request("test_point").eq(ClockSignal("word")),
        ]
        idc = [platform.request("idc", i) for i in range(8)]
        self.comb += [
            [_.dir.eq(1) for _ in idc],
            [_.io.eq(0) for _ in idc],
            Cat(idc[0].io).eq(
                Cat(self.link.delay[-4:], self.link.delay_relative[-4:])),
            idc[4].io.eq(Cat(
                ClockSignal("link"),
                ClockSignal("word"),
                ResetSignal("word"),
                self.link.sr.slip_req,
                self.spi.busy,
                self.frame.crc_good,
                self.frame.stb,
                self.frame.crc_err[0])),
        ]



if __name__ == "__main__":
    from banker import Platform
    platform = Platform()
    fastino = Fastino(platform)
    platform.build(fastino, build_name="fastino")
