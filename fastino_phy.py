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
        self.clk_buf = Signal()

        cd_link = ClockDomain("link", reset_less=True)
        platform.add_period_constraint(cd_link.clk, t_clk*self.n_div)
        self.clock_domains += cd_link
        self.comb += cd_link.clk.eq(self.clk_buf)

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
        self.delay = Signal(6, reset=0x20, reset_less=True)
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
                i_REFERENCECLK=self.clk_buf,
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
        self.submodules.sr = SlipSR(n_lanes=n_lanes + 2, n_div=self.n_div)

        helper = Signal(2*(n_lanes + 1))
        self.specials += [
            Instance(
                "SB_GB_IO",
                p_PIN_TYPE=0b000000,  # no output, i registered
                p_IO_STANDARD="SB_LVDS_INPUT",
                i_INPUT_CLK=cd_sr.clk,
                o_D_IN_0=self.sr.data[0],  # rising
                o_D_IN_1=helper[1],  # falling
                o_GLOBAL_BUFFER_OUTPUT=self.clk_buf,
                i_PACKAGE_PIN=clk,
            ),
            Instance(
                "SB_DFFN",
                i_D=helper[1],
                i_C=cd_sr.clk,
                o_Q=self.sr.data[1],
            ),
        ] + [
            [Instance(
                "SB_IO",
                p_PIN_TYPE=0b000000,
                p_IO_STANDARD="SB_LVDS_INPUT",
                i_INPUT_CLK=cd_sr.clk,
                o_D_IN_0=self.sr.data[i + 2],  # rising
                # o_D_IN_1=,  # falling
                # TODO: could be used for common/majority edge sampling on all data lanes
                i_PACKAGE_PIN=data[i],
            )]
            for i in range(n_lanes)
        ]

        self.payload = Signal(n_lanes*self.n_div)
        clk0 = Signal(7)
        clk1 = Signal(7)
        slip_good = Signal()
        delay_inc = Signal()
        delay_dec = Signal()
        settle = Signal(max=64, reset=63, reset_less=True)  # delay settling timer
        settle_done = Signal()
        self.align_err = Signal(8, reset_less=True)
        self.stb = Signal()

        n = len(self.sr.data)
        self.comb += [
            # EEM inversion
            clk0.eq(~self.sr.word[::n]),
            clk1.eq(~self.sr.word[1::n]),
            self.payload.eq(~Cat([self.sr.word[2 + i*n:(i + 1)*n]
                for i in range(self.n_div)])),
            # cycle 0 1 2 3 4 5 6
            # clk0  1 1 0 0 0 1 1
            # two of the quadrature samples should hit an edge
            # this is with the timing helper DFFN
            # clk1 1 1 x 0 0 x 1  # optimal, on edge
            # clk1 1 1 0 0 0 1 1  # late
            # clk1 1 1 1 0 0 0 1  # early
            slip_good.eq(clk0 == 0b1100011),
            # late sampling, decrease clock delay, increase feedback delay
            delay_inc.eq(clk1 == 0b1100011),
            # early sampling, increase clock delay, decrease feedback delay
            delay_dec.eq(clk1 == 0b1110001),
            settle_done.eq(settle == 0),
            self.stb.eq(slip_good & settle_done),
        ]
        self.sync.word += [
            self.sr.slip_req.eq(0),
            If(~settle_done,
                settle.eq(settle - 1),
            ).Else(
                If(~slip_good,
                    self.sr.slip_req.eq(1),
                    self.align_err.eq(self.align_err + 1),
                    self.delay.eq(self.delay.reset),
                    settle.eq(settle.reset),
                ).Elif(delay_inc,
                    If(self.delay != (1 << len(self.delay)) - 1,
                        self.delay.eq(self.delay + 1),
                        settle.eq(settle.reset),
                    )
                ).Elif(delay_dec,
                    If(self.delay != 0,
                        self.delay.eq(self.delay - 1),
                        settle.eq(settle.reset),
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
        eof_good = Signal()

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

        csi = Signal()
        i = Signal(max=n_bits)
        sr = [Signal(n_bits, reset_less=True) for i in range(n_channels)]
        mask = Signal(n_channels, reset_less=True)
        assert len(Cat(sr, mask)) == n

        self.sync.spi += [
            [sri[1:].eq(sri) for sri in sr],  # MSB first
            If(~csi & self.stb,
                csi.eq(1),
                Cat(mask, sr).eq(self.data)
            ),
            If(csi,
                i.eq(i + 1),
            ),
            If(i == n_bits - 1,
                csi.eq(0),
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
                    i_D_OUT_0=csi & sr[i][-1] & mask[i]),
                #Instance(
                #    "SB_IO",
                #    p_PIN_TYPE=C(0b011100, 6),  # output registered inverted
                #    p_IO_STANDARD="SB_LVCMOS",
                #    i_OUTPUT_CLK=ClockSignal("spi"),
                #    #i_CLOCK_ENABLE=ce,
                #    o_PACKAGE_PIN=cs[i],
                #    i_D_OUT_0=csi & mask[i]),
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
                    i_D_OUT_0=csi & mask[i],
                    i_D_OUT_1=0),
            ]


class Fastino(Module):
    def __init__(self, platform):
        n_bits = 16
        n_frame = 14
        t_clk = 4.

        link = Link(
            clk=platform.request("eem2_n", 0),
            data=[platform.request("eem2_n", i + 1) for i in range(6)],
            platform=platform,
            t_clk=t_clk)
        self.submodules += link

        cd_sys = ClockDomain("sys")
        self.clock_domains += cd_sys
        self.comb += [
            cd_sys.rst.eq(ResetSignal("word")),
            cd_sys.clk.eq(ClockSignal("word")),
        ]
        # platform.add_period_constraint(cd_sys.clk, t_clk*link.n_div)

        frame = Frame()
        self.submodules += frame
        self.comb += [
            frame.payload.eq(link.payload),
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
                i_OUTPUT_CLK=cd_sys.clk,
                o_PACKAGE_PIN=platform.request("eem2_p", 7),
                i_D_OUT_0=sdo),  # falling SCK
            Instance(
                "SB_IO",
                p_PIN_TYPE=C(0b011100, 6),  # output registered inverted
                p_IO_STANDARD="SB_LVCMOS",
                i_OUTPUT_CLK=cd_sys.clk,
                o_PACKAGE_PIN=platform.request("eem2_n", 7),
                i_D_OUT_0=sdo),  # falling SCK
        ]
        sr = Signal(n_frame//2, reset_less=True)
        status = Signal((1 << len(adr))*n_frame//2)
        status_ = Cat(C(0xfa, 8), locked,
                      link.delay, link.delay_relative,
                      link.align_err, frame.crc_err, cfg.raw_bits())
        assert len(status_) <= len(status)

        self.comb += [
            status.eq(status_),
            sdo.eq(sr[-1]),
            adr.eq(frame.body[len(cfg):]),
        ]
        self.sync += [
            sr[1:].eq(sr),
            If(frame.stb,
                cfg.eq(frame.body),
                sr.eq(Array([status[i*n_frame//2:(i + 1)*n_frame//2]
                    for i in range(1 << len(adr))])[adr]),
            )
        ]

        self.comb += [
            platform.request("user_led").eq(~ResetSignal()),
            platform.request("test_point").eq(link.clk_buf),
        ]
        idc = [platform.request("idc", i) for i in range(8)]
        self.comb += [
            [_.dir.eq(1) for _ in idc],
            [_.io.eq(0) for _ in idc],
            Cat(idc[0].io).eq(Cat(link.delay, link.delay_relative)),
            Cat(idc[2].io, idc[3].io).eq(link.payload),
            idc[4].io.eq(Cat(
                ClockSignal("word"),
                ResetSignal("word"),
                link.sr.slip_req,
                link.stb,
                link.align_err[0],
                frame.crc_good,
                frame.stb,
                frame.crc_err[0])),
            #idc[1].io.eq(link.sr.word[0::len(link.sr.data)]),
            Cat(idc[5].io).eq(Cat(frame.crc.next)),
            idc[6].io.eq(link.sr.word[::len(link.sr.data)]),
            idc[7].io.eq(link.sr.word[1::len(link.sr.data)]),
        ]

        cd_spi = ClockDomain("spi")
        self.clock_domains += cd_spi
        divr = 3 - 1
        divf = 4 - 1
        divq = 4
        t_out = t_clk*link.n_div*(divr + 1)/(divf + 1)  # *2**divq
        assert t_out >= 20., t_out
        assert t_out < t_clk*link.n_div
        platform.add_period_constraint(cd_spi.clk, t_out)
        print("min sys-spi clock delay", min((i*t_out) % (t_clk*link.n_div)
            for i in range(1, (divf + 1)//gcd(divr + 1, divf + 1))))
        t_idle = n_frame*t_clk*link.n_div - (n_bits + 1)*t_out
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
                i_RESETB=~(cfg.rst | cd_sys.rst),
                i_DYNAMICDELAY=Cat(link.delay[-4:], link.delay_relative[-4:]),
                i_REFERENCECLK=link.clk_buf,
                i_LATCHINPUTVALUE=cfg.latchinputvalue,
                o_LOCK=locked,
                o_PLLOUTGLOBAL=cd_spi.clk,
                # o_PLLOUTCORE=,
            ),
            AsyncResetSynchronizer(cd_spi, ~locked),
        ]

        spi = MultiSPI(platform)
        self.submodules += spi

        assert len(cfg) + len(adr) + len(spi.data) == len(frame.body)

        if False:
            xfer = BlindTransfer("sys", "spi", len(spi.data))
            assert len(spi.data) <= len(frame.body)
            self.submodules += xfer
            self.comb += [
                xfer.i.eq(frame.stb),
                xfer.data_i.eq(frame.body[-len(xfer.data_i):]),
                spi.stb.eq(xfer.o),
                spi.data.eq(xfer.data_o),
            ]
        else:
            # no cdc, assume timing is comensurate
            # max data delay <= min clock delay
            self.comb += [
                spi.stb.eq(frame.stb),
                spi.data.eq(frame.body[-len(spi.data):])
            ]


if __name__ == "__main__":
    from banker import Platform
    platform = Platform()
    fastino = Fastino(platform)
    platform.build(fastino, build_name="fastino")
