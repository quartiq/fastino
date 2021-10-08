from math import gcd

from migen import *
from migen.genlib.cdc import AsyncResetSynchronizer

from interpolator import Interpolator
from frame import Frame
from multispi import MultiSPI
from link import Link


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
            ("afe_pwr_n", 1),
            ("dac_clr", 1),
            ("clr_err", 1),
            ("led", 8),
            ("typ", 1),
            ("reserved", 7),
        ])
        unlock = Signal(reset=1)

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
        sr = Signal(n_frame, reset_less=True)
        # status register
        status = Signal((1 << len(adr))*len(sr))
        status_ = Cat(C(0xfa, 8),  # ID
                      platform.request("cbsel"),
                      platform.request("sw"),
                      platform.request("hw_rev"),
                      C(0, 3),  # gw version
                      unlock,
                      self.link.delay,
                      self.link.align_err,
                      self.frame.crc_err,
                      cfg.raw_bits())
        assert len(status_) <= len(status)

        self.comb += [
            status.eq(status_),
            sdo.eq(sr[-1]),  # MSB first
            adr.eq(self.frame.body[len(cfg):]),
        ]
        have_crc_err = Signal()
        self.sync += [
            sr[1:].eq(sr),
            If(self.frame.stb,
                cfg.raw_bits().eq(self.frame.body),
                # grab data from status register according to address
                sr.eq(Array([status[i*n_frame:(i + 1)*n_frame]
                    for i in range(1 << len(adr))])[adr]),
            ),
            If(cfg.clr_err,
                unlock.eq(0),
                self.frame.crc_err.eq(0),
            ),
            have_crc_err.eq(self.frame.crc_err != 0),
        ]
        have_align_err = Signal()
        self.sync.word += [
            If(cfg.clr_err,
                self.link.align_err.eq(0),
            ),
            have_align_err.eq(self.link.align_err != 0)
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
        print("Check 'Max delay posedge sys_clk -> posedge spi_clk' <",
            min((i*t_out) % (t_clk*self.link.n_div)
                for i in range(1, (divf + 1)//gcd(divr + 1, divf + 1))))
        t_idle = n_frame*t_clk*self.link.n_div - (n_bits + 1)*t_out
        assert t_idle >= 0, t_idle
        t_vco = t_out/2**divq
        assert .533 <= 1/t_vco <= 1.066, 1/t_vco

        locked = Signal()
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
                i_BYPASS=0,
                i_RESETB=~(cfg.rst | ResetSignal("word")),
                i_DYNAMICDELAY=Cat(self.link.delay, self.link.delay_relative),
                i_REFERENCECLK=ClockSignal("link"),
                i_LATCHINPUTVALUE=0,
                o_LOCK=locked,
                o_PLLOUTGLOBAL=cd_spi.clk,
                # o_PLLOUTCORE=,
            ),
            AsyncResetSynchronizer(cd_spi, ~locked),
        ]

        self.submodules.int0 = ClockDomainsRenamer("spi")(Interpolator)(
            n_channels=16)
        self.submodules.int1 = ClockDomainsRenamer("spi")(Interpolator)(
            n_channels=16)
        stb0 = Signal(reset_less=True)

        # no cdc, assume timing is synchronous and comensurate such that
        # max data delay sys-spi < min sys-spi clock delay over all alignments
        body = Cat(
                self.int0.en_in,
                self.int1.en_in,
                self.int0.data,
                self.int1.data,
        )
        self.sync.spi += [
            stb0.eq(~self.frame.stb),
            self.int0.stb.eq(self.frame.stb & stb0),
            self.int1.stb.eq(self.frame.stb & stb0),
            body.eq(self.frame.body[-len(body):]),
            self.int0.typ.eq(self.frame.body[len(cfg) - 8]),
            self.int1.typ.eq(self.frame.body[len(cfg) - 8]),
        ]

        self.submodules.spi = MultiSPI(platform)
        assert len(body) == len(self.spi.data)
        assert len(cfg) + len(adr) + len(self.spi.data) == len(self.frame.body)

        body = Cat(
                self.int0.en,
                self.int1.en,
                self.int0.y,
                self.int1.y,
        )
        assert len(body) == len(self.spi.data)
        self.comb += [
            self.spi.stb.eq(self.int0.valid),
            self.spi.data.eq(body),
        ]

        self.comb += [
            platform.request("dac_clr_n").eq(~cfg.dac_clr),
            platform.request("en_afe_pwr").eq(~cfg.afe_pwr_n),
            #platform.request("test_point", 2).eq(self.link.delay[0]),
            #platform.request("test_point", 3).eq(self.link.delay[1]),
            platform.request("test_point", 3).eq(self.spi.busy),
            platform.request("test_point", 4).eq(self.frame.stb),
            Cat(platform.request("user_led", i) for i in range(9)).eq(Cat(
                cfg.led,
                ResetSignal("spi") | have_align_err | have_crc_err,  # RED
            )),
        ]
        self.specials += [
            Instance(
                "SB_IO",
                p_PIN_TYPE=C(0b010000, 6),  # output registered DDR
                p_IO_STANDARD="SB_LVCMOS",
                i_OUTPUT_CLK=ClockSignal("link"),
                #i_CLOCK_ENABLE=1,
                o_PACKAGE_PIN=platform.request("test_point", 0),
                i_D_OUT_0=1,
                i_D_OUT_1=0),
            Instance(
                "SB_IO",
                p_PIN_TYPE=C(0b010000, 6),  # output registered DDR
                p_IO_STANDARD="SB_LVCMOS",
                i_OUTPUT_CLK=ClockSignal("word"),
                #i_CLOCK_ENABLE=1,
                o_PACKAGE_PIN=platform.request("test_point", 1),
                i_D_OUT_0=1,
                i_D_OUT_1=0),
        ]


if __name__ == "__main__":
    from fastino import Platform
    platform = Platform()
    fastino = Fastino(platform)
    platform.build(fastino, build_name="fastino")
