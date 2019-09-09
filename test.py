from migen import *
from migen.genlib.cdc import MultiReg, AsyncResetSynchronizer, BlindTransfer


class Banker(Module):
    def __init__(self, platform):
        n_channels = 32
        n_bits = 16
        n_lanes = 6

        self.comb += platform.request("drv_oe_n").eq(0)

        eem2_0 = platform.request("eem2_n", 0)
        cd_sys = ClockDomain("sys")
        platform.add_period_constraint(cd_sys.clk, 8.)
        self.clock_domains += cd_sys
        self.specials += [
            Instance(
                "SB_GB_IO",
                p_PIN_TYPE=C(0b000000, 6),
                p_IO_STANDARD="SB_LVDS_INPUT",
                io_PACKAGE_PIN=eem2_0,
                o_GLOBAL_BUFFER_OUTPUT=cd_sys.clk),
        ]

        cd_spi = ClockDomain("spi")
        cd_spi2 = ClockDomain("spi2")
        self.clock_domains += cd_spi, cd_spi2
        platform.add_period_constraint(cd_spi.clk, 20.)
        platform.add_period_constraint(cd_spi2.clk, 10.)
        rst = Signal()
        delay = Signal(8)
        locked = Signal()
        bypass = Signal()
        latchinputvalue = Signal()
        self.specials += [
            Instance(
                "SB_PLL40_2F_CORE",
                p_FEEDBACK_PATH="PHASE_AND_DELAY",   # out = in/r*f, vco = out*2**q
                p_DIVR=5-1,  # input
                p_DIVF=4-1,  # feedback
                p_DIVQ=3,  # vco
                p_FILTER_RANGE=2,
                p_SHIFTREG_DIV_MODE=0,  # div-by-4
                p_DELAY_ADJUSTMENT_MODE_FEEDBACK="DYNAMIC",
                p_FDA_FEEDBACK=0,
                p_DELAY_ADJUSTMENT_MODE_RELATIVE="DYNAMIC",
                p_FDA_RELATIVE=0,
                p_PLLOUT_SELECT_PORTA="GENCLK_HALF",
                p_PLLOUT_SELECT_PORTB="GENCLK",
                p_ENABLE_ICEGATE_PORTA=1,
                p_ENABLE_ICEGATE_PORTB=1,
                i_BYPASS=bypass,
                i_RESETB=~rst,
                i_DYNAMICDELAY=delay,
                i_REFERENCECLK=cd_sys.clk,
                i_LATCHINPUTVALUE=latchinputvalue,
                o_LOCK=locked,
                o_PLLOUTGLOBALA=cd_spi.clk,
                # o_PLLOUTCOREA=,
                o_PLLOUTGLOBALB=cd_spi2.clk,
                # o_PLLOUTCOREB=,
            ),
            AsyncResetSynchronizer(cd_spi, ~locked),
            AsyncResetSynchronizer(cd_spi2, ~locked),
        ]

        dat = Signal(2*n_lanes)
        self.specials += [
            Instance(
                "SB_IO",
                p_PIN_TYPE=(0b0000 << 2) |  # no output
                           (0b00 << 0),     # i registered ddr
                p_IO_STANDARD="SB_LVDS_INPUT",
                i_INPUT_CLK=cd_sys.clk,
                i_OUTPUT_ENABLE=0,
                o_D_IN_0=dat[i*2 + 0],
                o_D_IN_1=dat[i*2 + 1],
                io_PACKAGE_PIN=platform.request("eem2_n", i + 2)
            ) for i in range(len(dat)//2)]
        #dat, dat0 = Signal(len(dat), reset_less=True), dat
        #self.sync += dat.eq(dat0)

        n = 64
        sr = [Signal(n, reset_less=True) for i in range(n_lanes*2)]
        assert len(Cat(sr)) == len(dat)*n
        i = Signal(max=n)
        stb = Signal()
        self.comb += [
            stb.eq(~sr[0][0] & dat[0]),
        ]
        self.sync += [
            # MSB first
            [srj.eq(Cat(dat[j], srj)) for j, srj in enumerate(sr)],
            i.eq(i + 1),
            If(stb,
                i.eq(1),
            )
        ]
        sdo = Signal(reset_less=True)
        self.specials += [
            Instance(
                "SB_IO",
                p_PIN_TYPE=C(0b010100, 6),  # output registered
                p_IO_STANDARD="SB_LVCMOS",
                i_OUTPUT_CLK=cd_sys.clk,
                o_PACKAGE_PIN=platform.request("eem2_p", 1),
                i_D_OUT_0=sdo),  # falling SCK
            Instance(
                "SB_IO",
                p_PIN_TYPE=C(0b011100, 6),  # output registered inverted
                p_IO_STANDARD="SB_LVCMOS",
                i_OUTPUT_CLK=cd_sys.clk,
                o_PACKAGE_PIN=platform.request("eem2_n", 1),
                i_D_OUT_0=sdo),  # falling SCK
        ]

        # TODO
        crc = Signal(n_bits)
        crc_good = Signal(reset=1)
        adr = Signal(n_bits)
        cfg = Signal(n_bits)
        crc_en = Signal()
        self.comb += Cat(delay, rst, crc_en, bypass, latchinputvalue).eq(cfg)
        self.sync += sdo.eq(crc_good & stb)
        # available; sr[2] and remaining 48 bits of sr[1]

        # TODO: could be trimmed for lower latency as 20ns/8ns can
        # be tuned to have 4ns S/H margin
        xfer = BlindTransfer("sys", "spi", n_channels*(n_bits + 1))
        self.submodules += xfer
        assert len(Cat(xfer.data_i, crc, adr)) == len(Cat(sr[3:]))
        self.comb += [
            xfer.i.eq(stb & crc_good),
            Cat(xfer.data_i, crc, adr).eq(Cat(sr[3:])),
        ]
        self.sync += [
            If(xfer.i,
                Cat(cfg).eq(sr[1]),
            )
        ]

        mosi = platform.request("vhdci", 0)
        assert len(mosi.io) == n_channels
        cs = platform.request("vhdci", 1)
        assert len(cs.io) == n_channels
        self.comb += [
            mosi.dir.eq(0b1111),
            cs.dir.eq(0b1111)
        ]

        csi = Signal()
        i = Signal(max=n_bits)
        self.sync.spi += [
            If(xfer.o,
                csi.eq(1),
            ),
            If(csi,
                i.eq(i + 1),
            ),
            If(i == n_bits - 1,
                csi.eq(0),
            ),
        ]
        n = len(xfer.data_o)
        sr = [Signal(n_bits, reset_less=True) for i in range(n_channels)]
        mask = Signal(n_channels, reset_less=True)
        assert len(Cat(sr, mask)) == len(xfer.data_o)
        self.sync.spi += [
            [sri[1:].eq(sri) for sri in sr[:n_channels]],  # MSB first
            If(xfer.o,
                Cat(sr, mask).eq(xfer.data_o)
            ),
        ]
        for i, sri in enumerate(sr[:n_channels]):
            self.specials += [
                Instance(
                    "SB_IO",
                    p_PIN_TYPE=C(0b010100, 6),  # output registered
                    p_IO_STANDARD="SB_LVCMOS",
                    i_OUTPUT_CLK=cd_spi.clk,
                    o_PACKAGE_PIN=mosi.io[i],
                    i_D_OUT_0=sri[0] & ~mask[i]),
                Instance(
                    "SB_IO",
                    p_PIN_TYPE=C(0b010100, 6),  # output registered
                    p_IO_STANDARD="SB_LVCMOS",
                    i_OUTPUT_CLK=cd_spi.clk,
                    o_PACKAGE_PIN=cs.io[i],
                    i_D_OUT_0=csi & ~mask[i]),
            ]

        self.comb += platform.request("user_led").eq(locked)


if __name__ == "__main__":
    from banker import Platform
    platform = Platform()
    banker = Banker(platform)
    platform.build(banker)
