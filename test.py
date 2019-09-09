from migen import *
from migen.genlib.cdc import MultiReg, AsyncResetSynchronizer, BlindTransfer


class Banker(Module):
    def __init__(self, platform):
        n_channels = 32
        n_bits = 16
        n_lanes = 6
        t_clk = 4.
        n_div = 7
        n_frame = 14

        self.comb += platform.request("drv_oe_n").eq(0)

        clk = Signal()
        cd_sys = ClockDomain("sys")
        platform.add_period_constraint(cd_sys.clk, t_clk*n_div)
        self.clock_domains += cd_sys

        cd_sr = ClockDomain("sr")
        self.clock_domains += cd_sr
        divr = 1 - 1
        divf = 7 - 1
        divq = 2
        t_out = t_clk*n_div*(divr + 1)/(divf + 1)
        assert t_out == 4.
        t_vco = t_out/2**divq
        assert .533 <= 1/t_vco <= 1.066, 1/t_vco
        platform.add_period_constraint(cd_sr.clk, t_out)

        locked0 = Signal()
        delay0 = Signal(8)
        self.specials += [
            Instance(
                "SB_PLL40_2F_CORE",
                p_FEEDBACK_PATH="DELAY",   # out = in/r*f, vco = out*2**q
                p_DIVR=divr,  # input
                p_DIVF=divf,  # feedback
                p_DIVQ=divq,  # vco
                p_FILTER_RANGE=3,
                p_SHIFTREG_DIV_MODE=int(n_div == 7),  # div-by-7
                p_DELAY_ADJUSTMENT_MODE_FEEDBACK="DYNAMIC",
                p_FDA_FEEDBACK=0,
                p_DELAY_ADJUSTMENT_MODE_RELATIVE="DYNAMIC",
                p_FDA_RELATIVE=0,
                p_PLLOUT_SELECT_PORTA="SHIFTREG_0deg",
                p_PLLOUT_SELECT_PORTB="GENCLK",
                p_ENABLE_ICEGATE_PORTA=0,
                p_ENABLE_ICEGATE_PORTB=0,
                # i_BYPASS=0,
                i_RESETB=1,
                i_DYNAMICDELAY=delay0,
                i_REFERENCECLK=clk,
                # i_LATCHINPUTVALUE=0,
                o_LOCK=locked0,
                o_PLLOUTGLOBALA=cd_sys.clk,
                # o_PLLOUTCOREA=,
                o_PLLOUTGLOBALB=cd_sr.clk,
                # o_PLLOUTCOREB=,
            ),
            AsyncResetSynchronizer(cd_sr, ~locked0),
            AsyncResetSynchronizer(cd_sys, ~locked0),
        ]

        dat = Signal(n_lanes + 1)
        self.specials += [
            Instance(
                "SB_GB_IO",
                p_PIN_TYPE=0b000000,  # no output, i registered
                p_IO_STANDARD="SB_LVDS_INPUT",
                i_INPUT_CLK=cd_sr.clk,
                o_D_IN_0=dat[0],
                o_GLOBAL_BUFFER_OUTPUT=clk,
                i_PACKAGE_PIN=platform.request("eem2_n", 0)
            ),
            [Instance(
                "SB_IO",
                p_PIN_TYPE=0b000000,
                p_IO_STANDARD="SB_LVDS_INPUT",
                i_INPUT_CLK=cd_sr.clk,
                o_D_IN_0=dat[i + 1],
                io_PACKAGE_PIN=platform.request("eem2_n", i + 1)
            ) for i in range(n_lanes)]
        ]

        tap = Signal(3 + 4)
        self.comb += [
            delay0[:4].eq(tap[3:]),  # feedback delay in 150ps steps
            delay0[4:].eq(0b0000),  # delay of A (sys) to B (sr)
        ]

        word = Signal(n_div*len(dat), reset_less=True)
        buf = Signal.like(word)
        self.sync.sr += [
            buf.eq(Cat(dat, buf)),
            word[:len(dat)].eq(Array([buf[i*len(dat):(i + 1)*len(dat)]
                for i in range(n_div)])[tap[:3]]),
            word[len(dat):].eq(word),
        ]

        frame = Signal((len(word) - len(dat) - 1)*n_frame, reset_less=True)
        marker = Signal(reset_less=True)
        clk_word = Signal(n_div, reset_less=True)
        slip = Signal(4, reset_less=True)  # slip delay
        stb = Signal()
        align_err = Signal(8, reset_less=True)
        self.sync += [
            clk_word.eq(word[::len(dat)]),
            marker.eq(word[1]),  # marker is set on last word
            stb.eq(word[1] & ~marker),
            frame.eq(Cat(
                word[2:len(dat)],
                [word[i*len(dat) + 1:(i + 1)*len(dat)] for i in range(1, n_div)],
                frame
            )),
            slip[1:].eq(slip),
            If((slip[0] == slip[-1]) & (clk_word != 0b1100011),
                slip[0].eq(~slip[0]),
                tap.eq(tap + 1),
                align_err.eq(align_err + 1),
            ),
        ]

        # TODO
        crc = Signal(n_bits)
        crc_good = Signal(reset=1)
        adr = Signal(4)
        cfg = Signal(10)
        crc_err = Signal(8, reset_less=True)
        rst = Signal()
        bypass = Signal()
        latchinputvalue = Signal()
        locked1 = Signal()
        delay1 = Signal(8)
        self.comb += Cat(rst, bypass, latchinputvalue, delay1).eq(cfg)

        sdo = Signal(reset_less=True)
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
        sr = Signal(n_frame, reset_less=True)
        status = Signal((1 << len(adr))*n_frame)
        status_val = Cat(Signal(8, reset=0xfa), cfg, crc_good, locked0, locked1, tap, delay0, align_err, crc_err)
        assert len(status_val) <= len(status)
        self.comb += [
            status.eq(status_val),
            sdo.eq(sr[-1]),
            adr.eq(frame[len(cfg):]),
        ]
        self.sync += [
            sr[1:].eq(sr),
            If(stb,
                If(crc_good,
                    cfg.eq(frame),
                ).Else(
                    crc_err.eq(crc_err + 1),
                ),
                sr.eq(Array([status[i*n_frame:(i + 1)*n_frame]
                    for i in range(1 << len(adr))])[adr]),
            )
        ]

        # self.sync += platform.request("user_led").eq(frame != 0)

        cd_spi = ClockDomain("spi")
        self.clock_domains += cd_spi

        divr = 1 - 1
        divf = 22 - 1
        divq = 4
        t_out = t_clk*n_div*(divr + 1)/(divf + 1)*2**divq
        assert t_out >= 20., t_out
        t_idle = n_frame*t_clk*n_div - (n_bits + 1)*t_out
        assert t_idle >= 0, t_idle
        t_vco = t_out/2**divq
        assert .533 <= 1/t_vco <= 1.066, 1/t_vco
        platform.add_period_constraint(cd_spi.clk, t_out)

        self.specials += [
            Instance(
                "SB_PLL40_CORE",
                p_FEEDBACK_PATH="SIMPLE",
                p_DIVR=divr,  # input
                p_DIVF=divf,  # feedback
                p_DIVQ=divq,  # vco
                p_FILTER_RANGE=3,
                p_PLLOUT_SELECT="GENCLK",
                p_ENABLE_ICEGATE=1,
                i_BYPASS=bypass,
                i_RESETB=~(rst | cd_sys.rst),
                i_DYNAMICDELAY=delay1,
                i_REFERENCECLK=clk,
                i_LATCHINPUTVALUE=latchinputvalue,
                o_LOCK=locked1,
                o_PLLOUTGLOBAL=cd_spi.clk,
                # o_PLLOUTCORE=,
            ),
            AsyncResetSynchronizer(cd_spi, ~locked1),
        ]

        xfer = BlindTransfer("sys", "spi", n_channels*(n_bits + 1))
        assert len(xfer.data_i) + len(crc) + len(cfg) + len(adr) <= len(frame)
        self.submodules += xfer
        self.comb += [
            xfer.i.eq(stb & crc_good),
            xfer.data_i.eq(frame[-len(xfer.data_i):]),
        ]

        vhdci = [platform.request("vhdci", i) for i in range(2)]
        idc = [platform.request("idc", i) for i in range(8)]
        mosi = vhdci[0].io
        cs = vhdci[1].io
        sck = []
        ldac = []
        for _ in idc:
            sck.extend(_.io[i] for i in range(4))
            ldac.extend(_.io[i + 4] for i in range(4))
        self.comb += [
            [_.dir.eq(0b1111) for _ in vhdci],
            [_.dir.eq(1) for _ in idc],
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
            [sri[1:].eq(sri) for sri in sr],  # MSB first
            If(xfer.o,
                Cat(sr, mask).eq(xfer.data_o)
            ),
        ]
        for i in range(n_channels):
            self.specials += [
                Instance(
                    "SB_IO",
                    p_PIN_TYPE=C(0b010100, 6),  # output registered
                    p_IO_STANDARD="SB_LVCMOS",
                    i_OUTPUT_CLK=cd_spi.clk,
                    #i_CLOCK_ENABLE=mask[i],
                    o_PACKAGE_PIN=mosi[i],
                    i_D_OUT_0=sr[i][-1] & mask[i]),
                Instance(
                    "SB_IO",
                    p_PIN_TYPE=C(0b011100, 6),  # output registered inverted
                    p_IO_STANDARD="SB_LVCMOS",
                    i_OUTPUT_CLK=cd_spi.clk,
                    #i_CLOCK_ENABLE=mask[i],
                    o_PACKAGE_PIN=cs[i],
                    i_D_OUT_0=csi & mask[i]),
                Instance(
                    "SB_IO",
                    p_PIN_TYPE=C(0b011100, 6),  # output registered inverted
                    p_IO_STANDARD="SB_LVCMOS",
                    i_OUTPUT_CLK=cd_spi.clk,
                    #i_CLOCK_ENABLE=mask[i],
                    o_PACKAGE_PIN=ldac[i],
                    i_D_OUT_0=mask[i]),
                Instance(
                    "SB_IO",
                    p_PIN_TYPE=C(0b010000, 6),  # output registered DDR
                    p_IO_STANDARD="SB_LVCMOS",
                    i_OUTPUT_CLK=cd_spi.clk,
                    #i_CLOCK_ENABLE=mask[i],
                    o_PACKAGE_PIN=sck[i],
                    i_D_OUT_0=mask[i],
                    i_D_OUT_1=0),
            ]


if __name__ == "__main__":
    from banker import Platform
    platform = Platform()
    banker = Banker(platform)
    platform.build(banker)
