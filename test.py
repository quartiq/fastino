from math import gcd

from migen import *
from migen.genlib.cdc import MultiReg, AsyncResetSynchronizer, BlindTransfer
from misoc.cores.liteeth_mini.mac.crc import LiteEthMACCRCEngine

# word: n_clk = 7
# cyc  0 1 2 3 4 5 6
# clk0 1 1 0 0 0 1 1
# clk1  1 x 0 0 x 1 1   # optimal, on edge
# clk1   1 0 0 0 1 1 1   # late
# clk1 1 1 0 0 0 1 1   # early
# n_pay = n_lanes*n_clk = 42

# frame: n_frame = 14
# 0: body(n_pay)
# ...
# n_frame//2 - 1: body(n_pay)
# ...
# n_frame//2: marker(1), body(n_pay - 1)
# ...
# n_frame: marker(1), crc(n_crc = 16), body(n_pay - 1 - 16)
# n_body = n_frame//2*(2*n_pay - 1) - n_crc


class Link(Module):
    def __init__(self, clk, data, platform, t_clk=4.):
        self.n_div = 7
        n_lanes = len(data)

        # link clocking
        self.clk_buf = Signal()
        cd_word = ClockDomain("word")
        platform.add_period_constraint(cd_word.clk, t_clk*self.n_div)
        self.clock_domains += cd_word

        cd_sr = ClockDomain("sr")
        self.clock_domains += cd_sr
        divr = 1 - 1
        divf = self.n_div - 1
        divq = 2
        t_out = t_clk*self.n_div*(divr + 1)/(divf + 1)
        assert t_out == t_clk
        platform.add_period_constraint(cd_sr.clk, t_out)
        t_vco = t_out/2**divq
        assert .533 <= 1/t_vco <= 1.066, 1/t_vco

        # input clock PLL
        locked = Signal()
        self.delay = Signal(4, reset_less=True, reset=0b1111)
        self.delay_relative = Signal(4, reset_less=True)
        self.specials += [
            Instance(
                "SB_PLL40_2F_CORE",
                p_FEEDBACK_PATH="DELAY",   # out = in/r*f, vco = out*2**q
                p_DIVR=divr,  # input
                p_DIVF=divf,  # feedback
                p_DIVQ=divq,  # vco
                p_FILTER_RANGE=3,
                p_SHIFTREG_DIV_MODE=int(self.n_div == 7),  # div-by-7
                p_DELAY_ADJUSTMENT_MODE_FEEDBACK="DYNAMIC",
                p_FDA_FEEDBACK=0,
                p_DELAY_ADJUSTMENT_MODE_RELATIVE="DYNAMIC",
                p_FDA_RELATIVE=0,
                p_PLLOUT_SELECT_PORTA="GENCLK",
                p_PLLOUT_SELECT_PORTB="SHIFTREG_0deg",
                p_ENABLE_ICEGATE_PORTA=0,
                p_ENABLE_ICEGATE_PORTB=0,
                i_BYPASS=0,
                i_RESETB=1,
                i_DYNAMICDELAY=Cat(self.delay, self.delay_relative),
                i_REFERENCECLK=self.clk_buf,
                i_LATCHINPUTVALUE=0,
                o_LOCK=locked,
                o_PLLOUTGLOBALA=cd_sr.clk,
                # o_PLLOUTCOREA=,
                o_PLLOUTGLOBALB=cd_word.clk,
                # o_PLLOUTCOREB=,
            ),
            AsyncResetSynchronizer(cd_sr, ~locked),
            AsyncResetSynchronizer(cd_word, cd_sr.rst),
        ]

        # input PIO registers
        dat = Signal(n_lanes + 2)
        self.specials += [
            Instance(
                "SB_GB_IO",
                p_PIN_TYPE=0b000000,  # no output, i registered
                p_IO_STANDARD="SB_LVDS_INPUT",
                i_INPUT_CLK=cd_sr.clk,
                o_D_IN_0=dat[0],
                # o_D_IN_1=dat[1],  # doesn't meet timing
                o_GLOBAL_BUFFER_OUTPUT=self.clk_buf,
                i_PACKAGE_PIN=clk,
            ),
            [Instance(
                "SB_IO",
                p_PIN_TYPE=0b000000,
                p_IO_STANDARD="SB_LVDS_INPUT",
                i_INPUT_CLK=cd_sr.clk,
                o_D_IN_0=dat[i + 2],
                io_PACKAGE_PIN=data[i],
            ) for i in range(n_lanes)]
        ]

        # bitslip buffer
        self.tap = Signal(3, reset_less=True, reset=0)
        buf = Signal(self.n_div*len(dat), reset_less=True)
        word0 = Signal.like(buf)
        word = Signal.like(buf)
        self.sync.sr += [
            buf.eq(Cat(dat, buf)),
            word0.eq(Cat(Array([
                buf[i*len(dat):(i + 1)*len(dat)]
                for i in range(self.n_div)])[self.tap], word0)),
        ]
        # word clock retiming register
        self.sync.word += word.eq(word0)

        self.payload = Signal(n_lanes*self.n_div)
        clk0 = Signal(4)
        clk1 = Signal(4)
        slip_good = Signal()
        delay_inc = Signal()
        delay_dec = Signal()
        slip = Signal(3, reset_less=True)  # slip adjustment delay line
        slip_done = Signal()
        self.align_err = Signal(8, reset_less=True)
        self.stb = Signal()

        self.comb += [
            # relevant clock samples for bit slip alignment
            clk0.eq(Cat(word[i*len(dat)] for i in (1, 2, 4, 5))),
            # relevant clock samples for sub-sample delay alignment
            clk1.eq(Cat(word[1 + i*len(dat)] for i in (1, 4))),
            slip_good.eq(clk0 == 0b1001),
            # early sampling, increase clock delay, decrease feedback delay
            delay_dec.eq(clk1 == 0b10),
            # late sampling, decrease clock delay, increase feedback delay
            delay_inc.eq(clk1 == 0b01),
            slip_done.eq(~(slip[0] ^ slip[1])),
            self.stb.eq(slip_good & slip_done),
            self.payload.eq(Cat([word[2 + i*len(dat):(i + 1)*len(dat)]
                for i in range(self.n_div)])),
        ]
        self.sync.word += [
            # propagate through slip adjustment delay line
            slip[1:].eq(slip),
            If(slip_done,
                If(~slip_good,
                    slip[0].eq(~slip[0]),
                    self.tap.eq(self.tap + 1),
                    self.delay.eq(self.delay.reset),
                    self.align_err.eq(self.align_err + 1),
                ).Elif(delay_inc & (self.delay != 0xf),
                    slip[0].eq(~slip[0]),
                    self.delay.eq(self.delay + 0x1),
                ).Elif(delay_dec & (self.delay != 0x0),
                    slip[0].eq(~slip[0]),
                    self.delay.eq(self.delay - 0x1),
                ),
            ),
        ]


class Frame(Module):
    def __init__(self, n_frame=14):
        n = 7*6
        self.payload = Signal(n)
        checksum = Signal(16)
        self.body = Signal(n_frame*n -
                           n_frame//2 - 1 - len(checksum))
        self.stb = Signal()
        self.crc_err = Signal(8, reset_less=True)

        crc = LiteEthMACCRCEngine(
            data_width=n, width=len(checksum), polynom=0x1021)
        self.submodules += crc

        marker_good = Signal()

        self.comb += [
            crc.data.eq(Mux(marker_good,
                Cat(C(1, len(checksum) + 1), self.payload[1 + len(checksum):]),
                self.payload)),
            crc.last.eq(checksum),
        ]

        crc_good = Signal(reset=1)
        frame = Signal(n*n_frame, reset_less=True)

        body_ = Cat(
            frame[1 + len(checksum):n],  # most recent
            [frame[1 + i*n:(i + 1)*n] for i in range(1, 1 + n_frame//2)],
            frame[(1 + n_frame//2)*n:],
        )
        assert len(body_) == len(self.body)

        self.comb += [
            marker_good.eq(self.payload[0] & (
                Cat(frame[i*n] for i in range(n_frame//2)) == 0)),
            crc_good.eq(crc.next == self.payload[1:1 + len(checksum)]),
            self.body.eq(body_),
        ]
        self.sync += [
            frame.eq(Cat(self.payload, frame)),
            self.stb.eq(0),
            checksum.eq(crc.next),
            If(marker_good,
                checksum.eq(0),
                If(crc_good,
                    self.stb.eq(1),
                ).Else(
                    self.crc_err.eq(self.crc_err + 1),
                ),
            ),
        ]


class MultiSPI(Module):
    def __init__(self, platform, n_channels=32, n_bits=16):
        n = n_channels*(n_bits + 1)
        self.data = Signal(n)
        self.stb = Signal()

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
            platform.request("drv_oe_n").eq(0),
        ]

        csi = Signal()
        i = Signal(max=n_bits)
        self.sync.spi += [
            If(self.stb,
                csi.eq(1),
            ),
            If(csi,
                i.eq(i + 1),
            ),
            If(i == n_bits - 1,
                csi.eq(0),
            ),
        ]
        sr = [Signal(n_bits, reset_less=True) for i in range(n_channels)]
        mask = Signal(n_channels, reset_less=True)
        ce = Signal(1)
        n_ce = n_channels//len(ce)
        self.comb += [ce[i].eq(~mask[i*n_ce:(i + 1)*n_ce] == 0) for i in
                range(len(ce))]
        assert len(Cat(sr, mask)) == n
        self.sync.spi += [
            [sri[1:].eq(sri) for sri in sr],  # MSB first
            If(self.stb,
                Cat(sr, mask).eq(self.data)
            ),
        ]
        for i in range(n_channels):
            self.specials += [
                Instance(
                    "SB_IO",
                    p_PIN_TYPE=C(0b010100, 6),  # output registered
                    p_IO_STANDARD="SB_LVCMOS",
                    i_OUTPUT_CLK=ClockSignal("spi"),
                    # i_CLOCK_ENABLE=ce[i//n_ce],
                    o_PACKAGE_PIN=mosi[i],
                    i_D_OUT_0=sr[i][-1] & mask[i]),
                Instance(
                    "SB_IO",
                    p_PIN_TYPE=C(0b011100, 6),  # output registered inverted
                    p_IO_STANDARD="SB_LVCMOS",
                    i_OUTPUT_CLK=ClockSignal("spi"),
                    # i_CLOCK_ENABLE=ce[i//n_ce],
                    o_PACKAGE_PIN=cs[i],
                    i_D_OUT_0=csi & mask[i]),
                Instance(
                    "SB_IO",
                    p_PIN_TYPE=C(0b011100, 6),  # output registered inverted
                    p_IO_STANDARD="SB_LVCMOS",
                    i_OUTPUT_CLK=ClockSignal("spi"),
                    # i_CLOCK_ENABLE=ce[i//n_ce],
                    o_PACKAGE_PIN=ldac[i],
                    i_D_OUT_0=mask[i]),
                Instance(
                    "SB_IO",
                    p_PIN_TYPE=C(0b010000, 6),  # output registered DDR
                    p_IO_STANDARD="SB_LVCMOS",
                    i_OUTPUT_CLK=ClockSignal("spi"),
                    # i_CLOCK_ENABLE=ce[i//n_ce],
                    o_PACKAGE_PIN=sck[i],
                    i_D_OUT_0=mask[i],
                    i_D_OUT_1=0),
            ]


class Banker(Module):
    def __init__(self, platform):
        n_channels = 32
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
        platform.add_period_constraint(cd_sys.clk, t_clk*link.n_div)

        frame = Frame()
        self.submodules += frame
        self.comb += [
            frame.payload.eq(link.payload),
        ]

        adr = Signal(4)
        cfg = Signal(10)
        rst = Signal()
        bypass = Signal()
        latchinputvalue = Signal()
        locked = Signal()
        delay_feedback = Signal(4)
        delay_relative = Signal(4)
        self.comb += Cat(rst, bypass, latchinputvalue, delay_feedback,
                delay_relative).eq(cfg)

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
        status_val = Cat(Signal(8, reset=0xfa), cfg, locked, link.tap,
                         link.delay, link.align_err, frame.crc_err)
        assert len(status_val) <= len(status)
        self.comb += [
            status.eq(status_val),
            sdo.eq(sr[-1]),
            adr.eq(frame.body[len(cfg):]),
        ]
        self.sync += [
            sr[1:].eq(sr),
            If(frame.stb,
                cfg.eq(frame.body),
                sr.eq(Array([status[i*n_frame:(i + 1)*n_frame]
                    for i in range(1 << len(adr))])[adr]),
            )
        ]

        self.comb += platform.request("user_led").eq(~ResetSignal())

        cd_spi = ClockDomain("spi")
        self.clock_domains += cd_spi
        divr = 3 - 1
        divf = 4 - 1
        divq = 4
        t_out = t_clk*link.n_div*(divr + 1)/(divf + 1)  # *2**divq
        assert t_out >= 20., t_out
        assert t_out < t_clk*link.n_div
        platform.add_period_constraint(cd_spi.clk, t_out)
        print("min cd_sys-cd_spi gap", min((i*t_out) % (t_clk*link.n_div)
            for i in range(1, (divf + 1)//gcd(divr + 1, divf + 1))))
        t_idle = n_frame*t_clk*link.n_div - (n_bits + 1)*t_out
        assert t_idle >= 0, t_idle
        t_vco = t_out/2**divq
        assert .533 <= 1/t_vco <= 1.066, 1/t_vco

        self.specials += [
            Instance(
                "SB_PLL40_CORE",
                p_FEEDBACK_PATH="PHASE_AND_DELAY",
                p_DIVR=divr,  # input
                p_DIVF=divf,  # feedback
                p_DIVQ=divq,  # vco
                p_FILTER_RANGE=1,
                p_PLLOUT_SELECT="GENCLK",
                p_ENABLE_ICEGATE=1,
                p_DELAY_ADJUSTMENT_MODE_FEEDBACK="DYNAMIC",
                p_FDA_FEEDBACK=0,
                p_DELAY_ADJUSTMENT_MODE_RELATIVE="DYNAMIC",
                p_FDA_RELATIVE=0,
                i_BYPASS=bypass,
                i_RESETB=~(rst | cd_sys.rst),
                i_DYNAMICDELAY=Cat(delay_feedback, delay_relative),
                i_REFERENCECLK=link.clk_buf,
                i_LATCHINPUTVALUE=latchinputvalue,
                o_LOCK=locked,
                o_PLLOUTGLOBAL=cd_spi.clk,
                # o_PLLOUTCORE=,
            ),
            AsyncResetSynchronizer(cd_spi, ~locked),
        ]

        spi = MultiSPI(platform)
        self.submodules += spi

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
        else:  # no cdc, assume timing is comensurate, max delay < min gap
            self.comb += [
                spi.stb.eq(frame.stb),
                spi.data.eq(frame.body[-len(spi.data):])
            ]


if __name__ == "__main__":
    from banker import Platform
    platform = Platform()
    banker = Banker(platform)
    platform.build(banker)
