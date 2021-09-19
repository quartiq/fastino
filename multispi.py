from migen import *

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
        assert len(Cat(enable, sr)) == n

        enable0 = Signal.like(enable)
        self.sync.spi += [
            [sri[1:].eq(sri) for sri in sr],  # MSB first
            If(i == n_bits - 1,
                enable.eq(0),
                self.busy.eq(0),
            ),
            If(self.busy,
                i.eq(i + 1),
            ).Elif(self.stb,
                self.busy.eq(1),
                Cat(enable, sr).eq(self.data),
            ),
            enable0.eq(enable),
        ]
        for i in range(n_channels):
            self.specials += [
                Instance(
                    "SB_IO",
                    p_PIN_TYPE=C(0b010000, 6),  # output registered DDR
                    p_IO_STANDARD="SB_LVCMOS",
                    i_OUTPUT_CLK=ClockSignal("spi"),
                    #i_CLOCK_ENABLE=enable[i],
                    o_PACKAGE_PIN=spi[i].clk,
                    i_D_OUT_0=0,
                    i_D_OUT_1=enable0[i]),
                Instance(
                    "SB_IO",
                    p_PIN_TYPE=C(0b010100, 6),  # output registered
                    p_IO_STANDARD="SB_LVCMOS",
                    i_OUTPUT_CLK=ClockSignal("spi"),
                    #i_CLOCK_ENABLE=enable[i],
                    o_PACKAGE_PIN=spi[i].sdi,
                    i_D_OUT_0=sr[i][-1] & enable[i]),
                Instance(
                    "SB_IO",
                    p_PIN_TYPE=C(0b011100, 6),  # output registered inverted
                    p_IO_STANDARD="SB_LVCMOS",
                    i_OUTPUT_CLK=ClockSignal("spi"),
                    #i_CLOCK_ENABLE=enable[i],
                    o_PACKAGE_PIN=spi[i].nss,
                    i_D_OUT_0=enable[i]),
            ]
            # With LDAC tied permanently low, the rising edge of
            # CS loads the data to the DAC.
            self.comb += spi[i].ldacn.eq(0)
