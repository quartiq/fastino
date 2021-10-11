from migen import *

from cic import CIC

class Interpolator(Module):
    def __init__(self, n_channels=16, n_bits=16, n_mantissa=6, n_exp=4,
                 order=3):
        self.typ = Signal()
        self.stb_in = Signal()
        self.x = [Signal(n_bits, reset_less=True) for _ in range(n_channels)]
        self.en_in = Signal(n_channels)

        self.y = [Signal(n_bits) for _ in range(n_channels)]
        self.en_out = Signal(n_channels)
        self.stb_out = Signal()
        msb_flip = 1  # conversion between offset-binary and twos-complement

        ###

        cic = CEInserter()(CIC)(width=n_bits, order=order,
                  rate_width=n_bits, channels=n_channels)
        self.submodules += cic
        assert cic.latency < n_channels

        reset = Signal(n_channels, reset_less=True)
        enable = Signal(3*n_channels, reset_less=True)
        sr = [Signal(n_bits, reset_less=True)
              for _ in range(2*n_channels - cic.latency)]

        self.comb += [
            cic.x.eq(sr[0] ^ (msb_flip << n_bits - 1)),
            cic.reset.eq(reset[0]),
            cic.rate.eq(((sr[0][:n_mantissa] + 1) <<
                          sr[0][n_mantissa:n_mantissa + n_exp]) - 1),
            cic.shift.eq(sr[0][n_mantissa + n_exp:]),
            Cat(self.y).eq(Cat(sr[-n_channels:])),
            self.en_out.eq(enable[-n_channels:]),
        ]
        self.sync += [
            If(cic.ce,
                reset.eq(reset[1:]),
                enable.eq(Cat(enable[1:], cic.valid & enable[0])),
                Cat(sr[:-1]).eq(Cat(sr[1:])),
                sr[-1].eq(cic.y ^ (msb_flip << n_bits - 1)),
            ),
            If(cic.xi == n_channels - 1,
                cic.ce.eq(0),
            ),
            If(self.stb_in & ~cic.ce,
                Cat(sr[:n_channels]).eq(Cat(self.x)),
                cic.stb.eq(~self.typ),
                If(self.typ == 0,
                    enable[n_channels + cic.latency:2*n_channels + cic.latency].eq(
                        self.en_in),
                ).Elif(self.typ == 1,
                    enable[n_channels + cic.latency:2*n_channels + cic.latency].eq(0),
                    reset.eq(self.en_in),
                ),
                cic.ce.eq(1),
            ),
            self.stb_out.eq(cic.yi == n_channels - 1),
        ]
