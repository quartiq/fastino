from migen import *

from cic import CIC

class Interpolator(Module):
    def __init__(self, n_channels=16, n_bits=16, n_mantissa=6, n_exp=4,
                 order=3):
        self.typ = Signal()
        self.stb = Signal()
        self.data = Signal(n_channels + n_channels*n_bits, reset_less=True)

        self.y = [Signal(n_bits) for _ in range(n_channels)]
        self.en = Signal(n_channels)
        self.valid = Signal()
        msb_flip = 1

        ###

        cic = CEInserter()(CIC)(width=n_bits, order=order,
                  rate_width=n_bits, channels=n_channels)
        self.submodules += cic
        assert cic.latency < n_channels

        rate_reset = Signal(n_channels)
        enable_sr = Signal(2*n_channels)
        sr = [Signal(n_bits, reset_less=True)
              for _ in range(2*n_channels - cic.latency)]

        self.comb += [
            cic.x.eq(sr[0] + (msb_flip << n_bits - 1)),
            cic.stb.eq(enable_sr[cic.latency]),
            cic.reset.eq(rate_reset[0]),
            cic.rate.eq(((sr[0][:n_mantissa] + 1) <<
                          sr[0][n_mantissa:n_mantissa + n_exp]) - 1),
            cic.shift.eq(sr[0][n_mantissa + n_exp:]),
            Cat(self.y).eq(Cat(sr[-n_channels:])),
            self.en.eq(enable_sr[-n_channels:]),
        ]
        self.sync += [
            If(cic.ce,
                rate_reset.eq(rate_reset[1:]),
                enable_sr.eq(Cat(enable_sr[1:], enable_sr[0] & cic.valid)),
                Cat(sr).eq(Cat(sr[1:], cic.y + (msb_flip << n_bits - 1))),
            ),
            If(cic.xi == n_channels - 1,
                cic.ce.eq(0),
            ),
            If(self.stb & ~cic.ce,  # | (cic.xi == n_channels - 1),
                Cat(sr[:n_channels]).eq(self.data[n_channels:]),
                If(self.typ == 0,
                    enable_sr[cic.latency:cic.latency + n_channels].eq(
                        self.data),
                ).Elif(self.typ == 1,
                    enable_sr[cic.latency:cic.latency + n_channels].eq(0),
                    rate_reset.eq(self.data),
                ),
                cic.ce.eq(1),
            ),
            self.valid.eq(cic.yi == n_channels - 1),
        ]