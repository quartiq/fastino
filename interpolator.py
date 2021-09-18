from migen import *

from cic import CIC

class Interpolator(Module):
    def __init__(self, n_channels=32, n_bits=16, n_mantissa=6, n_exp=4,
                 div=2, order=3):
        self.typ = Signal()
        self.stb = Signal()
        self.data = Signal(n_channels + n_channels*n_bits, reset_less=True)

        self.y = [Signal(n_bits, reset_less=True) for _ in range(n_channels)]
        self.en = Signal(n_channels)
        self.valid = Signal()
        
        ###

        enable = Signal(n_channels)
        enable_sr = Signal(div*(2*order + 1))
        rate_reset = Signal(n_channels)
        cfg = [Signal(n_bits, reset_less=True) for _ in range(n_channels)]
        x = [Signal(n_bits, reset_less=True) for _ in range(n_channels)]
        run = Signal()

        cics = []
        for i in range(div):
            cic = CIC(width=n_bits, order=order,
                      rate_width=n_bits, channels=n_channels//div)
            cic = CEInserter()(cic)
            self.submodules += cic
            cics.append(cic)
            self.comb += [
                cic.x.eq(x[i] + (1 << n_bits - 1)),
                cic.stb.eq(enable[i]),
                cic.reset.eq(rate_reset[i]),
                cic.rate.eq(((cfg[i][:n_mantissa] + 1) <<
                              cfg[i][n_mantissa:n_mantissa + n_exp]) - 1),
                cic.shift.eq(cfg[i][n_mantissa + n_exp:]),
                cic.ce.eq(run),
            ]
        
        self.sync += [
            If(run,
                Cat(cfg).eq(Cat(cfg[div:], cfg[:div])),
                Cat(x).eq(Cat(x[div:], x[:div])),
                rate_reset.eq(Cat(rate_reset[div:], rate_reset[:div])),
                enable.eq(Cat(enable[div:], enable[:div])),
                enable_sr.eq(Cat(enable_sr[div:], enable[:div])),

                self.en.eq(Cat(self.en[div:],
                               enable_sr[:div] &
                               Cat([cic.valid for cic in cics]))),
                Cat(self.y).eq(Cat(
                    self.y[div:],
                    [cic.y + (1 << n_bits - 1) for cic in cics])),
            ),
            If(cics[0].xi == n_channels//div - 1,
                run.eq(0),
            ),
            If(self.stb & (~run | (cics[0].xi == n_channels//div - 1)),
                Array([
                    Cat(enable, x),
                    Cat(rate_reset, cfg),
                ])[self.typ].eq(self.data),
                run.eq(1),
            ),
            self.valid.eq(cics[0].yi == n_channels//div - 1),
        ]