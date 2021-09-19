from migen import *

class SlipSR(Module):
    """Shift register deserializer.

    Takes in a couple of fast lanes of serial `data` to be shifted in
    on the `sr` clock domain and outputs the entire `word` in the `word`
    domain.

    `slip_request` (level sensitive, from the `word` clock domain)
    performs a bit-slip operation on all lanes by one bit. The first
    slipped word appears between 2 and 3 word clock cycles later.
    """
    def __init__(self, n_lanes, n_div):
        self.data = Signal(n_lanes)
        self.slip_req = Signal()
        self.word = Signal(n_div*n_lanes, reset_less=True)

        # continuously shifting input register
        buf = Signal.like(self.word)
        # slipped snapshot of buf
        slipped = Signal.like(buf)
        # delayed word clock
        clk0 = Signal()
        # work alignment marker, rotating shift register in the sr domain
        # with a single high bit
        i = Signal(n_div, reset=1)
        # no slip active
        slip_n = Signal()
        self.sync.sr += [
            # shift in new data
            buf.eq(Cat(self.data, buf)),
            clk0.eq(ClockSignal("word")),
            slip_n.eq(~(self.slip_req & ClockSignal("word") & ~clk0)),
            # shift through alignment marker if not slipping
            If(slip_n,
                i.eq(Cat(i[-1], i)),
            ),
            # if alignment marker on top, coppy into slipped
            If(i[-1],
                slipped.eq(buf),
            ),
        ]
        # in word domain, coppy into output
        self.sync.word += [
            self.word.eq(slipped),
        ]
