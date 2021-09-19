from migen import *

class Aligner(Module):
    def __init__(self, cd_fast="spi", cd_slow="sys"):
        # the current fast period is fully contained within one slow period
        self.aligned = Signal()

        ###

        slow = Signal(reset_less=True)
        cd_slow = getattr(self.sync, cd_slow)
        cd_slow += [
            slow.eq(~slow),
        ]
        fast = Signal(reset_less=True)
        cd_fast = getattr(self.sync, cd_fast)
        cd_fast += [
            fast.eq(slow),
        ]
        self.comb += [
            self.aligned.eq(fast == slow),
        ]
