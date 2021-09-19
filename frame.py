from migen import *
from misoc.cores.liteeth_mini.mac.crc import LiteEthMACCRCEngine


class Frame(Module):
    """Frame alignment, checksum"""
    def __init__(self, n_frame=14):
        n = 7*6
        # input word from link
        self.word = Signal(n)
        checksum = Signal(12, reset_less=True)
        self.submodules.crc = LiteEthMACCRCEngine(
            data_width=n, width=len(checksum), polynom=0x80f)  # crc-12 telco
        # frame body
        self.body = Signal(n_frame*n - n_frame//2 - 1 - len(checksum))
        self.stb = Signal()
        self.crc_err = Signal(8)

        # frame: n_frame = 14
        # 0: body(n_word)
        # ...
        # n_frame//2 - 4: body(n_word)
        # ...
        # n_frame//2 - 3: marker(1), body(n_word - 1)
        # ...
        # n_frame - 2: marker(1), body(n_word - 1)
        # n_frame - 1: crc(n_crc = 12), body(n_word - 12)
        # n_body = n_frame*n_word - n_frame//2 - 1 - n_crc

        eof = Signal()
        frame = Signal(n*n_frame, reset_less=True)
        crc_good = Signal()

        body_ = Cat(
            # checksum in the most recent word
            frame[len(checksum):n],
            # skip eof marker bits in the next n_frame//2 + 1 words
            [frame[1 + i*n:(i + 1)*n] for i in range(1, 2 + n_frame//2)],
            # complete words
            frame[(2 + n_frame//2)*n:],
        )
        assert len(body_) == len(self.body)

        self.comb += [
            self.crc.last.eq(checksum),
            # LiteEthMACCRCEngine takes LSB first
            self.crc.data[::-1].eq(self.word),
            self.body.eq(body_),
            # CRC([x, CRC(x)]) == 0
            crc_good.eq(self.crc.next == 0),
        ]
        self.sync += [
            eof.eq(Cat(self.word[0], [frame[i*n] for i in range(n_frame//2)]) == 1),
            self.stb.eq(eof & crc_good),
            frame.eq(Cat(self.word, frame)),
            checksum.eq(self.crc.next),
            If(eof,
                checksum.eq(0),
                If(~crc_good,
                    self.crc_err.eq(self.crc_err + 1),
                ),
            ),
        ]