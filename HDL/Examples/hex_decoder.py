from amaranth import *

from readbin import readbin


class HexDecoder(Elaboratable):
    def __init__(self):
        # Input
        self.data = Signal(128)
        self.x = Signal(8)
        self.y = Signal(8)

        # Output
        self.color = Signal(16)

    def elaborate(self, platform):
        m = Module()

        y1 = Signal(8)
        y2 = Signal(3)
        x1 = Signal(8)
        x2 = Signal(8)
        xdiv = Signal(5)
        xmod = Signal(3)
        xmod2 = Signal(3)
        xdiv2 = Signal(5)
        hex_digit = Signal(4)
        pixel = Signal()

        # Create the font
        font_data = readbin("hex_font.mem")
        font = Memory(width=5, depth=len(font_data), init=font_data)
        m.submodules.fr = fr = font.read_port()

        m.d.comb += fr.addr.eq(Cat(~y2, hex_digit))

        # Pipeline stage 1
        m.d.sync += [
            xdiv.eq(self.x // 6),
            xmod.eq(self.x % 6),
            y1.eq(self.y),
            x1.eq(self.x)
        ]

        # Stage 2
        m.d.sync += [
            hex_digit.eq(self.data.word_select(xdiv, 4)),
            xmod2.eq(xmod),
            xdiv2.eq(xdiv),
            y2.eq(y1[:3]),
            x2.eq(x1)
        ]

        # Stage 3
        m.d.sync += pixel.eq(Mux((xmod2 < 5) & (x2 < 192), fr.data.bit_select(xmod2, 1), 0))

        # Stage 4
        m.d.sync += self.color.eq(Mux(pixel, 0xffff, 0x0000))

        return m
