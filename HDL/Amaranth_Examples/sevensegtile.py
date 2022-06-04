from amaranth import *

from HDL.Tiles.seven_seg_tile import SevenSegController, tile_resources

class SevenSegmentTile(Elaboratable):
    def __init__(self):
        self.leds = Signal(7)
        self.ca   = Signal(3)
        self.val  = Signal(12)

    def elaborate(self, platform):
        m = Module()

        m.submodules.seven = seven = SevenSegController()

        m.d.comb += self.leds.eq(seven.leds)

        timer = Signal(19)
        m.d.sync += timer.eq(timer + 1)

        for i in range(3):
            m.d.comb += self.ca[i].eq(timer[17:19] == i)

        with m.If(self.ca[2]):
            m.d.comb += seven.val.eq(self.val[8:])
        with m.If(self.ca[1]):
            m.d.comb += seven.val.eq(self.val[4:8])
        with m.If(self.ca[0]):
            m.d.comb += seven.val.eq(self.val[:4])

        return m