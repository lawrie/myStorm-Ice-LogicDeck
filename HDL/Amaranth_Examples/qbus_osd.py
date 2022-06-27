from amaranth import *
from amaranth.build import *
from amaranth_stdio.serial import AsyncSerial
from amaranth.lib.fifo import SyncFIFOBuffered

from mystorm_boards.icelogicbus import *

from HDL.Tiles import AAVC_tile
from HDL.Tiles import seven_seg_tile
from HDL.Misc.pll import PLL

from qspimem import QspiMem
from sevensegtile import SevenSegmentTile

from HDL.Misc.vga import VGADriver, VGATestPattern, vga_timings

from readhex import readhex
from readbin import readbin

from osd import Osd

BLADE = 1
SEG7_TILE = 3
VGA_TILE = 1
PMOD = 5

led_blade = [
    Resource("leds6", 0,
             Subsignal("leds", Pins("1 2 3 4 5 6", dir="o", invert=True, conn=("blade", BLADE))),
             Attrs(IO_STANDARD="SB_LVCMOS"))
]

# External uart connected to Pmod
uart_pmod = [
    Resource("ext_uart", 0,
             Subsignal("tx", Pins("10", dir="o", conn=("pmod", PMOD))),
             Subsignal("rx", Pins("4", dir="i", conn=("pmod", PMOD))),
             Subsignal("gnd", Pins("9", dir="o", conn=("pmod", PMOD))),
             Attrs(IO_STANDARD="SB_LVCMOS"))
]


class QbusOsd(Elaboratable):
    def __init__(self, start_x=128, start_y=48, chars_x=64, chars_y=24,
                 init_on=0, inverse=0, char_file="osd.mem",
                 font_file="font_bizcat8x16.mem"):
        self.start_x = start_x
        self.start_y = start_y
        self.chars_x = chars_x
        self.chars_y = chars_y
        self.init_on = init_on
        self.inverse = inverse
        self.char_file = char_file
        self.font_file = font_file

    def elaborate(self, platform):
        # Get pins
        qspi = platform.request("qspi")
        leds6 = platform.request("leds6")
        led = platform.request("led")

        m = Module()

        # Clock generator.
        clk_freq = 1e8
        clk_in = platform.request(platform.default_clk)
        # Create a Pll for 100Mhz clock
        m.submodules.pll = pll = PLL(freq_in_mhz=int(platform.default_clk_frequency / 1e6),
                                     freq_out_mhz=int(clk_freq / 1e6),
                                     domain_name="sync")
        # Set the sync domain to the pll domain
        m.domains.sync = cd_sync = pll.domain
        m.d.comb += pll.clk_pin.eq(clk_in)
        platform.add_clock_constraint(cd_sync.clk, clk_freq)

        # Add QspiMem submodule
        m.submodules.qspimem = qspimem = QspiMem()

        # Connect pins
        m.d.comb += [
            qspimem.qss.eq(qspi.cs),
            qspimem.qck.eq(qspi.clk),
            qspimem.qd_i.eq(qspi.data.i),
            qspi.data.o.eq(qspimem.qd_o),
            qspi.data.oe.eq(qspimem.qd_oe)
        ]

        # Add 7-segment display tile controller
        m.submodules.seven = seven = SevenSegmentTile()
        display = Signal(12)

        # Get pins
        seg_pins = platform.request("seven_seg_tile")
        leds7 = Cat([seg_pins.a, seg_pins.b, seg_pins.c, seg_pins.d,
                     seg_pins.e, seg_pins.f, seg_pins.g])

        # Connect pins and display value
        m.d.comb += [
            leds7.eq(seven.leds),
            seg_pins.ca.eq(seven.ca),
            seven.val.eq(display)
        ]

        # Create uart
        ext_uart = platform.request("ext_uart")
        divisor = int(clk_freq // 115200)

        m.submodules.serial = serial = AsyncSerial(divisor=divisor)

        m.d.comb += [
            # Set GND pin
            ext_uart.gnd.eq(0),
            # Connect uart pins
            serial.rx.i.eq(ext_uart.rx),
            ext_uart.tx.eq(serial.tx.o)
        ]

        # Create fifo for incoming uart data
        m.submodules.fifo = fifo = SyncFIFOBuffered(width=8, depth=32)

        # Create VGA instance with chosen timings
        m.domains.pixel = cd_pixel = ClockDomain("pixel")
        m.d.comb += ClockSignal("pixel").eq(clk_in)
        platform.add_clock_constraint(cd_pixel.clk, platform.default_clk_frequency)

        m.submodules.vga = vga = VGADriver(
            vga_timings['640x480@60Hz'],
            bits_x=16,
            # Play around with the sizes because sometimes
            bits_y=16
            # a smaller/larger value will make it pass timing.
        )

        # Create test pattern
        W = Signal(8)
        A = Signal(8)
        T = Signal(8)
        Z = Signal(6)

        # Test pattern fundamentals
        m.d.comb += [
            A.eq(Mux(
                (vga.o_beam_x[5:8] == 0b010) & (vga.o_beam_y[5:8] == 0b010),
                0xFF, 0)),
            W.eq(Mux(
                (vga.o_beam_x[:8] == vga.o_beam_y[:8]),
                0xFF, 0)),
            Z.eq(Mux(
                (vga.o_beam_y[3:5] == ~(vga.o_beam_x[3:5])),
                0xFF, 0)),
            T.eq(Repl(vga.o_beam_y[6], len(T))),
        ]

        # Mux Emit rgb test pattern pixels unless within blanking period
        with m.If(vga.o_vga_blank):
            m.d.pixel += [
                vga.i_r.eq(0),
                vga.i_g.eq(0),
                vga.i_b.eq(0),
            ]
        with m.Else():
            m.d.pixel += [
                vga.i_r.eq((Cat(0b00, vga.o_beam_x[:6] & Z) | W) & (~A)),
                vga.i_g.eq(((vga.o_beam_x[:8] & T) | W) & (~A)),
                vga.i_b.eq(vga.o_beam_x[:8] | W | A),
            ]

            # enable the clock
        m.d.comb += vga.i_clk_en.eq(1)

        # Add OSD
        x_start = self.start_x + 128
        x_stop = self.start_x + (8 * self.chars_x) - 1
        y_start = self.start_y + 48
        y_stop = self.start_y + (16 * self.chars_y) - 1
        #print("x_start", x_start)
        #print("x_stop", x_stop)
        #print("y_start", y_start)
        #print("y_stop", y_stop)
        m.submodules.osd = osd = Osd(x_start=x_start, x_stop=x_stop,
                                     y_start=y_start, y_stop=y_stop)

        osd_en = Signal(reset=self.init_on)
        osd_x = Signal(10)
        osd_y = Signal(10)
        dout = Signal(8)
        dout_align = Signal(8)
        osd_pixel = Signal()

        m.d.comb += [
            osd.clk_ena.eq(1),
            osd.i_r.eq(vga.i_r),
            osd.i_g.eq(vga.i_g),
            osd.i_b.eq(vga.i_b),
            osd.i_osd_r.eq(Mux(vga.o_vga_blank, 0x00, Mux(osd_pixel, C(0xff, 8), C(0x50, 8)))),
            osd.i_osd_g.eq(Mux(vga.o_vga_blank, 0x00, Mux(osd_pixel, C(0xff, 8), C(0x30, 8)))),
            osd.i_osd_b.eq(Mux(vga.o_vga_blank, 0x00, Mux(osd_pixel, C(0xff, 8), C(0x20, 8)))),
            osd.i_hsync.eq(vga.o_vga_hsync),
            osd.i_vsync.eq(vga.o_vga_vsync),
            osd.i_blank.eq(vga.o_vga_vblank),
            osd.i_osd_ena.eq(osd_en),
            osd_x.eq(osd.o_osd_x),
            osd_y.eq(osd.o_osd_y),
            dout_align.eq(Cat(dout[1:], dout[0])),
            osd_pixel.eq(dout_align.bit_select(7 - osd_x[:3], 1)),
        ]

        # Grab our VGA Tile resource
        av_tile = platform.request("av_tile")

        # Hook it up to the OSD output
        m.d.comb += [
            av_tile.red.eq(osd.o_r[5:]),
            av_tile.green.eq(osd.o_g[4:]),
            av_tile.blue.eq(osd.o_b[5:]),
            av_tile.hs.eq(osd.o_hsync),
            av_tile.vs.eq(osd.o_vsync)
        ]

        # Read in the tilemap
        tile_map = readhex(self.char_file)

        tile_data = Memory(width=8 + self.inverse, depth=self.chars_x * self.chars_y, init=tile_map)

        m.submodules.tr = tr = tile_data.read_port(domain="pixel")
        m.submodules.tw = tw = tile_data.write_port(domain="sync")

        # Read in the font
        font = readbin(self.font_file)

        font_data = Memory(width=8, depth=4096, init=font)

        m.submodules.fr = fr = font_data.read_port(domain="pixel")

        # Connect tilemap
        m.d.comb += [
            tw.addr.eq(qspimem.addr - 0x101),
            tw.en.eq(qspimem.wr & (qspimem.addr > 0x100)),
            tw.data.eq(qspimem.dout),
            tr.addr.eq((osd_y >> 4) * self.chars_x + ((osd_x + 1) >> 3)),
            dout.eq(fr.data),
            fr.addr.eq(Cat(osd_y[:4], tr.data))
        ]

        # Write to peripherals

        # 7-segment lower byte
        with m.If(qspimem.wr & (qspimem.addr == 0)):
            m.d.sync += display.eq(qspimem.dout)

            # 7-segment top nibble
        with m.If(qspimem.wr & (qspimem.addr == 1)):
            m.d.sync += display[8:].eq(qspimem.dout[:4])

            # Led blade and led
        with m.If(qspimem.wr & (qspimem.addr == 2)):
            m.d.sync += [
                leds6.eq(qspimem.dout[:6]),
                led.eq(qspimem.dout[7])
            ]

            # Uart
        m.d.comb += [
            # Write received data
            serial.tx.data.eq(fifo.r_data),
            # Write data to fifo when received
            fifo.w_data.eq(qspimem.dout),
            fifo.w_en.eq(qspimem.wr & (qspimem.addr >= 3) & (qspimem.addr < 0x100)),
            fifo.r_en.eq(serial.tx.rdy),
            serial.tx.ack.eq(fifo.r_rdy & serial.tx.rdy),
        ]

        # VGA OSD
        with m.If(qspimem.wr & (qspimem.addr == 0x100)):
            m.d.sync += osd_en.eq(qspimem.dout[0])

        return m


def synth():
    platform = IceLogicBusPlatform()
    platform.add_resources(seven_seg_tile.tile_resources(SEG7_TILE))
    platform.add_resources(AAVC_tile.tile_resources(VGA_TILE))
    platform.add_resources(led_blade)
    platform.add_resources(uart_pmod)
    platform.build(QbusOsd(), nextpnr_opts="--timing-allow-fail", do_program=True)
    # Send bus command
    addr = 0
    data = b'\x42\x06\xb8Hello World!\r\n'
    command = b'\x03' + addr.to_bytes(4, 'big') + len(data).to_bytes(4, 'big') + data
    print("Sending command: ", command)
    platform.bus_send(command)
    addr = 0x100
    # data = b'\x01Dummy data abcdefghijklmnopqrstuvwxyz 0123456789 ABCDEFGHIJKLMNOPQRSTUVWXYZ'
    data = b'\x01ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789abcdefghijklmnopqrstuvwxyz  ABCDEFGHIJKLMNOPQRSTUVWXYZ'
    command = b'\x03' + addr.to_bytes(4, 'big') + len(data).to_bytes(4, 'big') + data
    print("Sending command: ", command)
    platform.bus_send(command)


if __name__ == "__main__":
    synth()
