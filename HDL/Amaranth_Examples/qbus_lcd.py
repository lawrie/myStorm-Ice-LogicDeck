from amaranth import *
from amaranth.build import *
from amaranth_stdio.serial import AsyncSerial
from amaranth.lib.fifo import SyncFIFOBuffered

from mystorm_boards.icelogicbus import *

from HDL.Tiles.seven_seg_tile import tile_resources
from HDL.Misc.pll import PLL

from qspimem import QspiMem
from sevensegtile import SevenSegmentTile

from st7789 import *

from readbin import readbin

LED_BLADE = 1
LCD_BLADE = 3

TILE = 3
PMOD = 5

led_blade = [
    Resource("leds6", 0,
             Subsignal("leds", Pins("1 2 3 4 5 6", dir="o", invert=True, conn=("blade", LED_BLADE))),
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

lcd_blade = [
    Resource("oled", 0,
             Subsignal("oled_bl",   Pins("1", dir="o", conn=("blade", LCD_BLADE))),
             Subsignal("oled_resn",   Pins("2", dir="o", conn=("blade", LCD_BLADE))),
             Subsignal("oled_csn",   Pins("3", dir="o", conn=("blade", LCD_BLADE))),
             Subsignal("oled_clk", Pins("4", dir="o", conn=("blade", LCD_BLADE))),
             Subsignal("oled_dc", Pins("5", dir="o", conn=("blade", LCD_BLADE))),
             Subsignal("oled_mosi", Pins("6", dir="o", conn=("blade", LCD_BLADE))),
             Attrs(IO_STANDARD="SB_LVCMOS"))
]

class QbusLcd(Elaboratable):
    def __init__(self, font_file="font_bizcat8x16.mem"):
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

        # Add the st7789 LCD
        oled = platform.request("oled")
        m.submodules.st7789 = st7789 = ST7789(reset_delay=100000, reset_period=100000)

        m.d.comb += [
            oled.oled_clk.eq(st7789.spi_clk),
            oled.oled_mosi.eq(st7789.spi_mosi),
            oled.oled_dc.eq(st7789.spi_dc),
            oled.oled_resn.eq(st7789.spi_resn),
            oled.oled_csn.eq(st7789.spi_csn),
            oled.oled_bl.eq(0)
        ]

        # Create the tile map
        tile_data = Memory(width=8, depth=30 * 15)
        m.submodules.tr = tr = tile_data.read_port()
        m.submodules.tw = tw = tile_data.write_port()

        # Read in the font
        font = readbin(self.font_file)
        font_data = Memory(width=8, depth=4096, init=font)
        m.submodules.fr = fr = font_data.read_port()

        # Connect tilemap
        m.d.comb += [
            tw.addr.eq(qspimem.addr - 0x100),
            tw.en.eq(qspimem.wr & (qspimem.addr >= 0x100)),
            tw.data.eq(qspimem.dout),
            tr.addr.eq(st7789.y[4:] * 30 + st7789.x[3:]),
            fr.addr.eq(Cat(st7789.y[:4], tr.data)),
            st7789.color.eq(Mux(fr.data.bit_select(~st7789.x[:3], 1), 0xfff, 0x0000))
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
            fifo.w_en.eq(qspimem.wr & (qspimem.addr >= 3)),
            fifo.r_en.eq(serial.tx.rdy),
            serial.tx.ack.eq(fifo.r_rdy & serial.tx.rdy),
        ]

        return m


def synth():
    platform = IceLogicBusPlatform()
    platform.add_resources(tile_resources(TILE))
    platform.add_resources(led_blade)
    platform.add_resources(lcd_blade)
    platform.add_resources(uart_pmod)
    platform.build(QbusLcd(), nextpnr_opts="--timing-allow-fail", do_program=True)
    # Send bus command
    addr = 0
    data = b'\x42\x06\xb8Hello World!\r\n'
    command = b'\x03' + addr.to_bytes(4, 'big') + len(data).to_bytes(4, 'big') + data
    print("Sending command: ", command)
    platform.bus_send(command)
    addr = 0x100
    data = b'ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789abcdefghijklmnopqrstuvwxyz'
    command = b'\x03' + addr.to_bytes(4, 'big') + len(data).to_bytes(4, 'big') + data
    print("Sending command: ", command)
    platform.bus_send(command)


if __name__ == "__main__":
    synth()
