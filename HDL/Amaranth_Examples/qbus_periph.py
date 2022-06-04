from amaranth import *
from amaranth.build import *
from amaranth_stdio.serial import AsyncSerial
from amaranth.lib.fifo import SyncFIFOBuffered

from mystorm_boards.icelogicbus import *
from HDL.Tiles.seven_seg_tile import tile_resources
from qspimem import QspiMem
from HDL.Misc.pll import PLL

from sevensegtile import SevenSegmentTile

BLADE = 1
TILE = 3

led_blade = [
    Resource("leds6", 0,
                Subsignal("leds",
                          Pins("1 2 3 4 5 6", dir="o", invert=True, conn=("blade", BLADE)),
                          Attrs(IO_STANDARD="SB_LVCMOS")
                          )
             )
]

PMOD = 5

uart_pmod = [
    Resource("ext_uart", 0,
             Subsignal("tx", Pins("10", dir="o", conn=("pmod", PMOD))),
             Subsignal("rx", Pins("4", dir="i", conn=("pmod", PMOD))),
             Subsignal("gnd", Pins("9", dir="o", conn=("pmod", PMOD))),
             Attrs(IO_STANDARD="SB_LVCMOS"))
]





class QbusPeriph(Elaboratable):
    def elaborate(self, platform):
        qspi  = platform.request("qspi")
        qss   = qspi.cs
        qck   = qspi.clk
        qd_i  = qspi.data.i
        qd_o  = qspi.data.o
        qd_oe = qspi.data.oe
        leds6 = platform.request("leds6")
        led = platform.request("led")

        m = Module()

        # Clock generator.
        CLK_FREQ = 1e8
        clk_in = platform.request(platform.default_clk, dir='-')[0]
        # Create a Pll for 100Mhz clock
        m.submodules.pll = pll = PLL(freq_in_mhz=int(platform.default_clk_frequency / 1e6),
                                     freq_out_mhz=int(CLK_FREQ / 1e6),
                                     domain_name="sync")
        # Set the sync domain to the pll domain
        m.domains.sync = cd_sync = pll.domain
        m.d.comb += pll.clk_pin.eq(clk_in)
        platform.add_clock_constraint(cd_sync.clk, CLK_FREQ)

        # Add QspiMem submodule
        m.submodules.qspimem = qspimem = QspiMem()

        # Connect pins
        m.d.comb += [
            qspimem.qss.eq(qss),
            qspimem.qck.eq(qck),
            qspimem.qd_i.eq(qd_i),
            qd_o.eq(qspimem.qd_o),
            qd_oe.eq(qspimem.qd_oe)
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
        divisor = int(CLK_FREQ // 115200)

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

        # Write to peripherals

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

        return m

def synth():
    platform = IceLogicBusPlatform()
    platform.add_resources(tile_resources(TILE))
    platform.add_resources(led_blade)
    platform.add_resources(uart_pmod)
    platform.build(QbusPeriph(), nextpnr_opts="--timing-allow-fail", do_program=True)
    print("Sending QSPI data")
    platform.bus_send(bytearray(b'\x03\x00\x00\x00\x00\x00\x00\x00\x0a\x42\x06\xb8Hello\r\n'))
    print("Data sent")

if __name__ == "__main__":
    synth()
