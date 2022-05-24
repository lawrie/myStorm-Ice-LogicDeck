from amaranth import *
from amaranth_stdio.serial import AsyncSerial

from mystorm_boards.icelogicbus import *
from amaranth.build import *

TILE = 2

uart_tile = [
    Resource("ext_uart", 0,
            Subsignal("tx", Pins("12", dir="o", conn=("tile", TILE)), Attrs(IO_STANDARD="SB_LVCMOS")),
            Subsignal("rx", Pins("11", dir="i", conn=("tile", TILE)), Attrs(IO_STANDARD="SB_LVCMOS")),
            Subsignal("gnd", Pins("10", dir="o", conn=("tile", TILE)), Attrs(IO_STANDARD="SB_LVCMOS")))
]

class Uart(Elaboratable):
    def elaborate(self, platform):
        led = platform.request("led")
        ext_uart = platform.request("ext_uart")
        divisor = int(platform.default_clk_frequency // 115200)

        tx = ext_uart.tx
        rx = ext_uart.rx
        gnd = ext_uart.gnd

        timer = Signal(24)

        m = Module()

        # Create the uart
        m.submodules.serial = serial = AsyncSerial(divisor=divisor)

        m.d.sync += timer.eq(timer + 1)

        m.d.comb += [
            # Connect data out to data in
            serial.tx.data.eq(serial.rx.data),
            # Always allow reads
            serial.rx.ack.eq(1),
            # Write data when received
            serial.tx.ack.eq(serial.rx.rdy),
            # Blink the led
            led.eq(timer[-1]),
            # Set GND pin
            gnd.eq(0),
            # Connect uart pins
            serial.rx.i.eq(rx),
            tx.eq(serial.tx.o)
        ]

        return m

def synth():
    platform = IceLogicBusPlatform()
    platform.add_resources(uart_tile)
    platform.build(Uart(), do_program=True)


if __name__ == "__main__":
    synth()