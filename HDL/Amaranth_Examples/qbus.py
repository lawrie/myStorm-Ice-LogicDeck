from amaranth import *
from amaranth.hdl.ast import Rose
from amaranth.build import *
from amaranth_stdio.serial import AsyncSerial
from amaranth.lib.fifo import SyncFIFOBuffered

from mystorm_boards.icelogicbus import *
from HDL.Tiles.seven_seg_tile import SevenSegController, tile_resources

from amaranth.lib.cdc import FFSynchronizer

from HDL.Misc.pll import PLL
from qspimem import QspiMem

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

TEST_PMOD = 4

qspi_pmod = [
    Resource("qspi_test", 0,
             Subsignal("qss", Pins("10", dir="o", conn=("pmod", TEST_PMOD))),
             Subsignal("qck", Pins("9", dir="o", conn=("pmod", TEST_PMOD))),
             Subsignal("gnd", Pins("8", dir="o", conn=("pmod", TEST_PMOD))),
             Subsignal("qd", Pins("1 2 3 4", dir="o", conn=("pmod", TEST_PMOD))),
             Attrs(IO_STANDARD="SB_LVCMOS"))
]

PMOD = 5

uart_pmod = [
    Resource("ext_uart", 0,
             Subsignal("tx", Pins("10", dir="o", conn=("pmod", PMOD))),
             Subsignal("rx", Pins("4", dir="i", conn=("pmod", PMOD))),
             Subsignal("gnd", Pins("9", dir="o", conn=("pmod", PMOD))),
             Attrs(IO_STANDARD="SB_LVCMOS"))
]


class NibbleToHex(Elaboratable):
    def __init__(self):
        self.b = Signal(5)
        self.h = Signal(8)

    def elaborate(self, platform):
        m = Module()

        with m.If(~self.b[4]):
            with m.If(self.b[:4] < 10):
                m.d.comb += self.h.eq(ord('0') + self.b[:4])
            with m.Else():
                m.d.comb += self.h.eq(ord('a') + self.b[:4] - 10)
        with m.Else():
            m.d.comb += self.h.eq(Cat(self.b[:4], C(0, 4)))

        return m

class QbusTest(Elaboratable):
    def elaborate(self, platform):
        qspi  = platform.request("qspi")
        qss   = qspi.cs
        qck   = qspi.clk
        qd_i  = qspi.data.i
        qd_o  = qspi.data.o
        qd_oe = qspi.data.oe
        leds6 = platform.request("leds6")
        led = platform.request("led")
        qspi_test = platform.request("qspi_test")

        rd    = Signal()
        wr    = Signal()
        addr  = Signal(23)
        din   = Signal(8)
        dout  = Signal(8)

        m = Module()

        # Clock generator.
        clk_in = platform.request(platform.default_clk, dir='-')[0]
        # Create a Pll for 100Mhz clock
        m.submodules.pll = pll = PLL(freq_in_mhz=int(platform.default_clk_frequency / 1000000),
                                     freq_out_mhz=100,
                                     domain_name="sync")
        # Set the sync domain to the pll domain
        m.domains.sync = cd_sync = pll.domain
        m.d.comb += pll.clk_pin.eq(clk_in)
        platform.add_clock_constraint(cd_sync.clk, 100000000)

        pwr_on_reset = Signal(9)
        with m.If(~pwr_on_reset.all()):
            m.d.sync += pwr_on_reset.eq(pwr_on_reset + 1)

        m.d.comb += [
            qspi_test.qss.eq(qss),
            qspi_test.qck.eq(qck),
            qspi_test.qd.eq(qd_i),
            qspi_test.gnd.eq(0)
        ]

        # Add QspiMem submodule
        m.submodules.qspimem = qspimem = QspiMem()

        # Create memory to read and write
        mem = Memory(width=8, depth=4 * 1024)
        m.submodules.r = r = mem.read_port()
        m.submodules.w = w = mem.write_port()

        m.d.comb += [
            qspimem.qss.eq(qss),
            qspimem.qck.eq(qck),
            qspimem.qd_i.eq(qd_i),
            qspimem.din.eq(din),
            qd_o.eq(qspimem.qd_o),
            qd_oe.eq(qspimem.qd_oe),
            addr.eq(qspimem.addr),
            dout.eq(qspimem.dout),
            rd.eq(qspimem.rd),
            wr.eq(qspimem.wr),
            r.addr.eq(addr),
            din.eq(r.data),
            w.data.eq(dout),
            w.addr.eq(addr),
            w.en.eq(wr)
        ]

        with m.If(qspimem.wr):
            m.d.sync += led.eq(1)

        nibbles = Signal(8, reset=0)
        r_qss = Signal()
        r_qck = Signal()
        r_qd_i = Signal(4)

        m.submodules += FFSynchronizer(qss, r_qss, reset=1)
        m.submodules += FFSynchronizer(qck, r_qck, reset=0)
        m.submodules += FFSynchronizer(qd_i, r_qd_i, reset=0)

        with m.If(~r_qss & pwr_on_reset.all()):
            with m.If(Rose(r_qck)):
                m.d.sync += nibbles.eq(nibbles + 1)

        m.d.comb += leds6.eq(nibbles)

        # Put Data on 7-segment display
        m.submodules.seven = seven = SevenSegController()
        display = Signal(8)

        # Get pins
        seg_pins = platform.request("seven_seg_tile")
        leds7 = Cat([seg_pins.a, seg_pins.b, seg_pins.c, seg_pins.d,
                     seg_pins.e, seg_pins.f, seg_pins.g])

        timer = Signal(19)
        m.d.sync += timer.eq(timer + 1)

        m.d.comb += [
            leds7.eq(seven.leds)
        ]

        for i in range(3):
            m.d.comb += seg_pins.ca[i].eq(timer[17:19] == i)

        with m.If(seg_pins.ca[2]):
            m.d.comb += seven.val.eq(addr[:4])
        with m.If(seg_pins.ca[1]):
            m.d.comb += seven.val.eq(display[-4:])
        with m.If(seg_pins.ca[0]):
            m.d.comb += seven.val.eq(display[:4])

        with m.If(qspimem.wr):
            m.d.sync += display.eq(dout)

        ext_uart = platform.request("ext_uart")
        divisor = int(100000000 // 115200)

        tx = ext_uart.tx
        rx = ext_uart.rx
        gnd = ext_uart.gnd

        # Create the uart
        m.submodules.serial = serial = AsyncSerial(divisor=divisor)

        m.submodules.nibbletohex = nibbletohex = NibbleToHex()

        m.submodules.fifo = fifo = SyncFIFOBuffered(width=5, depth=256)

        tx_busy  = Signal(1, reset=0)
        tx_state = Signal(1, reset=0)
        nl_state = Signal(2, reset=0)

        # Connect the fifo
        m.d.comb += [
            fifo.w_en.eq((~r_qss & Rose(r_qck) & pwr_on_reset.all()) | nl_state > 0),
            fifo.r_en.eq(~tx_busy)
        ]

        # Add CRLF to end of line by pushing them to fifo with control character bit set
        with m.Switch(nl_state):
            with m.Case(0):
                m.d.comb += fifo.w_data.eq(Cat(r_qd_i, Cat(0, 1)))
            with m.Case(1):
                m.d.comb += fifo.w_data.eq(0x1d)
            with m.Case(2):
                m.d.comb += fifo.w_data.eq(0x1a)

        m.d.comb += [
            # Write hex data
            serial.tx.data.eq(nibbletohex.h),
            # Write data when received
            serial.tx.ack.eq(tx_busy),
            # Set GND pin
            gnd.eq(0),
            # Connect uart pins
            serial.rx.i.eq(rx),
            tx.eq(serial.tx.o)
        ]

        # Put carriage return, line feed in fifo at end of command
        with m.If(Rose(r_qss)):
            m.d.sync += nl_state.eq(1)
        with m.Elif(nl_state == 2):
            m.d.sync += nl_state.eq(0)
        with m.Elif(nl_state == 1):
            m.d.sync += nl_state.eq(2)

        with m.Switch(tx_state):
            with m.Case(0):
                with m.If(fifo.r_rdy):
                    m.d.sync += [
                        tx_state.eq(1),
                        nibbletohex.b.eq(fifo.r_data),
                        tx_busy.eq(1),
                    ]
            with m.Case(1):
                with m.If(serial.tx.rdy):
                    m.d.sync += [
                        tx_state.eq(0),
                        tx_busy.eq(0)
                    ]

        return m

def synth():
    platform = IceLogicBusPlatform()
    platform.add_resources(tile_resources(TILE))
    platform.add_resources(led_blade)
    platform.add_resources(qspi_pmod)
    platform.add_resources(uart_pmod)
    platform.build(QbusTest(), do_program=True)
    print("Sending QSPI data")
    platform.bus_send(bytearray(b'\x03\x00\x00\x00\x00\x00\x00\x00\x02\x42\x56'))
    print("Data sent")

if __name__ == "__main__":
    synth()
