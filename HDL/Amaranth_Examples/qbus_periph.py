from amaranth import *
from amaranth.hdl.ast import Rose, Fell
from amaranth.utils import bits_for
from amaranth.build import *
from amaranth_stdio.serial import AsyncSerial
from amaranth.lib.fifo import SyncFIFOBuffered

from mystorm_boards.icelogicbus import *
from HDL.Tiles.seven_seg_tile import SevenSegController, tile_resources

from amaranth.lib.cdc import FFSynchronizer

from HDL.Misc.pll import PLL

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

class QspiMem(Elaboratable):
    def __init__(self, addr_bits=23, data_bits=8):
        # parameters
        self.addr_bits    = addr_bits
        self.data_bits    = data_bits
        self.addr_nibbles = 4
        self.data_nibbles = 2

        # inputs
        self.qd_i  = Signal(4)
        self.qss   = Signal()
        self.qck   = Signal()
        self.din   = Signal(data_bits)

        # outputs
        self.addr  = Signal(addr_bits)
        self.qd_o  = Signal(4)
        self.qd_oe = Signal(4)
        self.dout  = Signal(data_bits)
        self.rd    = Signal()
        self.wr    = Signal()

    def elaborate(self, platform):
        m = Module()

        r_req_read     = Signal()
        r_req_write    = Signal()
        r_cmd          = Signal(self.data_bits)
        r_data         = Signal(self.data_bits)
        r_addr         = Signal(self.addr_bits)

        r_nibble_count = Signal(5)

        r_qd_i         = Signal(4)
        r_qck          = Signal()
        r_qss          = Signal()

        # Ignore spurious QSPI data after programming
        pwr_on_reset = Signal(9)
        with m.If(~pwr_on_reset.all()):
            m.d.sync += pwr_on_reset.eq(pwr_on_reset + 1)

        new_nibble = ~r_qss & pwr_on_reset.all() & Rose(r_qck)

        # Drive outputs
        m.d.comb += [
            self.rd.eq(r_req_read),
            self.wr.eq(r_req_write),
            self.dout.eq(r_data),
            self.addr.eq(r_addr),
        ]

        # De-glitch
        m.submodules += FFSynchronizer(self.qss, r_qss, reset=1)
        m.submodules += FFSynchronizer(self.qck, r_qck, reset=0)
        m.submodules += FFSynchronizer(self.qd_i, r_qd_i, reset=0)

        # Reset signals when qss is high
        with m.If(r_qss):
            m.d.sync += [
                r_req_read.eq(0),
                r_req_write.eq(0),
                r_nibble_count.eq(0),
                self.qd_oe.eq(0)
            ]
        with m.Else():  # qss == 0
            with m.If(new_nibble):
                m.d.sync += r_nibble_count.eq(r_nibble_count + 1)

        with m.FSM():
            with m.State("COMMAND"):
                with m.If(new_nibble):
                    # Read in the byte with the command bit and the top 7 address bits
                    m.d.sync += r_cmd.eq(Cat(r_qd_i, r_cmd[:-4]))
                    with m.If(r_nibble_count == 1):
                        m.next = "ADDRESS"
            with m.State("ADDRESS"):
                with m.If(new_nibble):
                    with m.If(r_nibble_count == self.addr_nibbles+1):
                        m.d.sync += r_addr.eq(Cat(r_qd_i, r_addr[:-11], r_cmd[:7]) - 1)
                        with m.If(r_cmd[7]):
                            m.d.sync += [
                                self.qd_oe.eq(Repl(0b1, 4)),
                                r_req_read.eq(1)
                            ]
                            m.next = "READ_DATA"
                        with m.Else():
                            m.next = "WRITE_DATA"
                    with m.Else():
                        m.d.sync += r_addr.eq(Cat(r_qd_i, r_addr[:-4])),
            with m.State("WRITE_DATA"):
                with m.If(new_nibble):
                    # write data
                    m.d.sync += r_data.eq(Cat(r_qd_i, r_data[:-4]))
                    with m.If(r_nibble_count[0]):
                        m.d.sync += r_req_write.eq(1)
                    with m.Else():
                        m.d.sync += [
                            r_req_write.eq(0),
                            r_addr.eq(r_addr + 1)
                        ]
                with m.Else():
                    m.d.sync += r_req_write.eq(0)

                with m.If(Rose(r_qss)):
                    m.next = "COMMAND"
            with m.State("READ_DATA"):
                with m.If(new_nibble):
                    with m.If(r_nibble_count[0]):
                        m.d.sync += [
                            r_req_read.eq(1),
                            r_addr.eq(r_addr + 1),
                            self.qd_o.eq(self.din[:4])
                        ]
                    with m.Else():
                        m.d.sync += [
                            r_req_read.eq(0),
                            self.qd_o.eq(self.din[4:]),
                        ]
                with m.If(Rose(r_qss)):
                    m.next = "COMMAND"

        return m


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
        clk_in = platform.request(platform.default_clk, dir='-')[0]
        # Create a Pll for 100Mhz clock
        m.submodules.pll = pll = PLL(freq_in_mhz=int(platform.default_clk_frequency / 1000000),
                                     freq_out_mhz=100,
                                     domain_name="sync")
        # Set the sync domain to the pll domain
        m.domains.sync = cd_sync = pll.domain
        m.d.comb += pll.clk_pin.eq(clk_in)
        platform.add_clock_constraint(cd_sync.clk, 100000000)

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

        # Put Data on 7-segment display
        m.submodules.seven = seven = SevenSegController()
        display = Signal(12)

        # Get pins
        seg_pins = platform.request("seven_seg_tile")
        leds7 = Cat([seg_pins.a, seg_pins.b, seg_pins.c, seg_pins.d,
                     seg_pins.e, seg_pins.f, seg_pins.g])

        timer = Signal(19)
        m.d.sync += timer.eq(timer + 1)

        m.d.comb += leds7.eq(seven.leds)

        for i in range(3):
            m.d.comb += seg_pins.ca[i].eq(timer[17:19] == i)

        with m.If(seg_pins.ca[2]):
            m.d.comb += seven.val.eq(display[8:])
        with m.If(seg_pins.ca[1]):
            m.d.comb += seven.val.eq(display[4:8])
        with m.If(seg_pins.ca[0]):
            m.d.comb += seven.val.eq(display[:4])

        # Create uart
        ext_uart = platform.request("ext_uart")
        divisor = int(100000000 // 115200)

        tx = ext_uart.tx
        rx = ext_uart.rx
        gnd = ext_uart.gnd

        # Create the uart
        m.submodules.serial = serial = AsyncSerial(divisor=divisor)

        # Create fifo for incoming uart data
        m.submodules.fifo = fifo = SyncFIFOBuffered(width=8, depth=32)

        # Write to peripherals

        # Uart
        m.d.comb += [
            # Write received data
            serial.tx.data.eq(Mux(fifo.r_data == 0x00, ord("\n"), fifo.r_data)),
            # Write data to fifo when received
            fifo.w_data.eq(qspimem.dout),
            fifo.w_en.eq(qspimem.wr & (qspimem.addr >= 3)),
            fifo.r_en.eq(serial.tx.rdy),
            serial.tx.ack.eq(fifo.r_rdy & serial.tx.rdy),
            # Set GND pin
            gnd.eq(0),
            # Connect uart pins
            serial.rx.i.eq(rx),
            tx.eq(serial.tx.o)
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
