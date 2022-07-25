from amaranth import *
from amaranth.build import *

from amaranth.lib.cdc import FFSynchronizer

from HDL.Misc.pll import PLL
from qspimem import QspiMem

from mystorm_boards.icelogicbus import *

from hyperflash_write import HyperflashWrite, HyperbusPins

import time

LED_BLADE = 1

led_blade = [
    Resource("leds6", 0,
             Subsignal("leds", Pins("1 2 3 4 5 6", dir="o", invert=True, conn=("blade", LED_BLADE))),
             Attrs(IO_STANDARD="SB_LVCMOS"))
]


class QbusHf(Elaboratable):
    def elaborate(self, platform):
        m = Module()

        # Qspi domain
        clk_freq = 100000000
        # Create a Pll for higher speed clock
        m.submodules.pll = pll = PLL(freq_in_mhz=int(platform.default_clk_frequency / 1e6),
                                     freq_out_mhz=int(clk_freq / 1e6),
                                     domain_name="qspi")
        # Set the sync domain to the pll domain
        m.domains.qspi = cd_qspi = pll.domain
        m.d.comb += pll.clk_pin.eq(ClockSignal())
        platform.add_clock_constraint(cd_qspi.clk, clk_freq)

        # Add QspiMem submodule
        m.submodules.qspimem = qspimem = QspiMem(domain="qspi")

        # Connect pins
        qspi = platform.request("qspi")
        led = platform.request("led")

        m.d.comb += [
            qspimem.qss.eq(qspi.cs),
            qspimem.qck.eq(qspi.clk),
            qspimem.qd_i.eq(qspi.data.i),
            qspi.data.o.eq(qspimem.qd_o),
            qspi.data.oe.eq(qspimem.qd_oe)
        ]

        mem = Memory(width=16, depth=128)
        m.submodules.rp = rp = mem.read_port()
        m.submodules.wp = wp = mem.write_port(domain="qspi")

        hbus = platform.request("hyperbus", 0)
        pins = HyperbusPins()

        m.submodules.hfw = hfw = HyperflashWrite(pins=pins, init_latency=16)

        # HyperFlash start address and command
        addr0 = Signal(8)
        addr1 = Signal(8)
        addr2 = Signal(8)
        cmd = Signal(2)

        #  Save lower byte of received data
        lb = Signal(8)
        with m.If(qspimem.wr & (qspimem.addr < 256) & ~qspimem.addr[0]):
            m.d.qspi += lb.eq(qspimem.dout)

        m.d.comb += [
            hbus.clk.o.eq(pins.clk_o),
            hbus.cs.o[1].eq(pins.csn_o),  # HyperFlash
            hbus.cs.o[0].eq(1),           # Disable HyperRAM

            hbus.rd.o.eq(pins.rwds_o),
            hbus.rd.oe.eq(pins.rwds_oe),
            pins.rwds_i.eq(hbus.rd.i),

            hbus.data.o.eq(pins.dq_o),
            hbus.data.oe.eq(pins.dq_oe),
            pins.dq_i.eq(hbus.data.i),

            wp.addr.eq(qspimem.addr[1:]),
            wp.en.eq(qspimem.wr & (qspimem.addr < 256) & qspimem.addr[0]),
            wp.data.eq(Cat(lb, qspimem.dout)),

            rp.addr.eq(Cat(~hfw.addr[0], hfw.addr[1:7])),

            hfw.start_addr.eq(Cat(addr0, addr1, addr2)),
            hfw.len.eq(128),
            hfw.cmd.eq(cmd),
            hfw.din.eq(rp.data)
        ]

        # QspiMem address 259 is cmd to execute
        with m.If(qspimem.wr & (qspimem.addr == 259)):
            m.d.qspi += cmd.eq(qspimem.dout)

        # Address 256 to 258 send the starting address or sector
        with m.If(qspimem.wr & (qspimem.addr == 256)):
            m.d.qspi += addr2.eq(qspimem.dout)

        with m.If(qspimem.wr & (qspimem.addr == 257)):
            m.d.qspi += addr1.eq(qspimem.dout)

        with m.If(qspimem.wr & (qspimem.addr == 258)):
            m.d.qspi += addr0.eq(qspimem.dout)

        # Get done signal from sync domain and latch it in r_done
        done = Signal()
        m.submodules += FFSynchronizer(hfw.done, done, o_domain="qspi")

        r_done = Signal()
        m.d.qspi += r_done.eq(done)

        # When done signal goes high, unset cmd
        with m.If(done & ~r_done):
            m.d.qspi += cmd.eq(0)

        # Set led when cmd done
        m.d.comb += led.eq(done)

        leds6 = platform.request("leds6")
        m.d.comb += leds6.eq(cmd)

        return m


def send_cmd(addr, data, debug=True):
    command = b'\x03' + addr.to_bytes(4, 'big') + len(data).to_bytes(4, 'big') + data
    if debug:
        print("Sending command: ", command)
    platform.bus_send(command)


def send_file(fn):
    """ Send file in 256 byte chunks """
    data = bytearray()
    addr = 0
    with open(fn, "rb") as f:
        print("Sending file: ", fn)
        byte = f.read(1)
        n = 0
        while True:
            while byte and n < 256:
                data += byte
                byte = f.read(1)
                n += 1
            print("Sending to address {0}, number of bytes {1}".format(addr, n))
            # Put data in BRAM buffer
            send_cmd(0, data, debug=False)
            # Program data
            send_cmd(256, addr.to_bytes(3, 'big') + b'\x02')
            # Wait for write buffer
            time.sleep(0.1)
            if not byte:
                break
            data = bytearray()
            n = 0
            addr += 256


if __name__ == "__main__":
    platform = IceLogicBusPlatform()
    platform.add_resources(led_blade)
    platform.build(QbusHf(), nextpnr_opts="--timing-allow-fail", do_program=True)

    # Erase segment 0
    data = b'\x00\x00\x00\x01'
    send_cmd(256, data, debug=True)
    time.sleep(1)
    # Send a file
    send_file("bios.test")
