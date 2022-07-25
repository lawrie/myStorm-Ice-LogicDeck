from amaranth import *
from amaranth.build import *

from mystorm_boards.icelogicbus import *

from hyperflash_write import HyperflashWrite, HyperbusPins

LED_BLADE = 1

led_blade = [
    Resource("leds6", 0,
             Subsignal("leds", Pins("1 2 3 4 5 6", dir="o", invert=True, conn=("blade", LED_BLADE))),
             Attrs(IO_STANDARD="SB_LVCMOS"))
]


def readbios():
    """ Read bios.bin into an array of integers """
    f = open("bios.bin", "rb")
    l = []
    while True:
        b = f.read(4)
        if b:
            l.append(int.from_bytes(b, "little"))
        else:
            break
    f.close()
    return l


class TestHyperflash(Elaboratable):
    def elaborate(self, platform):
        m = Module()

        bios = readbios()
        mem = Memory(width=32, depth=4096)
        mem.init = bios
        print("bios length:", len(bios))
        print("lines:", (len(bios) + 127) // 128)
        m.submodules.rp = rp = mem.read_port()

        hbus = platform.request("hyperbus", 0)
        pins = HyperbusPins()

        m.submodules.hfw = hfw = HyperflashWrite(pins=pins, init_latency=16)

        size = Signal(13)
        lines = Signal(6)
        lines_done = Signal(6, reset=0)
        cmd = Signal(2, reset=1)  # start with erase segment

        m.d.comb += [
            size.eq(len(bios) + 127),
            lines.eq(size[7:]),           # Number of 512 byte lines
            hbus.clk.o.eq(pins.clk_o),
            hbus.cs.o[1].eq(pins.csn_o),  # HyperFlash
            hbus.cs.o[0].eq(1),           # Disable HyperRAM

            hbus.rd.o.eq(pins.rwds_o),
            hbus.rd.oe.eq(pins.rwds_oe),
            pins.rwds_i.eq(hbus.rd.i),

            hbus.data.o.eq(pins.dq_o),
            hbus.data.oe.eq(pins.dq_oe),
            pins.dq_i.eq(hbus.data.i),
            hfw.start_addr.eq(Cat(C(0, 9), lines_done)),
            hfw.len.eq(256),
            rp.addr.eq(hfw.addr[1:]),
            hfw.cmd.eq(cmd)
        ]

        # Register the done signal
        r_done = Signal()
        m.d.sync += r_done.eq(hfw.done)

        # Set data to word address
        with m.If(hfw.next):
            m.d.sync += hfw.din.eq(Mux(hfw.addr[0], rp.data[:16], rp.data[16:]))

        # Do an erase command followed by series of write-buffer commands
        with m.If(hfw.done & ~r_done):
            with m.If((lines_done == lines - 1) & (cmd == 2)):
                m.d.sync += cmd.eq(0)   # All lines done
            with m.Else():
                with m.If(cmd == 2):
                    m.d.sync += lines_done.eq(lines_done + 1)
                with m.Else():
                    m.d.sync += cmd.eq(2)

        led = platform.request("led")
        leds6 = platform.request("leds6")

        m.d.comb += [
            led.eq(hfw.done),
            leds6.eq(hfw.err)
        ]

        return m


if __name__ == "__main__":
    platform = IceLogicBusPlatform()
    platform.add_resources(led_blade)
    platform.build(TestHyperflash(), nextpnr_opts="--timing-allow-fail", do_program=True)
