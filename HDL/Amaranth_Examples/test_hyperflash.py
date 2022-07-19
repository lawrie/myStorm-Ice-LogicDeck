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


class TestHyperflash(Elaboratable):
    def elaborate(self, platform):
        m = Module()

        hbus = platform.request("hyperbus", 0)
        pins = HyperbusPins()

        m.submodules.hfw = hfw = HyperflashWrite(pins=pins, init_latency=16)

        m.d.comb += [
            hbus.clk.o.eq(pins.clk_o),
            hbus.cs.o[1].eq(pins.csn_o),
            hbus.cs.o[0].eq(1),

            hbus.rd.o.eq(pins.rwds_o),
            hbus.rd.oe.eq(pins.rwds_oe),
            pins.rwds_i.eq(hbus.rd.i),

            hbus.data.o.eq(pins.dq_o),
            hbus.data.oe.eq(pins.dq_oe),
            pins.dq_i.eq(hbus.data.i),
            hfw.start_addr.eq(0),
            hfw.len.eq(256)
        ]

        # Set data to word address
        with m.If(hfw.next):
            m.d.sync += hfw.din.eq(hfw.addr)

        with m.If(hfw.done):
            m.d.sync += hfw.cmd.eq(0)
        with m.Else():
            m.d.sync += hfw.cmd.eq(2)

        return m

if __name__ == "__main__":
    platform = IceLogicBusPlatform()
    platform.add_resources(led_blade)
    platform.build(TestHyperflash(), nextpnr_opts="--timing-allow-fail", do_program=True)
