from amaranth import *

from mystorm_boards.icelogicbus import *

from hyperflash_write import HyperflashWrite, HyperbusPins


class TestHyperflash(Elaboratable):
    def elaborate(self, platform):
        m = Module()

        hbus = platform.request("hyperbus", 0)
        pins = HyperbusPins()

        m.submodules.hfw = hfw = HyperflashWrite(pins=pins)

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
            hfw.cmd.eq(2)
        ]

        return m

if __name__ == "__main__":
    platform = IceLogicBusPlatform()
    platform.build(TestHyperflash(), nextpnr_opts="--timing-allow-fail", do_program=True)
