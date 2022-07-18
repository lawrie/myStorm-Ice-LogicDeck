from amaranth import *
from amaranth.sim import Simulator, Passive
from hyperflash_write import HyperflashWrite, HyperbusPins

if __name__ == "__main__":
    m = Module()

    pins = HyperbusPins()
    m.submodules.hf = hf = HyperflashWrite(pins=pins)

    sim = Simulator(m)
    sim.add_clock(4e-8)

    def process():
        yield hf.cmd.eq(2)
        yield hf.start_addr.eq(0x00000)
        yield hf.len.eq(2)
        yield pins.csn_o.eq(1)
        yield
        yield hf.cmd.eq(0)
        yield Passive()

    sim.add_sync_process(process)

    with sim.write_vcd("test.vcd", "test.gtkw"):
        sim.run_until(5e-6, run_passive=True)
