from amaranth import *

class HyperbusPins(Record):
    def __init__(self, cs_count=1):
        layout = [
            ("clk_o", 1),
            ("csn_o", cs_count),
            ("rstn_o", 1),
            ("rwds_o", 1),
            ("rwds_oe", 1),
            ("rwds_i", 1),
            ("dq_o", 8),
            ("dq_oe", 8),
            ("dq_i", 8),
        ]
        super().__init__(layout)

IDLE = 0
ERASE = 1
WRITE = 2

class HyperflashWrite(Elaboratable):
    def __init__(self, pins):
        self.cmd  = Signal(2)
        self.addr = Signal(24)
        self.din  = Signal(8)
        self.pins = pins

    def elaborate(self, platform):
        m = Module()

        counter = Signal(8)
        clk = Signal()
        csn = Signal()

        # Data shift register
        sr = Signal(64)

        # Drive out clock on negedge while active
        m.domains += ClockDomain("neg", clk_edge="neg")
        m.d.comb += [
            ClockSignal("neg").eq(ClockSignal()),
            ResetSignal("neg").eq(ResetSignal()),
        ]

        with m.If(csn):
            # Reset clock if nothing active
            m.d.neg += clk.eq(0)
        with m.Elif(counter.any()):
            m.d.neg += clk.eq(~clk)
            m.d.sync += counter.eq(counter-1)
        with m.If(counter.any()):
            # move shift register (sample/output data) on posedge
            m.d.sync += sr.eq(Cat(self.pins.dq_i, sr[:-8]))

        m.d.comb += [
            self.pins.clk_o.eq(clk),
            self.pins.csn_o.eq(csn),
            self.pins.dq_o.eq(sr[-8:]),
        ]

        with m.FSM() as fsm:
            with m.State("IDLE"):
                m.d.sync += [
                    counter.eq(0),
                    self.pins.rwds_oe.eq(0),
                    csn.eq(1),
                    sr[63].eq(0),  # Always write
                    sr[62].eq(0),  # Memory
                    sr[61].eq(0),  # Burst type not used
                    sr[19:32].eq(0), # Reserved
                    self.pins.dq_oe.eq(1),
                ]
                with m.If(self.cmd == ERASE):
                    m.d.sync += [
                        csn.eq(0),
                        counter.eq(8),
                        sr[16:19].eq(0x5),  # Send Unlock 1
                        sr[32:61].eq(0xAA),
                        sr[0:16].eq(0x00AA)
                    ]
                    m.next = "EWAIT1"
                with m.If(self.cmd == WRITE):
                    m.d.sync += [
                        csn.eq(0),
                        counter.eq(8),
                        sr[16:19].eq(0x5),  # Send Unlock 1
                        sr[32:61].eq(0xAA),
                        sr[0:16].eq(0x00AA)
                    ]
                    m.next = "WWAIT1"
            with m.State("EWAIT1"):
                with m.If(counter == 0):
                    m.d.sync += csn.eq(1)
                    m.next = "EUNLOCK2"
            with m.State("EUNLOCK2"):
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8),
                    sr[16:19].eq(0x2),
                    sr[32:61].eq(0x55),
                    sr[0:16].eq(0x0055)
                ]
                m.next = "EWAIT2"
            with m.State("EWAIT2"):
                with m.If(counter == 0):
                    m.d.sync += csn.eq(1)
                    m.next = "ECMD1"
            with m.State("ECMD1"):
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8),
                    sr[16:19].eq(0x5),
                    sr[32:61].eq(0xAA),
                    sr[0:16].eq(0x0080)
                ]
                m.next = "EWAIT3"
            with m.State("EWAIT3"):
                with m.If(counter == 0):
                    m.d.sync += csn.eq(1)
                    m.next = "ECMD2"
            with m.State("ECMD2"):
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8),
                    sr[16:19].eq(0x5),
                    sr[32:61].eq(0xAA),
                    sr[0:16].eq(0x00AA)
                ]
                m.next = "EWAIT4"
            with m.State("EWAIT4"):
                with m.If(counter == 0):
                    m.d.sync += csn.eq(1)
                    m.next = "ECMD3"
            with m.State("ECMD3"):
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8),
                    sr[16:19].eq(0x2),
                    sr[32:61].eq(0x55),
                    sr[0:16].eq(0x0055)
                ]
                m.next = "EWAIT5"
            with m.State("EWAIT5"):
                with m.If(counter == 0):
                    m.d.sync += csn.eq(1)
                    m.next = "ERASE"
            with m.State("ERASE"):
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8),
                    sr[16:18].eq(0x0),
                    sr[32:61].eq(0x0), # Segment 0 for now
                    sr[0:16].eq(0x0030)
                ]
                m.next = "EWAIT6"
            with m.State("EWAIT6"):
                with m.If(counter == 0):
                    m.d.sync += csn.eq(1)
                    m.next = "STATUS"
            with m.State("WWAIT1"):
                with m.If(counter == 0):
                    m.d.sync += csn.eq(1)
                    m.next = "WUNLOCK2"
            with m.State("WUNLOCK2"):
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8),
                    sr[16:19].eq(0x2),
                    sr[32:61].eq(0x55),
                    sr[0:16].eq(0x0055)
                ]
                m.next = "WWAIT2"
            with m.State("WWAIT2"):
                with m.If(counter == 0):
                    m.d.sync += csn.eq(1)
                    m.next = "WCMD1"
            with m.State("WCMD1"):
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8),
                    sr[16:19].eq(0x0), # Sector address 0
                    sr[32:61].eq(0x0),
                    sr[0:16].eq(0x0025)
                ]
                m.next = "WWAIT3"
            with m.State("WWAIT3"):
                with m.If(counter == 0):
                    m.d.sync += csn.eq(1)
                    m.next = "WCMD2"
            with m.State("WCMD2"):
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8),
                    sr[16:19].eq(0x0),
                    sr[32:61].eq(0x0),
                    sr[0:16].eq(0x0001)  # write 2 words
                ]
                m.next = "WWAIT4"
            with m.State("WWAIT4"):
                with m.If(counter == 0):
                    m.d.sync += csn.eq(1)
                    m.next = "WDAT1"
            with m.State("WDAT1"):
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8),
                    sr[16:19].eq(0x0), # Write 0 to address 0
                    sr[32:61].eq(0x0),
                    sr[0:16].eq(0x0000)
                ]
                m.next = "WWAIT5"
            with m.State("WWAIT5"):
                with m.If(counter == 0):
                    m.d.sync += csn.eq(1)
                    m.next = "WDAT2"
            with m.State("WDAT2"):
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8),
                    sr[16:19].eq(0x1),  # Write 1 to address 1
                    sr[32:61].eq(0x0),
                    sr[0:16].eq(0x0001)
                ]
                m.next = "WWAIT6"
            with m.State("WWAIT6"):
                with m.If(counter == 0):
                    m.d.sync += csn.eq(1)
                    m.next = "WRITE"
            with m.State("WRITE"):
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8),
                    sr[16:18].eq(0x0),
                    sr[32:61].eq(0x0), # Segment 0
                    sr[0:16].eq(0x0029)
                ]
                m.next = "WWAIT7"
            with m.State("WWAIT7"):
                with m.If(counter == 0):
                    m.d.sync += csn.eq(1)
                    m.next = "STATUS"
            with m.State("STATUS"):
                # Todo read the status value
                m.next = "STATUS"

        return m
