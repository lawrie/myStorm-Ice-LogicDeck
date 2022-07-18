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
            ("dq_oe", 1),
            ("dq_i", 8),
        ]
        super().__init__(layout)

IDLE = 0
ERASE = 1
WRITE = 2

class HyperflashWrite(Elaboratable):
    def __init__(self, pins, init_latency=16):
        # Input
        self.cmd  = Signal(2)  # Erase, Write or None
        self.start_addr = Signal(24)  # byte address
        self.len = Signal(9)  # Length of data in words, max 256
        self.din  = Signal(16)  # Next data item to write

        # Output
        self.addr = Signal(24)  # byte address of next item
        self.next = Signal()  # next item requirted
        self.done = Signal()  # command done
        self.err  = Signal()  # command failed

        # Parameters
        self.pins = pins
        self.init_latency = init_latency

        self.latency = Signal(5, reset=self.init_latency)

    def elaborate(self, platform):
        m = Module()

        sector = self.start_addr[18:]

        word_addr = Signal(23)
        rem_words = Signal(10)

        if platform is None:
            leds6 = Signal(6)
            led = Signal()
        else:
            leds6 = platform.request("leds6")
            led = platform.request("led")

        counter = Signal(8)
        clk = Signal()
        csn = Signal(reset=1)

        # Data shift register
        sr = Signal(64)

        def set_sr(addr, data):
            m.d.sync += [
                sr[63].eq(0),  # write
                sr[62].eq(0),  # not used
                sr[61].eq(0),  # burst not used
                sr[32:61].eq(C(addr, 23)[3:]),  # half-page offset
                sr[19:32].eq(0),  # reserved
                sr[16:19].eq(C(addr, 23)[:3]),  # half page address
                sr[:16].eq(C(data, 16))
            ]

        def set_sr_sector(sector, data):
            m.d.sync += [
                sr[63].eq(0),  # write
                sr[62].eq(0),  # not used
                sr[61].eq(0),  # burst not used
                sr[46:61].eq(sector),
                sr[32:46].eq(0),
                sr[19:32].eq(0),  # reserved
                sr[16:19].eq(0x0),
                sr[0:16].eq(C(data, 16))
            ]


        # Incoming data
        dat_r = Signal(16, reset=0x80)

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
            self.pins.dq_o.eq(sr[-8:]),self.addr.eq(word_addr),
            led.eq(~dat_r[7]),
            leds6.eq(dat_r[:6])
        ]

        with m.FSM() as fsm:
            with m.State("IDLE"):
                m.d.sync += [
                    counter.eq(0),
                    self.pins.rwds_oe.eq(0),
                    csn.eq(1),
                    self.pins.dq_oe.eq(1)
                ]
                with m.If(self.cmd == ERASE):
                    set_sr(0x555, 0xAA)  # Unlock 1
                    m.d.sync += [
                        csn.eq(0),
                        counter.eq(8)
                    ]
                    m.next = "E_WAIT1"
                with m.If(self.cmd == WRITE):
                    set_sr(0x555, 0xAA)  # Unlock 1
                    m.d.sync += [
                        csn.eq(0),
                        counter.eq(8),
                        word_addr.eq(self.start_addr[1:]),
                        rem_words.eq(self.len)
                    ]
                    m.next = "W_WAIT1"
            with m.State("E_WAIT1"):
                with m.If(counter == 1):
                    m.d.sync += csn.eq(1)
                    m.next = "E_UNLOCK2"
            with m.State("E_UNLOCK2"):
                set_sr(0x2AA, 0x55)  # Unlock2
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8)
                ]
                m.next = "E_WAIT2"
            with m.State("E_WAIT2"):
                with m.If(counter == 1):
                    m.d.sync += csn.eq(1)
                    m.next = "E_CMD1"
            with m.State("E_CMD1"):
                set_sr(0x555, 0x80)
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8)
                ]
                m.next = "E_WAIT3"
            with m.State("E_WAIT3"):
                with m.If(counter == 1):
                    m.d.sync += csn.eq(1)
                    m.next = "E_CMD2"
            with m.State("E_CMD2"):
                set_sr(0x555, 0xAA)  # Unlock1
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8)
                ]
                m.next = "E_WAIT4"
            with m.State("E_WAIT4"):
                with m.If(counter == 1):
                    m.d.sync += csn.eq(1)
                    m.next = "E_CMD3"
            with m.State("E_CMD3"):
                set_sr(0x2AA, 0x55)  # Unlock2
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8)
                ]
                m.next = "E_WAIT5"
            with m.State("E_WAIT5"):
                with m.If(counter == 1):
                    m.d.sync += csn.eq(1)
                    m.next = "ERASE"
            with m.State("ERASE"):
                set_sr_sector(sector, 0x30)
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8),
                ]
                m.next = "E_WAIT6"
            with m.State("E_WAIT6"):
                with m.If(counter == 1):
                    m.d.sync += csn.eq(1)
                    m.next = "STATUS"
            with m.State("W_WAIT1"):
                with m.If(counter == 1):
                    m.d.sync += csn.eq(1)
                    m.next = "W_UNLOCK2"
            with m.State("W_UNLOCK2"):
                set_sr(0x2AA, 0x55)  # Unlock2
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8),
                ]
                m.next = "W_WAIT2"
            with m.State("W_WAIT2"):
                with m.If(counter == 1):
                    m.d.sync += csn.eq(1)
                    m.next = "W_CMD1"
            with m.State("W_CMD1"):
                set_sr_sector(sector, 0x25)
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8),
                ]
                m.next = "W_WAIT3"
            with m.State("W_WAIT3"):
                with m.If(counter == 1):
                    m.d.sync += csn.eq(1)
                    m.next = "W_CMD2"
            with m.State("W_CMD2"):
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8),
                    sr[63].eq(0),  # Always write
                    sr[62].eq(0),  # Not used
                    sr[61].eq(0),  # Burst type not used
                    sr[19:32].eq(0),  # Reserved
                    sr[16:19].eq(0x0),
                    sr[32:46].eq(0),
                    sr[46:61].eq(sector),
                    sr[0:16].eq(self.len - 1)  # write 2 words
                ]
                m.next = "W_WAIT4"
            with m.State("W_WAIT4"):
                with m.If(counter == 1):
                    m.d.sync += csn.eq(1)
                    m.d.comb += self.next.eq(1)
                    m.next = "W_DATA"
            with m.State("W_DATA"):
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8),
                    sr[63].eq(0),  # Always write
                    sr[62].eq(0),  # Not used
                    sr[61].eq(0),  # Burst type not used
                    sr[19:32].eq(0),  # Reserved
                    sr[16:19].eq(word_addr[:3]),  # data
                    sr[32:61].eq(word_addr[3:]),
                    sr[0:16].eq(self.din),
                    rem_words.eq(rem_words - 1),
                    word_addr.eq(word_addr + 1)
                ]
                m.next = "W_WAIT5"
            with m.State("W_WAIT5"):
                with m.If(counter == 1):
                    m.d.sync += csn.eq(1)
                    with m.If(rem_words > 0):
                        m.d.comb += self.next.eq(1)
                        m.next = "W_DATA"
                    with m.Else():
                        m.next = "WRITE"
            with m.State("WRITE"):
                set_sr_sector(sector, 0x29)
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8),
                ]
                m.next = "W_WAIT7"
            with m.State("W_WAIT7"):
                with m.If(counter == 1):
                    m.d.sync += csn.eq(1)
                    m.next = "STATUS"
            with m.State("STATUS"):
                set_sr(0x555, 0x70)
                m.d.sync += [
                    csn.eq(0),
                    counter.eq(8),
                ]
                m.next = "S_WAIT1"
            with m.State("S_WAIT1"):
                with m.If(counter == 1):
                    m.d.sync += csn.eq(1)
                    m.next = "S_READ"
            with m.State("S_READ"):
                m.d.sync += [
                    csn.eq(0),
                    self.pins.dq_oe.eq(1),
                    counter.eq(6),
                    # Assign CA
                    sr[63].eq(1),  # Read
                    sr[62].eq(0),  # Not used
                    sr[61].eq(0),  # Burst type not used
                    sr[32:61].eq(0),  # Status register
                    sr[19:32].eq(0),  # RFU
                    sr[16:19].eq(0),  # Status register
                ]
                m.next = "S_WAIT2"
            with m.State("S_WAIT2"):
                with m.If(counter == 1):
                    m.d.sync += counter.eq(2 * self.latency - 2)
                    m.next = "S_WAIT3"
            with m.State("S_WAIT3"):
                with m.If(counter == 1):
                    m.d.sync += [
                        self.pins.dq_oe.eq(0),
                        sr.eq(0),
                        counter.eq(2)
                    ]
                    m.next = "SHIFT_DAT"
            with m.State("SHIFT_DAT"):
                with m.If(counter == 1):
                    m.next = "ACK_XFER"
            with m.State("ACK_XFER"):
                m.d.sync += [
                    dat_r.eq(sr[:16]),
                    self.pins.dq_oe.eq(1),
                    csn.eq(1)
                ]
                m.next = "STATUS"

        return m
