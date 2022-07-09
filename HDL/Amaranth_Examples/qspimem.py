from amaranth import *
from amaranth.hdl.ast import Rose, Fell
from amaranth.lib.cdc import FFSynchronizer

class QspiMem(Elaboratable):
    def __init__(self, domain="sync", addr_bits=23, data_bits=8):
        # parameters
        self.domain       = domain
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
            m.d[self.domain] += pwr_on_reset.eq(pwr_on_reset + 1)

        new_nibble = ~r_qss & pwr_on_reset.all() & Rose(r_qck, domain=self.domain)

        # Drive outputs
        m.d.comb += [
            self.rd.eq(r_req_read),
            self.wr.eq(r_req_write),
            self.dout.eq(r_data),
            self.addr.eq(r_addr),
        ]

        # De-glitch
        m.submodules += FFSynchronizer(self.qss, r_qss, reset=1, o_domain=self.domain)
        m.submodules += FFSynchronizer(self.qck, r_qck, reset=0, o_domain=self.domain)
        m.submodules += FFSynchronizer(self.qd_i, r_qd_i, reset=0, o_domain=self.domain)

        # Reset signals when qss is high
        with m.If(r_qss):
            m.d[self.domain] += [
                r_req_read.eq(0),
                r_req_write.eq(0),
                r_nibble_count.eq(0),
                self.qd_oe.eq(0)
            ]
        with m.Else():  # qss == 0
            with m.If(new_nibble):
                m.d[self.domain] += r_nibble_count.eq(r_nibble_count + 1)

        with m.FSM(domain=self.domain):
            with m.State("COMMAND"):
                with m.If(new_nibble):
                    # Read in the byte with the command bit and the top 7 address bits
                    m.d[self.domain] += r_cmd.eq(Cat(r_qd_i, r_cmd[:-4]))
                    with m.If(r_nibble_count == 1):
                        m.next = "ADDRESS"
            with m.State("ADDRESS"):
                with m.If(new_nibble):
                    with m.If(r_nibble_count == self.addr_nibbles+1):
                        m.d[self.domain] += r_addr.eq(Cat(r_qd_i, r_addr[:-11], r_cmd[:7]) - 1)
                        with m.If(r_cmd[7]):
                            m.d[self.domain] += [
                                self.qd_oe.eq(Repl(0b1, 4)),
                                r_req_read.eq(1)
                            ]
                            m.next = "READ_DATA"
                        with m.Else():
                            m.next = "WRITE_DATA"
                    with m.Else():
                        m.d[self.domain] += r_addr.eq(Cat(r_qd_i, r_addr[:-4])),
            with m.State("WRITE_DATA"):
                with m.If(new_nibble):
                    # write data
                    m.d[self.domain] += r_data.eq(Cat(r_qd_i, r_data[:-4]))
                    with m.If(r_nibble_count[0]):
                        m.d[self.domain] += r_req_write.eq(1)
                    with m.Else():
                        m.d[self.domain] += [
                            r_req_write.eq(0),
                            r_addr.eq(r_addr + 1)
                        ]
                with m.Else():
                    m.d[self.domain] += r_req_write.eq(0)

                with m.If(Rose(r_qss, domain=self.domain)):
                    m.next = "COMMAND"
            with m.State("READ_DATA"):
                with m.If(new_nibble):
                    with m.If(r_nibble_count[0]):
                        m.d[self.domain] += [
                            r_req_read.eq(1),
                            r_addr.eq(r_addr + 1),
                            self.qd_o.eq(self.din[:4])
                        ]
                    with m.Else():
                        m.d[self.domain] += [
                            r_req_read.eq(0),
                            self.qd_o.eq(self.din[4:]),
                        ]
                with m.If(Rose(r_qss, domain=self.domain)):
                    m.next = "COMMAND"

        return m
