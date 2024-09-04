"""NRF24L01 driver for MicroPython
"""

from micropython import const
import utime

# nRF24L01+ registers
CONFIG = const(0x00)
AUTO_ACK = const(0x01)
EN_RXADDR = const(0x02)
SETUP_AW = const(0x03)
SETUP_RETR = const(0x04)
RF_CH = const(0x05)
RF_SETUP = const(0x06)
STATUS = const(0x07)
RX_ADDR_P0 = const(0x0A)
TX_ADDR = const(0x10)
RX_PW_P0 = const(0x11)
FIFO_STATUS = const(0x17)
DYNPD = const(0x1C)
TX_FEATURE = const(0x1D)

# CONFIG register
EN_CRC = const(0x08)  # enable CRC
CRCO = const(0x04)  # CRC encoding scheme; 0=1 byte, 1=2 bytes
PWR_UP = const(0x02)  # 1=power up, 0=power down
PRIM_RX = const(0x01)  # RX/TX control; 0=PTX, 1=PRX

# RF_SETUP register
POWER_0 = const(0x00)  # -18 dBm
POWER_1 = const(0x02)  # -12 dBm
POWER_2 = const(0x04)  # -6 dBm
POWER_3 = const(0x06)  # 0 dBm
SPEED_1M = const(0x00)
SPEED_2M = const(0x08)
SPEED_250K = const(0x20)

# STATUS register
RX_DR = const(0x40)  # RX data ready; write 1 to clear
TX_DS = const(0x20)  # TX data sent; write 1 to clear
MAX_RT = const(0x10)  # max retransmits reached; write 1 to clear

# FIFO_STATUS register
RX_EMPTY = const(0x01)  # 1 if RX FIFO is empty

# constants for instructions
R_RX_PL_WID = const(0x60)  # read RX payload width
R_RX_PAYLOAD = const(0x61)  # read RX payload
W_TX_PAYLOAD = const(0xA0)  # write TX payload
FLUSH_TX = const(0xE1)  # flush TX FIFO
FLUSH_RX = const(0xE2)  # flush RX FIFO
NOP = const(0xFF)  # use to read STATUS register


def address_repr(buf, reverse: bool = True, delimit: str = "") -> str:
    """Convert a buffer into a hexlified string."""
    order = range(len(buf) - 1, -1, -1) if reverse else range(len(buf))
    return delimit.join([f"{buf[byte]:02X}" for byte in order])


class NRF24L01:
    def __init__(self, spi, cs, ce, channel=46, payload_size=16, baudrate=4000000):
        assert payload_size <= 32

        self.buf = bytearray(1)
        self.send_done_status = 0

        # store the pins
        self.spi = spi
        self.cs = cs
        self.ce = ce

        # init the SPI bus and pins
        self.init_spi(baudrate)

        # reset everything
        ce.init(ce.OUT, value=0)
        cs.init(cs.OUT, value=1)

        self.payload_size = payload_size
        self.pipe0_read_addr = None
        utime.sleep_ms(5)

        self._pl_len = [32] * 6  # 32-byte static payloads for all pipes
        self._pipes = [bytearray(5)] * 2 + [0] * 4

        # set address width to 5 bytes and check for device present
        self.reg_write(SETUP_AW, 0b11)
        if self.reg_read(SETUP_AW) != 0b11:
            raise OSError("nRF24L01+ Hardware not responding")

        # capture RX addresses from registers
        for i in range(6):
            if i < 2:
                self._pipes[i] = self.reg_read_bytes(RX_ADDR_P0 + i)
            else:
                self._pipes[i] = self.reg_read(RX_ADDR_P0 + i)

        # test is nRF24L01 is a plus variant using a command specific to
        # non-plus variants
        self._open_pipes, self._is_plus_variant = (0, False)  # close all RX pipes
        self._features = self.reg_read(TX_FEATURE)
        self.reg_write(0x50, 0x73)  # derelict command toggles TX_FEATURE register
        after_toggle = self.reg_read(TX_FEATURE)
        if self._features == after_toggle:
            self._is_plus_variant = True
        elif not after_toggle:  # if features are disabled
            self.reg_write(0x50, 0x73)  # ensure they're enabled

        # disable dynamic payloads
        self.reg_write(DYNPD, 0)

        # auto retransmit delay: 1750us
        # auto retransmit count: 8
        self.reg_write(SETUP_RETR, (6 << 4) | 8)

        # set rf power and speed
        self.set_power_speed(POWER_3, SPEED_250K)  # Best for point to point links

        # init CRC
        self.set_crc(2)

        # clear status flags
        self.reg_write(STATUS, RX_DR | TX_DS | MAX_RT)

        # set channel
        self.set_channel(channel)

        # flush buffers
        self.flush_rx()
        self.flush_tx()

    def print_details(self, dump_pipes: bool = False) -> None:
        """This debugging function outputs all details about the nRF24L01."""
        observer = self.reg_read(8)
        _fifo = self.reg_read(FIFO_STATUS)
        self._config = self.reg_read(CONFIG)
        self._rf_setup = self.reg_read(RF_SETUP)
        self._retry_setup = self.reg_read(SETUP_RETR)
        self._channel = self.reg_read(RF_CH)
        self._addr_len = self.reg_read(0x03) + 2
        self._features = self.reg_read(TX_FEATURE)
        self._aa = self.reg_read(AUTO_ACK)
        self._dyn_pl = self.reg_read(DYNPD)

        for i in range(6):
            self._pl_len[i] = min(32, self.reg_read(RX_PW_P0 + i))

        _crc = (
            (2 if self._config & 4 else 1)
            if self._aa
            else max(0, ((self._config & 0x0C) >> 2) - 1)
        )
        d_rate = self._rf_setup & 0x28
        d_rate = (2 if d_rate == 8 else 250) if d_rate else 1
        _pa_level = (3 - ((self._rf_setup & 6) >> 1)) * -6
        dyn_p = (
            ("_Enabled" if self._dyn_pl else "Disabled")
            if self._dyn_pl == 0x3F or not self._dyn_pl
            else "0b" + "0" * (8 - len(bin(self._dyn_pl))) + bin(self._dyn_pl)[2:]
        )
        auto_a = (
            ("Enabled" if self._aa else "Disabled")
            if self._aa == 0x3F or not self._aa
            else "0b" + "0" * (8 - len(bin(self._aa))) + bin(self._aa)[2:]
        )
        pwr = (
            ("Standby-II" if self.ce else "Standby-I")
            if self._config & 2
            else "Off"
        )

        print(f"Is a plus variant_________{self.is_plus_variant}")
        print(
            f"Channel___________________{self._channel}",
            f"~ {(self._channel + 2400) / 1000} GHz",
        )
        print(
            f"RF Data Rate______________{d_rate}",
            "Mbps" if d_rate != 250 else "Kbps",
        )
        print(f"RF Power Amplifier________{_pa_level} dbm")
        print(f"RF Low Noise Amplifier____{"En" if bool(self._rf_setup & 1) else "Dis"}abled")
        print(f"CRC bytes_________________{_crc}")
        print(f"Address length____________{self._addr_len} bytes")
        print(f"TX Payload lengths________{self._pl_len[0]} bytes")
        print(f"Auto retry delay__________{((self._retry_setup & 0xF0) >> 4) * 250 + 250} microseconds")
        print(f"Auto retry attempts_______{self._retry_setup & 0x0F} maximum")
        print(f"Re-use TX FIFO____________{bool(_fifo & 64)}")
        print(f"Packets lost on current channel_____________________{observer >> 4}")
        print(f"Retry attempts made for last transmission___________{observer & 0xF}")
        """
        print(
            "IRQ on Data Ready__{}abled".format("Dis" if self._config & 64 else "_En"),
            "   Data Ready___________{}".format(self.irq_dr),
        )
        print(
            "IRQ on Data Fail___{}abled".format("Dis" if self._config & 16 else "_En"),
            "   Data Failed__________{}".format(self.irq_df),
        )
        print(
            "IRQ on Data Sent___{}abled".format("Dis" if self._config & 32 else "_En"),
            "   Data Sent____________{}".format(self.irq_ds),
        )
        """
        print(
            f"TX FIFO full__________{"_Tru" if _fifo & 0x20 else "Fals"}e",
            f"   TX FIFO empty________{bool(_fifo & 0x10)}",
        )
        print(
            f"RX FIFO full__________{"_Tru" if _fifo & 2 else "Fals"}e",
            f"   RX FIFO empty________{bool(_fifo & 1)}",
        )
        print(f"Ask no ACK_________{"_Allow" if self._features & 1 else "Disabl"}ed    Custom ACK Payload___{"En" if self._features & 2 else "Dis"}abled")
        print(f"Dynamic Payloads___{dyn_p}    Auto Acknowledgment__{auto_a}")
        print(
            f"Primary Mode_____________{"R" if self._config & 1 else "T"}X",
            f"   Power Mode___________{pwr}",
        )
        if dump_pipes:
            self.print_pipes()

    def print_pipes(self) -> None:
        """Prints all information specific to pipe's addresses, RX state, & expected
        static payload sizes (if configured to use static payloads)."""
        self._open_pipes = self.reg_read(EN_RXADDR)
        self._tx_address = self.reg_read_bytes(TX_ADDR)
        for i in range(6):
            if i < 2:
                self._pipes[i] = self.reg_read_bytes(RX_ADDR_P0 + i)
            else:
                self._pipes[i] = self.reg_read(RX_ADDR_P0 + i)
            self._pl_len[i] = self.reg_read(RX_PW_P0 + i)
        print(f"TX address____________ 0x{address_repr(self.address())}")
        for i in range(6):
            is_open = self._open_pipes & (1 << i)
            print(f"Pipe {i} ({" open " if is_open else "closed"}) bound: 0x{address_repr(self.address(i))}")
            if is_open and not self._dyn_pl & (1 << i):
                print(f"\t\texpecting {self._pl_len[i]} byte static payloads")

    @property
    def is_plus_variant(self) -> bool:
        """A `bool` describing if the nRF24L01 is a plus variant or not (read-only)."""
        return self._is_plus_variant

    def address(self, index: int = -1):
        """Returns the current TX address or optionally RX address. (read-only)"""
        if index > 5:
            raise IndexError(f"index {index} is out of bounds [0,5]")
        if index < 0:
            return self._tx_address
        if index <= 1:
            return self._pipes[index]
        return bytes([self._pipes[index]]) + self._pipes[1][1:]

    def init_spi(self, baudrate):
        try:
            master = self.spi.MASTER
        except AttributeError:
            self.spi.init(baudrate=baudrate, polarity=0, phase=0)
        else:
            self.spi.init(master, baudrate=baudrate, polarity=0, phase=0)

    def reg_read(self, reg):
        self.cs(0)
        self.spi.readinto(self.buf, reg)
        self.spi.readinto(self.buf)
        self.cs(1)
        return self.buf[0]

    def reg_read_bytes(self, reg, size=5):
        buf = bytearray(size)
        self.cs(0)
        self.spi.readinto(self.buf, reg)
        self.spi.readinto(buf)
        self.cs(1)
        return buf

    def reg_write_bytes(self, reg, buf):
        self.cs(0)
        self.spi.readinto(self.buf, 0x20 | reg)
        self.spi.write(buf)
        self.cs(1)
        return self.buf[0]

    def reg_write(self, reg, value):
        self.cs(0)
        self.spi.readinto(self.buf, 0x20 | reg)
        ret = self.buf[0]
        self.spi.readinto(self.buf, value)
        self.cs(1)
        return ret

    def flush_rx(self):
        self.cs(0)
        self.spi.readinto(self.buf, FLUSH_RX)
        self.cs(1)

    def flush_tx(self):
        self.cs(0)
        self.spi.readinto(self.buf, FLUSH_TX)
        self.cs(1)

    # power is one of POWER_x defines; speed is one of SPEED_x defines
    def set_power_speed(self, power, speed):
        setup = self.reg_read(RF_SETUP) & 0b11010001
        self.reg_write(RF_SETUP, setup | power | speed)

    # length in bytes: 0, 1 or 2
    def set_crc(self, length):
        config = self.reg_read(CONFIG) & ~(CRCO | EN_CRC)
        if length == 0:
            pass
        elif length == 1:
            config |= EN_CRC
        else:
            config |= EN_CRC | CRCO
        self.reg_write(CONFIG, config)

    def set_channel(self, channel):
        self.reg_write(RF_CH, min(channel, 125))

    # address should be a bytes object 5 bytes long
    def open_tx_pipe(self, address):
        assert len(address) == 5
        self.reg_write_bytes(RX_ADDR_P0, address)
        self.reg_write_bytes(TX_ADDR, address)
        self.reg_write(RX_PW_P0, self.payload_size)

    # address should be a bytes object 5 bytes long
    # pipe 0 and 1 have 5 byte address
    # pipes 2-5 use same 4 most-significant bytes as pipe 1, plus 1 extra byte
    def open_rx_pipe(self, pipe_id, address):
        assert len(address) == 5
        assert 0 <= pipe_id <= 5
        if pipe_id == 0:
            self.pipe0_read_addr = address
        if pipe_id < 2:
            self.reg_write_bytes(RX_ADDR_P0 + pipe_id, address)
        else:
            self.reg_write(RX_ADDR_P0 + pipe_id, address[0])
        self.reg_write(RX_PW_P0 + pipe_id, self.payload_size)
        self.reg_write(EN_RXADDR, self.reg_read(EN_RXADDR) | (1 << pipe_id))

    def start_listening(self):
        self.reg_write(CONFIG, self.reg_read(CONFIG) | PWR_UP | PRIM_RX)
        self.reg_write(STATUS, RX_DR | TX_DS | MAX_RT)

        if self.pipe0_read_addr is not None:
            self.reg_write_bytes(RX_ADDR_P0, self.pipe0_read_addr)

        self.flush_rx()
        self.flush_tx()
        self.ce(1)
        utime.sleep_us(130)

    def stop_listening(self):
        self.ce(0)
        self.flush_tx()
        self.flush_rx()

    # returns True if any data available to recv
    def any(self):
        return not bool(self.reg_read(FIFO_STATUS) & RX_EMPTY)

    def recv(self):
        # get the data
        self.cs(0)
        self.spi.readinto(self.buf, R_RX_PAYLOAD)
        buf = self.spi.read(self.payload_size)
        self.cs(1)
        # clear RX ready flag
        self.reg_write(STATUS, RX_DR)

        return buf

    # blocking wait for tx complete
    def send(self, buf, timeout=500):
        self.send_start(buf)
        start = utime.ticks_ms()
        result = None
        while result is None and utime.ticks_diff(utime.ticks_ms(), start) < timeout:
            result = self.send_done()  # 1 == success, 2 == fail
        if result == 2:
            raise OSError("send failed")

    # non-blocking tx
    def send_start(self, buf):
        # power up
        self.reg_write(CONFIG, (self.reg_read(CONFIG) | PWR_UP) & ~PRIM_RX)
        utime.sleep_us(150)
        # send the data
        self.cs(0)
        self.spi.readinto(self.buf, W_TX_PAYLOAD)
        self.spi.write(buf)
        if len(buf) < self.payload_size:
            self.spi.write(b"\x00" * (self.payload_size - len(buf)))  # pad out data
        self.cs(1)

        # enable the chip so it can send the data
        self.ce(1)
        utime.sleep_us(15)  # needs to be >10us
        self.ce(0)

        # Try a long delay here to allow responder ack to be seen w/o csn interference
        utime.sleep_us(2000)

    # returns None if send still in progress, 1 for success, 2 for fail
    def send_done(self):
        if not self.reg_read(STATUS) & (TX_DS | MAX_RT):
            return None  # tx not finished

        # either finished or failed: get and clear status flags, power down
        self.send_done_status = self.reg_write(STATUS, RX_DR | TX_DS | MAX_RT)
        self.reg_write(CONFIG, self.reg_read(CONFIG) & ~PWR_UP)
        return 1 if self.send_done_status & TX_DS else 2
