"""Test for nrf24l01 module.  Portable between MicroPython targets."""

import sys
import struct
import time
import random
from machine import Pin, SPI, SoftSPI
from micropython import const

# Responder pause between receiving data and checking for further packets.
_RX_POLL_DELAY = const(15)
# Responder pauses an additional _RESPONER_SEND_DELAY ms after receiving data and before
# transmitting to allow the (remote) initiator time to get into receive mode. The
# initiator may be a slow device. Value tested with Pyboard, ESP32 and ESP8266.
_RESPONDER_SEND_DELAY = const(10)

# Addresses are in little-endian format. They correspond to big-endian
# 0xf0f0f0f0e1, 0xf0f0f0f0d2
pipes = (b"\xe1\xf0\xf0\xf0\xf0", b"\xd2\xf0\xf0\xf0\xf0")

class NrfTest():
    def __init__(self, driver_name, ce_gpio, channel, power, speed):
        self.driver_name = driver_name
        self.ce_gpio = ce_gpio
        self.channel = channel
        self.power_string = power
        self.speed_string = speed
        self.driver = None
        self.power = None
        self.speed = None
        self.spi = None
        self.cfg = None
        self.nrf = None
        self.all_systems_go = True
        self.import_driver()
        self.init_spi()
        self.init_nrf()
        self.report()

    def import_driver(self):
        if not self.all_systems_go:
            return

        try:
            self.driver = getattr(__import__(f"drivers.{self.driver_name}"), self.driver_name)
            self.NRF24L01 = getattr(self.driver, "NRF24L01")
        except AttributeError:
            print(f"Unable to import driver {self.driver_name}")

        if self.driver is not None:
            try:
                self.power = getattr(self.driver, self.power_string)
            except AttributeError:
                print(f"Unable to get attribute {self.power_string}")
           
            try:
                self.speed = getattr(self.driver, self.speed_string)
            except AttributeError:
                print(f"Unable to get attribute {self.speed_string}")

        self.all_systems_go = not any(v is None for v in [self.driver, self.NRF24L01, self.power, self.speed])

    def init_spi(self):
        if not self.all_systems_go:
            return

        if sys.platform == "pyboard":
            spi = SPI(2)  # miso : Y7, mosi : Y8, sck : Y6
            cfg = {"spi": spi, "csn": "Y5", "ce": "Y4"}
        elif sys.platform == "esp8266":  # Hardware SPI
            spi = SPI(1)  # miso : 12, mosi : 13, sck : 14
            cfg = {"spi": spi, "csn": 4, "ce": 5}
        elif sys.platform == "esp32":  # Software SPI
            spi = SoftSPI(sck=Pin(25), mosi=Pin(33), miso=Pin(32))
            cfg = {"spi": spi, "csn": 26, "ce": 27}
        elif sys.platform == "rp2":  # Hardware SPI with explicit pin definitions
            spi = SPI(0, sck=Pin(18), mosi=Pin(19), miso=Pin(16))
            cfg = {"spi": spi, "csn": 17, "ce": 14, "led": 25}
        else:
            raise ValueError(f"Unsupported platform {sys.platform}")

        self.leds = []
        if "led" in cfg:
            self.leds.append(Pin(cfg["led"], Pin.OUT))

        cfg["ce"] = self.ce_gpio
        self.spi = spi
        self.cfg = cfg

        self.all_systems_go = not any(v is None for v in [self.spi, self.cfg])

    def init_nrf(self):
        if not self.all_systems_go:
            return

        csn = Pin(self.cfg["csn"], mode=Pin.OUT, value=1)
        ce = Pin(self.cfg["ce"], mode=Pin.OUT, value=0)
        spi = self.cfg["spi"]
        self.nrf = self.NRF24L01(spi, csn, ce, payload_size=8, channel=self.channel)
        self.gpio10 = Pin(10, mode=Pin.OUT, value=0)
        if self.power is not None and self.speed is not None:
            self.nrf.set_power_speed(self.power, self.speed)

        self.all_systems_go = not any(v is None for v in [self.nrf])

    def initiator(self, num_needed=1):
        if not self.all_systems_go:
            print("ERROR: all systems are NOT go")
            return

        self.nrf.open_tx_pipe(pipes[0])
        self.nrf.open_rx_pipe(1, pipes[1])
        self.nrf.start_listening()

        num_successes = 0
        num_failures = 0
        led_state = 0

        print(f"NRF24L01 initiator mode, sending {num_needed} packets...")

        while num_successes < num_needed and num_failures < num_needed:
            # stop listening and send packet
            self.nrf.stop_listening()
            millis = time.ticks_ms()
            led_state = random.randint(0, 15)
            print("sending:", millis, led_state)
            try:
                self.nrf.send(struct.pack("ii", millis, led_state))
            except OSError:
                pass

            # start listening again
            self.nrf.start_listening()

            # wait for response, with 250ms timeout
            start_time = time.ticks_ms()
            timeout = False
            while not self.nrf.any() and not timeout:
                if time.ticks_diff(time.ticks_ms(), start_time) > 250:
                    timeout = True

            if timeout:
                print("failed, response timed out")
                num_failures += 1

            else:
                # recv packet
                (got_millis,) = struct.unpack("i", self.nrf.recv())

                # print response and round-trip delay
                print(
                    "got response:",
                    got_millis,
                    "(delay",
                    time.ticks_diff(time.ticks_ms(), got_millis),
                    "ms)",
                )
                num_successes += 1

            # delay then loop
            time.sleep_ms(250)

        print("initiator finished sending; successes=%d, failures=%d" % (num_successes, num_failures))

    def responder(self):
        if not self.all_systems_go:
            print("ERROR: all systems are NOT go")
            return

        self.nrf.open_tx_pipe(pipes[1])
        self.nrf.open_rx_pipe(1, pipes[0])
        self.nrf.start_listening()

        print("NRF24L01 responder mode, waiting for packets... (ctrl-C to stop)")

        while True:
            if self.nrf.any():
                self.gpio10.value(1)
                while self.nrf.any():
                    buf = self.nrf.recv()
                    millis, led_state = struct.unpack("ii", buf)
                    print("received:", millis, led_state)
                    for led in self.leds:
                        if led_state & 1:
                            led.on()
                        else:
                            led.off()
                        led_state >>= 1
                    time.sleep_ms(_RX_POLL_DELAY)

                # Give initiator time to get into receive mode.
                time.sleep_ms(_RESPONDER_SEND_DELAY)
                self.nrf.stop_listening()
                try:
                    self.nrf.send(struct.pack("i", millis))
                except OSError:
                    pass
                self.gpio10.value(0)
                print("sent response")
                self.nrf.start_listening()

    def report(self):
        if not self.all_systems_go:
            print("ERROR: all systems are NOT go")
            return

        print("NRF24L01 test module loaded")
        print("NRF24L01 pinout for test:")
        print("    CE on", self.cfg["ce"])
        print("    CSN on", self.cfg["csn"])
        print("    SPI on", self.cfg["spi"])
        print("run <object>.responder() on responder, then <object>.initiator(num) on initiator")
