# multimedia_python/external/si470x.py

import time
import os

import RPi.GPIO as GPIO
import smbus


class Si4703Radio:
    """Driver for the Si4703 FM receiver with basic tuning, volume and RDS."""

    # Register indices
    DEVICEID = 0x00
    CHIPID = 0x01
    POWERCFG = 0x02
    CHANNEL = 0x03
    SYSCONFIG1 = 0x04
    SYSCONFIG2 = 0x05
    SYSCONFIG3 = 0x06
    OSCILLATOR = 0x07
    STATUSRSSI = 0x0A
    READCHAN = 0x0B
    RDSA = 0x0C
    RDSB = 0x0D
    RDSC = 0x0E
    RDSD = 0x0F

    def __init__(self, i2c_addr=0x10, reset_pin=23, bus_id=1, enable_pin=0):
        """
        :param i2c_addr: I2C address of the Si4703 (default 0x10)
        :param reset_pin: GPIO pin used as RESET for the Si4703
        :param bus_id: I2C bus number (1 on Raspberry Pi)
        :param enable_pin: GPIO pin used for SEN/enable (like original code used GPIO 0)
        """
        self.i2c_addr = i2c_addr
        self.reset_pin = reset_pin
        self.enable_pin = enable_pin

        # I2C and GPIO setup
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.reset_pin, GPIO.OUT)
        GPIO.setup(self.enable_pin, GPIO.OUT)

        self.i2c = smbus.SMBus(bus_id)

        # Internal register image
        self.reg = [0] * 16
        self._read_buf = [0] * 32
        self._write_buf = [0] * 11

        # Last RDS message
        self._rds_text = ""

        # For bit string padding in RDS
        self._z = "0" * 15

        # Initialize the chip
        self._init_radio()

    # -------------------- Low level I2C helpers --------------------

    def _write_registers(self):
        # starts writing at register 2, first byte in I2C command
        cmd, self._write_buf[0] = divmod(self.reg[self.POWERCFG], 1 << 8)
        self._write_buf[1], self._write_buf[2] = divmod(self.reg[self.CHANNEL], 1 << 8)
        self._write_buf[3], self._write_buf[4] = divmod(self.reg[self.SYSCONFIG1], 1 << 8)
        self._write_buf[5], self._write_buf[6] = divmod(self.reg[self.SYSCONFIG2], 1 << 8)
        self._write_buf[7], self._write_buf[8] = divmod(self.reg[self.SYSCONFIG3], 1 << 8)
        self._write_buf[9], self._write_buf[10] = divmod(self.reg[self.OSCILLATOR], 1 << 8)

        self.i2c.write_i2c_block_data(self.i2c_addr, cmd, self._write_buf)
        self._read_buf[16] = cmd
        self._read_registers()

    def _read_registers(self):
        self._read_buf = self.i2c.read_i2c_block_data(self.i2c_addr, self._read_buf[16], 32)

        self.reg[10] = self._read_buf[0] * 256 + self._read_buf[1]
        self.reg[11] = self._read_buf[2] * 256 + self._read_buf[3]
        self.reg[12] = self._read_buf[4] * 256 + self._read_buf[5]
        self.reg[13] = self._read_buf[6] * 256 + self._read_buf[7]
        self.reg[14] = self._read_buf[8] * 256 + self._read_buf[9]
        self.reg[15] = self._read_buf[10] * 256 + self._read_buf[11]
        self.reg[0] = self._read_buf[12] * 256 + self._read_buf[13]
        self.reg[1] = self._read_buf[14] * 256 + self._read_buf[15]
        self.reg[2] = self._read_buf[16] * 256 + self._read_buf[17]
        self.reg[3] = self._read_buf[18] * 256 + self._read_buf[19]
        self.reg[4] = self._read_buf[20] * 256 + self._read_buf[21]
        self.reg[5] = self._read_buf[22] * 256 + self._read_buf[23]
        self.reg[6] = self._read_buf[24] * 256 + self._read_buf[25]
        self.reg[7] = self._read_buf[26] * 256 + self._read_buf[27]
        self.reg[8] = self._read_buf[28] * 256 + self._read_buf[29]
        self.reg[9] = self._read_buf[30] * 256 + self._read_buf[31]

    # -------------------- Init / basic control --------------------

    def _init_radio(self):
        """
        Put the chip into 2-wire (I2C) mode and configure basic settings.
        Mirrors the original init() logic.
        """
        # Enter 2-wire mode: enable_pin low, then toggle reset
        GPIO.output(self.enable_pin, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(self.reset_pin, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(self.reset_pin, GPIO.HIGH)
        time.sleep(0.1)

        self._read_registers()
        # Turn on oscillator
        self.reg[self.OSCILLATOR] = 0x8100
        self._write_registers()
        time.sleep(1.0)

        self._read_registers()
        # Enable radio, unmute
        self.reg[self.POWERCFG] = 0x4001
        self._write_registers()
        time.sleep(0.1)

        self._read_registers()
        # Enable RDS, min volume, extended range
        self.reg[self.SYSCONFIG1] |= (1 << 12)       # RDS
        self.reg[self.SYSCONFIG2] &= 0xFFF0          # clear volume bits
        self.reg[self.SYSCONFIG2] = 0x0000           # min volume
        self.reg[self.SYSCONFIG3] = 0x0100           # extended volume range
        self._write_registers()

    # -------------------- Public API used by your node --------------------

    def si4703SetVolume(self, new_volume: int):
        """Set volume 0–15."""
        if new_volume > 15:
            new_volume = 15
        if new_volume < 0:
            new_volume = 0

        self._read_registers()
        self.reg[self.SYSCONFIG2] &= 0xFFF0  # Clear volume bits
        self.reg[self.SYSCONFIG2] |= new_volume
        self._write_registers()

    def si4703GetChannel(self) -> int:
        """
        Get current tuned channel in '0.1 MHz steps', e.g. 1045 = 104.5 MHz.
        Matches your use of Int32 frequency.
        """
        self._read_registers()
        channel = self.reg[self.READCHAN] & 0x03FF
        channel *= 2
        channel += 875
        return channel

    def si4703SetChannel(self, freq_01mhz: int):
        """
        Set tuned channel.
        freq_01mhz = 1045 => 104.5 MHz.
        Valid range: 87.8–108.0 MHz -> 878..1080 (same as original).
        """
        if freq_01mhz < 878 or freq_01mhz > 1080:
            return

        self._read_registers()

        # Convert from 0.1 MHz units to channel bits (integer math)
        new_channel = freq_01mhz * 10          # e.g. 10450
        new_channel -= 8750
        new_channel //= 20                     # original formula

        # Clear channel bits and set new channel
        self.reg[self.CHANNEL] &= 0xFE00
        self.reg[self.CHANNEL] |= new_channel
        self.reg[self.CHANNEL] |= (1 << 15)    # TUNE bit
        self._write_registers()

        # Wait for tune complete (STC)
        while True:
            self._read_registers()
            if (self.reg[self.STATUSRSSI] & (1 << 14)) != 0:
                break

        # Clear TUNE bit
        self.reg[self.CHANNEL] &= ~(1 << 15)
        self._write_registers()

    def _seek(self, direction_up: bool):
        """
        Internal seek helper. direction_up=True => Seek up, False => down.
        """
        self._read_registers()

        # Set SEEK bit
        self.reg[self.POWERCFG] |= (1 << 10)

        if direction_up:
            self.reg[self.POWERCFG] |= (1 << 9)   # SEEKUP
        else:
            self.reg[self.POWERCFG] &= ~(1 << 9)  # clear SEEKUP

        self.reg[self.POWERCFG] |= (1 << 8)       # START SEEK
        self._write_registers()

        # Wait for STC
        while True:
            self._read_registers()
            if (self.reg[self.STATUSRSSI] & (1 << 14)) != 0:
                break

        # Clear SEEK bit
        self.reg[self.POWERCFG] &= ~(1 << 8)
        self._write_registers()

    def si4703SeekUp(self):
        self._seek(True)

    def si4703SeekDown(self):
        self._seek(False)

    # -------------------- RDS handling (simplified from logger) --------------------

    def si4703ClearRDSBuffers(self):
        self._rds_text = ""

    def si4703ProcessRDS(self, max_groups: int = 500):
        """
        Poll RDS data for up to max_groups changes and build a message
        (roughly equivalent to the "msg" creation in the original logger).
        """
        msg = ""
        mi = 0
        h2 = ""
        h3 = ""
        h4 = ""
        wc = 0

        while wc < max_groups:
            self._read_registers()

            # RDSR bit (RDS ready)
            if self.reg[self.STATUSRSSI] & (1 << 15):
                z = self._z
                r2 = z[:16 - len(bin(self.reg[self.RDSB])[2:])] + bin(self.reg[self.RDSB])[2:]
                r3 = z[:16 - len(bin(self.reg[self.RDSC])[2:])] + bin(self.reg[self.RDSC])[2:]
                r4 = z[:16 - len(bin(self.reg[self.RDSD])[2:])] + bin(self.reg[self.RDSD])[2:]

                if h2 != r2 or h3 != r3 or h4 != r4:
                    wc += 1
                    h2 = r2
                    h3 = r3
                    h4 = r4

                    value = int(r2[:4], 2)
                    value2 = int(r2[5:-5], 2)
                    mtype = "A" if value2 == 0 else "B"
                    code = f"{value}{mtype}"

                    # The original script used "2B" (Radiotext B)
                    if code == "2B":
                        chars = (
                            chr(int(r3[:8], 2)) +
                            chr(int(r3[9:], 2)) +
                            chr(int(r4[:8], 2)) +
                            chr(int(r4[9:], 2))
                        )
                        index = int(r2[12:], 2)

                        # New cycle => full message collected
                        if index == 0 and mi != 0:
                            # Clean message
                            msg = msg.translate({ord(','): None})
                            msg = msg.translate({c: None for c in range(ord('\x00'), ord('\x1f'))})
                            self._rds_text = msg
                            return

                        if index == mi:
                            msg += chars
                            mi += 1

        # If we get here, we didn't finish a full message
        if msg:
            msg = msg.translate({ord(','): None})
            msg = msg.translate({c: None for c in range(ord('\x00'), ord('\x1f'))})
            self._rds_text = msg

    def si4703GetProgramService(self) -> str:
        """Return last collected RDS text (Radiotext B in this implementation)."""
        return self._rds_text or ""
