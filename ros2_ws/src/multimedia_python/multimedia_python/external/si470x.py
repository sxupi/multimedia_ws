#!/usr/bin/env python3

# RDS FM Radio logger based on the si470x break-out board from spark-fun connected to a raspberry pi
# The si470x IC is continually polled and the RDS Message is logged to file
# File format is <UNIX TIMESTAMP>,<STATION IN MHZ>,<RDS MESSAGE>
# Cobbled together by Grant Trebbin www.grant-trebbin.com
# The hard work was done by the people below

# Original code by KansasCode on the Raspberry Pi Forums
# www.raspberrypi.org/forums/viewtopic.php?t=28920
# Logging code from Stephen Phillips
# blog.scphillips.com/2013/07/getting-a-python-script-to-run-in-the-background-as-a-service-on-boot/

# Ported to Python 3

import RPi.GPIO as GPIO
import smbus
import time
import logging
import logging.handlers
import os

# configure logger
LOG_FILENAME = "/var/log/si470x/si470x.log"
LOG_LEVEL = logging.INFO
logger = logging.getLogger(__name__)
logger.setLevel(LOG_LEVEL)

# Make sure the directory exists (optional but helpful)
os.makedirs(os.path.dirname(LOG_FILENAME), exist_ok=True)

handler = logging.handlers.TimedRotatingFileHandler(
    LOG_FILENAME,
    when="h",
    backupCount=6720
)
logger.addHandler(handler)

GPIO.setwarnings(False)

i2c = smbus.SMBus(1)
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(0, GPIO.OUT)

# create 16 registers for SI4703
reg = [0] * 16

# Register Descriptions
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

z = "000000000000000"

# found using i2cdetect utility
si4703_addr = 0x10

# create list to write registers
# only need to write registers 2-7 and since first byte is in the write
# command then only need 11 bytes to write
writereg = [0] * 11

# read 32 bytes
readreg = [0] * 32


def write_registers():
    # starts writing at register 2
    # but first byte is in the i2c write command
    global writereg
    global reg
    global readreg

    cmd, writereg[0] = divmod(reg[2], 1 << 8)
    writereg[1], writereg[2] = divmod(reg[3], 1 << 8)
    writereg[3], writereg[4] = divmod(reg[4], 1 << 8)
    writereg[5], writereg[6] = divmod(reg[5], 1 << 8)
    writereg[7], writereg[8] = divmod(reg[6], 1 << 8)
    writereg[9], writereg[10] = divmod(reg[7], 1 << 8)

    i2c.write_i2c_block_data(si4703_addr, cmd, writereg)
    readreg[16] = cmd  # readreg
    read_registers()


def read_registers():
    global readreg
    global reg
    readreg = i2c.read_i2c_block_data(si4703_addr, readreg[16], 32)

    reg[10] = readreg[0] * 256 + readreg[1]
    reg[11] = readreg[2] * 256 + readreg[3]
    reg[12] = readreg[4] * 256 + readreg[5]
    reg[13] = readreg[6] * 256 + readreg[7]
    reg[14] = readreg[8] * 256 + readreg[9]
    reg[15] = readreg[10] * 256 + readreg[11]
    reg[0] = readreg[12] * 256 + readreg[13]
    reg[1] = readreg[14] * 256 + readreg[15]
    reg[2] = readreg[16] * 256 + readreg[17]
    reg[3] = readreg[18] * 256 + readreg[19]
    reg[4] = readreg[20] * 256 + readreg[21]
    reg[5] = readreg[22] * 256 + readreg[23]
    reg[6] = readreg[24] * 256 + readreg[25]
    reg[7] = readreg[26] * 256 + readreg[27]
    reg[8] = readreg[28] * 256 + readreg[29]
    reg[9] = readreg[30] * 256 + readreg[31]


def getchannel():
    read_registers()
    channel = reg[READCHAN] & 0x03FF
    channel *= 2
    channel += 875
    return channel


def changechannel(newchannel):
    if newchannel < 878 or newchannel > 1080:
        return
    global reg
    # Keep everything integer (Python 3: use // instead of /)
    newchannel *= 10
    newchannel -= 8750
    newchannel //= 20  # integer division

    read_registers()
    reg[CHANNEL] &= 0xFE00  # Clear out the channel bits
    reg[CHANNEL] |= newchannel  # Mask in the new channel
    reg[CHANNEL] |= (1 << 15)  # Set the TUNE bit to start
    write_registers()
    while True:
        read_registers()
        if (reg[STATUSRSSI] & (1 << 14)) != 0:
            break
    reg[CHANNEL] &= ~(1 << 15)
    write_registers()


def setvolume(newvolume):
    global reg
    if newvolume > 15:
        newvolume = 15
    if newvolume < 0:
        newvolume = 0
    read_registers()
    reg[SYSCONFIG2] &= 0xFFF0  # Clear volume bits
    reg[SYSCONFIG2] = newvolume  # Set volume to lowest
    write_registers()


def init():
    # init code needs to activate 2-wire (i2c) mode
    # the si4703 will not show up in i2cdetect until
    # you do these steps to put it into 2-wire (i2c) mode

    GPIO.output(0, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(23, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(23, GPIO.HIGH)
    time.sleep(0.1)

    read_registers()
    # do init step, turn on oscillator
    reg[OSCILLATOR] = 0x8100
    write_registers()
    time.sleep(1)

    read_registers()
    reg[POWERCFG] = 0x4001  # Enable the Radio IC and turn off muted
    write_registers()
    time.sleep(0.1)

    read_registers()
    reg[SYSCONFIG1] |= (1 << 12)  # Enable RDS
    reg[SYSCONFIG2] &= 0xFFF0  # Clear volume bits
    reg[SYSCONFIG2] = 0x0000  # Set volume to lowest
    # Set extended volume range (too loud for me without this)
    reg[SYSCONFIG3] = 0x0100
    write_registers()


def seek(direction):
    read_registers()
    reg[POWERCFG] |= (1 << 10)
    if direction == 0:
        reg[POWERCFG] &= ~(1 << 1)
    else:
        reg[POWERCFG] |= (1 << 9)
    reg[POWERCFG] |= (1 << 8)
    write_registers()
    while True:
        read_registers()
        if (reg[STATUSRSSI] & (1 << 14)) != 0:
            break
    print("Trying Station", float(getchannel()) / 10.0)
    read_registers()
    valuesfbl = reg[STATUSRSSI] & (1 << 13)  # noqa: F841 (unused)
    reg[POWERCFG] &= ~(1 << 8)
    write_registers()


currvol = 15
init()  # init stuff

changechannel(1045)  # 104.5 MHz
setvolume(currvol)

while True:
    logdata = " "
    logdata = str(float(getchannel()) / 10.0)
    msg = u''
    mi = 0
    h2 = ""
    h3 = ""
    h4 = ""
    wc = 0

    while True:
        read_registers()
        if reg[STATUSRSSI] & (1 << 15):
            r2 = z[:16 - len(bin(reg[RDSB])[2:])] + bin(reg[RDSB])[2:]
            r3 = z[:16 - len(bin(reg[RDSC])[2:])] + bin(reg[RDSC])[2:]
            r4 = z[:16 - len(bin(reg[RDSD])[2:])] + bin(reg[RDSD])[2:]

            if h2 != r2 or h3 != r3 or h4 != r4:
                wc += 1
                h2 = r2
                h3 = r3
                h4 = r4

                value = int(r2[:4], 2)
                value2 = int(r2[5:-5], 2)

                if value2 == 0:
                    Mtype = "A"
                else:
                    Mtype = "B"

                code = str(value) + Mtype

                if code == "2B":
                    chars = (
                        chr(int(r3[:8], 2)) +
                        chr(int(r3[9:], 2)) +
                        chr(int(r4[:8], 2)) +
                        chr(int(r4[9:], 2))
                    )
                    index = int(r2[12:], 2)

                    if index == 0 and mi != 0:
                        # remove comma from message to prevent corruption of csv file
                        msg = msg.translate(dict.fromkeys([ord(',')], None))

                        # remove control characters, they can corrupt the message
                        msg = msg.translate(
                            dict.fromkeys(
                                range(ord('\x00'), ord('\x1f')), None)
                        )

                        # log and print only the ASCII characters
                        safe = (
                            str(int(time.time())) + "," + logdata + "," + msg
                        )
                        safe_ascii = safe.encode(
                            'ascii', 'ignore').decode('ascii', 'ignore')
                        logger.info(safe_ascii)
                        print(safe_ascii)
                        break

                    if index == mi:
                        # In Python 3, chars is already str
                        msg += chars
                        mi += 1

                if wc == 500:
                    break
