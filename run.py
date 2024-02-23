import time
import spidev
from globals import *

BUS = 0 # We only have SPI bus 0 available to us on the Pi
DEVICE = 0 # Device is the chip select pin. Set to 0 or 1, depending on the connections

spi = spidev.SpiDev()
spi.open(BUS, DEVICE)
spi.max_speed_hz = 10000000
spi.mode = 0

# for 16mhz, 1mbit can:
CFG1 = 0x00
CFG2 = 0xD0
CFG3 = 0x82
# 16mhz, 500k can:
# CFG1 = 0x00
# CFG2 = 0xF0
# CFG3 = 0x86



spi.xfer2([INSTRUCTION_RESET])

time.sleep(0.010)


reply = spi.xfer2([INSTRUCTION_READ, MCP_RXF0SIDH, 0])
print(reply)
