import time
import spidev
from globals import *

BUS = 0 # We only have SPI bus 0 available to us on the Pi
DEVICE = 0 # Device is the chip select pin. Set to 0 or 1, depending on the connections

def main():
	spi = spidev.SpiDev()
	spi.open(BUS, DEVICE)
	spi.max_speed_hz = 10000000
	spi.mode = 0

	# for 16mhz, 1mbit can:
	MCP_16MHz_1000kBPS_CFG1
	MCP_16MHz_1000kBPS_CFG2
	MCP_16MHz_1000kBPS_CFG3

	# MCP_16MHz_500kBPS_CFG1
	# MCP_16MHz_500kBPS_CFG2
	# MCP_16MHz_500kBPS_CFG3


	spi.xfer2([INSTRUCTION_RESET])

	time.sleep(0.010)


	reply = spi.xfer2([INSTRUCTION_READ, MCP_RXF0SIDH, 0])
	print(reply)


def reset():
    spi.xfer2([INSTRUCTION_RESET])

    time.sleep(0.010)

    zeros = []
    for _ in range(14):
    	zeros.append(0)
    setRegisters(MCP_TXB0CTRL, zeros)
    setRegisters(MCP_TXB1CTRL, zeros)
    setRegisters(MCP_TXB2CTRL, zeros)

    setRegister(MCP_RXB0CTRL, 0)
    setRegister(MCP_RXB1CTRL, 0)

    setRegister(MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF)

    # receives all valid messages using either Standard or Extended Identifiers that
    # meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for RXB1
    modifyRegister(MCP_RXB0CTRL,
                   RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT | RXB0CTRL_FILHIT_MASK,
                   RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT | RXB0CTRL_FILHIT)
    modifyRegister(MCP_RXB1CTRL,
                   RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK,
                   RXBnCTRL_RXM_STDEXT | RXB1CTRL_FILHIT)

    # clear filters and masks
    # do not filter any standard frames for RXF0 used by RXB0
    # do not filter any extended frames for RXF1 used by RXB1
    filters = [RXF0, RXF1, RXF2, RXF3, RXF4, RXF5]
    for i in range(len(filters)):
        ext = (i == 1)
        result = setFilter(filters[i], ext, 0)
        if result != ERROR_OK:
            return result

    masks = [MASK0, MASK1]
    for i in range(len(masks)):
        ERROR result = setFilterMask(masks[i], true, 0)
        if (result != ERROR_OK):
            return result

    return ERROR_OK

def setRegister(reg, value)
	setRegisters(reg, [value])

def setRegisters(reg, values):
	"""
	values is an array/list
	"""
	command = [INSTRUCTION_WRITE, reg]
	command += values
	spi.xfer2(command)

def modifyRegister(reg, mask, data)
	command = [INSTRUCTION_BITMOD, reg, mask, data]
	spi.xfer2(command)

def setFilter(num, ext, ulData):
    ERROR res = setConfigMode()
    if res != ERROR_OK:
        return res

    if num == RXF0:
    	reg = MCP_RXF0SIDH
    elif num == RXF1:
    	reg = MCP_RXF1SIDH
    elif num == RXF2:
    	reg = MCP_RXF2SIDH
    elif num == RXF3:
    	reg = MCP_RXF3SIDH
    elif num == RXF4:
    	reg = MCP_RXF4SIDH
    elif num == RXF5:
    	reg = MCP_RXF5SIDH
    else:
        return ERROR_FAIL

    buffer = prepareId(ext, ulData)
    setRegisters(reg, buffer, 4)

    return ERROR_OK

def setConfigMode():
    return setMode(CANCTRL_REQOP_CONFIG)

def setMode(mode):
    modifyRegister(MCP_CANCTRL, CANCTRL_REQOP, mode)
    endTime = time.time() + 10
    modeMatch = False
    while time.time() < endTime:
        newmode = readRegister(MCP_CANSTAT)
        newmode &= CANSTAT_OPMOD
        modeMatch = (newmode == mode)
        if modeMatch:
            break
    if modeMatch:
    	return ERROR_OK
    else:
    	return ERROR_FAIL

def prepareId(ext, id)
	buffer = []
    canid = id & 0x0FFFF

    if ext:
        buffer[MCP_EID0] = (canid & 0xFF)
        buffer[MCP_EID8] = (canid >> 8)
        canid = (id >> 16)
        buffer[MCP_SIDL] = (canid & 0x03)
        buffer[MCP_SIDL] += ((canid & 0x1C) << 3)
        buffer[MCP_SIDL] |= TXB_EXIDE_MASK
        buffer[MCP_SIDH] = (uint8_t) (canid >> 5)
    else:
        buffer[MCP_SIDH] = (canid >> 3)
        buffer[MCP_SIDL] = ((canid & 0x07 ) << 5)
        buffer[MCP_EID0] = 0
        buffer[MCP_EID8] = 0

    return buffer

def setFilterMask(mask, ext, ulData):

    ERROR res = setConfigMode()
    if (res != ERROR_OK):
        return res
    
    
    tbufdata = prepareId(ext, ulData)

    # REGISTER reg;
    if mask == MASK0:
    	reg = MCP_RXM0SIDH
    elif mask == MASK1:
    	reg = MCP_RXM1SIDH
   	else:
        return ERROR_FAIL

    setRegisters(reg, tbufdata, 4)
    return ERROR_OK

if __name__ == "__main__":
	main():
