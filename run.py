"""
maybe change the errors to raise exception from inside the functions
"""
import time
import spidev
from globals import *

BUS = 0 # We only have SPI bus 0 available to us on the Pi
DEVICE = 0 # Device is the chip select pin. Set to 0 or 1, depending on the connections
spi = spidev.SpiDev()

def main():
    spi.open(BUS, DEVICE)
    spi.max_speed_hz = 10000000
    spi.mode = 0

    print("resetting chip")
    if reset() != ERROR_OK:
        print("some error running reset")
        exit(1)

    print("setting CAN baud and chip crystal frequency")
    if setBitrate(CAN_1000KBPS, MCP_16MHZ) != ERROR_OK:
        print("error setting bitrate")
        exit(1)

    print("setting CAN normal mode")
    if setNormalMode() != ERROR_OK:
        print("error setting normal mode")
        exit(1)
    time.sleep(0.025)

    print("starting to read messages if available...")
    n_frames = 0
    while True:
        error, can_msg = readMessage()
        if error == ERROR_OK:
            n_frames += 1
            print("got a can message", hex(can_msg["can_id"]), n_frames, "total CAN frames")
            print(can_msg)
            # print(hex(can_msg["can_id"]))
        else:
            # print("didnt get a can message")
            time.sleep(0.010)

def getStatus():
    return spi.xfer2([INSTRUCTION_READ_STATUS, 0x00])[1]

def modifyRegister(reg, mask, data):
    command = [INSTRUCTION_BITMOD, reg, mask, data]
    spi.xfer2(command)

def readRegister(reg):
    return spi.xfer2([INSTRUCTION_READ, reg, 0x00])[2]

def readRegisters(reg, n):
    command = [INSTRUCTION_READ, reg]
    # mcp2515 has auto-increment of address-pointer
    for i in range(n):
        command.append(0x00)
    data = spi.xfer2(command)[2:]
    return data


def readMessage():
    stat = getStatus()
    if ( stat & STAT_RX0IF ):
        rc, frame = readMessage_rxbn(RXB0)
    elif ( stat & STAT_RX1IF ):
        rc, frame = readMessage_rxbn(RXB1)
    else:
        rc, frame = ERROR_NOMSG, None
    return rc, frame

def readMessage_rxbn(rxbn):
    frame = {}

    tbufdata = readRegisters(RXB[rxbn][SIDH], 5)

    id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5)

    if ( (tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) ==  TXB_EXIDE_MASK ):
        id = (id<<2) + (tbufdata[MCP_SIDL] & 0x03)
        id = (id<<8) + tbufdata[MCP_EID8]
        id = (id<<8) + tbufdata[MCP_EID0]
        id |= CAN_EFF_FLAG

    dlc = (tbufdata[MCP_DLC] & DLC_MASK)
    if (dlc > CAN_MAX_DLEN):
        return ERROR_FAIL, None

    ctrl = readRegister(RXB[rxbn][CTRL])
    if (ctrl & RXBnCTRL_RTR):
        id |= CAN_RTR_FLAG

    frame["can_id"] = id
    frame["can_dlc"] = dlc

    frame["data"] = readRegisters(RXB[rxbn][DATA], dlc)

    modifyRegister(MCP_CANINTF, RXB[rxbn][CANINTF_RXnIF], 0)

    return ERROR_OK, frame


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
        result = setFilterMask(masks[i], True, 0)
        if (result != ERROR_OK):
            return result

    return ERROR_OK


def setRegister(reg, value):
    setRegisters(reg, [value])

def setRegisters(reg, values):
    """
    values is an array/list
    """
    command = [INSTRUCTION_WRITE, reg]
    command += values
    spi.xfer2(command)

def setFilter(num, ext, ulData):
    res = setConfigMode()
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
    setRegisters(reg, buffer)

    return ERROR_OK

def setConfigMode():
    return setMode(CANCTRL_REQOP_CONFIG)

def setNormalMode():
    return setMode(CANCTRL_REQOP_NORMAL)

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

def prepareId(ext, id):
    buffer = [0, 0, 0, 0]
    canid = id & 0x0FFFF

    if ext:
        buffer[MCP_EID0] = (canid & 0xFF)
        buffer[MCP_EID8] = (canid >> 8)
        canid = (id >> 16)
        buffer[MCP_SIDL] = (canid & 0x03)
        buffer[MCP_SIDL] += ((canid & 0x1C) << 3)
        buffer[MCP_SIDL] |= TXB_EXIDE_MASK
        buffer[MCP_SIDH] = (canid >> 5)
    else:
        buffer[MCP_SIDH] = (canid >> 3)
        buffer[MCP_SIDL] = ((canid & 0x07 ) << 5)
        buffer[MCP_EID0] = 0
        buffer[MCP_EID8] = 0

    return buffer

def setFilterMask(mask, ext, ulData):

    res = setConfigMode()
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

    setRegisters(reg, tbufdata)
    return ERROR_OK


def setBitrate(canSpeed, canClock):

    error = setConfigMode();
    if (error != ERROR_OK):
        return error

    set = 1

    # MOVE THIS OUT TO THE INCLUDE?
    if canClock == MCP_8MHZ:
        if canSpeed == CAN_5KBPS:
            cfg1 = MCP_8MHz_5kBPS_CFG1;
            cfg2 = MCP_8MHz_5kBPS_CFG2;
            cfg3 = MCP_8MHz_5kBPS_CFG3;

        elif canSpeed == CAN_10KBPS:
            cfg1 = MCP_8MHz_10kBPS_CFG1;
            cfg2 = MCP_8MHz_10kBPS_CFG2;
            cfg3 = MCP_8MHz_10kBPS_CFG3;


        elif canSpeed == CAN_20KBPS:
            cfg1 = MCP_8MHz_20kBPS_CFG1;
            cfg2 = MCP_8MHz_20kBPS_CFG2;
            cfg3 = MCP_8MHz_20kBPS_CFG3;

        elif canSpeed == CAN_31K25BPS:
            cfg1 = MCP_8MHz_31k25BPS_CFG1;
            cfg2 = MCP_8MHz_31k25BPS_CFG2;
            cfg3 = MCP_8MHz_31k25BPS_CFG3;

        elif canSpeed == CAN_33KBPS:
            cfg1 = MCP_8MHz_33k3BPS_CFG1;
            cfg2 = MCP_8MHz_33k3BPS_CFG2;
            cfg3 = MCP_8MHz_33k3BPS_CFG3;

        elif canSpeed == CAN_40KBPS:
            cfg1 = MCP_8MHz_40kBPS_CFG1;
            cfg2 = MCP_8MHz_40kBPS_CFG2;
            cfg3 = MCP_8MHz_40kBPS_CFG3;

        elif canSpeed == CAN_50KBPS:
            cfg1 = MCP_8MHz_50kBPS_CFG1;
            cfg2 = MCP_8MHz_50kBPS_CFG2;
            cfg3 = MCP_8MHz_50kBPS_CFG3;

        elif canSpeed == CAN_80KBPS:
            cfg1 = MCP_8MHz_80kBPS_CFG1;
            cfg2 = MCP_8MHz_80kBPS_CFG2;
            cfg3 = MCP_8MHz_80kBPS_CFG3;

        elif canSpeed == CAN_100KBPS:
            cfg1 = MCP_8MHz_100kBPS_CFG1;
            cfg2 = MCP_8MHz_100kBPS_CFG2;
            cfg3 = MCP_8MHz_100kBPS_CFG3;

        elif canSpeed == CAN_125KBPS:
            cfg1 = MCP_8MHz_125kBPS_CFG1;
            cfg2 = MCP_8MHz_125kBPS_CFG2;
            cfg3 = MCP_8MHz_125kBPS_CFG3;

        elif canSpeed == CAN_200KBPS:
            cfg1 = MCP_8MHz_200kBPS_CFG1;
            cfg2 = MCP_8MHz_200kBPS_CFG2;
            cfg3 = MCP_8MHz_200kBPS_CFG3;

        elif canSpeed == CAN_250KBPS:
            cfg1 = MCP_8MHz_250kBPS_CFG1;
            cfg2 = MCP_8MHz_250kBPS_CFG2;
            cfg3 = MCP_8MHz_250kBPS_CFG3;

        elif canSpeed == CAN_500KBPS:
            cfg1 = MCP_8MHz_500kBPS_CFG1;
            cfg2 = MCP_8MHz_500kBPS_CFG2;
            cfg3 = MCP_8MHz_500kBPS_CFG3;

        elif canSpeed == CAN_1000KBPS:
            cfg1 = MCP_8MHz_1000kBPS_CFG1;
            cfg2 = MCP_8MHz_1000kBPS_CFG2;
            cfg3 = MCP_8MHz_1000kBPS_CFG3;

        else:
            set = 0

    elif canClock == MCP_16MHZ:
        if canSpeed == CAN_5KBPS:
            cfg1 = MCP_16MHz_5kBPS_CFG1;
            cfg2 = MCP_16MHz_5kBPS_CFG2;
            cfg3 = MCP_16MHz_5kBPS_CFG3;

        elif canSpeed == CAN_10KBPS:
            cfg1 = MCP_16MHz_10kBPS_CFG1;
            cfg2 = MCP_16MHz_10kBPS_CFG2;
            cfg3 = MCP_16MHz_10kBPS_CFG3;

        elif canSpeed == CAN_20KBPS:
            cfg1 = MCP_16MHz_20kBPS_CFG1;
            cfg2 = MCP_16MHz_20kBPS_CFG2;
            cfg3 = MCP_16MHz_20kBPS_CFG3;

        elif canSpeed == CAN_33KBPS:
            cfg1 = MCP_16MHz_33k3BPS_CFG1;
            cfg2 = MCP_16MHz_33k3BPS_CFG2;
            cfg3 = MCP_16MHz_33k3BPS_CFG3;

        elif canSpeed == CAN_40KBPS:
            cfg1 = MCP_16MHz_40kBPS_CFG1;
            cfg2 = MCP_16MHz_40kBPS_CFG2;
            cfg3 = MCP_16MHz_40kBPS_CFG3;

        elif canSpeed == CAN_50KBPS:
            cfg1 = MCP_16MHz_50kBPS_CFG1;
            cfg2 = MCP_16MHz_50kBPS_CFG2;
            cfg3 = MCP_16MHz_50kBPS_CFG3;

        elif canSpeed == CAN_80KBPS:
            cfg1 = MCP_16MHz_80kBPS_CFG1;
            cfg2 = MCP_16MHz_80kBPS_CFG2;
            cfg3 = MCP_16MHz_80kBPS_CFG3;

        elif canSpeed == CAN_83K3BPS:
            cfg1 = MCP_16MHz_83k3BPS_CFG1;
            cfg2 = MCP_16MHz_83k3BPS_CFG2;
            cfg3 = MCP_16MHz_83k3BPS_CFG3;

        elif canSpeed == CAN_95KBPS:
            cfg1 = MCP_16MHz_95kBPS_CFG1;
            cfg2 = MCP_16MHz_95kBPS_CFG2;
            cfg3 = MCP_16MHz_95kBPS_CFG3;

        elif canSpeed == CAN_100KBPS:
            cfg1 = MCP_16MHz_100kBPS_CFG1;
            cfg2 = MCP_16MHz_100kBPS_CFG2;
            cfg3 = MCP_16MHz_100kBPS_CFG3;

        elif canSpeed == CAN_125KBPS:
            cfg1 = MCP_16MHz_125kBPS_CFG1;
            cfg2 = MCP_16MHz_125kBPS_CFG2;
            cfg3 = MCP_16MHz_125kBPS_CFG3;

        elif canSpeed == CAN_200KBPS:
            cfg1 = MCP_16MHz_200kBPS_CFG1;
            cfg2 = MCP_16MHz_200kBPS_CFG2;
            cfg3 = MCP_16MHz_200kBPS_CFG3;

        elif canSpeed == CAN_250KBPS:
            cfg1 = MCP_16MHz_250kBPS_CFG1;
            cfg2 = MCP_16MHz_250kBPS_CFG2;
            cfg3 = MCP_16MHz_250kBPS_CFG3;

        elif canSpeed == CAN_500KBPS:
            cfg1 = MCP_16MHz_500kBPS_CFG1;
            cfg2 = MCP_16MHz_500kBPS_CFG2;
            cfg3 = MCP_16MHz_500kBPS_CFG3;

        elif canSpeed == CAN_1000KBPS:
            cfg1 = MCP_16MHz_1000kBPS_CFG1;
            cfg2 = MCP_16MHz_1000kBPS_CFG2;
            cfg3 = MCP_16MHz_1000kBPS_CFG3;

        else:
            set = 0

    elif canClock == MCP_20MHZ:
        if canSpeed == CAN_33KBPS:
            cfg1 = MCP_20MHz_33k3BPS_CFG1;
            cfg2 = MCP_20MHz_33k3BPS_CFG2;
            cfg3 = MCP_20MHz_33k3BPS_CFG3;

        elif canSpeed == CAN_40KBPS:
            cfg1 = MCP_20MHz_40kBPS_CFG1;
            cfg2 = MCP_20MHz_40kBPS_CFG2;
            cfg3 = MCP_20MHz_40kBPS_CFG3;

        elif canSpeed == CAN_50KBPS:
            cfg1 = MCP_20MHz_50kBPS_CFG1;
            cfg2 = MCP_20MHz_50kBPS_CFG2;
            cfg3 = MCP_20MHz_50kBPS_CFG3;

        elif canSpeed == CAN_80KBPS:
            cfg1 = MCP_20MHz_80kBPS_CFG1;
            cfg2 = MCP_20MHz_80kBPS_CFG2;
            cfg3 = MCP_20MHz_80kBPS_CFG3;

        elif canSpeed == CAN_83K3BPS:
            cfg1 = MCP_20MHz_83k3BPS_CFG1;
            cfg2 = MCP_20MHz_83k3BPS_CFG2;
            cfg3 = MCP_20MHz_83k3BPS_CFG3;

        elif canSpeed == CAN_100KBPS:
            cfg1 = MCP_20MHz_100kBPS_CFG1;
            cfg2 = MCP_20MHz_100kBPS_CFG2;
            cfg3 = MCP_20MHz_100kBPS_CFG3;

        elif canSpeed == CAN_125KBPS:
            cfg1 = MCP_20MHz_125kBPS_CFG1
            cfg2 = MCP_20MHz_125kBPS_CFG2
            cfg3 = MCP_20MHz_125kBPS_CFG3

        elif canSpeed == CAN_200KBPS:
            cfg1 = MCP_20MHz_200kBPS_CFG1
            cfg2 = MCP_20MHz_200kBPS_CFG2
            cfg3 = MCP_20MHz_200kBPS_CFG3

        elif canSpeed == CAN_250KBPS:
            cfg1 = MCP_20MHz_250kBPS_CFG1
            cfg2 = MCP_20MHz_250kBPS_CFG2
            cfg3 = MCP_20MHz_250kBPS_CFG3

        elif canSpeed == CAN_500KBPS:
            cfg1 = MCP_20MHz_500kBPS_CFG1
            cfg2 = MCP_20MHz_500kBPS_CFG2
            cfg3 = MCP_20MHz_500kBPS_CFG3

        elif canSpeed == CAN_1000KBPS:
            cfg1 = MCP_20MHz_1000kBPS_CFG1
            cfg2 = MCP_20MHz_1000kBPS_CFG2
            cfg3 = MCP_20MHz_1000kBPS_CFG3

        else:
            set = 0

    else:
        set = 0

    if (set):
        setRegister(MCP_CNF1, cfg1)
        setRegister(MCP_CNF2, cfg2)
        setRegister(MCP_CNF3, cfg3)
        return ERROR_OK
    else:
        return ERROR_FAIL



if __name__ == "__main__":
    main()
