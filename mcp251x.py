import time
import spidev

class MCP251x():

    def __init__(self, bus, device, spi_bitrate=10000000):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = spi_bitrate
        self.spi.mode = 0


    def getStatus(self):
        return self.spi.xfer2([INSTRUCTION_READ_STATUS, 0x00])[1]

    def modifyRegister(self, reg, mask, data):
        command = [INSTRUCTION_BITMOD, reg, mask, data]
        self.spi.xfer2(command)

    def readRegister(self, reg):
        return self.spi.xfer2([INSTRUCTION_READ, reg, 0x00])[2]

    def readRegisters(self, reg, n):
        command = [INSTRUCTION_READ, reg]
        # mcp2515 has auto-increment of address-pointer
        for i in range(n):
            command.append(0x00)
        data = self.spi.xfer2(command)[2:]
        return data


    def readMessage(self):
        stat = self.getStatus()
        if ( stat & STAT_RX0IF ):
            rc, frame = self.readMessage_rxbn(RXB0)
        elif ( stat & STAT_RX1IF ):
            rc, frame = self.readMessage_rxbn(RXB1)
        else:
            rc, frame = ERROR_NOMSG, None
        return rc, frame

    def readMessage_rxbn(self, rxbn):
        frame = {}

        tbufdata = self.readRegisters(RXB[rxbn][SIDH], 5)

        id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5)

        if ( (tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) ==  TXB_EXIDE_MASK ):
            id = (id<<2) + (tbufdata[MCP_SIDL] & 0x03)
            id = (id<<8) + tbufdata[MCP_EID8]
            id = (id<<8) + tbufdata[MCP_EID0]
            id |= CAN_EFF_FLAG

        dlc = (tbufdata[MCP_DLC] & DLC_MASK)
        if (dlc > CAN_MAX_DLEN):
            return ERROR_FAIL, None

        ctrl = self.readRegister(RXB[rxbn][CTRL])
        if (ctrl & RXBnCTRL_RTR):
            id |= CAN_RTR_FLAG

        frame["can_id"] = id
        frame["can_dlc"] = dlc

        frame["data"] = self.readRegisters(RXB[rxbn][DATA], dlc)

        self.modifyRegister(MCP_CANINTF, RXB[rxbn][CANINTF_RXnIF], 0)

        return ERROR_OK, frame


    def reset(self):
        self.spi.xfer2([INSTRUCTION_RESET])

        time.sleep(0.010)

        zeros = []
        for _ in range(14):
            zeros.append(0)
        self.setRegisters(MCP_TXB0CTRL, zeros)
        self.setRegisters(MCP_TXB1CTRL, zeros)
        self.setRegisters(MCP_TXB2CTRL, zeros)

        self.setRegister(MCP_RXB0CTRL, 0)
        self.setRegister(MCP_RXB1CTRL, 0)

        self.setRegister(MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF)

        # receives all valid messages using either Standard or Extended Identifiers that
        # meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for RXB1
        self.modifyRegister(MCP_RXB0CTRL,
                       RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT | RXB0CTRL_FILHIT_MASK,
                       RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT | RXB0CTRL_FILHIT)
        self.modifyRegister(MCP_RXB1CTRL,
                       RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK,
                       RXBnCTRL_RXM_STDEXT | RXB1CTRL_FILHIT)

        # clear filters and masks
        # do not filter any standard frames for RXF0 used by RXB0
        # do not filter any extended frames for RXF1 used by RXB1
        filters = [RXF0, RXF1, RXF2, RXF3, RXF4, RXF5]
        for i in range(len(filters)):
            ext = (i == 1)
            result = self.setFilter(filters[i], ext, 0)
            if result != ERROR_OK:
                return result

        masks = [MASK0, MASK1]
        for i in range(len(masks)):
            result = self.setFilterMask(masks[i], True, 0)
            if (result != ERROR_OK):
                return result

        return ERROR_OK


    def setRegister(self, reg, value):
        self.setRegisters(reg, [value])

    def setRegisters(self, reg, values):
        """
        values is an array/list
        """
        command = [INSTRUCTION_WRITE, reg]
        command += values
        self.spi.xfer2(command)

    def setFilter(self, num, ext, ulData):
        res = self.setConfigMode()
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

        buffer = self.prepareId(ext, ulData)
        self.setRegisters(reg, buffer)

        return ERROR_OK

    def setConfigMode(self):
        return self.setMode(CANCTRL_REQOP_CONFIG)

    def setNormalMode(self):
        return self.setMode(CANCTRL_REQOP_NORMAL)

    def setMode(self, mode):
        self.modifyRegister(MCP_CANCTRL, CANCTRL_REQOP, mode)
        endTime = time.time() + 10
        modeMatch = False
        while time.time() < endTime:
            newmode = self.readRegister(MCP_CANSTAT)
            newmode &= CANSTAT_OPMOD
            modeMatch = (newmode == mode)
            if modeMatch:
                break
        if modeMatch:
            return ERROR_OK
        else:
            return ERROR_FAIL

    def prepareId(self, ext, id):
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

    def setFilterMask(self, mask, ext, ulData):

        res = self.setConfigMode()
        if (res != ERROR_OK):
            return res
        
        
        tbufdata = self.prepareId(ext, ulData)

        # REGISTER reg;
        if mask == MASK0:
            reg = MCP_RXM0SIDH
        elif mask == MASK1:
            reg = MCP_RXM1SIDH
        else:
            return ERROR_FAIL

        self.setRegisters(reg, tbufdata)
        return ERROR_OK


    def setBitrate(self, canSpeed, canClock):

        error = self.setConfigMode();
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
            self.setRegister(MCP_CNF1, cfg1)
            self.setRegister(MCP_CNF2, cfg2)
            self.setRegister(MCP_CNF3, cfg3)
            return ERROR_OK
        else:
            return ERROR_FAIL

# Speed 8M
MCP_8MHz_1000kBPS_CFG1 = 0x00
MCP_8MHz_1000kBPS_CFG2 = 0x80
MCP_8MHz_1000kBPS_CFG3 = 0x80
MCP_8MHz_500kBPS_CFG1 = 0x00
MCP_8MHz_500kBPS_CFG2 = 0x90
MCP_8MHz_500kBPS_CFG3 = 0x82
MCP_8MHz_250kBPS_CFG1 = 0x00
MCP_8MHz_250kBPS_CFG2 = 0xB1
MCP_8MHz_250kBPS_CFG3 = 0x85
MCP_8MHz_200kBPS_CFG1 = 0x00
MCP_8MHz_200kBPS_CFG2 = 0xB4
MCP_8MHz_200kBPS_CFG3 = 0x86
MCP_8MHz_125kBPS_CFG1 = 0x01
MCP_8MHz_125kBPS_CFG2 = 0xB1
MCP_8MHz_125kBPS_CFG3 = 0x85
MCP_8MHz_100kBPS_CFG1 = 0x01
MCP_8MHz_100kBPS_CFG2 = 0xB4
MCP_8MHz_100kBPS_CFG3 = 0x86
MCP_8MHz_80kBPS_CFG1 = 0x01
MCP_8MHz_80kBPS_CFG2 = 0xBF
MCP_8MHz_80kBPS_CFG3 = 0x87
MCP_8MHz_50kBPS_CFG1 = 0x03
MCP_8MHz_50kBPS_CFG2 = 0xB4
MCP_8MHz_50kBPS_CFG3 = 0x86
MCP_8MHz_40kBPS_CFG1 = 0x03
MCP_8MHz_40kBPS_CFG2 = 0xBF
MCP_8MHz_40kBPS_CFG3 = 0x87
MCP_8MHz_33k3BPS_CFG1 = 0x47
MCP_8MHz_33k3BPS_CFG2 = 0xE2
MCP_8MHz_33k3BPS_CFG3 = 0x85
MCP_8MHz_31k25BPS_CFG1 = 0x07
MCP_8MHz_31k25BPS_CFG2 = 0xA4
MCP_8MHz_31k25BPS_CFG3 = 0x84
MCP_8MHz_20kBPS_CFG1 = 0x07
MCP_8MHz_20kBPS_CFG2 = 0xBF
MCP_8MHz_20kBPS_CFG3 = 0x87
MCP_8MHz_10kBPS_CFG1 = 0x0F
MCP_8MHz_10kBPS_CFG2 = 0xBF
MCP_8MHz_10kBPS_CFG3 = 0x87
MCP_8MHz_5kBPS_CFG1 = 0x1F
MCP_8MHz_5kBPS_CFG2 = 0xBF
MCP_8MHz_5kBPS_CFG3 = 0x87

# speed 16M
MCP_16MHz_1000kBPS_CFG1 = 0x00
MCP_16MHz_1000kBPS_CFG2 = 0xD0
MCP_16MHz_1000kBPS_CFG3 = 0x82
MCP_16MHz_500kBPS_CFG1 = 0x00
MCP_16MHz_500kBPS_CFG2 = 0xF0
MCP_16MHz_500kBPS_CFG3 = 0x86
MCP_16MHz_250kBPS_CFG1 = 0x41
MCP_16MHz_250kBPS_CFG2 = 0xF1
MCP_16MHz_250kBPS_CFG3 = 0x85
MCP_16MHz_200kBPS_CFG1 = 0x01
MCP_16MHz_200kBPS_CFG2 = 0xFA
MCP_16MHz_200kBPS_CFG3 = 0x87
MCP_16MHz_125kBPS_CFG1 = 0x03
MCP_16MHz_125kBPS_CFG2 = 0xF0
MCP_16MHz_125kBPS_CFG3 = 0x86
MCP_16MHz_100kBPS_CFG1 = 0x03
MCP_16MHz_100kBPS_CFG2 = 0xFA
MCP_16MHz_100kBPS_CFG3 = 0x87
MCP_16MHz_95kBPS_CFG1 = 0x03
MCP_16MHz_95kBPS_CFG2 = 0xAD
MCP_16MHz_95kBPS_CFG3 = 0x07
MCP_16MHz_83k3BPS_CFG1 = 0x03
MCP_16MHz_83k3BPS_CFG2 = 0xBE
MCP_16MHz_83k3BPS_CFG3 = 0x07
MCP_16MHz_80kBPS_CFG1 = 0x03
MCP_16MHz_80kBPS_CFG2 = 0xFF
MCP_16MHz_80kBPS_CFG3 = 0x87
MCP_16MHz_50kBPS_CFG1 = 0x07
MCP_16MHz_50kBPS_CFG2 = 0xFA
MCP_16MHz_50kBPS_CFG3 = 0x87
MCP_16MHz_40kBPS_CFG1 = 0x07
MCP_16MHz_40kBPS_CFG2 = 0xFF
MCP_16MHz_40kBPS_CFG3 = 0x87
MCP_16MHz_33k3BPS_CFG1 = 0x4E
MCP_16MHz_33k3BPS_CFG2 = 0xF1
MCP_16MHz_33k3BPS_CFG3 = 0x85
MCP_16MHz_20kBPS_CFG1 = 0x0F
MCP_16MHz_20kBPS_CFG2 = 0xFF
MCP_16MHz_20kBPS_CFG3 = 0x87
MCP_16MHz_10kBPS_CFG1 = 0x1F
MCP_16MHz_10kBPS_CFG2 = 0xFF
MCP_16MHz_10kBPS_CFG3 = 0x87
MCP_16MHz_5kBPS_CFG1 = 0x3F
MCP_16MHz_5kBPS_CFG2 = 0xFF
MCP_16MHz_5kBPS_CFG3 = 0x87

# speed 20M
MCP_20MHz_1000kBPS_CFG1 = 0x00
MCP_20MHz_1000kBPS_CFG2 = 0xD9
MCP_20MHz_1000kBPS_CFG3 = 0x82
MCP_20MHz_500kBPS_CFG1 = 0x00
MCP_20MHz_500kBPS_CFG2 = 0xFA
MCP_20MHz_500kBPS_CFG3 = 0x87
MCP_20MHz_250kBPS_CFG1 = 0x41
MCP_20MHz_250kBPS_CFG2 = 0xFB
MCP_20MHz_250kBPS_CFG3 = 0x86
MCP_20MHz_200kBPS_CFG1 = 0x01
MCP_20MHz_200kBPS_CFG2 = 0xFF
MCP_20MHz_200kBPS_CFG3 = 0x87
MCP_20MHz_125kBPS_CFG1 = 0x03
MCP_20MHz_125kBPS_CFG2 = 0xFA
MCP_20MHz_125kBPS_CFG3 = 0x87
MCP_20MHz_100kBPS_CFG1 = 0x04
MCP_20MHz_100kBPS_CFG2 = 0xFA
MCP_20MHz_100kBPS_CFG3 = 0x87
MCP_20MHz_83k3BPS_CFG1 = 0x04
MCP_20MHz_83k3BPS_CFG2 = 0xFE
MCP_20MHz_83k3BPS_CFG3 = 0x87
MCP_20MHz_80kBPS_CFG1 = 0x04
MCP_20MHz_80kBPS_CFG2 = 0xFF
MCP_20MHz_80kBPS_CFG3 = 0x87
MCP_20MHz_50kBPS_CFG1 = 0x09
MCP_20MHz_50kBPS_CFG2 = 0xFA
MCP_20MHz_50kBPS_CFG3 = 0x87
MCP_20MHz_40kBPS_CFG1 = 0x09
MCP_20MHz_40kBPS_CFG2 = 0xFF
MCP_20MHz_40kBPS_CFG3 = 0x87
MCP_20MHz_33k3BPS_CFG1 = 0x0B
MCP_20MHz_33k3BPS_CFG2 = 0xFF
MCP_20MHz_33k3BPS_CFG3 = 0x87


# enum CAN_CLOCK {
MCP_20MHZ = 0
MCP_16MHZ = 1
MCP_8MHZ = 2
# };

# enum CAN_SPEED {
CAN_5KBPS = 0
CAN_10KBPS = 1
CAN_20KBPS = 2
CAN_31K25BPS = 3
CAN_33KBPS = 4
CAN_40KBPS = 5
CAN_50KBPS = 6
CAN_80KBPS = 7
CAN_83K3BPS = 8
CAN_95KBPS = 9
CAN_100KBPS = 10
CAN_125KBPS = 11
CAN_200KBPS = 12
CAN_250KBPS = 13
CAN_500KBPS = 14
CAN_1000KBPS = 15
# };

# enum CAN_CLKOUT {
CLKOUT_DISABLE = -1
CLKOUT_DIV1    = 0x0
CLKOUT_DIV2    = 0x1
CLKOUT_DIV4    = 0x2
CLKOUT_DIV8    = 0x3
# };

ERROR_OK        = 0
ERROR_FAIL      = 1
ERROR_ALLTXBUSY = 2
ERROR_FAILINIT  = 3
ERROR_FAILTX    = 4
ERROR_NOMSG     = 5

MASK0 = 0
MASK1 = 1

RXF0 = 0
RXF1 = 1
RXF2 = 2
RXF3 = 3
RXF4 = 4
RXF5 = 5

RXB0 = 0
RXB1 = 1

TXB0 = 0
TXB1 = 1
TXB2 = 2

CANINTF_RX0IF = 0x01
CANINTF_RX1IF = 0x02
CANINTF_TX0IF = 0x04
CANINTF_TX1IF = 0x08
CANINTF_TX2IF = 0x10
CANINTF_ERRIF = 0x20
CANINTF_WAKIF = 0x40
CANINTF_MERRF = 0x80


EFLG_RX1OVR = (1<<7)
EFLG_RX0OVR = (1<<6)
EFLG_TXBO   = (1<<5)
EFLG_TXEP   = (1<<4)
EFLG_RXEP   = (1<<3)
EFLG_TXWAR  = (1<<2)
EFLG_RXWAR  = (1<<1)
EFLG_EWARN  = (1<<0)

CANCTRL_REQOP = 0xE0
CANCTRL_ABAT = 0x10
CANCTRL_OSM = 0x08
CANCTRL_CLKEN = 0x04
CANCTRL_CLKPRE = 0x03

CANCTRL_REQOP_NORMAL     = 0x00
CANCTRL_REQOP_SLEEP      = 0x20
CANCTRL_REQOP_LOOPBACK   = 0x40
CANCTRL_REQOP_LISTENONLY = 0x60
CANCTRL_REQOP_CONFIG     = 0x80
CANCTRL_REQOP_POWERUP    = 0xE0

CANSTAT_OPMOD = 0xE0
CANSTAT_ICOD = 0x0E

CNF3_SOF = 0x80

TXB_EXIDE_MASK = 0x08
DLC_MASK       = 0x0F
RTR_MASK       = 0x40

RXBnCTRL_RXM_STD    = 0x20
RXBnCTRL_RXM_EXT    = 0x40
RXBnCTRL_RXM_STDEXT = 0x00
RXBnCTRL_RXM_MASK   = 0x60
RXBnCTRL_RTR        = 0x08
RXB0CTRL_BUKT       = 0x04
RXB0CTRL_FILHIT_MASK = 0x03
RXB1CTRL_FILHIT_MASK = 0x07
RXB0CTRL_FILHIT = 0x00
RXB1CTRL_FILHIT = 0x01

MCP_SIDH = 0
MCP_SIDL = 1
MCP_EID8 = 2
MCP_EID0 = 3
MCP_DLC  = 4
MCP_DATA = 5

# enum /*class*/ STAT : uint8_t {
STAT_RX0IF = (1<<0)
STAT_RX1IF = (1<<1)
# };

STAT_RXIF_MASK = STAT_RX0IF | STAT_RX1IF

EFLG_ERRORMASK = EFLG_RX1OVR | EFLG_RX0OVR | EFLG_TXBO | EFLG_TXEP | EFLG_RXEP


# instructions
INSTRUCTION_WRITE       = 0x02
INSTRUCTION_READ        = 0x03
INSTRUCTION_BITMOD      = 0x05
INSTRUCTION_LOAD_TX0    = 0x40
INSTRUCTION_LOAD_TX1    = 0x42
INSTRUCTION_LOAD_TX2    = 0x44
INSTRUCTION_RTS_TX0     = 0x81
INSTRUCTION_RTS_TX1     = 0x82
INSTRUCTION_RTS_TX2     = 0x84
INSTRUCTION_RTS_ALL     = 0x87
INSTRUCTION_READ_RX0    = 0x90
INSTRUCTION_READ_RX1    = 0x94
INSTRUCTION_READ_STATUS = 0xA0
INSTRUCTION_RX_STATUS   = 0xB0
INSTRUCTION_RESET       = 0xC0

# registers
MCP_RXF0SIDH = 0x00
MCP_RXF0SIDL = 0x01
MCP_RXF0EID8 = 0x02
MCP_RXF0EID0 = 0x03
MCP_RXF1SIDH = 0x04
MCP_RXF1SIDL = 0x05
MCP_RXF1EID8 = 0x06
MCP_RXF1EID0 = 0x07
MCP_RXF2SIDH = 0x08
MCP_RXF2SIDL = 0x09
MCP_RXF2EID8 = 0x0A
MCP_RXF2EID0 = 0x0B
MCP_CANSTAT  = 0x0E
MCP_CANCTRL  = 0x0F
MCP_RXF3SIDH = 0x10
MCP_RXF3SIDL = 0x11
MCP_RXF3EID8 = 0x12
MCP_RXF3EID0 = 0x13
MCP_RXF4SIDH = 0x14
MCP_RXF4SIDL = 0x15
MCP_RXF4EID8 = 0x16
MCP_RXF4EID0 = 0x17
MCP_RXF5SIDH = 0x18
MCP_RXF5SIDL = 0x19
MCP_RXF5EID8 = 0x1A
MCP_RXF5EID0 = 0x1B
MCP_TEC      = 0x1C
MCP_REC      = 0x1D
MCP_RXM0SIDH = 0x20
MCP_RXM0SIDL = 0x21
MCP_RXM0EID8 = 0x22
MCP_RXM0EID0 = 0x23
MCP_RXM1SIDH = 0x24
MCP_RXM1SIDL = 0x25
MCP_RXM1EID8 = 0x26
MCP_RXM1EID0 = 0x27
MCP_CNF3     = 0x28
MCP_CNF2     = 0x29
MCP_CNF1     = 0x2A
MCP_CANINTE  = 0x2B
MCP_CANINTF  = 0x2C
MCP_EFLG     = 0x2D
MCP_TXB0CTRL = 0x30
MCP_TXB0SIDH = 0x31
MCP_TXB0SIDL = 0x32
MCP_TXB0EID8 = 0x33
MCP_TXB0EID0 = 0x34
MCP_TXB0DLC  = 0x35
MCP_TXB0DATA = 0x36
MCP_TXB1CTRL = 0x40
MCP_TXB1SIDH = 0x41
MCP_TXB1SIDL = 0x42
MCP_TXB1EID8 = 0x43
MCP_TXB1EID0 = 0x44
MCP_TXB1DLC  = 0x45
MCP_TXB1DATA = 0x46
MCP_TXB2CTRL = 0x50
MCP_TXB2SIDH = 0x51
MCP_TXB2SIDL = 0x52
MCP_TXB2EID8 = 0x53
MCP_TXB2EID0 = 0x54
MCP_TXB2DLC  = 0x55
MCP_TXB2DATA = 0x56
MCP_RXB0CTRL = 0x60
MCP_RXB0SIDH = 0x61
MCP_RXB0SIDL = 0x62
MCP_RXB0EID8 = 0x63
MCP_RXB0EID0 = 0x64
MCP_RXB0DLC  = 0x65
MCP_RXB0DATA = 0x66
MCP_RXB1CTRL = 0x70
MCP_RXB1SIDH = 0x71
MCP_RXB1SIDL = 0x72
MCP_RXB1EID8 = 0x73
MCP_RXB1EID0 = 0x74
MCP_RXB1DLC  = 0x75
MCP_RXB1DATA = 0x76

CTRL = 0
SIDH = 1
DATA = 2
CANINTF_RXnIF = 3

DEFAULT_SPI_CLOCK = 10000000 # 10MHz

# N_TXBUFFERS = 3
# N_RXBUFFERS = 2

TXB = [
    [MCP_TXB0CTRL, MCP_TXB0SIDH, MCP_TXB0DATA],
    [MCP_TXB1CTRL, MCP_TXB1SIDH, MCP_TXB1DATA],
    [MCP_TXB2CTRL, MCP_TXB2SIDH, MCP_TXB2DATA]
]

RXB = [
    [MCP_RXB0CTRL, MCP_RXB0SIDH, MCP_RXB0DATA, CANINTF_RX0IF],
    [MCP_RXB1CTRL, MCP_RXB1SIDH, MCP_RXB1DATA, CANINTF_RX1IF]
]

CAN_EFF_FLAG = 0x80000000 # /* EFF/SFF is set in the MSB */
CAN_RTR_FLAG = 0x40000000 # /* remote transmission request */
CAN_ERR_FLAG = 0x20000000 # /* error message frame */
CAN_MAX_DLEN = 8

# /* CAN payload length and DLC definitions according to ISO 11898-1 */
# #define CAN_MAX_DLC 8
# #define CAN_MAX_RAW_DLC 15
# #define CAN_MAX_DLEN 8

# /* CAN FD payload length and DLC definitions according to ISO 11898-7 */
# #define CANFD_MAX_DLC 15
# #define CANFD_MAX_DLEN 64

# /*
#  * CAN XL payload length and DLC definitions according to ISO 11898-1
#  * CAN XL DLC ranges from 0 .. 2047 => data length from 1 .. 2048 byte
#  */
# #define CANXL_MIN_DLC 0
# #define CANXL_MAX_DLC 2047
# #define CANXL_MAX_DLC_MASK 0x07FF
# #define CANXL_MIN_DLEN 1
# #define CANXL_MAX_DLEN 2048