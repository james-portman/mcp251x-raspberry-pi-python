import time
import mcp251x

BUS = 0 # We only have SPI bus 0 available to us on the Pi
DEVICE = 0 # Device is the chip select pin. Set to 0 or 1, depending on the connections

def main():
    can = mcp251x.MCP251x(BUS, DEVICE)

    print("resetting chip")
    if can.reset() != mcp251x.ERROR_OK:
        print("some error running reset")
        exit(1)

    print("setting CAN baud and chip crystal frequency")
    if can.setBitrate(mcp251x.CAN_1000KBPS, mcp251x.MCP_16MHZ) != mcp251x.ERROR_OK:
        print("error setting bitrate")
        exit(1)

    print("setting CAN normal mode")
    if can.setNormalMode() != mcp251x.ERROR_OK:
        print("error setting normal mode")
        exit(1)
    time.sleep(0.025)

    print("starting to read messages if available...")
    n_frames = 0
    while True:
        error, can_msg = can.readMessage()
        if error == mcp251x.ERROR_OK:
            n_frames += 1
            print("got a can message", hex(can_msg["can_id"]), ", ", n_frames, "total CAN frames")
            print(can_msg)
            # print(hex(can_msg["can_id"]))
        else:
            # print("didnt get a can message")
            time.sleep(0.010)



if __name__ == "__main__":
    main()
