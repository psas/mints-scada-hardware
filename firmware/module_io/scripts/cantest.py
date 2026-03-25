#!.venv/bin/python
import can
import time
import threading

# Configuration for the SLCAN device
slcan_device = '/dev/ttyACM1'
baud_rate = 1000000  # Set the appropriate baud rate for your setup

running = True

def receive(canbus):
    rxi = 0
    print("Receiver running")
    # As long as this thread is supposed to be running
    while running:
        # If the CAN bus is broke, break
        if canbus is None:
            raise RuntimeError("The CAN bus has broken")
        # Get the incoming data packet
        try:
            bm = canbus.recv(0.1)
            # Process the incoming DataPacket
            if bm is not None:
                # print("RXd", bm)
                rxi += 1
                print(f"RXd {rxi:03d} {bm.arbitration_id:03X} | {' '.join([f'{d:02X}' for d in bm.data]):<23s} | ", end='')
                for bt in bm.data:
                    if bt == 0:
                        print(' ', end='')
                    elif ord(' ') <= bt <= ord('~'):
                        print(chr(bt), end='')
                    else:
                        print('@', end='')
                print()
        except Exception as e:
            print(e)
    # When the thread stops
    print("Receiver stopped")

print("Hello")
# Create a CAN bus instance using the SLCAN interface
with can.interface.Bus(interface='slcan', channel=slcan_device, bitrate=baud_rate) as bus:
    print("CAN started")
    # Define a simple CAN message
    can_id = 0x123  # CAN ID
    data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]  # 8 bytes of data

    # Create a CAN message
    message = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)

    receiverThread = threading.Thread(target=receive, args=(bus,), name="CAN-receiver")
    receiverThread.start()
    print("Thread started")

    try:
        while True:
            # Send the CAN message
#            bus.send(message)
#            print(f"Sent: {message}")

            # Wait for a second before sending the next message
            time.sleep(10)

    except KeyboardInterrupt:
        running = False
        receiverThread.join()
        print("Stopped by user")

    except can.CanError as e:
        print(f"CAN error: {e}")

