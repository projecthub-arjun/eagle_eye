# This python script receives ground station data through a socket
# and sends it to the Flight Controller through serial UART.

import socket, time, struct, sys
import signal
import serial

# Eagle Eye drone IP
EAGLE_EYE_DRONE_IP = "192.168.0.111"
EAGLE_EYE_DRONE_PORT = 8000

# Serial communication parameters
ARDUINO_SERIAL_PORT = "/dev/ttyAMA0"
ARDUINO_SERIAL_BAUD = 115200

# Token required for receiving data from socket
EOF = "\n"

# Parameters for starting sky station server
MAX_CLIENTS = 5
MAX_SOCKET_DATA_RECV_LEN = 1024

# Arduino serial connection object
arduino = None

# Ground station sokcet object
ground_station = None

# Tokens for parsing the received data from ground station
PITCH_TOKEN = "P="
ROLL_TOKEN = "&R="
YAW_TOKEN = "&Y="
THROTTLE_TOKEN = "&T="
BUTTON_TOKEN = "&B="

# This function parses the raw socket data based on the token
def GetTokenValue(message,token):
    split_list = message.split(token)
    if(len(split_list) == 2):
        value_list = split_list[1].split("&")
        return value_list[0], True

    return "", False

# This function initalizes the serial interface with arduino
def InitalizeSerialInterface():
    try:
        print "Starting Serial Interface"
        arduino = serial.Serial(ARDUINO_SERIAL_PORT,ARDUINO_SERIAL_BAUD)
    except:
        print "Unable to Start Serial Interface"
        sys.exit()

# This function waits for the ground station to come alive
def WaitForGroundStation():
    print 'Starting Eagle Eye Sky Station...'
    eagle_eye = socket.socket()
    eagle_eye.bind(EAGLE_EYE_DRONE_IP, EAGLE_EYE_DRONE_PORT)
    eagle_eye.listen(MAX_CLIENTS)
    print 'Listening on Port --> ', EAGLE_EYE_DRONE_PORT
    ground_station, addr = eagle_eye.accept()
    return ground_station

# This function receives data from the ground station socket
def ReceiveData():
    socket_data = ""
    while(True):
        try:
            socket_data += ground_station.recv(MAX_SOCKET_DATA_RECV_LEN)
            if(EOF in socket_data):
                break
        except:
            return "", False
    if not data:
        return "", False

    return socket_data, True

# This function cleans up the socket and serial connections on exit
def CleanUp():
    if(ground_station):
        ground_station.close()
    if(arduino):
        arduino.close()

# This function parses the raw ground station data and extracts
# pitch, roll, yaw, throttle, and button information
def ParseData(raw_data):
    pitch, result = GetTokenValue(raw_data, PITCH_TOKEN)
    if(not result):
        return "", "", "", "", "", False

    roll, result = GetTokenValue(raw_data, ROLLTOKEN)
    if(not result):
        return "", "", "", "", "", False

    yaw, result = GetTokenValue(raw_data, YAW_TOKEN)
    if(not result):
        return "", "", "", "", "", False

    throttle, result = GetTokenValue(raw_data, THROTTLE_TOKEN)
    if(not result):
        return "", "", "", "", "", False

    buttons, result = GetTokenValue(raw_data, BUTTON_TOKEN)
    if(not result):
        return "", "", "", "", "", False
    else:
        # To reduce load on arduino, the status of only 7 buttons is
        # sent, this is sent as a byte and the value of that byte cannot
        # be 1 as this value is chosen as the header. So we have 7 bits 
        # each bit represent a button on the joystick
        buttons = str(buttons[:7], 2) * 2

    return pitch, roll, yaw, throttle, buttons, True

# This function is called if the user exits the program
def ExitSignalHandler(signum, frame):
    CleanUp()

# Call the ExitSignalHandler function if the user presses Ctrl + Z
signal.signal(signal.SIGTSTP, ExitSignalHandler)

# This function receives data from the ground station, parses it and
# sends it to the arduino
def SendDataToArduino():
    while(True):
        raw_data, result = ReceiveData()
        if(result):
            pitch, roll, yaw, throttle, buttons, result = ParseData(raw_data)
            if(result):
                print "P = {} R = {} Y = {} T = {} B = {}".format(pitch, roll, yaw, throttle, buttons)
                ser.write(struct.pack('>B', 1))
                ser.write(struct.pack('>H', int(pitch)))
                ser.write(struct.pack('>H', int(roll)))
                ser.write(struct.pack('>H', int(yaw)))
                ser.write(struct.pack('>H', int(throttle)))
                ser.write(struct.pack('>B', int(buttons)))

# Main
if __name__ == '__main__':
    try:
        # Start the serial connection with arduino
        InitalizeSerialInterface()

        # Wait for the ground station, this call will block
        # until the ground station tries to connect.
        WaitForGroundStation()

        # This function receives data from the socket, changes its
        # format and sends it to the arduino.
        SendDataToArduino()

    except:
        print "Something went wrong..."

    finally:
        CleanUp()


