# Install the pyautogui and pyserial libraries
import serial
import time
import pyautogui
import ctypes
import string
import math
#import pynput

# Below are the strings used for cursor movements
# taken from the pyautogui reference:
# https://pyautogui.readthedocs.io/en/latest/keyboard.html
#
# For mouse commands use these:
# https://pyautogui.readthedocs.io/en/latest/mouse.html


pyautogui.FAILSAFE = False

SendInput = ctypes.windll.user32.SendInput

# C struct redefinitions 
PUL = ctypes.POINTER(ctypes.c_ulong)
class KeyBdInput(ctypes.Structure):
    _fields_ = [("wVk", ctypes.c_ushort),
                ("wScan", ctypes.c_ushort),
                ("dwFlags", ctypes.c_ulong),
                ("time", ctypes.c_ulong),
                ("dwExtraInfo", PUL)]

class HardwareInput(ctypes.Structure):
    _fields_ = [("uMsg", ctypes.c_ulong),
                ("wParamL", ctypes.c_short),
                ("wParamH", ctypes.c_ushort)]

class MouseInput(ctypes.Structure):
    _fields_ = [("dx", ctypes.c_long),
                ("dy", ctypes.c_long),
                ("mouseData", ctypes.c_ulong),
                ("dwFlags", ctypes.c_ulong),
                ("time",ctypes.c_ulong),
                ("dwExtraInfo", PUL)]

class Input_I(ctypes.Union):
    _fields_ = [("ki", KeyBdInput),
                 ("mi", MouseInput),
                 ("hi", HardwareInput)]

class Input(ctypes.Structure):
    _fields_ = [("type", ctypes.c_ulong),
                ("ii", Input_I)]

# Actuals Functions

def PressKey(hexKeyCode):
    extra = ctypes.c_ulong(0)
    ii_ = Input_I()
    ii_.ki = KeyBdInput( 0, hexKeyCode, 0x0008, 0, ctypes.pointer(extra) )
    x = Input( ctypes.c_ulong(1), ii_ )
    ctypes.windll.user32.SendInput(1, ctypes.pointer(x), ctypes.sizeof(x))

def ReleaseKey(hexKeyCode):
    extra = ctypes.c_ulong(0)
    ii_ = Input_I()
    ii_.ki = KeyBdInput( 0, hexKeyCode, 0x0008 | 0x0002, 0, ctypes.pointer(extra) )
    x = Input( ctypes.c_ulong(1), ii_ )
    ctypes.windll.user32.SendInput(1, ctypes.pointer(x), ctypes.sizeof(x))






str_up="up"
str_down="down"
str_left="left"
str_right="right"

key_Forward=0x14
key_Left=0x21
key_Right=0x23
key_Backward=0x22

key_Jump=0x24
key_Crouch=0x1F

w_move=0;
a_move=0;
s_move=0;
d_move=0;
inputKeys = True;


def move(move_string):
        w_move = move_string[0]
        a_move = move_string[1]
        s_move = move_string[2]
        d_move = move_string[3]

        moveString = "W: " + w_move + " A: " + a_move + " S: " + s_move + " D: " + d_move
        #print(moveString)

#'''

        if(inputKeys):
            if w_move == '1':
                PressKey(key_Forward)
            else:
                ReleaseKey(key_Forward)
            if a_move == '1':
                PressKey(key_Left)
            else:
                ReleaseKey(key_Left)
            if s_move == '1':
                PressKey(key_Backward)
            else:
                ReleaseKey(key_Backward)
            if d_move == '1':
                PressKey(key_Right)
            else:
                ReleaseKey(key_Right)
#'''

# Make sure the baudrate matches that of the Arduino
# Serial.begin(xxxx)


print("Starting Program...")
ArduinoSerial = serial.Serial(port='COM5', baudrate=500000)
time.sleep(2)

while 1:
        incoming = str (ArduinoSerial.readline()) #Read one byte from serial port
        #print(incoming)
        if(incoming[2] == 'Q'):
                move_string = incoming[12:16]
                move(move_string)
                #print(incoming[33]);
                if(incoming[33] == '1'):
                    #print("Jump");
                    PressKey(key_Jump)
                else:
                    ReleaseKey(key_Jump)
                vision_string = incoming[43:]
                [x,y] = vision_string.split();
                if(y[6] == '\r'):
                    y = y[0:6]

                else:
                    y = y[0:5]

                if(incoming[25] == '1'):
                    pyautogui.mouseDown();
                    #print("Click");
                else:
                    pyautogui.mouseUp();
                    #print("Don't Click");    

                
                #print(x)
                #print(y)
                #print(vision_string)

                x_current, y_current = pyautogui.position()
                scalingFactor = 100;
                xScalingFactor = 12;
                yScalingFactor = 3;
                x = float(x);
                x = x * abs(x);
                y = float(y);
                y = y * abs(y);
                pyautogui.moveTo(-(float(x) * scalingFactor * xScalingFactor) + x_current, y_current + (float(y) * yScalingFactor * scalingFactor))

                #for i in range(3):
                #    pyautogui.moveTo(-(float(x) * scalingFactor * 1/(3 - i)) + x_current, y_current + (float(y) * scalingFactor * 1/(3-i)))
                #    time.sleep(0.001)


                    

        ArduinoSerial.reset_input_buffer()  # Flush the serial port

        #mouse = pynput.mouse.Controller()
        #mouse.move(10, 10)


 
