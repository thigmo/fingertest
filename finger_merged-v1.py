# V1
# Merged code bases for Steve's work on motor control and Dave's work on face detect
# 4/16/25

import cv2 as cv
import numpy as np
import argparse
from picamera2 import Picamera2, Preview
#import time
import math
from pynput import keyboard
import random

from time import sleep, time
import spidev
from math import sqrt
import json

#sbr>>>>>
# raspberry pi SPI settings
bus = 0 # only one available
device = 0 # Select which CE pin to use, here CE0

# TMC5160 SilentStepStick Settings
DRIVER_REGS_FN = "TMC5160_regs.json"
SPI_MAX_HZ = 100000 # specs says max is 4MHz assuming min on-chip oscillator freq
SPI_MODE = 3
RSENSE = 0.075
TMC5160_VFS = 0.325 # Volts full scale across sense resistor = Vsrt in datasheet

# Stepper settings
#MOTOR_IRMS = 0.350 # Adafruit Nema-17 motor 350mA
MOTOR_IRMS = 1.3 # StepperOnline Nema-17 motor 1.5A

class Register:
    def __init__(self, name, init_list):
        self.name = name
        self.address = int(init_list[0],16) #convert hex string to json to int
        self.rw = init_list[1]
        self.numbits = init_list[2]
        # register value, put reset defaults in json file, so that you can OR write only regs
        try:
            self.value = int(init_list[3])
        except ValueError:
            self.value = int(init_list[3],16)
        
class Setting:
    registers = None
    def __init__(self, name, init_list):
        self.name = name
        self.register = init_list[0]
        self.numbits = init_list[1]
        self.lsb_pos = init_list[2]
        

def read(address):
    if 0 < address < 0x74 : # valid address range
        #address &= ~(1 << 7)  # clear read bit b7 but you know this since < 0x80
        msg = [address] 
        data = bytearray(4)
        msg = msg + [x for x in data]
        
        spi.writebytes(msg) # send read command
        result = spi.readbytes(5)
        status = result[0]
        data = int.from_bytes(result[1:]) # defaults to big-endian
        return status, data
    else:
        raise ValueError("Read address is outside of valid range")

def write(address, data):
    if 0 <= address < 0x74: # valid address range, this might be different on different chip
        address |= 0x80  # set the write bit
        msg = bytearray([address])
        data = data & 0xffffffff # limit to 4 bytes
        msg = msg + data.to_bytes(4)
        spi.writebytes(msg)
    else:
        raise ValueError("write address is outside of valid range")


def set_val(setting_obj, value):
    if isinstance(setting_obj, Register):
        write(setting_obj.address, value)
    elif isinstance(setting_obj, Setting):
        # get value of appropriate register
        register = registers[setting_obj.register]
        # create the mask for bit posns
        if setting_obj.numbits + setting_obj.lsb_pos <= 32: # sanity check on value
            notmask = ~((2**setting_obj.numbits-1) << setting_obj.lsb_pos)
        else:
            raise ValueError('Requested set value is greater than 32 bits')
#     	print(f'{"original setting:":<25}{register.value:032b}')
        # clear out bits where new value will go
        register.value &= notmask
#     	print(f'{"cleared setting:":<25}{register.value:032b}')
        # set the new value
        if value < 2**setting_obj.numbits:
            register.value |= value << setting_obj.lsb_pos
        else:
            raise ValueError('Requested set value is greater than bit depth!')
#     	print(f'{"new setting:":<25}{register.value:032b}')
        # write the register
        if register.rw in ['rw','w']: # todo handle 'rwc'
            write(register.address, register.value)
        else:
            raise ValueError('Requested setting is not in a writeable register')
    
def get(get_obj):
    # note this reads the register from the device not just the value in memory
    if isinstance(get_obj, Register):
        status, data = read(get_obj.address)

    elif isinstance(get_obj, Setting):
        register = registers[get_obj.register]
#         if register.rw not in ['rw', 'r', 'rwc']:  # move this to read
#             raise ValueError("Setting to get is not in a readable register")
        status, data = read(register.address)
        register.value = data # update local copy
        # shift to the requested value
        data = data >> get_obj.lsb_pos
        # mask off any upper bits
        mask = (2**get_obj.numbits - 1)
        data &= mask
    else:
        raise ValueError('Get object is not a register or setting!')
    
    get_obj.value = data # update copy in memory, 
    return data
        
def calcCsScaler(A):
    # calculate initial values for CS and GLOBALSCALER, prior to autotuning
    cs = 32 # start with max value + 1
    # globalscaler must be 31 < globalscaler <= 255, 0 (write 256) for full scale
    #		255 for next highest, minimum recommended: 128
    globalscaler = 0 
    while globalscaler < 128:
        cs -= 1
        globalscaler = A * 256 * 32 * RSENSE * sqrt(2)/((cs + 1) * TMC5160_VFS)
        
    if globalscaler > 255:
        globalscaler = 256
    return cs, globalscaler

def load_json():
    try:
        with open(DRIVER_REGS_FN, 'r') as read_json:
            reg_dicts = json.load(read_json)
            # create register objects
            for key in reg_dicts["registers"].keys():
                registers[key] = Register(key, reg_dicts["registers"][key])
            # create setting objects
            for key in reg_dicts["settings"].keys():
                settings[key] = Setting(key, reg_dicts["settings"][key])
            # fill class variable
            #Setting.registers = registers    
    except IOError:
        print("Driver board's register json file not found")
        raise

    print("Read registers from json file: ")
    for k,v in registers.items():
        print(f'{k:<15} 0x{v.address:>02x}, {v.rw:>5}, {v.numbits}, 0x{v.value:>02x}')
        
    print("Read settings from json file: ")
    for k,v in settings.items():
        print(f'{k}: {v.register}, {v.numbits}, {v.lsb_pos}')
        
def begin():
    ''' even though these are default reset values, you want to reset them because
        you will inevitably run begin() more than once, without powering down the
        TMC board, so you need to return the TMC board to its reset state.  Make
        sure any register you plan to manipulate is in this list
        '''
    for send_reg_name in ["GCONF", "DRV_CONF","CHOPCONF", "PWMCONF", "IHOLD_IRUN"]:
        send_reg = registers[send_reg_name]
        print(f'writing register: {send_reg_name}, {send_reg.address:#0x}, {send_reg.value:#0x}')
        write(send_reg.address,send_reg.value)
    set_val(settings["TOFF"], 8)
    set_val(settings["TBL"], 1)
    write(registers["XTARGET"].address, 0)
    write(registers["XACTUAL"].address, 0)
    
def rms_current(A):
    # calculate cs for globalscaler > 128
    cs, globalscaler = calcCsScaler(A)
    print(f'rms_current({A}) setting GLOBALSCALER to {int(globalscaler)} and IRUN to {int(cs)}')
    write(registers['GLOBALSCALER'].address, int(globalscaler)) # note this is a register not a 'setting'
    set_val(settings["IRUN"], int(cs))
    set_val(settings["IHOLD"], 3)

def autotune():
    high_cs = 15 # move this to constant at beginning
    autotune1_time = .2
    #Phase 1 High current at standstill, need to take at least one step to turn on IRUN
    xactual = get(registers["XACTUAL"]) # should be 0, in case autotune after xactual has changed
    set_val(settings["IRUN"],high_cs) # arduino code had set this to 25 but with the adafruit motor this might be too much
    write(registers['A1'].address, 500)
    write(registers['AMAX'].address, 500)
    write(registers['V1'].address, 1000)
    write(registers['VMAX'].address, 2000)
    write(registers['VSTOP'].address, 10)
    write(registers['D1'].address, 200)
    write(registers['DMAX'].address, 400)
    write(registers['RAMPMODE'].address, 0)
    write(registers['XTARGET'].address, xactual + 1) # take one step
    starttime = time()
    done = False
    fullcurrent = 0
    while(not done): # need at least 150ms, check that CS_ACTUAL reaches full value
        deltatime = time() - starttime
        if deltatime > autotune1_time:
            done = True
            print(f'Autotune phase 1 full current: {fullcurrent}, target: {high_cs}') 
        cs_actual = get(settings['CS_ACTUAL'])
        if fullcurrent < cs_actual:
            fullcurrent = cs_actual

    # Autotuning Phase 2: Move the motor at constant velocity at least 400 full steps
    xactual = get(registers["XACTUAL"])
    write(registers['A1'].address, 65000)
    write(registers['AMAX'].address, 65000)
    write(registers['V1'].address, 32000)
    write(registers['VMAX'].address, 100000)
    write(registers['VSTOP'].address, 10)
    write(registers['D1'].address, 20000)
    write(registers['DMAX'].address, 20000)
    write(registers['XTARGET'].address, 256 * 800)
    # check for PWM_AUTOSCALE to approach 0 during spin
    done = False
    min_mon = 2**8
#     sleep(2) # appears to finish prior to motor spinning
    curr_xactual = xactual
    while(not done):
        while(curr_xactual < xactual + 256 * 400):
            #wait until motor half way to target
            curr_xactual = get(registers["XACTUAL"])
            
        monitor = get(settings["PWM_SCALE_AUTO"]) # this is a 9 bit signed number
        if monitor & 0x100 == 0x100:
            # find the absolute value, i.e. ignore sign, for 2s complement you'd need to * -1
            monitor = (~monitor + 1) & 0xFF 
        else:
            monitor = (monitor & 0xff)
        if monitor < min_mon:
            min_mon = monitor
        if monitor == 0: # indicates sucessful autotune, provide for a timeout fail
            done = True
            print('Autotuning results-----------------------------')
            print(f'PWM_SCALE_AUTO minimum during spin: {min_mon}')
            print(f'PWM_OFS_AUTO: {get(settings["PWM_OFS_AUTO"])}, PWM_GRAD_AUTO: {get(settings["PWM_GRAD_AUTO"])}')
    
                 



contourMinimum = 500
# video capture settings - OpenCV capture, not used here, use picam capture instead 
picam2 = Picamera2()
camWidth = 640
camHeight = 480
camera_config = picam2.create_video_configuration(main={"format":'XRGB8888', "size":(camWidth,camHeight)})
picam2.configure(camera_config)
picam2.start()
sleep(1)
controls = picam2.camera_controls
# picam2.controls.AeExposureMode = 2
# picam2.controls.ExposureTime = 10000
picam2.controls.ExposureValue = 0.0
#turn off auto gain exposure - but want it on for this project
picam2.controls.AeEnable = True
#Exposure Mode = 0 -3, Normal, short, long
picam2.controls.AeExposureMode = 3

controls = picam2.camera_controls

print(controls)
#picam2.controls.AnalogueGain
#controls2 = picam2.set_controls({"ExposureTime":10000,"AnalogueGain":10.0})

 # downloaded and moved the classifier file to data/
 # from https://github.com/opencv/opencv/tree/4.x/data
parser = argparse.ArgumentParser(description='Code for Cascade Classifier Tutorial')
parser.add_argument('--face_cascade', help='Path to face cascade', default='data/haarcascade_frontalface_alt.xml')
parser.add_argument('--camera',help='Camera divide number.', type=int, default =0)
args = parser.parse_args()
# cascade classifier sttings
face_cascade_name = args.face_cascade
face_cascade = cv.CascadeClassifier()

verbose = True

if not face_cascade.load(cv.samples.findFile(face_cascade_name)):
    print('error loading eyes cascade')
    exit()
    
trackedFace = None
showFrame = True
showMovement = showGray =False
frameWindowDestroyed = False
grayWindowDestroyed = movementWindowDestroyed = True
showMovement = showGray = False
matchDistance = 100
lifeLength = 10
camFov = 53.5
flipCam = True


def random_color():
    randColor = (random.randint(0,255),random.randint(0,255),random.randint(0,255))
    return randColor

def onpress(key):
    pass
   

def onrelease(key):
    global showFrame,showMovement,showGray
    
    try:
#         print(f'key {key.char} released in onrelease' , showFrame, showGray)
        if(key.char == 'f'):
            showFrame = not showFrame
            if not showFrame:
                 cv.destroyWindow('frame')
        if(key.char == 'm'):
            showMovement = not showMovement
            if not showMovement:
                cv.destroyWindow('movement')
        if(key.char == 'g'):
            showGray = not showGray
            if not showGray:
                cv.destroyWindow('gray')
    except AttributeError:
        
        print(f'special key {key} pressed')
        #stop listening if escape
        if(key == keyboard.Key.esc):
           return False
        raise



def drawFaces(faces,frame):
    for face in faces:
        center = ((face[0]+face[2])//2,(face[1]+face[3])//2)
        frame = cv.rectangle(frame,(face[0],face[1]),(face[0]+face[2],face[1]+face[3]),(255,0,0),3)

def drawFace(face,frame):
    center = (face['x']+face['w']//2,face['y']+face['h']//2)
    frameColor = face['color']
    frame = cv.rectangle(frame,(face['x'],face['y']),(face['x']+face['w'],face['y']+face['h']),frameColor,3)
    
    cv.putText(frame,str(face['life']),(face['x']+10,face['y']+10),cv.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),2,cv.LINE_AA,False)
    
#     faceCenter = face['x']+face['w']/2
#     xDist = camWidth/2 - faceCenter
#     offsetPercent = xDist/(camWidth*.5)
#     angle = camFov/2*offsetPercent
#     if verbose:
#         print(angle)



def detectFaces(moveframe,frame):
    global trackedFace
    
    currentFaces = face_cascade.detectMultiScale(moveframe)
    
    # try iterating through the current faces and see if one is close enough to the last tracked position
    
    #0 zero faces
    if len(currentFaces) == 0:
        if trackedFace is not None:
            trackedFace['life'] = trackedFace['life'] - 1
 
    #1 - only one face
    if len(currentFaces)==1:
        currentFace = currentFaces[0]
        if trackedFace is None:
            # updates face pos
            trackedFace = dict(x = currentFace[0], y = currentFace[1], w = currentFace[2], h= currentFace[3], life = lifeLength, color = random_color())
        else:
            # calc the dist and decide if it's the same face
            distance = math.sqrt(((currentFace[0]-trackedFace['x'])**2+(currentFace[1] -trackedFace['y'])**2))
            if distance< matchDistance:
                    #same face update position
                    trackedFace = dict(x = currentFace[0], y = currentFace[1], w = currentFace[2], h= currentFace[3], life =trackedFace['life'], color = trackedFace['color'])
                    trackedFace['life'] = lifeLength
            else:
                # different face, decrease life
                trackedFace['life'] = trackedFace['life'] - 1
   
    
            
    #2 -  more than one face - find closest face to tracked
    elif len(currentFaces)>1:
        if trackedFace is not None:
            # search for closest face
            for face in currentFaces:
                distance = math.sqrt(((face[0]-trackedFace['x'])**2+(face[1] -trackedFace['y'])**2))
                
                if distance< matchDistance:
                    #same face update position
                    trackedFace = dict(x = face[0], y = face[1], w = face[2], h= face[3], life =lifeLength, color= trackedFace['color'])
                    
        else:
            #pick a random face
            randFace = currentFaces[random.randint(0,len(currentFaces)-1)]
            trackedFace = dict(x = randFace[0], y = randFace[1], w = randFace[2], h= randFace[3], life =lifeLength,color = random_color())
            
     # kill off dead faces
    if trackedFace is not None and trackedFace['life'] <0:
        trackedFace = None
   
    drawFaces(currentFaces,frame)
    if trackedFace is not None:
        drawFace(trackedFace,frame);
        
        #move angle calc here
    


    cv.putText(frame,str(len(currentFaces)),(10,10),cv.FONT_HERSHEY_SIMPLEX,1.0,(255,255,255),2,cv.LINE_AA,False)
                
                
#sbr>>>>>>>>>>>>>>>>>>                
registers = {}
settings = {}    
regDict = {}

spi = spidev.SpiDev()
spi.open(bus, device)
spi.max_speed_hz =  SPI_MAX_HZ
spi.mode = SPI_MODE
load_json()


listener = keyboard.Listener(on_press = onpress, on_release = onrelease)
listener.start()

# Bg subtractor  - KNN parameters - history, dist2threshold, detectshadows
backSub = cv.createBackgroundSubtractorKNN(30,400,False)
#backSub = cv.createBackgroundSubtractorKNN(500,400,False)


try:
    begin()
    rms_current(MOTOR_IRMS) 
    set_val(settings["en_pwm_mode"],1)
    set_val(settings["pwm_autoscale"],1)
    set_val(settings["pwm_autograd"],1)
    set_val(settings["TOFF"],5) # any number other than 0 turns on the driver, should already be on
    autotune()
    
    while(True):
        
        #der>>>>>>>
        #picam2 capture
        frame = picam2.capture_array()
        
        if flipCam:
            frame = cv.flip(frame,-1)
        if frame is None:
            break
        #bg subtraction
        fgMask = backSub.apply(frame)
        morphKernel = np.ones((3,3),np.uint8)
        dilated = cv.dilate(fgMask,morphKernel,iterations = 1)
        
        # find contours
        retrMethod = cv.RETR_EXTERNAL
        conApprox = cv.CHAIN_APPROX_SIMPLE
        contours, hierarchy = cv.findContours(dilated,retrMethod,conApprox)
        # draw movment into a separate frame for face detection
        movement_frame = np.zeros((480,640),np.uint8)

        for i in range(len(contours)):
            contour = contours[i]
            
            if cv.contourArea(contour)>contourMinimum:
                x,y,w,h = cv.boundingRect(contour)
                frame_gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
                #Contrast Limited Adaptive Histogram
                clahe = cv.createCLAHE(clipLimit = 2.0, tileGridSize = (8,8))
                frame_clahe = clahe.apply(frame_gray)
                #copy contents of large contours to movement buffer
                movement_frame[y:y+h,x:x+w] = frame_clahe[y:y+h,x:x+w]
      
        detectFaces(movement_frame,frame)
            
        if showFrame:
            cv.imshow('frame',frame)
            cv.imshow('frame_clahe',frame_clahe)
        if showGray:
            cv.imshow('gray', frame_gray)
        #cv.imshow('clahe', frame_clahe)
        if showMovement:
            cv.imshow('movement',movement_frame)
        
        key = cv.waitKey(1)
        if key==27:
            raise KeyError
        
        # get data here
        if trackedFace is not None:
            faceCenter = trackedFace['x']+trackedFace['w']/2
            xDist = camWidth/2 - faceCenter
            offsetPercent = xDist/(camWidth*.5)
            angle = camFov/2*offsetPercent
            print(angle)
            point_angle = int(200*256*angle/360)
            print(f'point_angle: {point_angle}')
            set_val(registers['XTARGET'], -point_angle)       
        #sleep(8)
except:
    
    picam2.stop_preview()
    cv.destroyAllWindows()

    # on any error, turn off driver
    begin() # restore defaults to motor board
    set_val(settings["TOFF"],0)
    sleep(1)
    spi.close()
    raise
    
# while True:
#     #picam2 capture
#     frame = picam2.capture_array()
#     
#     if flipCam:
#         frame = cv.flip(frame,-1)
#     if frame is None:
#         break
#     #bg subtraction
#     fgMask = backSub.apply(frame)
#     morphKernel = np.ones((3,3),np.uint8)
#     dilated = cv.dilate(fgMask,morphKernel,iterations = 1)
#     
#     # find contours
#     retrMethod = cv.RETR_EXTERNAL
#     conApprox = cv.CHAIN_APPROX_SIMPLE
#     contours, hierarchy = cv.findContours(dilated,retrMethod,conApprox)
#     # draw movment into a separate frame for face detection
#     movement_frame = np.zeros((480,640),np.uint8)
# 
#     for i in range(len(contours)):
#         contour = contours[i]
#         
#         if cv.contourArea(contour)>contourMinimum:
#             x,y,w,h = cv.boundingRect(contour)
#             frame_gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
#             #Contrast Limited Adaptive Histogram
#             clahe = cv.createCLAHE(clipLimit = 2.0, tileGridSize = (8,8))
#             frame_clahe = clahe.apply(frame_gray)
#             #copy contents of large contours to movement buffer
#             movement_frame[y:y+h,x:x+w] = frame_clahe[y:y+h,x:x+w]
#   
#     detectFaces(movement_frame,frame)
#         
#     if showFrame:
#         cv.imshow('frame',frame)
#     if showGray:
#         cv.imshow('gray', frame_gray)
#     #cv.imshow('clahe', frame_clahe)
#     if showMovement:
#         cv.imshow('movement',movement_frame)
#     
#     key = cv.waitKey(1)
   

    
# 
# picam2.stop_preview()
# cv.destroyAllWindows()
# exit()

    

