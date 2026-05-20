from gpiozero import Device,PWMLED,Button
from time import sleep
from ultralytics import YOLO
import cv2
import numpy as np
from pupil_apriltags import Detector
import time

from smbus2 import SMBus


START_TIME = None


PARKING_TAG_ID = 2

model = YOLO("best_ncnn_model",task="detect")

CONFIDENCE_LEVEL = 0.7
MATCH_LEN_SEC = 175
ALIGN_AREA_W = 20

at_detector = Detector(
        families="tag36h11",
        nthreads=4,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
)


IW = 425
IH = 240
    
MOTRF = 20
MOTRB = 21
MOTLF = 16
MOTLB = 12
SW = 6

motlf = PWMLED(MOTLF)
motlb = PWMLED(MOTLB)
motrf = PWMLED(MOTRF)
motrb = PWMLED(MOTRB)
sw = Button(SW)

motlf.frequency = (3333);
motlb.frequency = (3333);
motrf.frequency = (3333);
motrb.frequency = (3333);


def how_long_till_end():
    return MATCH_LEN_SEC - (time.time() - START_TIME)


def mot_left_turn(damn):
    if damn >= 0:
        motlf.value = damn;
        motlb.off();
    else:
        motlf.off();
        motlb.value = -damn;

def go_straight(speed):
    mot_right_turn(speed)
    mot_left_turn(speed)
    
# ~ def go_left(speed,leftness):
    # ~ mot_right_turn(speed)
    # ~ mot_left_turn(-speed)

def go_left(speed):
    mot_right_turn(speed)
    mot_left_turn(-speed)

def go_right():
    mot_left_turn(-speed)
    mot_right_turn(speed)

def mot_right_turn(damn):
        if damn > 0:
                motrf.value = damn;
                motrb.off();
        else:
                motrf.off();
                motrb.value = -damn;

def activate_collection_spinner():
    motspin = PWMLED(13)
    motspin.value = 0.2
    return

def switch_is_pressed():
    return True
    return sw.is_pressed
    

feature_params = dict(maxCorners=1,qualityLevel=.6,minDistance=25,blockSize=9)
def should_i_quit():
    if not switch_is_pressed():
        return True
    if how_long_till_end() < 0: 
        return True

    return False
def camera_check():
    #print("SHOW IMAGE!")
    ret, img = cap.read();
    #img2 = img.img_to_array(img, dtype='uint8')
    #ball = cv2.goodFeaturesToTrack(img, **feature_params)
    results = model(img, stream=True,verbose=False)
    
    y = False
    
    for result in results:
        for box in result.boxes:
            if box.conf[0] > CONFIDENCE_LEVEL:
                y = True
    
    if y:
        print("WOOOO", end="", flush=True)
    cv2.imshow("Image", img);
    cv2.waitKey(1)
    return y;


cap = cv2.VideoCapture(0,cv2.CAP_V4L);
cap.set(cv2.CAP_PROP_FRAME_WIDTH, IW)  
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,IH )
cap.set(cv2.CAP_PROP_FPS, 60)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)



bus = SMBus(1)

def to_signed16(n):
    n = n & 0xffff
    return n | (-(n & 0x8000))



def compass_wait_data():
    while (bus.read_byte_data(0x2c, 0x09) & 1) == 0:
        sleep(0) 

def init_compass():
    bus.write_byte_data(0x2c, 0xb, 0x80)
    sleep(0.1)
    
    bus.write_byte_data(0x2c, 0x29, 0x6)
    bus.write_byte_data(0x2c, 0xb, 0x00)    
    bus.write_byte_data(0x2c, 0xa, 0xc3)

    compass_wait_data()

    fa = bus.read_word_data(0x2c, 0x1)
    fb = bus.read_word_data(0x2c, 0x3)
    fc = bus.read_word_data(0x2c, 0x5)
    
    bus.write_byte_data(0x2c, 0xb, 0x40)
    
    sleep(0.2)
    
    compass_wait_data()

    sa = bus.read_word_data(0x2c, 0x1)
    sb = bus.read_word_data(0x2c, 0x3)
    sc = bus.read_word_data(0x2c, 0x5)
    
    global dx, dy, dz
    dx = to_signed16(fa) - to_signed16(sa)
    dy = to_signed16(sb) - to_signed16(fb)
    dz = to_signed16(sc) - to_signed16(fc)



    while (bus.read_byte_data(0x2c, 0xb) & 0x40) != 0:
        sleep(0) 
    bus.write_byte_data(0x2c, 0x29, 0x6)
    bus.write_byte_data(0x2c, 0xb, 0x00)
    bus.write_byte_data(0x2c, 0xa, 0xcd)    


def met_parking_criteria(colcnt):
    if colcnt > 8:
        return True
    return False

def compass_get_range():
    rngbit= (bus.read_byte_data(0x2c,0xb) >> 2)  & 0b11
    match(rngbit):
        case 0:
            return 1000;
        case 1:
            return 2500;
        case 2:
            return 3750;
        case 3:
            return 15000;
    
from math import atan2, pi

_ctx = None
_cty = None
_cbx = None
_cby = None

def _update_cal(x, y):
    global _ctx, _cty, _cbx, _cby
    if _ctx is None:
        _ctx = _cbx = x
        _cty = _cby = y
    else:
        if x < _ctx: _ctx = x
        if x > _cbx: _cbx = x
        if y < _cty: _cty = y
        if y > _cby: _cby = y

def get_heading():
    compass_wait_data()
    raw_x = to_signed16(bus.read_word_data(0x2c, 0x1))
    raw_y = to_signed16(bus.read_word_data(0x2c, 0x3))

    _update_cal(raw_x, raw_y)
    cal_x = raw_x - (_ctx + _cbx) / 2.0
    cal_y = raw_y - (_cty + _cby) / 2.0

    heading = atan2(cal_y, cal_x) * 180.0 / pi
    if heading < 0:
        heading += 360.0
    return heading


def get_targ_tag():
    ret, rawimg = cap.read()
    img = cv2.cvtColor(rawimg, cv2.COLOR_BGR2GRAY)

    
    tags  = at_detector.detect(img)
    
    cv2.imshow("Image", img);
    cv2.waitKey(1)

    ret = None

    for i in tags:
        if i.tag_id == PARKING_TAG_ID:
            ret = i
            break

    return ret

def backwards_and_180():
    
    go_straight(-0.4)
    sleep(0.4)
    go_straight(0);
    
    curr_heading = get_heading();
    
    targ_heading = (curr_heading + 180) % 360;
    
    while(get_heading() < (targ_heading - TARG_HEADING_TOLERANCE) or get_heading() > (targ_heading + TARG_HEADING_TOLERANCE)):
        go_left(0.5)
        sleep(0.1)
        go_left(0)
        sleep(0.1)
    return
    
def activate_unload():
    return

def align_to_tag():
    if how_long_till_end() < 20:
        return
    aligned = False
    while(not aligned):
        tag = get_targ_tag()
        if not aligned:
            go_straight(-1)
            sleep(0.1)
            go_left(1);
            sleep(0.1)
        else:
            if tag.center > (IW + ALIGN_AREA_W):
                go_right(0.9)
                sleep(0.1)
            elif tag.center < (IW - ALIGN_AREA_W):
                go_left(0.9)
                sleep(0.1)
            else:
                aligned = True
                

    

def park_and_unload():
    
    for i in range(0, 4):
        align_to_tag()
        go_straight(0.5);
        wait(2)
        go_straight(-0.5)
        wait(0.5)
    backwards_and_180()
    activate_unload()


def wait_start():
    #gpio detection logic goes here
    start = False
    while not start:
        if switch_is_pressed:
            start = True 
activate_collection_spinner();
state = "scan"

counter = 0;
init_compass();

camera_check()
wait_start()


START_TIME = time.time()





while not should_i_quit():
    print(get_heading());

    found_balls = camera_check();
    if met_parking_criteria(counter):
        park_and_unload();  
        counter = 0


    if not found_balls:
        print("Turning left...")
        
        go_straight(-0.5);
        sleep(0.1)
        go_left(0.5);
        sleep(0.1)
    else:
        found_balls = camera_check(); #confirmation
        print ("Straight on!")
        go_straight(0.5);
        sleep(2.5)
        counter += 1;
    

