from gpiozero import Device,PWMLED
from time import sleep
from ultralytics import YOLO
import cv2
import numpy as np
from pupil_apriltags import Detector
import time


START_TIME = None


PARKING_TAG_ID = 2

model = YOLO("best_ncnn_model",task="detect")

CONFIDENCE_LEVEL = 0.7
MATCH_LEN_SEC = 180
ALIGN_AREA_W = 20

IW = 427
IH = 240
    
MOTLF = 21
MOTLB = 20
MOTRF = 16
MOTRB = 12

motlf = PWMLED(MOTLF)
motlb = PWMLED(MOTLB)
motrf = PWMLED(MOTRF)
motrb = PWMLED(MOTRB)

motlf.frequency = (3333);
motlb.frequency = (3333);
motrf.frequency = (3333);
motrb.frequency = (3333);


def how_long_till_end():
    return MATCH_LEN_SEC - time.time - START_TIME


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
    return

feature_params = dict(maxCorners=1,qualityLevel=.6,minDistance=25,blockSize=9)
def should_i_quit():
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


cap = cv2.VideoCapture(0);
cap.set(cv2.CAP_PROP_FRAME_WIDTH, IW)  
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,IH )
cap.set(cv2.CAP_PROP_FPS, 60)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)




def met_parking_criteria(colcnt):
    if colcnt > 8 or True:
        return True
    return False


def get_targ_tag():
    ret, rawimg = cap.read()
    tag = get_targ_tag(rawimg)
    img = cv2.cvtColor(rawimg, cv2.COLOR_BGR2GRAY)
    at_detector = Detector(
        families="tag36h11",
        nthreads=4,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
    )
    tags  = at_detector.detect(img)
    cv2.imshow("Image", img);
    cv2.waitKey(1)

    ret = None

    for i in tags:
        if i.tag_id == PARKING_TAG_ID:
            ret = i
            break

    return ret

def align_to_tag():
    if how_long_till_end() < 20:
        return
    aligned = False
    while(not end):
        tag = get_targ_tag(rawimg)
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
        go_straight(1);
        wait(2)
        go_straight(-1)
        wait(0.5)


def wait_start():
    #gpio detection logic goes here
    start = False
    while not start:
        start = True #FIXME: GPIO BUTTON 
activate_collection_spinner();
state = "scan"

counter = 0;

wait_start()


START_TIME = time.time()

while not should_i_quit():
    found_balls = camera_check();
    if met_parking_criteria(counter):
        park_and_unload();
        
        counter = 0


    if not found_balls:
        print("Turning left...")
        
        go_straight(-1);
        sleep(0.1)
        go_left(1);
        sleep(0.1)
    else:
        found_balls = camera_check(); #confirmation
        print ("Straight on!")
        go_straight(1);
        sleep(2.5)
        counter += 8;
    

