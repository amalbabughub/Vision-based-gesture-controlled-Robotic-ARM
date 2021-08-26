import cv2
import mediapipe as mp
import math
import pybullet as p
import pybullet_data
import utils_ur5_robotiq140
from collections import deque
import numpy as np
import time

def calculate_angle(a,b,c):
    a = np.array(a) 
    b = np.array(b) 
    c = np.array(c) 
    
    radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
    angle = np.abs(radians*180.0/np.pi)
    
    if angle >180.0:
        angle = 360-angle
        
    return angle

mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic
serverMode = p.GUI
sisbotUrdfPath = "/home/tomin/Project/robotSim/pybullet-ur5-equipped-with-robotiq-140/urdf/ur5_robotiq_140.urdf"
physicsClient = p.connect(serverMode)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeID = p.loadURDF("plane.urdf")

robotStartPos = [0,0,0]
robotStartOrn = p.getQuaternionFromEuler([0,0,0])
robotID = p.loadURDF(sisbotUrdfPath, robotStartPos, robotStartOrn,useFixedBase = True)                    
joints,controlRobotiqC2, controlJoints, mimicParentName = utils_ur5_robotiq140.setup_sisbot(p, robotID)
eefID = 7
for i in range(1,13):
    p.setJointMotorControl2(robotID, jointIndex=i,controlMode=p.POSITION_CONTROL,
    targetPosition = 0)
gripper_opening_length = 0.08
gripper_opening_angle = 0.715 - math.asin((gripper_opening_length - 0.010) / 0.1143)    
controlRobotiqC2(controlMode=p.POSITION_CONTROL, targetPosition=gripper_opening_angle)
for _ in range (100):
    p.stepSimulation ()
s_l=0;s_h=0;avg_l=0;avg_h=1;avg_i=0;avg_pl=0;avg_ph=0
loop_id=0;im_h=1;P_Co=0;wrist=[0]*2;centre=[0]*2;dis_p=0
t_tip=[0]*2;m_tip=[0]*2
angle1_p=0;angle2_p=0;angle3_p=0;angle4_p=0;an3=0;le_p=0

cap = cv2.VideoCapture(0)
with mp_holistic.Holistic(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as holistic:
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      print("Ignoring empty camera frame.")
      continue
    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
    image.flags.writeable = False
    results = holistic.process(image)
    image.flags.writeable = True
    image_height, image_width, _ = image.shape
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    angle_h=0
    
    try:

        landmarks = results.pose_landmarks.landmark

        landmarks_h = results.left_hand_landmarks.landmark
        
        wrist_h = [landmarks_h[mp_holistic.HandLandmark.WRIST.value].x,landmarks_h[mp_holistic.HandLandmark.WRIST.value].y]
        t_tip = [landmarks_h[mp_holistic.HandLandmark.THUMB_TIP.value].x,landmarks_h[mp_holistic.HandLandmark.THUMB_TIP.value].y]

        m_tip = [landmarks_h[mp_holistic.HandLandmark.MIDDLE_FINGER_TIP.value].x,landmarks_h[mp_holistic.HandLandmark.MIDDLE_FINGER_TIP.value].y]
        angle_h= round(calculate_angle(t_tip, wrist_h, m_tip),1)
        if angle_h > 30:
            dis=0.12
            print(1)
        else:
            dis=0.8
            print(2)
        r_shoulder= [landmarks[mp_holistic.PoseLandmark.RIGHT_SHOULDER.value].x,landmarks[mp_holistic.PoseLandmark.RIGHT_SHOULDER.value].y]
        l_shoulder = [landmarks[mp_holistic.PoseLandmark.LEFT_SHOULDER.value].x,landmarks[mp_holistic.PoseLandmark.LEFT_SHOULDER.value].y]
        elbow = [landmarks[mp_holistic.PoseLandmark.LEFT_ELBOW.value].x,landmarks[mp_holistic.PoseLandmark.LEFT_ELBOW.value].y]
        wrist = [landmarks[mp_holistic.PoseLandmark.LEFT_WRIST.value].x,landmarks[mp_holistic.PoseLandmark.LEFT_WRIST.value].y]
        hip = [landmarks[mp_holistic.PoseLandmark.LEFT_HIP.value].x,landmarks[mp_holistic.PoseLandmark.LEFT_HIP.value].y]
        in_hand=[landmarks[mp_holistic.PoseLandmark.LEFT_PINKY.value].x,landmarks[mp_holistic.PoseLandmark.LEFT_PINKY.value].y]

        le=math.hypot(elbow[0]*image_width-r_shoulder[0]*image_width,elbow[1]*image_height-r_shoulder[1]*image_height)
        le=round(le,1)

        angle1= round(calculate_angle(l_shoulder, elbow, wrist),1)
        angle2= round(calculate_angle(hip, l_shoulder, elbow),1)
        angle3= round(calculate_angle(elbow, wrist,in_hand),1)
        
        if(loop_id==0):
            dis_p=dis
            le_p=le
            pan_p=0
        else:
            abl=abs(le_p-le)
            if abl >70 :
                le=le_p
            if angle3 > 165:
                an3=0
            else:
                an3=3.14/2
            pan=le_p-le
            pan_an=round(3.14*pan/90,2)
            #print(pan_an)
            gripper_opening_angle = dis
            p.setJointMotorControl2(robotID, jointIndex=2,controlMode=p.POSITION_CONTROL,
                targetPosition = -3.14*angle2/180+3.14/2)
            p.setJointMotorControl2(robotID, jointIndex=3,controlMode=p.POSITION_CONTROL,
                targetPosition = (3.14*angle1/180-3.14))
            p.setJointMotorControl2(robotID, jointIndex=6,controlMode=p.POSITION_CONTROL,
                targetPosition = an3)
            p.setJointMotorControl2(robotID, jointIndex=1,controlMode=p.POSITION_CONTROL,
                targetPosition = -(pan_p+pan_an))         
            controlRobotiqC2(controlMode=p.POSITION_CONTROL, targetPosition=gripper_opening_angle)
            for _ in range (30):
                p.stepSimulation()
            cv2.putText(image, str(angle1), 
                           tuple(np.multiply(elbow, [640, 480]).astype(int)), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA
                                )

            cv2.putText(image, str(angle2), 
                           tuple(np.multiply(l_shoulder, [640, 480]).astype(int)), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA
                                )
            cv2.putText(image, str(angle3), 
                           tuple(np.multiply(wrist, [640, 480]).astype(int)), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA
                                )
            cv2.putText(image, str(angle4), 
                           tuple(np.multiply(r_shoulder, [640, 480]).astype(int)), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            pan_p=pan_p+pan_an
        le_p=le
        dis_p=dis
        loop_id+=1
    except:
        pass 
    mp_drawing.draw_landmarks(
        image, results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS)
    mp_drawing.draw_landmarks(
        image, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS)
    cv2.imshow('MediaPipe Holistic', image)
    if cv2.waitKey(5) & 0xFF == 27:
      break
cap.release()
