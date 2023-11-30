#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import mediapipe as mp
from geometry_msgs.msg import Point, Pose
from xarm_planner.srv import pose_plan, exec_plan


def main():
        
        cam = cv2.VideoCapture(0)
        face_mesh = mp.solutions.face_mesh.FaceMesh(refine_landmarks=True)
        blink=-20000
        counter=0
        while True:
            _, frame = cam.read()
            frame = cv2.flip(frame, 1)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            output = face_mesh.process(rgb_frame)
            landmark_points = output.multi_face_landmarks
            frame_h, frame_w, _ = frame.shape
            if landmark_points:
                landmarks = landmark_points[0].landmark
                for id, landmark in enumerate(landmarks[474:478]):
                    x = int(landmark.x * frame_w)
                    y = int(landmark.y * frame_h)
                    cv2.circle(frame, (x, y), 3, (0, 255, 0))

                left = [landmarks[145], landmarks[159]]
                
                #conteo de parpadeos
                if (left[0].y - left[1].y) < 0.004:
                    print ("blink", blink)
                    if blink == 0:
                        counter=0
                    blink=blink+1
                #detener el codigo despues de 3 parpadeos
                if 5 < blink:
                    print ("stop")
                    blink=10
                    counter=-10
                    blanc=0
                    counter2=0
                    while blanc < 5:
                        _, frame = cam.read()            
                        frame = cv2.flip(frame, 1)
                        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        output = face_mesh.process(rgb_frame)
                        landmark_points = output.multi_face_landmarks
                        frame_h, frame_w, _ = frame.shape
                        if landmark_points:
                            landmarks = landmark_points[0].landmark
                            for id, landmark in enumerate(landmarks[474:478]):
                                x = int(landmark.x * frame_w)
                                y = int(landmark.y * frame_h)
                                cv2.circle(frame, (x, y), 3, (0, 255, 0))
            
                            left = [landmarks[145], landmarks[159]]
                
                        if (left[0].y - left[1].y) < 0.004:
                            print ("blanc", blanc)
                            if blanc == 0:
                                counter2=0
                            blanc=blanc+1
                        if 200 < counter2:
                            counter2=0
                            blanc=0
                    blink=0
                    counter=0
                                
                else:
                    for landmark in left:
                        x = int(landmark.x * frame_w)
                        y = int(landmark.y * frame_h)
                        # Publica las coordenadas en el tópico
                        xe=(x / 1000) 
                        ye=frame_h/600-(y / 600)
                        target_pose = Pose()
                        target_pose.position.x = 0.2
                        target_pose.position.y = xe
                        target_pose.position.z = ye
                        pub.publish(target_pose)
                        print (target_pose.position.z) 
                        print (target_pose.position.z) 
                        print (frame_h)    
                        cv2.circle(frame, (x, y), 3, (0, 255, 255))
                        counter=counter+1

            print (counter)
            if 50 < counter:
                 counter=0
                 blink=0
            
            cv2.imshow('Eye Controlled Mouse', frame)
            cv2.waitKey(1)



if __name__ == '__main__':
    rospy.init_node("visual_controller")
    pub = rospy.Publisher("/xarm6_coordinates", Pose, queue_size=10)

    rate = rospy.Rate(10)

  

    x = 0.3
    y = 0.2
    z = 0.2

    target_pose = Pose()
    
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = 0.2
    pub.publish(target_pose)

    # target_pose.position.x = x
    # target_pose.position.y = y
    # target_pose.position.z = z
    # target_pose.orientation.x = 1.0
    # target_pose.orientation.y = 0.0
    # target_pose.orientation.z = 0.0
    # target_pose.orientation.w = 0.0

    # coordinates.pub.publish(target_pose)

    # start = input("press any key to start traking")

    try:
        while not rospy.is_shutdown():
            main()


            
            pub.publish(target_pose)



    except rospy.ROSInterruptException:
        pass
