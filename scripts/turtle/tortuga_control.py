import rospy
import cv2
import mediapipe as mp
from geometry_msgs.msg import Twist

def main():
        rospy.init_node('opencv_to_ros_turtle')
        pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
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
                if (left[0].y - left[1].y) < 0.004:
                    print ("blink", blink)
                    if blink == 0:
                        counter=0
                    blink=blink+1
                for landmark in left:
                    x = int(landmark.x * frame_w)
                    y = int(landmark.y * frame_h)
                    # Publica las coordenadas en el tópico
                    xe=(x // 60) -4
                    ye=(-y // 60)-- 4
                    twist = Twist()
                    twist.linear.x = xe
                    twist.linear.y = ye
                    print (x) 
                    print (y) 
                    pub.publish(twist)
                    cv2.circle(frame, (x, y), 3, (0, 255, 255))
            
            counter=counter+1
            if 5 < blink:
                 print ("stop!")
                 blink=0
                 break
            if 20 < counter:
                 counter=0
                 blink=0
            
            cv2.imshow('Eye Controlled Mouse', frame)
            cv2.waitKey(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


