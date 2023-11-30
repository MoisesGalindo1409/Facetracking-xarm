import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from turtlesim.msg import Pose  # Importa la información de posición de la tortuga
import math

# Variables globales para almacenar la posición y las coordenadas objetivo
current_pose = Pose()
target_coordinates = Point()

def pose_callback(data):
    global current_pose
    current_pose = data

def coordinates_callback(data):
    global target_coordinates
    target_coordinates = data

def main():
    rospy.init_node('turtle_controller')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)  # Suscribe a la posición de la tortuga
    rospy.Subscriber('/coordenadas_objetivo', Point, coordinates_callback)  # Ajusta el nombre del tópico según tu configuración

    # Velocidades máximas lineal y angular
    max_linear_speed = 100.0  # Ajusta según tu preferencia
    max_angular_speed = 100.0  # Ajusta según tu preferencia

    # Umbral para considerar que la tortuga ha llegado a las coordenadas deseadas
    threshold = 0.0001  # Ajusta según tus necesidades

    rate = rospy.Rate(10)  # Frecuencia de publicación (10 Hz)

    while not rospy.is_shutdown():
        # Calcula el error en las coordenadas
        error_x = target_coordinates.x - current_pose.x
        error_y = target_coordinates.y - current_pose.y

        # Calcula el ángulo hacia las coordenadas deseadas
        desired_angle = math.atan2(error_y, error_x)

        # Calcula la distancia al punto deseado
        distance = math.sqrt(error_x ** 2 + error_y ** 2)

        # Ajusta la velocidad lineal y angular
        linear_speed = 0.0
        angular_speed = 0.0

        if abs(desired_angle - current_pose.theta) > 0.1:
            angular_speed = 1.0 if desired_angle > current_pose.theta else -1.0
        else:
            linear_speed = min(1.0, distance)

        # Si la distancia es menor que el umbral, detén la tortuga
        if distance < threshold:
            linear_speed = 0.0
            angular_speed = 0.0

        # Crea el mensaje de Twist para controlar la tortuga
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed

        # Publica el mensaje en el tópico /turtle1/cmd_vel
        pub.publish(twist)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
