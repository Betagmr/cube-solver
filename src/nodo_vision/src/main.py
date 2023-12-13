import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node("primer_subscriber",anonymous=True)
bridge = CvBridge()

imagen = None

def recibir_mensajes(data: Image) -> None:
    global imagen
    imagen = bridge.imgmsg_to_cv2(data)
    imagen = cv2.cvtColor(imagen, cv2.COLOR_BGR2RGB)

rospy.Subscriber("/usb_cam/image_raw", Image, recibir_mensajes)

while imagen is None:
    rospy.sleep(0.1)

while True:
    cv2.imshow("dawaw", imagen)
    cv2.waitKey(1)

# rospy.spin()