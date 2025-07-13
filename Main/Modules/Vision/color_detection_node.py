import rclpy 
from rclpy.node import Node 
import cv2 
from std_msgs.msg import String
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from ultralytics import YOLO  # YOLOv8

#Variables de deteccion de color

class ColorDetectionNode(Node):
    
    def __init__(self):

        super().__init__('color_detection_node')

        self.bridge = CvBridge()

        #Publicador y suscriptor de imagenes
        self.sub = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)
        self.pub = self.create_publisher(Image, 'stop_light_camera', 10)

        ##Publicador al topico stop_light
        self.pub_color = self.create_publisher(String, 'color_detected', 10)

        self.image_received_flag = False
        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)
        self.get_logger().info("Vision node started")

        self.colorDetected = "green"

        # Carga del modelo YOLOv8 (con clases: red, yellow, green)
        self.model = YOLO("/home/danieldrg/Python/runs/detect/train/weights/best.pt")  # Ruta al modelo entrenado



    def camera_callback(self, msg):
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            #self.cv_img = self.cv_img = self.cv_img[0:460, 160:320] # (y1:y2, x1:x2) Simulacion
            self.cv_img = self.cv_img[0:120, 0:160] #SOLO PARA PUZZLEBOT
            self.image_received_flag = True
        except:
            self.get_logger().info("Failed to get an image")

    def timer_callback(self):
        if not self.image_received_flag:
            return

        self.image_received_flag = False
        image = self.cv_img.copy()

        # YOLOv8 - Inferencia
        results = self.model(image, verbose=False)[0]
        detected_color = None

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            label = self.model.names[cls_id]
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            area = (x2 - x1) * (y2 - y1)
            min_area = 180

            if conf > 0.70 and label in ['red', 'yellow', 'green'] and area > min_area:
                print(conf, "area:", area)
                detected_color = label
                cv2.rectangle(image, (x1, y1), (x2, y2), (255,0,0), 1)
                cv2.putText(image, label.upper(), (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                break  # Tomar solo el primer semáforo detectado

        if detected_color:
            self.pub_color.publish(String(data=detected_color))
            print(f"Color detectado: {detected_color.upper()}")
            print(conf)
        else:
            self.pub_color.publish(String(data="none"))

        resized_image = cv2.resize(image, (160,120)) #(width, height) 
        self.pub.publish(self.bridge.cv2_to_imgmsg(resized_image, 'bgr8')) 


def main(args=None):
        rclpy.init(args=args)
        node = ColorDetectionNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()