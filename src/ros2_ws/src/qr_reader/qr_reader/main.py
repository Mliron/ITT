import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from PIL import Image as PIL_Image
from pyzbar.pyzbar import decode
import io

class QRReaderNode(Node):
    def __init__(self):
        super().__init__('QR_Scanner')
        self.publisher_ = self.create_publisher(String, '/esp32/led_strip/color', 1)
        self.subscription = self.create_subscription(Image, '/esp32/camera/image_jpeg', self.image_callback, 1)
        self.subscription  # prevent unused variable warning


    def image_callback(self, msg):
        img = PIL_Image.open(io.BytesIO(bytes(msg.data)))
        decoded_objects = decode(img)

        # Print the decoded data
        for obj in decoded_objects:
            color = obj.data.decode("utf-8")
            print("Type:", obj.type)
            print("Data:", color, "\n")
            if color[0] == "#":
                tmp = String()
                tmp.data = color
                self.publisher_.publish(tmp)


def main(args=None):
    rclpy.init(args=args)
    node = QRReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()