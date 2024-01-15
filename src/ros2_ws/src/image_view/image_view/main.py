import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from flask import Flask, render_template, Response, request
from threading import Thread
from time import sleep

app = Flask(__name__)

latest_image = None
go_stream = False
node = None


class ControllerNode(Node):
    def __init__(self):
        super().__init__('ESP32_Camera_Web_Server')
        self.publisher_ = self.create_publisher(String, '/esp32/camera/config', 1)
        self.subscription = self.create_subscription(Image, '/esp32/camera/image_jpeg', self.image_callback, 1)
        self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        global latest_image, go_stream
        latest_image = bytes(msg.data)
        go_stream = True

    def publish_config(self, cfg:str):
        msg = String()
        msg.data = cfg.replace("=on", "=1").replace("=off", "=0")
        print(msg.data)
        self.publisher_.publish(msg)


def generate_image_stream():
    global latest_image, go_stream
    while True:
        if latest_image is not None and go_stream:
            go_stream = False
            content_length = str(len(latest_image)).encode()

            # Yield the image data as bytes
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\nContent-Length: ' + content_length + b'\r\n\r\n' + latest_image + b'\r\n\r\n')
        else:
            sleep(0.01)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/command")
def command():    
    global node
    if node is not None:
        node.publish_config(",".join([f"{key}={val}" for (key, val) in request.args.items()]))

    return ""

@app.route('/stream')
def image_stream():
    return Response(generate_image_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

def main(args=None):
    global node
    rclpy.init(args=args)
    node = ControllerNode()
    t = Thread(target=lambda: app.run(host="0.0.0.0", port=8000, debug=False, threaded=True), daemon=True)
    t.start()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
