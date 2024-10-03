import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from threading import Thread
from io import BytesIO
import cv2
from cv_bridge import CvBridge
import numpy as np
BUFFER_SIZE = 1024

class TCP_IP(Node):
    def __init__(self):
        super().__init__('tcp_ip_node')
        self.server_ip = "192.168.0.3"
        self.server_port = 12345
        self.limit = False
        self.scale = False
        
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.server_socket.bind((self.server_ip, self.server_port))
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.listen(1)
    
        # pub
        self.limit_on_pub = self.create_publisher(String, '/tcp_ip_node/limit_on', 10)
        self.scale_data_pub = self.create_publisher(Float64MultiArray,'tcp_ip_node/scale_data',10)
        
        # sub
        self.action_state_sub = self.create_subscription(Int32, '/tcp_ip_node/action_state',self.sub_callback_action_state,1)
        self.box_imformation_sub = self.create_subscription(String, '/tcp_ip_node/box_imformation',self.sub_box_imformation, 10)
        self.box_image_sub = self.create_subscription(Image, '/tcp_ip_node/box_image',self.sub_box_image, 10)


        self.gripper = False
        self.object = False
        self.scale = False
        self.imformation = False
        self.image = False
        self.all_process_done = False
        self.run()

    def close_socket(self):
        self.server_socket.close()
    
    def sub_callback_action_state(self,msg):
        action_state = msg.data
        if action_state == 0:
            self.get_logger().info("action_state: {}, 리니어상자잡기".format(msg.data))
            self.gripper = True
        
        elif action_state == 1:
            self.get_logger().info("sub_data: {}, 저울상자놓기".format(msg.data))
            self.object = True
        
        elif action_state == 2:
            self.get_logger().info("sub_data: {}, 적재단상자놓기".format(msg.data))
            self.all_process_done = True
        action_state = 0

    # 상자 정보 보내기 sub callback
    def sub_box_imformation(self, msg):
            self.get_logger().info("상자정보: {}".format(msg.data))
            self.box_imformation = msg.data
            self.imformation = True
    
    # 상자 이미지 sub callback
    def sub_box_image(self, msg):
            self.box_image = msg
            self.image = True


    # 클라이언트로부터 데이터 수신
    def handle_client(self,client_socket, client_address):
        self.get_logger().info('ready')
        while rclpy.ok():
            received_data_str = client_socket.recv(BUFFER_SIZE).decode("utf-8")
            print("받은 데이터 : {}".format(received_data_str))
            if received_data_str == "q":
                self.close_socket()
                self.get_logger().info('close TCP/IP server')
                self.destroy_node()
                rclpy.shutdown()

            # 리니어 이송단 도착
            elif received_data_str == "리밋스위치on":
                self.get_logger().info("리밋스위치on")
                msg = String()
                msg.data = "리밋스위치on"
                self.limit_on_pub.publish(msg)
                self.limit = True
                self.get_logger().info("ok")
            
            # 저울 측정 완료
            elif received_data_str == "저울측정완료":
                self.get_logger().info("저울측정완료")
                self.scale = True
            
            # 저울로 측정한 무게중심 데이터 전송
            elif self.scale:
                if received_data_str != "저울측정완료":
                    received_data_list = [float(value) for value in received_data_str[1:-1].split(",")]
                    msg = Float64MultiArray()
                    msg.data = received_data_list
                    self.scale_data_pub.publish(msg)
                    self.scalse = False


    def run(self):
        self.get_logger().info(f"Waiting for connection on {self.server_ip}:{self.server_port}")
        self.client_socket, self.client_address = self.server_socket.accept()
        client_thread = Thread(target=self.handle_client, args=(self.client_socket, self.client_address))
        client_thread.daemon = True
        client_thread.start()
        self.get_logger().info("thread_start")
        while True:

            rclpy.spin_once(self)
            if self.gripper:
                data_str = "그리퍼클로즈"
                self.client_socket.sendall(data_str.encode("utf-8"))
                self.get_logger().info("action_state : 0, tcp통신완료")
                self.gripper = False
            
            if self.object:
                data_str = "저울물체놓기완료"
                self.client_socket.sendall(data_str.encode("utf-8"))
                self.get_logger().info("action_state : 1, tcp통신완료")
                self.object = False
            
            if self.all_process_done:
                data_str = "물체놓기완료"
                self.client_socket.sendall(data_str.encode("utf-8"))
                self.get_logger().info("action_state : 2, tcp통신완료")
                self.all_process_done = False

            if self.imformation:
                data_str = self.box_imformation
                self.client_socket.sendall(data_str.encode("utf-8"))
                self.get_logger().info("상자정보보내기완료")
                self.imformation = False

            # Window PC에 상자 크기 및 무게중심이 나타난 사진 보내기
            if self.image:
                bridge = CvBridge()
                cv_image = bridge.imgmsg_to_cv2(self.box_image, "bgr8")
                _, image_buffer = cv2.imencode(".png", cv_image)
                png_data = image_buffer.tobytes()
                self.image = False
                
                total_size = len(png_data)
                
                offset = 0
                chunk_size = 4096
                self.count = 0
                while offset < total_size:
                    self.count += 1
                    end_offset = min(offset + chunk_size, total_size)
                    chunk = png_data[offset:end_offset]
                    self.client_socket.sendall(chunk)
                    offset = end_offset
                print("상자 데이터 완전히 보냄")
                self.image = False

    def on_shutdown(self):
        self.get_logger().info('Shutting down')

def main(args=None):
    rclpy.init(args=args)
    node = TCP_IP()

    try:
        rclpy.spin(node)
    except:
        node.server_socket.close()
        node.on_shutdown()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()