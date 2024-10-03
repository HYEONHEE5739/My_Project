import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray
import pyrealsense2 as rs
import numpy as np
import cv2
import math

def create_window(self):
    cv2.namedWindow("Trackbar Windows")
    cv2.createTrackbar("Threshold1_minValue", "Trackbar Windows", 0, 255, update_thresholds)
    cv2.createTrackbar("Threshold2_maxValue", "Trackbar Windows", 0, 255, update_thresholds)
    cv2.setTrackbarPos("Threshold1_minValue", "Trackbar Windows", 80)
    cv2.setTrackbarPos("Threshold2_maxValue", "Trackbar Windows", 255)
    cv2.waitKey(1)

def update_thresholds(self,*args):
    pass

class MyNode(Node):
    # 노드 초기화
    def __init__(self):
        super().__init__('my_camera_node')
        self.x, self.y, self.w, self.h = 730, 220, 520, 510
       
        self.check = False
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        self.profile = self.pipeline.start(self.config)
        self.detector = cv2.QRCodeDetector()
        # 저울 데이터 pub / sub
        self.sub_scale_data = self.create_subscription(Float64MultiArray,'Scale_data',self.listener_callback, 10)
        self.sub_scale_data
        
        # 길이 데이터 pub 
        self.pub_length_data = self.create_publisher(Float64MultiArray, 'Length_data', 10)
        
        # 무게중심 pub
        self.pub_cg_data = self.create_publisher(Float64MultiArray, 'Cg_data',10)

        # Timer callback
        self.frequency = 20  # Hz : 1초에 20번 타이머 콜백 실행
        self.timer = self.create_timer(1/self.frequency, self.timer_callback)
        self.data = 0
        self.wp2 = 0
        self.wp3 = 0
        self.wp4= 0
        self.hori_length_list = []
        self.vert_length_list = []
        self.height_length_list = []
        self.hori_length = 0
        self.vert_length = 0
        self.height_length = 0
        self.count = 0
        self.check = False
        self.next = False
        self.data_get = False
        self.create_window()
        
    def listener_callback(self, msg):
        self.get_logger().info(f"I heard: {msg.data}")
        self.data = msg.data
        self.data_get = True
    
    def create_window(self):
        cv2.namedWindow("Trackbar Windows")
        cv2.createTrackbar("Threshold1_minValue", "Trackbar Windows", 0, 255, self.update_thresholds)
        cv2.createTrackbar("Threshold2_maxValue", "Trackbar Windows", 0, 255, self.update_thresholds)
        cv2.setTrackbarPos("Threshold1_minValue", "Trackbar Windows", 80)
        cv2.setTrackbarPos("Threshold2_maxValue", "Trackbar Windows", 255)
        cv2.waitKey(1)

    def update_thresholds(self,*args):
        return
    
    # 타이머 콜백으로 카메라 동작 반복
    def timer_callback(self):
        if self.data_get:
            self.get_logger().info(f"good")
            self.camera_process()
            if self.next:
                self.get_logger().info(f"next")
                # 데이터 수집
                self.hori_length_list.append(self.hori_length)
                self.vert_length_list.append(self.vert_length)
                self.height_length_list.append(self.height_length)
                self.wp2 = self.wp2 + self.w_p2 
                self.wp3 = self.wp3 + self.w_p3
                self.wp4 = self.wp4 + self.w_p4
                self.next = False
                self.get_logger().info(f"callback_finished")
                # 20번 평균내기    
                if self.count == 20:
                    hori_avg = np.mean(self.hori_length_list)
                    vert_avg = np.mean(self.vert_length_list)
                    height_avg = np.mean(self.height_length_list)
                    
                    wp2_avg = self.wp2 / 20
                    wp3_avg = self.wp3 / 20
                    wp4_avg = self.wp4 / 20
                    x_offset , _ = self.D_and_Dgree(wp2_avg,wp3_avg,self.data)
                    y_offset , theta = self.D_and_Dgree(wp3_avg,wp4_avg,self.data)
                    w_cg = [wp3_avg[0] + x_offset, wp3_avg[1] + y_offset, wp3_avg[2]]
                    


                    # 길이 정보 보내기
                    avg_length = Float64MultiArray()
                    avg_length.data = [hori_avg, vert_avg, height_avg]
                    self.pub_length_data.publish(avg_length)
                    
                    # 무게중심 정보 보내기
                    cg_length = Float64MultiArray()
                    cg_length.data = [x_offset, y_offset]
                    self.pub_cg_data.publish(cg_length)

                    # ur3 상자 좌표 보내기
                    ur3_pub_point = self.rotation_point(wp3_avg, x_offset, y_offset, vert_avg, theta)
                    print( ur3_pub_point)
                    
                    
                    # 리스트 초기화
                    self.hori_length_list = []  # reset the lists for the next batch of iterations
                    self.vert_length_list = []
                    self.height_length_list = []
                    self.wp2 = 0
                    self.wp3 = 0
                    self.wp4 = 0
                    self.count = 0
        if self.data == False:
            self.get_logger().info(f"check : False self.check == False")

    # QR 코드 인식하기
    def QR_detect (self, img, detector):
        value, points, qrcode = detector.detectAndDecode(img)
        if value != "":
            x1 = points[0][0][0]
            y1 = points[0][0][1]
            x2 = points[0][2][0]
            y2 = points[0][2][1]

            x_center = (x2 - x1) / 2 + x1
            y_center = (y2 - y1) / 2 + y1

            qr_center = [int(x_center),int(y_center)]
            cv2.circle(img, (int(x_center),int(y_center)), 4, (255,0,0),-1)

            return qr_center, value
       
        else:
            return None
    
    # QR 정보로 cg 계산하기, ur3로 보내줄 정보 반환
    def QR_cg_point_to_ur3(self,filtered_contours, qr_center, w_p3, value, theta):
        boxpoints = filtered_contours
        qr_center = qr_center
        x_len, y_len, z_len, x_box,y_box,z_box = value
        theta = theta
        w_p3 = w_p3
        point_x = []
        point_y = []
        
        for point in boxpoints:
            corner_x, corner_y = point[0]
            point_x.append(corner_x)
            point_y.append(corner_y)
    
        points = np.array(list(zip(point_x, point_y)), dtype=np.int32)
        aligned_points = self.sort_points(points)
        
        distances = np.linalg.norm(aligned_points - qr_center, axis=1)
        min_distance_index = np.argmin(distances)
        
        homogeneous_mt = np.array([[math.cos(theta), -math.sin(theta), 0, w_p3[0]],
                                    [math.sin(theta), math.cos(theta) , 0, w_p3[1]],
                                    [0,0,1,w_p3[2]],
                                    [0,0,0,1]], dtype=np.float32)
        if min_distance_index == 0:
            box_coor_point = np.array([x_len-x_box, y_len - y_box, 0, 1])
            to_ur3 = np.array([x_len/2, y_len - y_box, 0, 1])
            scale_coor_cg = np.dot(homogeneous_mt, np.hstack((box_coor_point)))
            print("무게 중심 좌표(qr) : {}".format(scale_coor_cg))
            scale_coor_to_ur3 = np.dot(homogeneous_mt, np.hstack((to_ur3)))
            data_to_ur3 = [scale_coor_to_ur3, x_len/2 -x_box, theta]
            return data_to_ur3
        
        elif min_distance_index == 1: 
            box_coor_point = np.array([y_box, x_len - x_box, 0, 1])
            to_ur3 = np.array([y_len/2, y_len - y_box, 0, 1])
            scale_coor_cg = np.dot(homogeneous_mt, np.hstack((box_coor_point)))
            print("무게 중심 좌표(qr) : {}".format(scale_coor_cg))
            scale_coor_to_ur3 = np.dot(homogeneous_mt, np.hstack((to_ur3)))
            data_to_ur3 = [scale_coor_to_ur3, y_box - y_len/2, theta]
            return data_to_ur3
    
        elif min_distance_index == 2: 
            box_coor_point = np.array([x_box, y_box, 0, 1])
            to_ur3 = np.array([x_len/2, y_len - y_box, 0, 1])
            scale_coor_cg = np.dot(homogeneous_mt, np.hstack((box_coor_point)))
            print("무게 중심 좌표(qr) : {}".format(scale_coor_cg))  
            scale_coor_to_ur3 = np.dot(homogeneous_mt, np.hstack((to_ur3)))
            data_to_ur3 = [scale_coor_to_ur3, x_box- x_len/2, theta]
            return data_to_ur3
        
        elif min_distance_index == 3: 
            box_coor_point = np.array([y_len-y_box, x_box, 0, 1])
            to_ur3 = np.array([y_len/2, y_len - y_box, 0, 1])
            scale_coor_cg = np.dot(homogeneous_mt, np.hstack((box_coor_point)))
            print("무게 중심 좌표(qr) : {}".format(scale_coor_cg))
            scale_coor_to_ur3 = np.dot(homogeneous_mt, np.hstack((to_ur3)))
            data_to_ur3 = [scale_coor_to_ur3, y_len/2 -y_box, theta]
            return data_to_ur3

    # 좌표변환 (회전)
    def rotation_point(self, w_p3, x_offset, y_offset, L1, theta):
        w_p3 = w_p3
        # 가로 길이의 중간지점
        point = [L1/2, y_offset, 0, 1]
        # 상자의 무게중심점
        cg_point = [x_offset, y_offset, 0, 1]
        d = x_offset - (L1/2)
        homogeneous_mt = np.array([[math.cos(theta), -math.sin(theta), 0, w_p3[0]],
                                        [math.sin(theta), math.cos(theta) , 0, w_p3[1]],
                                        [0,0,1,w_p3[2]],
                                        [0,0,0,1]], dtype=np.float32)
        scale_coor_cg = np.dot(homogeneous_mt, np.hstack((cg_point)))
        print("무게 중심 좌표(sensor) : {}".format(scale_coor_cg))
        scale_coor_to_ur3 = np.dot(homogeneous_mt, np.hstack((point)))    
        data_to_ur3 = [scale_coor_to_ur3, d, theta]
        
        return data_to_ur3 
        
        # 4x4 동차 좌표 변환 행렬 생성


    # 좌표계변환 (카메라 -> 월드(저울))   
    def camera_to_world_point(self,c_p, rvec, tvec):
        # 회전 벡터를 회전 행렬로 변환
        c_p = np.asarray(c_p)
        R, _ = cv2.Rodrigues(rvec)
        # 3x4 변환 행렬 생성
        RT = np.hstack([R, tvec])

        # 4x4 동차 좌표 변환 행렬 생성
        homogeneous_RT = np.vstack([RT, [0, 0, 0, 1]])
        
        # 월드 좌표계를 계산하기 위한 역 변환 행렬
        inverse_RT = np.linalg.inv(homogeneous_RT)
        w_p = np.dot(inverse_RT, np.hstack((c_p,1)))

        return w_p[:-1]
    
    #선분 ab를 p:q 내분점 구하기
    def divide_segment_p_q(self,a, b, p, q):
        # a와 b의 x, y 좌표 추출
        x1, y1 = a
        x2, y2 = b

        # 선분 ab를 1:3으로 내분하는 점 P의 좌표 계산
        xp = (x1 + q * x2) / (p+q)
        yp = (y1 + q * y2) / (p+q)

        point = [int(xp),int(yp)]
        # 내분점 P의 좌표를 튜플로 반환
        return point
    
    # 무게 중심과 변의 거리, 상자 각도 구하기
    def D_and_Dgree (self,p1,p2,p3):
        x1, y1 = p1[0], p1[1]
        x2, y2 = p2[0], p2[1]
        x3, y3 = p3[0], p3[1]
        if x1 == x2: # 직선의 기울기가 무한인 경우
            slope = float('inf')
            slope_radians = np.actan(slope)
            return abs(x3 - x1), slope_radians
        else:
            k = (y2 - y1) / (x2 - x1) # 직선의 기울기
            b = y1 - k * x1 # 직선의 y 절편
            d = abs(k * x3 - y3 + b) / (k**2 + 1)**0.5 # 점 (x3, y3)와 직선 (x1, y1)-(x2, y2)간의 수직 거리
            slope_radians = np.actan(k)
            return d, slope_radians
    
    # 꼭지점과 중점 벡터의 각도 구하기
    def calculate_angle(self,center, point):
        return np.arctan2(center[1]-point[1], point[0] - center[0])

    # 꼭지점 위치 정렬하기
    def sort_points(self,points):
        center = np.mean(points, axis=0)
        sorted_points = sorted(points, key=lambda point: (self.calculate_angle(center, point)) % (2 * np.pi))

        top_right, top_left, bottom_left, bottom_right = sorted_points

        return np.array([top_right, top_left, bottom_left, bottom_right], dtype=np.int32)

    # 상자 길이, 꼭지점 좌표 구하기
    def Length_measure_3D (self,frame, image, points, x, y):
        image = image
        point_x = []
        point_y = []

        for point in points:
            corner_x, corner_y = point[0]
            point_x.append(corner_x+x)
            point_y.append(corner_y+y)
        center_x = int(np.mean(point_x))
        center_y = int(np.mean(point_y))
        center = [center_x,center_y]
        
        points = np.array(list(zip(point_x, point_y)), dtype=np.int32)
        aligned_points = self.sort_points(points)
        print(aligned_points)
        px1, px2, px3, px4 = aligned_points

        # 뎁스 추정
        dp1  = self.divide_segment_p_q(px1, center, 1,4)
        dp2  = self.divide_segment_p_q(px2, center, 1,4)
        dp3  = self.divide_segment_p_q(px3, center, 1,4)
        dp4  = self.divide_segment_p_q(px4, center, 1,4)

       
        Right_Up_Depth = frame.get_distance(dp1[0], dp1[1])
        Left_Up_Depth = frame.get_distance(dp2[0],dp2[1])
        Left_Down_Depth = frame.get_distance(dp3[0], dp3[1])
        Right_Down_Depth = frame.get_distance(dp4[0], dp4[1])
        Center_Depth = frame.get_distance(center[0],center[1])
        

        # 3D point
        p1 = rs.rs2_deproject_pixel_to_point(self.intrinsics, [px1[0], px1[1]], Right_Up_Depth)
        p2 = rs.rs2_deproject_pixel_to_point(self.intrinsics, [px2[0], px2[1]], Left_Up_Depth)
        p3 = rs.rs2_deproject_pixel_to_point(self.intrinsics, [px3[0], px3[1]], Left_Down_Depth)
        p4 = rs.rs2_deproject_pixel_to_point(self.intrinsics, [px4[0], px4[1]], Right_Down_Depth)
        p5 = rs.rs2_deproject_pixel_to_point(self.intrinsics, [center[0], center[1]], Center_Depth)
        # m 단위
        p1 = np.asarray(p1)
        p2 = np.asarray(p2)
        p3 = np.asarray(p3)
        p4 = np.asarray(p4)
        p5 = np.asarray(p5)

        distance_3d_Right = math.sqrt((p1[0] - p4[0]) ** 2 +
                            (p1[1] - p4[1]) ** 2 +
                            (p1[2] - p4[2]) ** 2)
        distance_3d_Left = math.sqrt((p3[0] - p2[0]) ** 2 +
                            (p3[1] - p2[1]) ** 2 +
                            (p3[2] - p2[2]) ** 2)
        distance_3d_Down = math.sqrt((p4[0] - p3[0]) ** 2 +
                            (p4[1] - p3[1]) ** 2 +
                            (p4[2] - p3[2]) ** 2)
        distance_3d_Up = math.sqrt((p1[0] - p2[0]) ** 2 +
                            (p1[1] - p2[1]) ** 2 +
                            (p1[2] - p2[2]) ** 2)
        # 상자 길이
        Distance_list = [distance_3d_Up, distance_3d_Down, distance_3d_Right, distance_3d_Left, Center_Depth]
        #꼭지점, 중점좌표
        Camera_Point_list = [p1,p2,p3,p4,p5]
        return Distance_list, Camera_Point_list, image
    
    # 좌표 변동 최소값 정하기
    def is_approx_too_close(self,approx, prev_contours,min_distance_to_prev):
        for prev_approx in prev_contours:
            # Compute the distance between the first points of the approximations
            distance = np.linalg.norm(approx[0][0] - prev_approx[0][0])
            if distance < min_distance_to_prev:
                return True
        return False
    
    # 카메라 이미지처리
    def camera_process (self):
        
        self.get_logger().info(f"camera_process : in")
        
        frames = self.pipeline.wait_for_frames()
        if frames:

            aligned_frame = self.align.process(frames)
            depth_frame = aligned_frame.get_depth_frame()
            color_frame = aligned_frame.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            self.intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            
            threshold1 = cv2.getTrackbarPos("Threshold1_minValue", "Trackbar Windows")
            threshold2 = cv2.getTrackbarPos("Threshold2_maxValue", "Trackbar Windows")
            
            colorizer = rs.colorizer()
            colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

            colorized_depth = colorized_depth[self.y:self.y+self.h, self.x:self.x+self.w]
            color_image = color_image[self.y:self.y+self.h, self.x:self.x+self.w]

            alpha = 0.5
            merged_image = cv2.addWeighted(color_image, alpha, colorized_depth, 1 - alpha, 0)
            self.get_logger().info(f"camera_process : gray")
            gray= cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            t = cv2.bilateralFilter(gray, -1, 10, 10)
            k = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
            edge = cv2.Canny(t,threshold1, threshold2)
          
            contours, hierarchy = cv2.findContours(edge, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)            
            
            self.filtered_contours = []
            prev_contours = []
            for contour in contours:
                p = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.01* p, True)
                vtc = len(approx)
                area = cv2.contourArea(contour)
                if (vtc == 4) and (area / (1920 *1080) > 0.001):
                    angles = []
                    for i in range(4):
                        p1 = approx[i][0]
                        p2 = approx[(i + 1) % 4][0]
                        p3 = approx[(i + 2) % 4][0]
                        vec1 = p1 - p2
                        vec2 = p3 - p2
                        dot_product = np.dot(vec1, vec2)
                        length_product = np.linalg.norm(vec1) * np.linalg.norm(vec2)
                        angle_rad = np.arccos(dot_product / length_product)
                        angle_deg = np.degrees(angle_rad)
                        angles.append(angle_deg)

            # 허용범위 안에서 각이 90도에 해당하는지 체크
                    angle_tolerance = 3  # 허용치 3도
                    if all(abs(angle - 90) < angle_tolerance for angle in angles):
                        if not self.is_approx_too_close(approx, prev_contours,30):
                            self.filtered_contours.append(approx)
                            prev_contours.append(approx)

            if len(self.filtered_contours) == 1:
                self.get_logger().info(f"detect contours")
                point_x = []
                point_y = []
                self.count += 1
                
                #상자 길이, 상자 꼭지점
                L_list, P_list, merged_image= self.Length_measure_3D(depth_frame,merged_image,self.filtered_contours[0],self.x,self.y)
                cv2.drawContours(color_image, self.filtered_contours, -1, (255,255,255), 4)

                world_points = np.array([[0, 0, 0], [0.300, 0, 0], [0, 0.300, 0], [0.300, 0.300, 0]], dtype=np.float32)
                image_points = np.array([[730,743], [1257,740], [724,212], [1260,210]], dtype=np.float32)

                camera_matrix = np.array([[1369.01, 0, 969.185],
                                [0, 1365.55, 563.157],
                                [0, 0, 1]], dtype=np.float32)
                dist_coeffs = np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float32)
                _, rvec, tvec= cv2.solvePnP(world_points, image_points, camera_matrix, dist_coeffs)

                self.hori_length = (L_list[0]+L_list[1]) / 2
                self.vert_length = (L_list[2]+L_list[3]) / 2

                w_p1 = self.camera_to_world_point(P_list[0],rvec, tvec)
                w_p2 = self.camera_to_world_point(P_list[1],rvec, tvec)
                w_p3 = self.camera_to_world_point(P_list[2],rvec, tvec)
                w_p4 = self.camera_to_world_point(P_list[3],rvec, tvec)
                w_p5 = self.camera_to_world_point(P_list[4],rvec, tvec)

                center_height = (w_p1[2]+w_p2[2]+w_p3[2]+w_p4[2])/4
                height = w_p5[2]
                self.height_length = height
                self.w_p2 = np.array(w_p2)
                self.w_p3 = np.array(w_p3) 
                self.w_p4 = np.array(w_p4)             
                self.next = True
            

                cv2.imshow("edge", edge)
                cv2.imshow("color_image", color_image)
                cv2.imshow("colorized_depth", merged_image)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    rclpy.spin(my_node)

if __name__ == '__main__':
    main()