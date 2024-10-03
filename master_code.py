import rclpy
from rclpy.node import Node
import serial
import spidev
import time
import numpy as np
import gpiozero as gpio
from ass_msgs.msg import ArmMasterCommInt
from rclpy.qos import qos_profile_sensor_data, qos_profile_default
from typing import Optional

class ARM_MASTER(Node):
    def __init__(self):
        super().__init__("arm_master")
        self.ser = serial.Serial(
            port='/dev/ttyCUI',  
            baudrate=2000000,         
            timeout=0.003
        )              

        self.zero_addr = [b'\x02\x5E', b'\x22\x5E', b'\x42\x5E', b'\x62\x5E', b'\x82\x5E', b'\xA2\x5E',
                               b'\12\x5E', b'\x32\x5E', b'\x52\x5E', b'\x72\x5E', b'\x92\x5E', b'\xB2\x5E']
        
        left_cui_addr = [b'\x00', b'\x20', b'\x40', b'\x60', b'\x80', b'\xA0']
        right_cui_addr = [b'\x10', b'\x30', b'\x50', b'\x70', b'\x90', b'\xB0']
        
        self.cui_addr = left_cui_addr + right_cui_addr
        self.count_p_r = None
        self.count_n_r = None
        self.count_p_l = None
        self.count_n_l = None
        self.previous_value_r = None
        self.previous_value_l = None
        self.zero_state = False
        
        # SPI 통신
        self.spi = spidev.SpiDev()

        SPI_CHANNEL = 0  # SPI 채널 (CE0)
        SPI_SPEED = 1000000  # 1 MHz

        self.spi = spidev.SpiDev()
        self.spi.open(0, SPI_CHANNEL)
        self.spi.max_speed_hz = SPI_SPEED
        # pub
        self.pub_cmd_input = self.create_publisher(ArmMasterCommInt, 'cmd_input', qos_profile_default)

        
        # setup
        self.setup_gpio()
        self.mode = 0 # 1: 데이터 받기, 2: setzero 설정

 
    def setup_gpio(self):
        self.l_joy_Button_u = gpio.Button(0)
        self.l_joy_Button_d = gpio.Button(6)
        self.l_joy_Button_r = gpio.Button(12)
        self.l_joy_Button_l = gpio.Button(5)
        self.r_joy_Button_u = gpio.Button(19)
        self.r_joy_Button_d = gpio.Button(21)
        self.r_joy_Button_r = gpio.Button(20)
        self.r_joy_Button_l = gpio.Button(26)
        self.lifter_up = gpio.Button(2)
        self.lifter_down = gpio.Button(3)
        self.pump_on = gpio.Button(4)
        self.pump_off = gpio.Button(14) 

      
    def readEncData(self):
        self.ser.reset_input_buffer()
        t_s = time.time_ns()
        for cui_addr in self.cui_addr:
            self.ser.reset_output_buffer()
            self.ser.write(cui_addr)
            t0 = time.time_ns()
            while True:
                t1 = (time.time_ns() - t0)/1e9
                if t1 >= 4e-6:         # time before encoder responds with position = 3us
                    break
       
        while True:  # blocks until buffer filled or 2ms passed
            in_waiting = self.ser.in_waiting
            if in_waiting == 24:
                break
            if time.time_ns() - t_s >= 2e6:
                break

        if self.ser.in_waiting == 24:
            data:bytes = self.ser.read(24)
            data_array = np.frombuffer(buffer=data, dtype=np.uint8).reshape(-1, 2)
            
            if self.checksum(data_array):
                return self.raw_to_deg(data_array)
            else:
                return None          
        else:
            return None
            
    # channel 0: left, channel 1: right
    def read_adc(self,channel):
        if channel < 0 or channel > 3:
            raise ValueError("채널은 0에서 3 사이여야 합니다.")
    
        # MCP3204 채널에 맞는 구성 비트 설정
        start_bit = 0x01
        config_bits = (0xA0 | channel << 6)
        read_command = [start_bit, config_bits, 0x00]

        adc_response = self.spi.xfer2(read_command)

        # 12비트 결과 추출
        adc_value = ((adc_response[1] & 0x0F) << 8) + adc_response[2]
        # 0 ~ 4096 --> 0 ~ 1000
        adc_value = adc_value /4096 * 1000
        return round(adc_value)


    # 조이스틱 신호 읽기 [ close : -1 | open : 1 ]
    def read_Joy(self,index): 
        L7 = -(self.l_joy_Button_r.value - self.l_joy_Button_l.value)
        L8 = self.l_joy_Button_u.value - self.l_joy_Button_d.value
        R7 = self.r_joy_Button_r.value - self.r_joy_Button_l.value
        R8 = self.r_joy_Button_u.value - self.r_joy_Button_d.value
        joy_data = [L7, L8, R7, R8]
        return joy_data[index]

    # down : -1, up : 1
    def read_Lifter(self):
        lifter = self.lifter_up.value - self.lifter_down.value
        return lifter  
    
    # off : -1, on: 1
    def read_Pump(self):
        pump = self.pump_on.value - self.pump_off.value
        return pump  
    


    def setZero(self):
        self.zero_state = True
        self.ser.reset_input_buffer()
        for cui_addr in self.zero_addr:
            self.ser.reset_output_buffer()
            self.ser.write(cui_addr)
            time.sleep(1e-2)

        self.ser.reset_input_buffer()
        for cui_addr in self.cui_addr:
            self.ser.reset_output_buffer()
            self.ser.write(cui_addr)
            time.sleep(1e-2)

        data:bytes = self.ser.read(24)
        data_array = np.frombuffer(buffer=data, dtype=np.uint8).reshape(-1, 2)
        zero_response = self.raw_to_deg(data_array)
        self.zero_state = False

        print("Degree : ", zero_response)

    def multi_turn_algorithm(self, val_degree, total_bytes, index):
        # 오른팔 5축
        if index == 1:
            current_value_r = total_bytes[10]
            
            if self.previous_value_r is not None:
                difference_r = current_value_r - self.previous_value_r
            else:
                difference_r = 0
            
            self.previous_value_r = current_value_r
            
            # 경계값을 넘었는지 확인
            if difference_r > 8000:
                self.count_n_r = True
                self.count_p_r = False
            elif difference_r < -8000:
                self.count_p_r = True
                self.count_n_r= False
            
            # 조건에 따라 각도를 계산
            if self.count_n_r:
                val = (val_degree[10] - 360) / 2
                return val
            
            elif self.count_p_r:
                val = val_degree[10] / 2
                return val

            else:
                return None   
        # 왼팔 5축
        if index == 0:
            current_value_l = total_bytes[4]
            
            if self.previous_value_l is not None:
                difference_l = current_value_l - self.previous_value_l
            else:
                difference_l = 0
            
            self.previous_value_l = current_value_l
            
            # 경계값을 넘었는지 확인
            if difference_l > 8000:
                self.count_n_l = False
                self.count_p_l = True
            elif difference_l < -8000:
                self.count_p_l = False
                self.count_n_l = True
            
            # 조건에 따라 각도를 계산
            if self.count_n_l:
                val = val_degree[4] / 2
                return val
            
            elif self.count_p_l:
                val = (val_degree[4] - 360) / 2
                return val

            else:
                return None
           
    def raw_to_deg(self, a: np.ndarray) -> Optional[np.ndarray]:
        # 모든 행에 공통적으로 적용되는 연산
        a = a & [0b11111111, 0b00111111]
        total_bytes = (a @ np.array([[1], [2 ** 8]])).reshape(-1)
        val_degree = total_bytes* 360 / 2**14
        val = val_degree - (val_degree > 180) * 360
        
        if self.zero_state:
            return val
        
        # 함수 index 1: 오른팔, 0: 왼팔
        val[4] = self.multi_turn_algorithm(val_degree, total_bytes, 0)
        val[10] = self.multi_turn_algorithm(val_degree, total_bytes, 1)

         # 0: 음수, 1: 양수
        mask = np.array([
            1, 0, 0, 1, 0, 1,
             0, 1, 1, 0, 1, 0
            ])
        val = val * mask + val * (1 - mask) * -1
        
        #return val
        if (not np.isnan(val[4])) and (not np.isnan(val[10])):
            return val
        else:
            return None

    @staticmethod              
    def checksum(a: np.ndarray) -> bool:
        total_bytes = a @ np.array([[1], [2**8]])  # shape of total_bytes: (-1,1)
        odd_masked = total_bytes & np.uint16(0b1010101010101010)
        even_masked = total_bytes & np.uint16(0b0101010101010101)
        
        even_counts = np.char.count(list(map(bin, even_masked[:, 0])), "1") % 2
        odd_counts = np.char.count(list(map(bin, odd_masked[:, 0])), "1") % 2
        
        return (even_counts & odd_counts).all().item()
    
   
    def run(self):
        msg = ArmMasterCommInt()
        while rclpy.ok():
            # index --> left : 0~5  | right : 6~11 
            cui_degree_data = self.readEncData()
            if cui_degree_data is None:
                continue
            else: 
                msg.l1 , msg.l2, msg.l3, msg.l4, msg.l5, msg.l6 = cui_degree_data[0:6]
                msg.r1 , msg.r2, msg.r3, msg.r4, msg.r5, msg.r6 = cui_degree_data[6:12]
                msg.lifter = self.read_Lifter()
                msg.pump = self.read_Pump()
                msg.l7 = self.read_Joy(0)
                msg.l8 = self.read_Joy(1)
                msg.r7 = self.read_Joy(2)
                msg.r8 = self.read_Joy(3)
                msg.lever_0 = self.read_adc(0)  # 왼팔
                msg.lever_1 = self.read_adc(1)  # 오른팔
                
                self.pub_cmd_input.publish(msg)



def main(args=None):
    rclpy.init(args=args, domain_id=1)
    node = ARM_MASTER()
    
    try: 
        user_input = input("1: Run\n2: SetZero\nInitializing Mode : ")
        node.mode = int(user_input)

        if node.mode == 1:
            node.run()

        elif node.mode == 2:
            node.setZero()

        rclpy.spin(node)
    
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    
    except ValueError:
        node.get_logger().error('Invalid input! Please enter 1 or 2.')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()