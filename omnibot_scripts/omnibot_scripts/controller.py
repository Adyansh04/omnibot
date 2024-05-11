#!/usr/bin/env python3

"""pip install pynput
pip install keyboard
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from time import time
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
import subprocess
# from pynput import keyboard
# import keyboard

class PS4JoyNode(Node):
    def __init__(self):
        super().__init__('ps4_joy_node')
        self.get_logger().info("PS4 Joy Node has been started")

        self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        self.create_subscription(
            Twist,
            'cmd_vel_fast',
            self.cmd_vel_callback,
            10
        )

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.2, self.timer_callback)

        # self.lift_service = self.create_client(SetBool, 'service_lift')
        # self.claw_service = self.create_client(SetBool, 'service_claw')

        # # Register keyboard event handlers
        # keyboard.on_press_key('w', lambda _: self.lift_up())     # 'w' key for lift up
        # keyboard.on_press_key('s', lambda _: self.lift_down())    # 's' key for lift down
        # keyboard.on_press_key('a', lambda _: self.claw_open())    # 'a' key for claw open
        # keyboard.on_press_key('d', lambda _: self.claw_close())   # 'd' key for claw close
        # keyboard.on_press_key('1', lambda _: self.luna_silo_1())  # '1' key for Luna Silo 1
        # keyboard.on_press_key('2', lambda _: self.luna_silo_2())  # '2' key for Luna Silo 2
        # keyboard.on_press_key('3', lambda _: self.luna_silo_3())  # '3' key for Luna Silo 3

        # self.keyboard_listener = keyboard.Listener(on_press=self.on_press)
        # self.keyboard_listener.start()

        self.lin_x = 0.0
        self.lin_y = 0.0
        self.ang_z = 0.0

        self.is_service_executing = False

        self.lift_down_old = 0
        self.lift_up_old = 0
        self.claw_open_old = 0
        self.claw_close_old = 0
        self.luna_1_old = 0
        self.luna_2_old = 0
        self.luna_3_old = 0
        

    def cmd_vel_callback(self, msg):
        self.lin_x = msg.linear.x
        self.lin_y = msg.linear.y
        self.ang_z = msg.angular.z

    def timer_callback(self):
        if not self.is_service_executing :
            msg = Twist()
            msg.linear.x = round(self.lin_x,3)
            msg.linear.y = round(self.lin_y,3) 
            msg.angular.z = round(self.ang_z,3) 

            self.publisher.publish(msg)
    
    def on_press(self, key):
        try:
            if key.char == 'w':
                self.lift_up()
            elif key.char == 's':
                self.lift_down()
            elif key.char == 'a':
                self.claw_open()
            elif key.char == 'd':
                self.claw_close()
            elif key.char == '1':
                self.luna_silo_1()
            elif key.char == '2':
                self.luna_silo_2()
            elif key.char == '3':
                self.luna_silo_3()
        except AttributeError:
            pass
                
    def call_service(self, service, data):

        self.get_logger().info('Giving Value True')

        self.is_service_executing = True

        while not service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = SetBool.Request()
        request.data = data
        future = service.call_async(request)
        future.add_done_callback(self.handle_service_response)
        

    def handle_service_response(self, future):
        try:
            response = future.result()
            self.get_logger().info('Service response: %r' % response)
            self.get_logger().info('Giving Value False')

            self.is_service_executing = False

        except Exception as e:
            self.get_logger().error('Exception while calling service: %r' % e)

    def lift_up(self):
        self.get_logger().info("Lift Up")
        self.call_service(self.lift_service, True)
        return
    
    def lift_down(self):
        self.get_logger().info("Lift Down")
        self.call_service(self.lift_service, False)
        return

    def claw_open(self):
        self.get_logger().info("Claw Open")
        self.call_service(self.claw_service, False)
        return

    def claw_close(self):
        self.get_logger().info("Claw Close")
        self.call_service(self.claw_service, True)
        return
    
    def luna_silo_1(self):
        self.get_logger().info("Luna Silo 1")
        self.is_service_executing = True
        cmd = "gnome-terminal -- ros2 run luna_control luna_wall_align --ros-args -p silo_number:=1"
        subprocess.Popen(cmd, shell=True)
        self.is_service_executing = False

        return
    
    def luna_silo_2(self):
        self.get_logger().info("Luna Silo 2")
        self.is_service_executing = True

        cmd = "gnome-terminal -- ros2 run luna_control luna_wall_align --ros-args -p silo_number:=2"
        subprocess.Popen(cmd, shell=True)
        self.is_service_executing = False

        return
    
    def luna_silo_3(self):
        self.get_logger().info("Luna Silo 3")
        self.is_service_executing = True

        cmd = "gnome-terminal -- ros2 run luna_control luna_wall_align --ros-args -p silo_number:=3"
        subprocess.Popen(cmd, shell=True)
        self.is_service_executing = False

        return

    def joy_callback(self, msg):

        # lift_up_new = msg.buttons[3] #Triangle
        # lift_down_new = msg.buttons[0] #Cross
        # claw_open_new = msg.buttons[1]  #Circle
        # claw_close_new = msg.buttons[2] #Square

        # luna_silo_1 = msg.buttons[11] #Up Arrow
        # luna_silo_2 = msg.buttons[14] #Rigt Arrow
        # luna_silo_3 = msg.buttons[12] #Down Arrow

        # self.get_logger().info(f"Life Up: {lift_up_new}, Life Down: {lift_down_new}, Claw Open: {claw_open_new}, Claw Close: {claw_close_new}")


        # if ((lift_up_new == 1) and (self.lift_up_old == 0)):
        #     self.lift_up()

        # if ((lift_down_new == 1) and (self.lift_down_old == 0)):
        #     self.lift_down()

        # if ((claw_open_new == 1) and (self.claw_open_old == 0)):
        #     self.claw_open()

        # if ((claw_close_new == 1) and (self.claw_close_old == 0)):
        #     self.claw_close()
        
        # if ((luna_silo_1 == 1) and (self.luna_1_old == 0)):
        #     self.luna_silo_1()

        # if ((luna_silo_2 == 1) and (self.luna_2_old == 0)):
        #     self.luna_silo_2()

        # if ((luna_silo_3 == 1) and (self.luna_3_old == 0)):
        #     self.luna_silo_3()

        # self.lift_up_old = lift_up_new 
        # self.lift_down_old = lift_down_new
        # self.claw_open_old = claw_open_new
        # self.claw_close_old = claw_close_new

        # self.luna_1_old = luna_silo_1
        # self.luna_2_old = luna_silo_2
        # self.luna_3_old = luna_silo_3
        pass

        
def main(args=None):
    rclpy.init(args=args)

    ps4_joy_node = PS4JoyNode()

    rclpy.spin(ps4_joy_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()




