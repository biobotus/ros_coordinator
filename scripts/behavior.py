#!/usr/bin/env python

# imports
import rospy
from std_msgs.msg import Float32, String, Int32, Float32MultiArray, Int32MultiArray

# variables
node_name = 'motor_control_y'

class behavior():
    def __init__(self):
        self.delta_x = 0
        self.delta_y = 0
        self.delta_z = 0

        self.actual_pos_x = 0
        self.actual_pos_y = 0
        self.actual_pos_z = 0

        self.new_pos_x = 0
        self.new_pos_y = 0
        self.new_pos_z = 0

        self_pulse_x = 0
        self_pulse_y = 0
        self_pulse_z = 0
        
        # Constants
        self.dist_step = 0.127
        self.mode_step = 0.25 # 4 pulses per step
        self.pulse = self.dist_step * self.mode_step
        
        # Error control
        self.err_pulse_x = 0
        self.err_pulse_y = 0
        self.err_pulse_z = 0

        # ROS publishments
        self.pub_pulse_x = rospy.Publisher('Pulse_x',Int32MultiArray, queue_size=10) 
        self.pub_pulse_y = rospy.Publisher('Pulse_y',Int32MultiArray, queue_size=10)
        self.pub_pulse_z = rospy.Publisher('Pulse_z',Int32MultiArray, queue_size=10)               
        self.pub_platform_init = rospy.Publisher('platform_init',Int32,queue_size=10)        

        # ROS subscriptions 
        self.subscriber = rospy.Subscriber('New_pos',Float32MultiArray, self.callback_pos)
        self.subscriber = rospy.Subscriber('Motor_Kill',String, self.callback_kill)
        self.subscriber = rospy.Subscriber('Error',String,self.callback_error)        

        # ROS init
        self.rate = rospy.Rate(10) # 10Hz

    # Callback position
    def callback_pos(self,data):
        self.new_pos_x = data.data[0]
        self.new_pos_y = data.data[1]
        self.new_pos_z = data.data[2]
        self.behavior_output_pulse()

    # Error management
    def callback_error(self,data):
        # switch on error code 
        self.pub_platform_init.publish(1)

    # Compute and publish the number of pulse for each axes
    def behavior_ouptut__pulse(self):
        self.delta_x = self.new_pos_x - self.actual_pos_x
        self.delta_y = self.new_pos_y - self.actual_pos_y
        self.delta_z = self.new_pos_z - self.actual_pos_z

        pulse_temp_x = self.delta_x / self.pulse 
        pulse_temp_y = self.delta_y / self.pulse
        pulse_temp_z = self.delta_z / self.pulse
                
        self.pulse_x = int(round(pulse_temp_x))
        self.pulse_y = int(round(pulse_temp_y))
        self.pulse_z = int(round(pulse_temp_z))

        # Adjust values if error is greater than 1 pulse
        self.err_pulse_x += pulse_temp_x - self.pulse_x
        self.err_pulse_y += pulse_temp_y - self.pulse_y
        self.err_pulse_z += pulse_temp_z - self.pulse_z

        if self.err_pulse_x < -1 :
            self.err_pulse_x += 1
            self.pulse_x -= 1 
        elif self.err_pulse_x > 1 :
            self.err_pulse_x -= 1
            self.pulse_x += 1

        if self.err_pulse_y < -1 :
            self.err_pulse_y += 1
            self.pulse_y -= 1
        elif self.err_pulse_y > 1 :
            self.err_pulse_y -= 1
            self.pulse_y += 1

        if self.err_pulse_z < -1 :
            self.err_pulse_z += 1
            self.pulse_z -= 1
        elif self.err_pulse_z > 1 :
            self.err_pulse_z -= 1
            self.pulse_z += 1

        self.pub_pulse_x.publish(self.pulse_x)
        self.pub_pulse_y.publish(self.pulse_y)
        self.pub_pulse_z.publish(self.pulse_z)  

    # callback to kill motor
    def callback_kill(self,data):
        if data.data == node_name:
            rospy.signal_shutdown(node_name)

    def listener(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

# Main function 
if __name__ == '__main__':
    rospy.init_node(node_name, anonymous = True)

    try:
        mcy = behavior()
        mcy.listener()

    except rospy.ROSInterruptException as e:
        print(e)
