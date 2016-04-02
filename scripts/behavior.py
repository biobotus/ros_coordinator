#!/usr/bin/python

# imports
from ErrorCode import error_code
from ros_behavior.msg import IntList, FloatList
import rospy
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Float32, String, Int32, Bool


class Behavior():
    def __init__(self):
        # ROS init
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10Hz

        # ROS subscriptions
        self.subscriber = rospy.Subscriber('New_Step', Float32MultiArray, \
                                           self.callback_pos)
        self.subscriber = rospy.Subscriber('New_Step_Step', Int32MultiArray, \
                                           self.callback_pos_step)    # Pour test
        self.subscriber = rospy.Subscriber('Error', String, self.callback_error)
        #self.subscriber = rospy.Subscriber('Done_Move',String,self.callback_done_move)

        # ROS publishments
        self.motor_kill = rospy.Publisher('Motor_Kill', String, queue_size=10)
        self.platform_init = rospy.Publisher('Platform_Init', Bool, queue_size=10)
        self.pub_pulse_xy = rospy.Publisher('Pulse_XY', IntList, queue_size=10)
        self.pub_pulse_z = rospy.Publisher('Pulse_Z', IntList, queue_size=10)
        self.step_state = rospy.Publisher('Step_Done', Bool, queue_size=10)


        self.delta_x = 0
        self.delta_y = 0
        self.delta_simplepip = 0
        self.delta_multipip = 0
        self.delta_gripper = 0

        self.actual_pos_x = 0
        self.actual_pos_y = 0
        self.actual_pos_simplepip = 0
        self.actual_pos_multipip = 0
        self.actual_pos_gripper = 0

        self.new_pos_x = 0.0
        self.new_pos_y = 0.0
        self.new_pos_simplepip = 0.0
        self.new_pos_multipip = 0.0
        self.new_pos_gripper = 0.0

        self.pulse_x = 0
        self.pulse_y = 0
        self.pulse_simplepip = 0
        self.pulse_multipip = 0
        self.pulse_gripper = 0

        self.z_id = 0

        # Constants
        self.dist_step = 0.127
        self.mode_step = 0.25  # 4 pulses per step
        self.pulse = self.dist_step * self.mode_step

        # Error control
        self.err_pulse_x = 0
        self.err_pulse_y = 0
        self.err_pulse_simplepip = 0
        self.err_pulse_multipip = 0
        self.err_pulse_gripper = 0




    # Callback position
    def callback_pos(self,data):
        self.new_pos_x = data.data[0]
        self.new_pos_y = data.data[1]
        self.new_pos_simplepip = data.data[2]
        self.new_pos_multipip = data.data[2]
        self.new_pos_gripper = data.data[2]
        self.behavior_output_pulse()

    # Callback position for step
    def callback_pos_step(self,data):
        step_x = data.data[0]
        step_y = data.data[1]

        #Publish number of pulse for all axis
        pulse_XY = IntList()
        pulse_XY.data = [step_x, step_y]
        print(pulse_XY.data)
        self.pub_pulse_xy.publish(pulse_XY)


    # Error management
    def callback_error(self,data):
        self.platform_init.publish(True)

    # Compute and publish the number of pulse for each axes
    def behavior_output_pulse(self):
        self.delta_x = self.new_pos_x - self.actual_pos_x
        self.delta_y = self.new_pos_y - self.actual_pos_y
        self.delta_simplepip = self.new_pos_simplepip - self.actual_pos_simplepip
        self.delta_multipip = self.new_pos_multipip - self.actual_pos_multipip
        self.delta_gripper = self.new_pos_gripper - self.actual_pos_gripper

        pulse_temp_x = self.delta_x / self.pulse
        pulse_temp_y = self.delta_y / self.pulse
        pulse_temp_simplepip = self.delta_simplepip / self.pulse
        pulse_temp_multipip = self.delta_multipip / self.pulse
        pulse_temp_gripper = self.delta_gripper / self.pulse

        self.pulse_x = int(round(pulse_temp_x))
        self.pulse_y = int(round(pulse_temp_y))
        self.pulse_simplepip = int(round(pulse_temp_simplepip))
        self.pulse_multipip = int(round(pulse_temp_multipip))
        self.pulse_gripper = int(round(pulse_temp_gripper))

        # Adjust values if error is greater than 1 pulse
        self.err_pulse_x += pulse_temp_x - self.pulse_x
        self.err_pulse_y += pulse_temp_y - self.pulse_y
        self.err_pulse_simplepip += pulse_temp_simplepip - self.pulse_simplepip
        self.err_pulse_multipip += pulse_temp_multipip - self.pulse_multipip
        self.err_pulse_gripper += pulse_temp_gripper - self.pulse_gripper

        if self.err_pulse_x < -1:
            self.err_pulse_x += 1
            self.pulse_x -= 1
        elif self.err_pulse_x > 1:
            self.err_pulse_x -= 1
            self.pulse_x += 1

        if self.err_pulse_y < -1:
            self.err_pulse_y += 1
            self.pulse_y -= 1
        elif self.err_pulse_y > 1 :
            self.err_pulse_y -= 1
            self.pulse_y += 1

        if self.err_pulse_simplepip < -1:
            self.err_pulse_simplepip += 1
            self.pulse_simplepip -= 1
        elif self.err_pulse_simplepip > 1:
            self.err_pulse_simplepip -= 1
            self.pulse_simplepip += 1

        if self.err_pulse_multipip < -1:
            self.err_pulse_multipip += 1
            self.pulse_multipip -= 1
        elif self.err_pulse_multipip > 1:
            self.err_pulse_multipip -= 1
            self.pulse_multipip += 1

        if self.err_pulse_gripper < -1:
            self.err_pulse_gripper += 1
            self.pulse_gripper -= 1
        elif self.err_pulse_gripper > 1:
            self.err_pulse_gripper -= 1
            self.pulse_gripper += 1


        #Publish number of pulse for all axis
        pulse_XY = IntList()
        pulse_XY.data = [self.pulse_x, self.pulse_y]
        print(pulse_XY.data)
        self.pub_pulse_xy.publish(pulse_XY)

        if (self.pulse_simplepip):
            self.z_id = 0
        elif (self.pulse_multipip):
            self.z_id = 1
        elif (self.pulse_gripper):
            self.z_id = 2

       # z_tools = IntList()
       # z_tools.data = [self.pulse_simplepip, self.pulse_multipip, self.pulse_gripper]
       # Pulse_Z = [self.z_id, z_tools.data]
       # self.pub_pulse_z.publish(Pulse_Z)


    def listener(self):
        # rospy.spin()
        try:
            while True:
                a = raw_input("Pres enter to init platform / CTRL+D to quit\n")
                self.platform_init.publish(True)
                print("Published init...")
        except EOFError:
            return

# Main function
if __name__ == '__main__':

    try:
        bh = Behavior()
        bh.listener()

    except rospy.ROSInterruptException as e:
        print(e)

