#!/usr/bin/python

# imports
import ast
from ErrorCode import error_code
from ros_behavior.msg import IntList, FloatList
import rospy
import numbers
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Float32, String, Int32, Bool


class Behavior():
    def __init__(self):
        # ROS init
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10Hz

        # ROS subscriptions
        self.subscriber = rospy.Subscriber('New_Step', String, \
                                           self.callback_new_step)
        self.subscriber = rospy.Subscriber('New_Step_Step', Int32MultiArray, \
                                           self.callback_pos_step)    # Pour test
        self.subscriber = rospy.Subscriber('New_Step_XY', Int32MultiArray, self.callback_XY_step)
        self.subscriber = rospy.Subscriber('New_Step_Z', Int32MultiArray, self.callback_Z_step)
        self.subscriber = rospy.Subscriber('Error', String, self.callback_error)
        self.subscriber = rospy.Subscriber('Done_Module',String,self.callback_done_module)

        # ROS publishments
        self.motor_kill = rospy.Publisher('Motor_Kill', String, queue_size=10)
        self.platform_init = rospy.Publisher('Platform_Init', String, queue_size=10)
        self.pub_pulse_xy = rospy.Publisher('Pulse_XY', IntList, queue_size=10)
        self.pub_pulse_z = rospy.Publisher('Pulse_Z', IntList, queue_size=10)
        self.pub_pulse_sp = rospy.Publisher('Pulse_SP', IntList, queue_size=10)
        self.pub_gripper_pos = rospy.Publisher('Gripper_Pos', String, queue_size=10)
        self.step_done = rospy.Publisher('Step_Done', Bool, queue_size=10)

        #Robot position inits
        self.delta_x = 0
        self.delta_y = 0
        self.delta_z = [0, 0, 0]

        self.actual_pos_x = 0.0
        self.actual_pos_y = 0.0
        self.actual_pos_z = [0.0, 0.0, 0.0]
        self.actual_pos_sp = 0
        self.actual_pos_mp = 0

        self.new_pos_x = 0.0
        self.new_pos_y = 0.0
        self.new_pos_z = [0.0, 0.0, 0.0]

        self.pulse_x = 0
        self.pulse_y = 0
        self.pulse_z = [0, 0, 0]
        self.pulse_sp = 0
        self.pulse_mp = 0

        # Constants
        self.dist_step_xy = 0.127
        self.mode_step_xy = 0.25  # 4 pulses per step
        self.pulse_cst_xy = self.dist_step_xy * self.mode_step_xy

        self.dist_step_z = 0.04
        self.mode_step_z = 0.25  # 4 pulses per step
        self.pulse_cst_z = self.dist_step_z * self.mode_step_z

        self.pip_lim = 70000  # number of pulse limits

        # pipette tips equations variables
        self.pip_slope_tip20 = 97.61071086
        self.pip_intercept_tip20 = 63.73267033

        self.pip_slope_tip200 = 99.91081613
        self.pip_intercept_tip200 = 68.43652967

        self.pip_slope_tip1000 = 97.84199255
        self.pip_intercept_tip1000 = 328.9959479

        # Error control
        self.err_pulse_x = 0
        self.err_pulse_y = 0
        self.err_pulse_z = [0, 0, 0]

        # Others
        self.step_dict = {}
        self.done_module = []
        self.z_id = -1

    # Callback position
    def callback_new_step(self, data):
        try:
            self.step_dict = ast.literal_eval(data.data)
            assert type(self.step_dict) == dict
            assert 'module_type' in self.step_dict
            getattr(self.__class__, "send_{}".format(self.step_dict['module_type']))(self)

        except (AssertionError, AttributeError) as e:
            print("Error : {}".format(e))
            return None

    # Callback position for step
    def callback_pos_step(self,data):
        step_x = data.data[0]
        step_y = data.data[1]
        step_z = data.data[2]

        #Publish number of pulse for all axis
        pulse_XY = IntList()
        pulse_XY.data = [step_x, step_y]
        print(pulse_XY.data)

        self.pub_pulse_xy.publish(pulse_XY)

        pulse_Z = IntList()
        pulse_Z.data = [0, step_z]
        print(pulse_Z.data)

        self.pub_pulse_z.publish(pulse_Z)

    # Callback position for step
    def callback_XY_step(self,data):
        step_x = data.data[0]
        step_y = data.data[1]

        #Publish number of pulse for all axis
        pulse_XY = IntList()
        pulse_XY.data = [step_x, step_y]
        print(pulse_XY.data)

        self.pub_pulse_xy.publish(pulse_XY)

    # Callback position for step
    def callback_Z_step(self,data):
        z_id_test = data.data[0]
        step_z = data.data[1]

        pulse_Z = IntList()
        pulse_Z.data = [z_id_test, step_z]
        print(pulse_Z.data)

        self.pub_pulse_z.publish(pulse_Z)

    def callback_done_module(self, data):

        try:
            assert data.data in self.done_module

        except AssertionError:
            print("Error : wrong done_move received: {}".format(data.data))
            return None
        
        self.done_module.remove(data.data)
        
        print("Il reste {} dans done_move".format(self.done_module))
        if not self.done_module:
            self.actual_pos_x = self.new_pos_x
            self.actual_pos_y = self.new_pos_y
            self.actual_pos_z = self.new_pos_z[:]  # [:] important to clone the list
            self.actual_pos_sp = self.pulse_sp + self.actual_pos_sp
            self.actual_pos_mp = self.pulse_mp + self.actual_pos_mp
            self.pulse_mp = 0
            self.pulse_sp = 0
            print("Publishing step done")
            self.step_done.publish(True)

    # Error management
    def callback_error(self,data):
        pass
        #self.platform_init.publish(True)

    # Publish pipette_s topics in function of self.step_dict args
    def send_pipette_s(self):
        if self.step_dict['params']['name'] == 'pos':
            self.z_id = 0
            return self.send_pos()

        try:
            assert self.step_dict['params']['name'] == 'manip'

        except (AssertionError, AtributeError) as e:
            print("Error with params name in dict: {}".format(e))
            return None

        vol = self.step_dict['params']['args']['vol']

        if abs(vol) < 10 :
            self.pulse_sp = self.pip_slope_tip20*vol + self.pip_intercept_tip20
        elif (abs(vol) >= 10) and (abs(vol) < 100):
            self.pulse_sp = self.pip_slope_tip200*vol + self.pip_intercept_tip200
        elif (abs(vol) > 100) and (abs(vol) < 800):
            self.pulse_sp = self.pip_slope_tip1000*vol + self.pip_intercept_tip1000
        else :
            print("Error wrong volume entered")
            return None

        try :
            assert (self.pulse_sp + self.actual_pos_sp) > self.pip_lim
            assert (self.pulse_sp + self.actual_pos_sp) < 0

        except (AssertionError, AtributeError) as e:
            print("Impossible SP manip, volume out of range: {}".format(e))
            return None

        freq_sp = round((self.step_dict['params']['args']['speed'] * self.pulse_sp) / vol)

        #Publish number of pulse for simple pip
        pulse_SP = IntList()
        pulse_SP.data = [freq_sp, self.pulse_sp]
        print("pulse_sp : {}".format(pulse_SP.data))
        self.pub_pulse_sp.publish(pulse_SP)
        self.done_module.append('MotorControlSP')

    # Publish pipette_mp topics in function of self.step_dict args
    def send_pipette_mp(self):
        if self.step_dict['params']['name'] == 'pos':
            self.z_id = 1
            return self.send_pos()

        try:
            assert self.step_dict['params']['name'] == 'manip'

        except (AssertionError, AtributeError) as e:
            print("Error with params name in dict: {}".format(e))
            return None


        vol = self.step_dict['params']['args']['vol']

        if abs(vol) < 10 :
            self.pulse_mp = self.pip_slope_tip20*vol + self.pip_intercept_tip20
        elif (abs(vol) >= 10) and (abs(vol) < 100):
            self.pulse_mp = self.pip_slope_tip200*vol + self.pip_intercept_tip200
        elif (abs(vol) > 100) and (abs(vol) < 800):
            self.pulse_mp = self.pip_slope_tip1000*vol + self.pip_intercept_tip1000
        else :
            print("Error wrong volume entered")
            return None

        try :
            assert (self.pulse_mp + self.actual_pos_mp) > self.pip_lim
            assert (self.pulse_mp + self.actual_pos_mp) < 0

        except (AssertionError, AtributeError) as e:
            print("Impossible MP manip, volume out of range: {}".format(e))
            return None

        freq_mp = round((self.step_dict['params']['args']['speed'] * self.pulse_mp) / vol)
        self.pulse_mp = round(self.pulse_mp)
        #Publish number of pulse for simple pip
        pulse_MP = IntList()
        pulse_MP.data = [freq_mp, self.pulse_mp]
        print("pulse_mp : {}".format(pulse_MP.data))
        self.pub_pulse_mp.publish(pulse_MP)
        self.done_module.append('MotorControlMP')

    # Publish gripper topics in function of self.step_dict args
    def send_gripper(self):
        if self.step_dict['params']['name'] == 'pos':
            self.z_id = 2
            return self.send_pos()

        try:
            assert self.step_dict['params']['name'] == 'manip'

        except (AssertionError, AtributeError) as e:
            print("Error with params name in dict: {}".format(e))
            return None

        gripper = self.step_dict['params']['args']
        self.pub_gripper_pos.publish(gripper)
        self.done_module.append('Gripper')


    # Compute and publish the number of pulse for each axes
    def send_pos(self):
        try:
            assert isinstance(self.step_dict['params']['args']['x'], numbers.Real)
            assert isinstance(self.step_dict['params']['args']['y'], numbers.Real)
            assert isinstance(self.step_dict['params']['args']['z'], numbers.Real)

        except (AssertionError, AttributeError) as e:
            print("Error : wrong argument type {}".format(e))
            return None

        self.new_pos_x = self.step_dict['params']['args']['x']
        self.new_pos_y = self.step_dict['params']['args']['y']
        self.new_pos_z[self.z_id] = self.step_dict['params']['args']['z']
        
        self.delta_x = self.new_pos_x - self.actual_pos_x
        self.delta_y = self.new_pos_y - self.actual_pos_y
        self.delta_z[self.z_id] = self.new_pos_z[self.z_id] - self.actual_pos_z[self.z_id]

        pulse_temp_x = self.delta_x / self.pulse_cst_xy
        pulse_temp_y = self.delta_y / self.pulse_cst_xy
        pulse_temp_z = self.delta_z[self.z_id] / self.pulse_cst_z
        
        self.pulse_x = int(round(pulse_temp_x))
        self.pulse_y = int(round(pulse_temp_y))
        self.pulse_z[self.z_id] = int(round(pulse_temp_z))


        # Adjust values if error is greater than 1 pulse
        self.err_pulse_x += pulse_temp_x - self.pulse_x
        self.err_pulse_y += pulse_temp_y - self.pulse_y
        self.err_pulse_z[self.z_id] += pulse_temp_z - self.pulse_z[self.z_id]

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
            
        if self.err_pulse_z[self.z_id] < -1:
            self.err_pulse_z[self.z_id] += 1
            self.pulse_z[self.z_id] -= 1
        elif self.err_pulse_z[self.z_id] > 1:
            self.err_pulse_z[self.z_id] -= 1
            self.pulse_z[self.z_id] += 1

        
        if self.pulse_x != 0 or self.pulse_y != 0:
            self.done_module.append('MotorControlXY')
        if self.pulse_z[self.z_id] != 0:
            print(self.pulse_z)
            self.done_module.append('MotorControlZ')
        
        #Publish number of pulse for all axis
        if self.pulse_x != 0 or self.pulse_y != 0:
            print("pulse_xy: {}".format([self.pulse_x, self.pulse_y]))
            pulse_XY = IntList()
            pulse_XY.data = [self.pulse_x, self.pulse_y]
            self.pub_pulse_xy.publish(pulse_XY)
                        
        if self.pulse_z != 0:
            while 'MotorControlXY' in self.done_module:
                self.rate.sleep()
            z_tools = IntList()
            Pulse_Z = [self.z_id, self.pulse_z[self.z_id]]
            self.pub_pulse_z.publish(Pulse_Z)
        
    def listener(self):
        rospy.spin()

# Main function
if __name__ == '__main__':

    try:
        bh = Behavior()
        bh.listener()

    except rospy.ROSInterruptException as e:
        print(e)

