#!/usr/bin/python

# imports
import ast
from biobot_ros_msgs.msg import FloatList, IntList
from ErrorCode import error_code
import numbers
import rospy
from std_msgs.msg import Bool, String

class Behavior():
    def __init__(self):
        # ROS init
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10Hz

        # ROS subscriptions
        self.subscriber = rospy.Subscriber('New_Step', String, \
                                           self.callback_new_step_abs)
        self.subscriber = rospy.Subscriber('New_Step_Rel', String, \
                                           self.callback_new_step_rel)
        self.subscriber = rospy.Subscriber('Error', String, self.callback_error)
        self.subscriber = rospy.Subscriber('Done_Module', String, self.callback_done_module)

        # ROS publishments
        self.motor_kill = rospy.Publisher('Motor_Kill', String, queue_size=10)
        self.platform_init = rospy.Publisher('Platform_Init', String, queue_size=10)
        self.pub_pulse_xy = rospy.Publisher('Pulse_XY', IntList, queue_size=10)
        self.pub_pulse_z = rospy.Publisher('Pulse_Z', IntList, queue_size=10)
        self.pub_pulse_sp = rospy.Publisher('Pulse_SP', IntList, queue_size=10)
        self.pub_pulse_mp = rospy.Publisher('Pulse_MP', IntList, queue_size=10)
        self.pub_gripper_pos = rospy.Publisher('Gripper_Pos', String, queue_size=10)
        self.step_done = rospy.Publisher('Step_Done', Bool, queue_size=10)
        self.refresh_pos = rospy.Publisher('Refresh_Pos', FloatList, queue_size=10)
        self.global_enable = rospy.Publisher('Global_Enable', Bool, queue_size=10)

        #Robot position inits
        self.delta_x = 0.0
        self.delta_y = 0.0
        self.delta_z = [0.0, 0.0, 0.0]

        self.actual_pos_x = None
        self.actual_pos_y = None
        self.actual_pos_z = [None, None, None]
        self.actual_pos_sp = 0.0
        self.actual_pos_mp = 0.0

        self.new_pos_x = 0.0
        self.new_pos_y = 0.0
        self.new_pos_z = [0.0, 0.0, 0.0]
        self.actual_vol_sp = 0
        self.actual_vol_mp = 0

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

        self.pip_lim = 70000  # Number of pulse limits

        # Pipette tips equations variables
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

        # Upper axis limits
        self.limit_x = 1050 #Ajouter message d'erreur et etre constant avec planner
        self.limit_y = 585
        self.limit_z = [320, 360, 280]

        # Volume limits
        self.limit_vol_up_sp = 500
        self.limit_vol_up_mp = 500
        self.limit_vol_down_sp = 0
        self.limit_vol_down_mp = 0

        # Speed limits (HZ)
        self.limit_spd_sp = 30000
        self.limit_spd_mp = 30000

        # Others
        self.behavior_step = False
        self.step_dict = {}
        self.done_module = []
        self.z_id = -1
        self.valid_motor_names = ['MotorControlXY', 'MotorControlZ']
        self.move_mode = 'abs'
        self.init_mod = []

    # Callback for new step
    def callback_new_step_abs(self, data):
        while self.done_module:
            self.rate.sleep()

        self.move_mode = 'abs'
        self.callback_new_step(data)

    # Callback for new step (special case for relative movements from web page)
    def callback_new_step_rel(self, data):
        while self.done_module:
            self.rate.sleep()

        self.move_mode = 'rel'
        self.callback_new_step(data)

    def callback_new_step(self, data):
        # Make sure Behavior knows where the platform is, else do an init
        if self.actual_pos_x is None or self.actual_pos_y is None \
                                      or None in self.actual_pos_z:
            self.step_dict = {"module_type": "init", "params": \
                             ['MotorControlXY', 'MotorControlZ']}
            actual_pos = False
            self.behavior_step = True

            self.send_init()

            while not actual_pos:
                self.rate.sleep()
                if not (self.actual_pos_x is None or \
                   self.actual_pos_y is None or None in self.actual_pos_z):
                    actual_pos = True

        try:
            self.step_dict = ast.literal_eval(data.data)
            assert type(self.step_dict) == dict
            assert 'module_type' in self.step_dict
            getattr(self.__class__, "send_{}".format(self.step_dict['module_type']))(self)

        except (AssertionError, AttributeError) as e:
            print("Error : {}".format(e))
            return None

    def callback_done_module(self, data):
        print('done_module: {}'.format(data.data))
        if data.data not in self.done_module:
            print("Error : wrong done_module received: {}".format(data.data))
            return None

        self.done_module.remove(data.data)

        if not self.done_module:
            if self.step_dict['module_type'] == 'init':

                if 'MotorControlXY' in self.init_mod :
                    self.actual_pos_x = 0.0
                    self.actual_pos_y = 0.0

                if 'MotorControlZ' in self.init_mod :
                    self.actual_pos_z = [0.0, 0.0, 0.0]

            else:
                if None not in [self.actual_pos_x, self.actual_pos_y, self.actual_pos_z[self.z_id]]:
                    if self.move_mode == 'abs':
                        self.actual_pos_x = self.new_pos_x
                        self.actual_pos_y = self.new_pos_y
                        self.actual_pos_z[self.z_id] = self.new_pos_z[self.z_id]

                    elif self.move_mode == 'rel':
                        self.actual_pos_x += self.new_pos_x
                        self.actual_pos_y += self.new_pos_y
                        self.actual_pos_z[self.z_id] += self.new_pos_z[self.z_id]

                self.actual_pos_sp += self.pulse_sp
                self.actual_pos_mp += self.pulse_mp
                self.pulse_mp = 0
                self.pulse_sp = 0

            self.end_of_done_module()

    def end_of_done_module(self):
        # Publish actual position for web interface
        refresh = FloatList()
        refresh.data = [self.actual_pos_x, self.actual_pos_y, \
                        self.actual_pos_z[0], self.actual_pos_z[1], \
                        self.actual_pos_z[2], self.actual_vol_sp]
        self.refresh_pos.publish(refresh)

        # If step origins from Behavior, don't publish Step Done
        if self.behavior_step:
            self.behavior_step = False
        else:
            print("Publishing step done")
            self.step_done.publish(True)

    # Error management
    def callback_error(self, data):
        self.actual_pos_x = None
        self.actual_pos_y = None
        self.actual_pos_z = [None, None, None]
        self.actual_pos_sp = 0
        self.actual_pos_mp = 0

        try:
            new_error_dict = ast.literal_eval(data.data)
            assert type(new_error_dict) == dict
            assert 'error_code' in new_error_dict
            assert 'name' in new_error_dict
            getattr(self.__class__, error_code[new_error_dict['error_code']])(self, new_error_dict['name'])

        except (AssertionError, AttributeError) as e:
            print("Error : {}".format(e))
            return None

    # send platform_init
    def send_init(self):
        try:
            assert type(self.step_dict['params']) == list

        except (AssertionError):
            print('Invalid params type for init : {}'.format(self.step_dict))
            return None

        if 'MotorControlZ' in self.step_dict['params'] and self.actual_pos_z == [0.0, 0.0, 0.0]:
            self.step_dict['params'].remove('MotorControlZ')

        if 'MotorControlXY' in self.step_dict['params'] and self.actual_pos_x == 0.0 and self.actual_pos_y == 0.0:
            self.step_dict['params'].remove('MotorControlXY')

        self.init_mod = self.step_dict['params'][:]

        for axis in self.step_dict['params']:
            if axis in self.valid_motor_names:
                self.done_module.append(axis)

        if self.done_module:
            if 'MotorControlZ' in self.step_dict['params']:
                self.platform_init.publish('MotorControlZ')
                self.step_dict['params'].remove('MotorControlZ')

                while 'MotorControlZ' in self.done_module:
                    self.rate.sleep()

            for axis in self.step_dict['params']:
                if axis in self.valid_motor_names:
                    self.platform_init.publish(axis)

        else:
            self.end_of_done_module()

    # Publish pipette_s topics in function of self.step_dict args
    def send_pipette_s(self):
        if self.step_dict['params']['name'] == 'pos':
            self.z_id = 0
            return self.send_pos()

        elif self.step_dict['params']['name'] == 'manip':
            vol = self.step_dict['params']['args']['vol']

            print(vol)
            print(self.limit_vol_up_sp)
            print(self.limit_vol_up_sp)

            self.actual_vol_sp = self.actual_vol_sp + vol

            print(self.actual_vol_sp)

            if self.actual_vol_sp > self.limit_vol_up_sp or self.actual_vol_sp < self.limit_vol_down_sp:
                self.actual_vol_sp = self.actual_vol_sp - vol
                print('vol limit')
                return None

            # Linear empiric relations
            if abs(vol) < 10 :
                self.pulse_sp = int(round(self.pip_slope_tip20*abs(vol) + \
                                                 self.pip_intercept_tip20))
            elif (abs(vol) >= 10) and (abs(vol) < 100):
                self.pulse_sp = int(round(self.pip_slope_tip200*abs(vol) + \
                                                 self.pip_intercept_tip200))
            elif (abs(vol) >= 100) and (abs(vol) < 600):
                self.pulse_sp = int(round(self.pip_slope_tip1000*abs(vol) + \
                                                 self.pip_intercept_tip1000))
            else:
                print("Error wrong volume entered")
                return None

            if vol < 0:
                self.pulse_sp *= -1

            # try :
            #     assert (self.pulse_sp + self.actual_pos_sp) < self.pip_lim
            #     assert (self.pulse_sp + self.actual_pos_sp) > 0

            # except AssertionError:
            #     print("Impossible SP manip, volume out of range: {}".format(vol))
            #     return None

            freq_sp = int(round(self.step_dict['params']['args']['speed'] * \
                                                   abs(self.pulse_sp / vol)))
            print(freq_sp)
            print(self.limit_spd_sp)

            if freq_sp < 0 or freq_sp > self.limit_spd_sp:
                self.actual_vol_sp = self.actual_vol_sp - vol
                print('speed limit')
                return None
            # Publish number of pulse for simple pip
            pulse_SP = IntList()
            pulse_SP.data = [freq_sp, self.pulse_sp]
            # print("pulse_sp : {}".format(pulse_SP.data))
            self.pub_pulse_sp.publish(pulse_SP)
            self.done_module.append('MotorControlSP')

    # Publish pipette_mp topics in function of self.step_dict args
    def send_pipette_m(self):
        if self.step_dict['params']['name'] == 'pos':
            self.z_id = 1
            return self.send_pos()

        elif self.step_dict['params']['name'] == 'manip':
            vol = self.step_dict['params']['args']['vol']

            # Linear empiric relations
            if abs(vol) < 10 :
                self.pulse_mp = int(round(self.pip_slope_tip20*abs(vol) + \
                                                 self.pip_intercept_tip20))
            elif (abs(vol) >= 10) and (abs(vol) < 100):
                self.pulse_mp = int(round(self.pip_slope_tip200*abs(vol) + \
                                                 self.pip_intercept_tip200))
            elif (abs(vol) >= 100) and (abs(vol) < 600):
                self.pulse_mp = int(round(self.pip_slope_tip1000*abs(vol) + \
                                                 self.pip_intercept_tip1000))
            else:
                print("Error wrong volume entered")
                return None

            if vol > 0:
                self.pulse_mp *= -1

            # try :
            #     assert (self.pulse_mp + self.actual_pos_mp) < self.pip_lim
            #     assert (self.pulse_mp + self.actual_pos_mp) > 0

            # except AssertionError:
            #     print("Impossible MP manip, volume out of range: {}".format(vol))
            #     return None

            freq_mp = int(round(self.step_dict['params']['args']['speed'] * \
                                                   abs(self.pulse_mp / vol)))

            # Publish number of pulse for simple pip
            pulse_mp = IntList()
            pulse_mp.data = [freq_mp, self.pulse_mp/4]  # TODO : Remove "/4"
            # print("pulse_mp : {}".format(pulse_mp.data))
            self.pub_pulse_mp.publish(pulse_mp)
            self.done_module.append('MotorControlMP')

    # Publish gripper topics in function of self.step_dict args
    def send_gripper(self):
        if self.step_dict['params']['name'] == 'pos':
            self.z_id = 2
            return self.send_pos()

        if self.step_dict['params']['name'] == 'manip':
            gripper = str(self.step_dict['params']['args'])
            self.pub_gripper_pos.publish(gripper)
            self.done_module.append('Gripper')

        else:
            print("Error with params name in dict: {}".format(e))
            return None

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

        if self.move_mode == 'abs':
            self.delta_x = self.new_pos_x - self.actual_pos_x
            self.delta_y = self.new_pos_y - self.actual_pos_y
            self.delta_z[self.z_id] = self.new_pos_z[self.z_id] - \
                                      self.actual_pos_z[self.z_id]

            if self.new_pos_x > self.limit_x or self.new_pos_x < 0:
                # Add warning for pos
                return None
            if self.new_pos_y > self.limit_y or self.new_pos_y < 0:
                # Add warning for pos
                return None
            if self.new_pos_z[self.z_id] > self.limit_z[self.z_id] or \
                                         self.new_pos_z[self.z_id] < 0:
                # Add warning for pos
                return None

        elif self.move_mode == 'rel':
            self.delta_x = self.new_pos_x
            self.delta_y = self.new_pos_y
            self.delta_z[self.z_id] = self.new_pos_z[self.z_id]

            abs_pos_x = (self.new_pos_x + self.actual_pos_x)
            abs_pos_y = (self.new_pos_y + self.actual_pos_y)
            abs_pos_z = (self.new_pos_z[self.z_id] + self.actual_pos_z[self.z_id])

            if  abs_pos_x > self.limit_x or abs_pos_x < 0:
                # Add warning for pos
                return None
            if  abs_pos_y  > self.limit_y or abs_pos_y < 0:
                # Add warning for pos
                return None
            if  abs_pos_z > self.limit_z[self.z_id] or abs_pos_z < 0:
                # Add warning for pos
                return None

        else:
            print("Invalid move mode: {0}".format(self.move_mode))
            return None

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
            self.done_module.append('MotorControlZ')

        # Publish number of pulse for all axis
        if self.pulse_x != 0 or self.pulse_y != 0:
            pulse_XY = IntList()
            pulse_XY.data = [self.pulse_x, self.pulse_y]
            # print("pulse_xy: {}".format(pulse_XY.data))
            self.pub_pulse_xy.publish(pulse_XY)

        if self.pulse_z != 0:
            while 'MotorControlXY' in self.done_module:
                self.rate.sleep()
            pulse_Z = IntList()
            pulse_Z.data = [self.z_id, self.pulse_z[self.z_id]]
            # print("pulse_z{0}: {1}".format(self.z_id, self.pulse_z[self.z_id]))
            self.pub_pulse_z.publish(pulse_Z)

    def global_disable(self, node):
        print("Error coming from {0}, disabling BioBot".format(node))
        self.global_enable.publish(False)
        self.step_done.publish(False)

    def motor_kill_err(self, axis):
        self.motor_kill.publish(axis)

    def platform_init_err(self, axis):
        self.platform_init.publish(axis)

    def listener(self):
        rospy.spin()

# Main function
if __name__ == '__main__':

    try:
        bh = Behavior()
        bh.listener()

    except rospy.ROSInterruptException as e:
        print(e)

