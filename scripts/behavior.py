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
        self.subscriber = rospy.Subscriber('Error', String, self.callback_error)
        self.subscriber = rospy.Subscriber('Done_Move',String,self.callback_done_move)

        # ROS publishments
        self.motor_kill = rospy.Publisher('Motor_Kill', String, queue_size=10)
        self.platform_init = rospy.Publisher('Platform_Init', Bool, queue_size=10)
        self.pub_pulse_xy = rospy.Publisher('Pulse_XY', IntList, queue_size=10)
        self.pub_pulse_z = rospy.Publisher('Pulse_Z', IntList, queue_size=10)
        self.step_done = rospy.Publisher('Step_Done', Bool, queue_size=10)

        
        #Robot position inits
        self.delta_x = 0
        self.delta_y = 0
        self.delta_z = [0, 0, 0]

        self.actual_pos_x = 0.0
        self.actual_pos_y = 0.0
        self.actual_pos_z = [0.0, 0.0, 0.0]
        
        self.new_pos_x = 0.0
        self.new_pos_y = 0.0
        self.new_pos_z = [0.0, 0.0, 0.0]

        self.pulse_x = 0
        self.pulse_y = 0
        self.pulse_z = [0, 0, 0]

        # Constants
        self.dist_step_xy = 0.127
        self.mode_step_xy = 0.25  # 4 pulses per step
        self.pulse_cst_xy = self.dist_step_xy * self.mode_step_xy
        
        self.dist_step_z = 0.127
        self.mode_step_z = 0.25  # 4 pulses per step
        self.pulse_cst_z = self.dist_step_z * self.mode_step_z
        
        
        # Error control
        self.err_pulse_x = 0
        self.err_pulse_y = 0
        self.err_pulse_z = [0, 0, 0]
        
        # others
        self.step_dict = {}
        self.done_move = ""
        
    # Callback position
    def callback_new_step(self, data):
        try:
            self.step_dict = ast.literal_eval(data.data)
            assert type(self.step_dict) == dict 
            assert 'module_type' in self.step_dict
            getattr(self.__class__, "send_{}".format(self.step_dict['module_type']))(self)

        except (AssertionError, AttributeError) as e:
            print("Erreur : {}".format(e))
            return -1
            
    # Callback position for step
    def callback_pos_step(self,data):
        step_x = data.data[0]
        step_y = data.data[1]


        #Publish number of pulse for all axis
        pulse_XY = IntList()
        pulse_XY.data = [step_x, step_y]
        print(pulse_XY.data)
        self.pub_pulse_xy.publish(pulse_XY)

    def callback_done_move(self, data):
        
        try:
            assert data.data in self.done_move
            self.done_move.remove(data.data)
            
            if not self.done_move:
                self.actual_pos_x = self.new_pos_x
                self.actual_pos_y = self.new_pos_y
                self.actual_pos_z = self.new_pos_z
            
                print(self.actual_pos_x)
                print(self.actual_pos_y)
                print(self.actual_pos_z)
            
                self.step_done.publish(True)
                
        except (AssertionError, AttributeError) as e:
            print("Error : wrong done_move received: {}".format(e))
            return -1
        
    # Error management
    def callback_error(self,data):
        self.platform_init.publish(True)

    def send_pipette_s(self):
        if self.step_dict['params']['name'] == 'pos':
            return self.send_pos(0)
        #manip still to implement
        return 
        
        
    # Compute and publish the number of pulse for each axes
    def send_pos(self, z_id):

        try:
            assert isinstance(self.step_dict['params']['args']['x'], numbers.Real)
            assert isinstance(self.step_dict['params']['args']['y'], numbers.Real)
            assert isinstance(self.step_dict['params']['args']['z'], numbers.Real)
            
        except (AssertionError, AttributeError) as e:
            print("Error : wrong argument type {}".format(e))
            return None
            
            
        self.new_pos_x = self.step_dict['params']['args']['x']
        self.new_pos_y = self.step_dict['params']['args']['y']
        self.new_pos_z[z_id] = self.step_dict['params']['args']['z']
    
        self.delta_x = self.new_pos_x - self.actual_pos_x
        self.delta_y = self.new_pos_y - self.actual_pos_y
        self.delta_z[z_id] = self.new_pos_z[z_id] - self.actual_pos_z[z_id]


        pulse_temp_x = self.delta_x / self.pulse_cst_xy
        pulse_temp_y = self.delta_y / self.pulse_cst_xy
        pulse_temp_z = self.delta_z[z_id] / self.pulse_cst_z

        self.pulse_x = int(round(pulse_temp_x))
        self.pulse_y = int(round(pulse_temp_y))
        self.pulse_z[z_id] = int(round(pulse_temp_z))


        # Adjust values if error is greater than 1 pulse
        self.err_pulse_x += pulse_temp_x - self.pulse_x
        self.err_pulse_y += pulse_temp_y - self.pulse_y
        self.err_pulse_z[z_id] += pulse_temp_z - self.pulse_z[z_id]


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

        if self.err_pulse_z[z_id] < -1:
            self.err_pulse_z[z_id] += 1
            self.pulse_z[z_id] -= 1
        elif self.err_pulse_z[z_id] > 1:
            self.err_pulse_z[z_id] -= 1
            self.pulse_z[z_id] += 1
            
        #Publish number of pulse for all axis
        pulse_XY = IntList()
        pulse_XY.data = [self.pulse_x, self.pulse_y]
        print("pulse_xy : ", pulse_XY.data)
        self.pub_pulse_xy.publish(pulse_XY)

        z_tools = IntList()
        Pulse_Z = [z_id, self.pulse_z[z_id]]
        self.pub_pulse_z.publish(Pulse_Z)
        print("z_id : ", z_id)
        print("pulse_z : ", self.pulse_z[z_id])

        self.done_move = ['MotorControlXY', 'MotorControlZ']
            
        
    def listener(self):
        rospy.spin()

# Main function
if __name__ == '__main__':

    try:
        bh = Behavior()
        bh.listener()

    except rospy.ROSInterruptException as e:
        print(e)

