# ros_behavior

Behavior is management ROS node used to link the planner
to platform_control's nodes. Behavior is the manager of the
cells in general, it translate and divide planner's tasks


It received these ROS topics:

    -**New_Step, received from planner**
    -**Done_Module, received from all platform_control's nodes**
    -**Error, received from all plaform_control's nodes**

The information reveived from planner is a python dictionnary
ROS topics that behavior manage and redistribute the informations
to the right platform_control nodes. The dictionnary is that form:

"{"module_type": "pipette_s", "params": {"name": "pos", "args":
{"x": float(mm), "y": float(mm), "z": float(mm)}}}"

If the name is pos like in this example the position in the
dictionnary are translate in number of pulse by behavior and
sended to the right platform_control nodes. If the name is
manip, behavior threat the args and translate the dictionnary
information in function of the module.

Done_Module is the ROS topics that indicate to behavior when
a platform_control node has finish his task.

Error is a string ROS topics that behavior received and threat
in function of the error code. It can bring to a platform
initialization or to a total stop of the entire cells for
example.

Behavior publish these topics :
    -**Motor_Kill**
    -**Platform_Init**
    -**Pulse_XY**
    -**Pulse_Z**
    -**Pulse_SP**
    -**Pulse_MP**
    -**Gripper_Pos**
    -**Step_Done**

Motor_Kill is sended to stop motor movement of the desire axis.

Platform_Init is sended for the homing.

Pulse_XY is a intList that contains the number of pulse sended to
X and Y axis motors, sames for Pulse_Z but this one also contains
and id that motor_control_z read to know wich Z axis to move.

Pulse_SP is sended to the motor_control_sp to control the volume
taken by the simple pipette, same for Pulse_MP.

These topics output are calculated from an empirical test equation
in function of the pipette's tip used.

Gripper_Pos is a dictionnary containing the gripper angles for each
motor.

Step_Done is the ROS topics sended to planner to indicate that all
of the New_Step task are done by the cells.
