#!/usr/bin/env python3
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32
from std_srvs.srv import Trigger, TriggerResponse

class OdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self.veh_name = rospy.get_namespace().strip("/")
        rospy.loginfo("[OdometryNode]: Vehicle Name = %s" %self.veh_name)

        # Get static parameters
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)

        self._Ntotal = 135
        self._travelledLeft = 0.0
        self._travelledRight = 0.0
        self._commandLeft = 0.0
        self._commandRight = 0.0
        self._previousTicksLeft = 0.0
        self._ticksAtStartLeft = 0.0
        self._initializedLeftTicks = False
        self._previousTicksRight = 0.0
        self._ticksAtStartRight = 0.0
        self._initializedRightTicks = False
        self._calibrationDistance = 2.0

        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_data_left) #TODO: topic name
        self.sub_encoder_ticks_right = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_data_right) #TODO: topic name
        self.sub_executed_commands = rospy.Subscriber(f'/{self.veh_name}/wheels_driver_node/wheels_cmd_executed', WheelsCmdStamped, self.cb_executed_commands)

        # Publishers
        self.pub_integrated_distance_left = rospy.Publisher(f'/{self.veh_name}/integrated_distance_left', Float32, queue_size=10)
        self.pub_integrated_distance_right = rospy.Publisher(f'/{self.veh_name}/integrated_distance_right', Float32, queue_size=10)

        # Services
        self.serv_wheel_calibrator_ = rospy.Service(f'/{self.veh_name}/get_wheel_radius', Trigger, self.wheel_calibrator)

        self.log("Initialized")

    def cb_encoder_data_left(self, msg):
        """ Update encoder distance information from ticks. Tick tata is given as total number of ticks, forward and backward ticks are counted as the same
        """
        if(not self._initializedLeftTicks):
            self._ticksAtStartLeft = msg.data
            self._previousTicksLeft = msg.data
            self._initializedLeftTicks = True

        #if(self._commandLeft>=0.0): #going forward
        #    self._travelledLeft += 2.0 * np.pi * self._radius * ((msg.data - self._previousTicksLeft)/self._Ntotal)
        #elif(self._commandLeft<0.0): #going backward
        #    self._travelledLeft -= 2.0 * np.pi * self._radius * ((msg.data - self._previousTicksLeft)/self._Ntotal)
        
        self._travelledLeft += 2.0 * np.pi * self._radius * ((msg.data - self._previousTicksLeft)/self._Ntotal)

        self._previousTicksLeft = msg.data


    def cb_encoder_data_right(self, msg):
        """ Update encoder distance information from ticks.
        """
        if(not self._initializedRightTicks):
            self._ticksAtStartRight = msg.data
            self._previousTicksRight = msg.data
            self._initializedRightTicks = True

        #if(self._commandRight>=0.0): #going forward
        #    self._travelledRight += 2.0 * np.pi * self._radius * ((msg.data - self._previousTicksRight)/self._Ntotal)
        #elif(self._commandRight<0.0): #going backward
        #    self._travelledRight -= 2.0 * np.pi * self._radius * ((msg.data - self._previousTicksRight)/self._Ntotal)

        self._travelledRight += 2.0 * np.pi * self._radius * ((msg.data - self._previousTicksRight)/self._Ntotal)

        self._previousTicksRight = msg.data


    def cb_executed_commands(self, msg):
        """ Use the executed commands to determine the direction of travel of each wheel.
        """
        self._commandLeft = msg.vel_left
        self._commandRight = msg.vel_right

    def wheel_calibrator(self, request):
        
        wheelRadiusLeft = (self._calibrationDistance * self._Ntotal) / (2.0 * np.pi * (self._previousTicksLeft - self._ticksAtStartLeft))
        wheelRadiusRight = (self._calibrationDistance * self._Ntotal) / (2.0 * np.pi * (self._previousTicksRight - self._ticksAtStartRight))
        wheelRadius = (wheelRadiusLeft + wheelRadiusRight) / 2.0
        self.reset()

        return TriggerResponse(
        success=True,
        message="Assuming you travelled {0}m, your wheel radius is: {1}m. Variables are reset.".format(self._calibrationDistance,wheelRadius)
        )

    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():

            self.pub_integrated_distance_left.publish(self._travelledLeft)
            self.pub_integrated_distance_right.publish(self._travelledRight)
            rospy.loginfo("_travelledLeft: %f" %self._travelledLeft)
            rospy.loginfo("_travelledRight: %f" %self._travelledRight)

            rate.sleep()

    def reset(self):
        self._initializedLeftTicks = False
        self._initializedRightTicks = False
        self._travelledRight = 0.0
        self._travelledLeft = 0.0
        rospy.loginfo("[WheelEncoderOdometry]: Reset!")


if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    # Keep it spinning to keep the node alive
    rospy.loginfo("wheel_encoder_node is up and running...")
    node.run()

    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")