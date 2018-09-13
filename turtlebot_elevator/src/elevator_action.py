#!/usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Float64
from turtlebot_elevator.msg import *
from kobuki_msgs.msg import DigitalOutput
#from dynamic_reconfigure.server import Server
import dynamic_reconfigure.client

pub = None
class ElevatorServer:
  
  def __init__(self):
    self.output_topic = rospy.get_param('~digital_out_topic','digital_output')
    self.action_name = rospy.get_param('~action_name','set_elevator')
    self.simulation = rospy.get_param('~simulation', False)
    

    self.feedback = SetElevatorFeedback()
    self.result = SetElevatorResult()
    if self.simulation:
      self.output_topic = "joint_elevator_controller/command"
      self.server = actionlib.SimpleActionServer(self.action_name, SetElevatorAction, self.setElevatorSimCb,False)
      self.pub = rospy.Publisher(self.output_topic, Float64, queue_size=1)
    else:
      self.server = actionlib.SimpleActionServer(self.action_name, SetElevatorAction, self.setElevatorCb,False)
      self.pub = rospy.Publisher(self.output_topic, DigitalOutput, queue_size=1)

    self.server.start()
    
  def setfootprint(self, radius):    	  
	  client = dynamic_reconfigure.client.Client("move_base/global_costmap")	  
	  client.update_configuration({"robot_radius":radius-0.2})	  
	  client = dynamic_reconfigure.client.Client("move_base/local_costmap")	  
	  client.update_configuration({"robot_radius":radius})
	  
  def setmaxvelocity(self, vel):
	  client = dynamic_reconfigure.client.Client("move_base/TebLocalPlannerROS")	  
	  client.update_configuration({"max_vel_x":vel})	  
	  client.update_configuration({"max_vel_theta":vel})
	  
  def setElevatorCb(self, goal):
    digital_output = DigitalOutput()
    digital_output.mask[0] = True
    digital_output.mask[1] = True

    if(goal.pos.value == SetPosition.RAISE):
      self.feedback.value = "RAISING"
      digital_output.values[0] = False
      digital_output.values[1] = True  
      self.setfootprint(0.75)
      
    elif(goal.pos.value == SetPosition.LOWER):
      self.feedback.value = "LOWERING"
      digital_output.values[1] = False
      digital_output.values[0] = True
      self.setfootprint(0.2)    
    else:
      self.feedback.value = "STOPPED"
      self.result.value = False
      self.server.set_aborted(self.result)
    
    self.pub.publish(digital_output)
    self.server.publish_feedback(self.feedback)
    d = rospy.Duration(4,0)
    rospy.sleep(d)

    self.result.value = True
    self.server.set_succeeded(self.result)
    return True

  def setElevatorSimCb(self, goal):
    value = Float64();
    if(goal.pos.value == SetPosition.RAISE):
      self.feedback.value = "RAISING"
      value.data= 1.0
      
      self.setfootprint(0.75)
      
    elif(goal.pos.value == SetPosition.LOWER):
      self.feedback.value = "LOWERING"
      value.data = 0.0    
      self.setfootprint(0.2)    
      self.setmaxvelocity(0.5)   
    else:
      self.feedback.value = "STOPPED"
      self.result.value = False
      self.server.set_aborted(self.result)
    
    self.pub.publish(value)
    self.server.publish_feedback(self.feedback)
    d = rospy.Duration(4,0)
    rospy.sleep(d)

    self.result.value = True
    self.server.set_succeeded(self.result)
    return True

if __name__ == '__main__':
  try:
    #print "AQUI" 
    rospy.init_node('elevator_node', anonymous=True)
    action_server = ElevatorServer()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
