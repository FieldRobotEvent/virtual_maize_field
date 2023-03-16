#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 31 05:26:51 2021

@author: fre
"""
import rospy
from gazebo_msgs.srv import GetModelState, SpawnModel
from std_msgs.msg import String
from urdf_parser_py.urdf import URDF

counter = 0
robot_name = ""


MARKER = """
<sdf version="1.6">  
  <model name="{name}"> 
    <static>true</static> 
    <pose frame="">0 0 0 0 0 0</pose> 
    <link name="marker_link"> 
      <visual name="visual"> 
        <geometry> 
          <cylinder> 
            <radius>0.375</radius> 
            <length>1</length> 
          </cylinder> 
        </geometry> 
        <material> 
          <ambient>{ambient}</ambient>
          <diffuse>{diffuse}</diffuse> 
          <specular>{specular}</specular> 
          <emissive>{emissive}</emissive>
        </material> 
      </visual> 
      <self_collide>0</self_collide> 
      <enable_wind>0</enable_wind> 
      <kinematic>0</kinematic> 
    </link> 
  </model> 
</sdf>"
"""


def callback(data):
    global counter
    global robot_name

    ms = gms(robot_name, "")
    ms.pose.position.z += 1

    name = "marker_%d" % counter
    counter += 1

    if data.data == "weed":
        ambient = "1.0 0.0 0.0 0.4"
        diffuse = "1.0 0.0 0.0 0.4"
        specular = "1.0 0.0 0.0 0.4"
        emissive = "0.0 0.0 0.0 0.0"
    elif data.data == "litter":
        ambient = "0.0 0.0 1.0 0.4"
        diffuse = "0.0 0.0 1.0 0.4"
        specular = "0.0 0.0 1.0 0.4"
        emissive = "0.0 0.0 0.0 0.0"

    marker = MARKER.format(
        name=name,
        ambient=ambient,
        diffuse=diffuse,
        specular=specular,
        emissive=emissive,
    )
    spawn_model(name, marker, "marker", ms.pose, "world")


if __name__ == "__main__":
    rospy.init_node("detection_spawner", anonymous=True)

    rospy.loginfo("Wating for gazebo services...")

    rospy.wait_for_service("gazebo/spawn_sdf_model")
    rospy.wait_for_service("/gazebo/get_model_state")

    gms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    rospy.loginfo("gazebo services started")

    has_printed = False
    while not rospy.has_param("robot_description") and not rospy.is_shutdown():
        if not has_printed:
            print("Waiting for the robot description in the param server")
            has_printed = True

        rospy.sleep(0.5)

    rospy.loginfo("Found the robot description in the param server")
    robot_name = URDF.from_parameter_server().name

    rospy.Subscriber("fre_detections", String, callback)
    rospy.spin()
