#!/usr/bin/env python3

import rospy
import pandas as pd
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def spawn_model_at(x, y, z, model_name):
    model_path = "~/.gazebo/models/yellow_sphere/model.sdf"
    with open(os.path.expanduser(model_path), "r") as f:
        model_xml = f.read()

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model(model_name, model_xml, "", pose, "world")
        rospy.loginfo(f"✅ Spawned {model_name} at ({x:.2f}, {y:.2f})")
    except rospy.ServiceException as e:
        rospy.logerr(f"Spawn failed: {e}")

def main():
    rospy.init_node('spawn_path_markers')
    path = pd.read_csv("wps.csv", header=None).values[:, :2]  # x, y

    for i, (x, y) in enumerate(path[::10]):  # spawn every 20th waypoint
        spawn_model_at(x, y, 0.05, f"waypoint_{i}")

if __name__ == '__main__':
    import os
    main()

