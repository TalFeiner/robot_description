#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client

def callback_inflation(config):
    rospy.loginfo("Config set to {inflation_radius}".format(**config))
def callback_TebLocalPlannerROS(config):
    rospy.loginfo("Config set to {dt_ref}, {dt_hysteresis}, {max_global_plan_lookahead_dist}, {min_obstacle_dist}, {costmap_obstacles_behind_robot_dist}, {costmap_obstacles_behind_robot_dist}".format(**config))


if __name__ == "__main__":
    rospy.init_node("dynamic_client")

    client_global_inflation = dynamic_reconfigure.client.Client("move_base/global_costmap/inflation_global", timeout=30, config_callback=callback_inflation)
    client_local_inflation = dynamic_reconfigure.client.Client("move_base/local_costmap/inflation", timeout=30, config_callback=callback_inflation)
    client_TebLocalPlannerROS = dynamic_reconfigure.client.Client("move_base/TebLocalPlannerROS", timeout=30, config_callback=callback_TebLocalPlannerROS)
    
    client_global_inflation.update_configuration({"inflation_radius":1.4})
    client_local_inflation.update_configuration({"inflation_radius":0.2})
    client_TebLocalPlannerROS.update_configuration({"dt_ref":0.3, "dt_hysteresis":0.1, "max_global_plan_lookahead_dist":3.0, "min_obstacle_dist":0.4, "costmap_obstacles_behind_robot_dist":0.3})
    