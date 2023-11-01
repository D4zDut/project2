#!/usr/bin/env python3

import rospy
import numpy as np
import json
from std_msgs.msg import String
from project2.msg import human_msg


def bbox_camera_callback(data, processed_bbox_publisher_camera):
    bbox_data = json.loads(data.data)
    msg = human_msg()

    for obj in bbox_data:
        bbox = obj.get("bbox")
        distance = obj.get("distance", 0)
        angle_bearing = obj.get("angle", [0, 0])[0]
        angle_elevation = obj.get("angle", [0, 0])[1]
        human_position_x = distance * np.cos(angle_elevation) * np.sin(angle_bearing)
        human_position_y = distance * np.cos(angle_elevation) * np.cos(angle_bearing)

        if bbox and len(bbox) == 4:
            msg.human_header.frame_id = "human1"
            msg.human_header.stamp = rospy.Time.now()
            msg.human_location.position.x = human_position_x
            msg.human_location.position.y = human_position_y

            msg.human_location.orientation.w = 1.0

            msg.camera_location.position.x = human_position_x
            msg.camera_location.position.y = human_position_y

            processed_bbox_publisher_camera.publish(msg)


def bbox_lidar_callback(data, processed_bbox_publisher_lidar):
    bbox_data = json.loads(data.data)
    msg = human_msg()

    for obj in bbox_data:
        bbox = obj.get("bbox")
        distance = obj.get("distance", 0)
        angle_bearing = obj.get("angle", [0, 0])[0]
        angle_elevation = obj.get("angle", [0, 0])[1]
        human_position_x = distance * np.cos(angle_elevation) * np.sin(angle_bearing)
        human_position_y = distance * np.cos(angle_elevation) * np.cos(angle_bearing)

        if bbox and len(bbox) == 4:
            msg.human_header.frame_id = "human1"
            msg.human_header.stamp = rospy.Time.now()
            msg.human_location.position.x = human_position_x
            msg.human_location.position.y = human_position_y

            msg.human_location.orientation.w = 1.0

            msg.camera_location.position.x = human_position_x
            msg.camera_location.position.y = human_position_y

            processed_bbox_publisher_lidar.publish(msg)


def main():
    rospy.init_node("bbox_processor_node", anonymous=True)
    human_list = ["human1"]
    rospy.set_param("human_list", human_list)
    processed_bbox_publisher_camera = rospy.Publisher(
        "/human1_position_camera", human_msg, queue_size=10
    )

    processed_bbox_publisher_lidar = rospy.Publisher(
        "/human1_position_lidar", human_msg, queue_size=10
    )

    bbox_subcriber = rospy.Subscriber(
        "/bbox_topic",
        String,
        callback=lambda data: bbox_camera_callback(
            data, processed_bbox_publisher_camera
        ),
    )

    bbox_subcriber = rospy.Subscriber(
        "/bbox_topic",
        String,
        callback=lambda data: bbox_lidar_callback(data, processed_bbox_publisher_lidar),
    )

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
