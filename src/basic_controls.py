#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
from sensor_msgs.msg import Image
import copy
import cv2
from cv_bridge import CvBridge, CvBridgeError

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster

import json

from random import random
from math import sin

server = None
menu_handler = MenuHandler()
br = None
counter = 0

def frameCallback( msg ):
    global counter, br
    time = rospy.Time.now()
    br.sendTransform( (0, 0, sin(counter/140.0)*2.0), (0, 0, 0, 1.0), time, "base_link", "moving_frame" )
    counter += 1


def processFeedback( feedback ):
    # print(feedback)

    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo( s + ": button click" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo( s + ": pose changed")
# TODO
#          << "\nposition = "
#          << feedback.pose.position.x
#          << ", " << feedback.pose.position.y
#          << ", " << feedback.pose.position.z
#          << "\norientation = "
#          << feedback.pose.orientation.w
#          << ", " << feedback.pose.orientation.x
#          << ", " << feedback.pose.orientation.y
#          << ", " << feedback.pose.orientation.z
#          << "\nframe: " << feedback.header.frame_id
#          << " time: " << feedback.header.stamp.sec << "sec, "
#          << feedback.header.stamp.nsec << " nsec" )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo( s + ": mouse down" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo( s + ": mouse up" + mp + "." )
    server.applyChanges()


def rand( min_, max_ ):
    return min_ + random()*(max_-min_)

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker


#####################################################################
# Marker Creation

bridge = CvBridge()


def json_callback(feedback):
    createJSONFile('dataset/test', data)


def button_callback(feedback):
    global save_image
    global name_image
    global number_image
    # global D
    # D['collections'][number_image] = {'data': 'left_camera' 'right_camera'}
    name_image = "left_camera"
    save_image = 1
    rospy.Subscriber('left/image_raw', Image, image_callback)
    save_image = 1
    rospy.Subscriber('center/image_raw', Image, image_callback_1)
    save_image = 1
    rospy.Subscriber('center2/image_raw', Image, image_callback_2)
    save_image = 1
    rospy.Subscriber('five/image_raw', Image, image_callback_3)
    save_image = 1
    rospy.Subscriber('right/image_raw', Image, image_callback_4)
    # number_image += 1
    # name_image = "center2_camera"
    # global sub_once_3
    # sub_once_3 = rospy.Subscriber('center2/image_raw', Image, image_callback)
    # name_image = "center3_camera"
    # global sub_once_4
    # sub_once_4 = rospy.Subscriber('five/image_raw', Image, image_callback)
    # name_image = "right_camera"
    # global sub_once_5
    # sub_once_5 = rospy.Subscriber('right/image_raw', Image, image_callback)


def image_callback(msg):
    global save_image
    global number_image
    global name_image

    if save_image == 1:
        print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg
            # if i == 1:
            cv2.imwrite('dataset/' + str(name_image) + '_' + str(number_image) + '.jpg', cv2_img)
            data['collections'][number_image]['data']['left_camera']['data_file'] = 'left_camera_' + str(number_image) + '.jpg'
            # save_image = 0

def image_callback_1(msg):
    global save_image
    global number_image
    global name_image
    global D1
    if save_image == 1:
        print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg
            # if i == 1:
            cv2.imwrite('dataset/center_camera_' + str(number_image) + '.jpg', cv2_img)
            data['collections'][number_image]['data']['center_camera']['data_file'] = 'center_camera_' + str(
                number_image) + '.jpg'
            # sub_once_2.unregister()
            # number_image += 1
            save_image = 0

def image_callback_2(msg):
    global save_image
    global number_image
    global name_image
    if save_image == 1:
        print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg
            # if i == 1:
            cv2.imwrite('dataset/center2_camera_' + str(number_image) + '.jpg', cv2_img)
            data['collections'][number_image]['data']['center2_camera']['data_file'] = 'center2_camera_' + str(
                number_image) + '.jpg'
            # sub_once_2.unregister()
            # number_image += 1
            save_image = 0

def image_callback_3(msg):
    global save_image
    global number_image
    global name_image
    if save_image == 1:
        print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg
            # if i == 1:
            cv2.imwrite('dataset/center3_camera_' + str(number_image) + '.jpg', cv2_img)
            data['collections'][number_image]['data']['center3_camera']['data_file'] = 'center3_camera_' + str(
                number_image) + '.jpg'
            # sub_once_2.unregister()
            # number_image += 1
            save_image = 0


def image_callback_4(msg):
    global save_image
    global number_image
    global name_image
    if save_image == 1:
        print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg
            # if i == 1:
            cv2.imwrite('dataset/right_camera_' + str(number_image) + '.jpg', cv2_img)
            data['collections'][number_image]['data']['right_camera']['data_file'] = 'right_camera_' + str(number_image) + '.jpg'
            # sub_once_2.unregister()
            number_image += 1
            save_image = 0


def makeMenuMarker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    int_marker.scale = 1

    int_marker.name = "context_menu"
    int_marker.description = "Context Menu\n(Right Click)"

    # make one control using default visuals
    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.description="Options"
    control.name = "menu_only_control"
    int_marker.controls.append(copy.deepcopy(control))

    # make one control showing a box
    marker = makeBox( int_marker )
    control.markers.append( marker )
    control.always_visible = True
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )


def createJSONFile(output_file, D):
    print("Saving the json output file to " + str(output_file) + ", please wait, it could take a while ...")
    f = open(output_file, 'w')
    json.encoder.FLOAT_REPR = lambda f: ("%.6f" % f)  # to get only four decimal places on the json file
    print >> f, json.dumps(D, indent=2, sort_keys=True)
    f.close()
    print("Completed.")


class AutoTree(dict):
    def __missing__(self, key):
        value = self[key] = type(self)()
        return value


if __name__=="__main__":
    rospy.init_node("basic_controls")
    br = TransformBroadcaster()
    global save_image
    global number_image
    global data
    number_image = 0
    data = AutoTree()
    data['calibration_config']['calibration_pattern']['size'] = 0.019
    data['calibration_config']['calibration_pattern']['dimensions'] = {0: 9, 1: 6}
    data['sensors'] = {'left_camera': {'camera_info': {}}, 'right_camera': {'camera_info': {}}}

    i = 0

    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.01), frameCallback)

    server = InteractiveMarkerServer("basic_controls")

    menu_handler.insert("Collect Data", callback=button_callback)
    menu_handler.insert("Save Json File", callback=json_callback)

    makeMenuMarker(Point(0, 0, 0))

    server.applyChanges()

    rospy.spin()

