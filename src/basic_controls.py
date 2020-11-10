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
import argparse

import rospy
import numpy as np
from sensor_msgs.msg import Image
import copy
import cv2
import glob
import os
from os.path import expanduser
from colorama import Fore
from cv_bridge import CvBridge, CvBridgeError

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster

import json

from random import random
from math import sin

global server
server = None
menu_handler = MenuHandler()
br = None
counter = 0


def save_image(name_cam, msg):
    global folder
    global folder2
    global intrinsic
    global number_image_intrinsic
    global number_image
    if intrinsic == 1:
        fold = folder2
        image = number_image_intrinsic

    else:
        fold = folder
        image = number_image
    print fold
    print("ill " + Fore.GREEN + "start " + Fore.RESET + name_cam + " image callback")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg
        cv2.imwrite(str(fold) + str(name_cam) + '_camera_' + str(image) + '.jpg', cv2_img)
        if intrinsic == 0:
            data['collections'][image]['data'][str(name_cam) + '_camera']['data_file'] = str(name_cam) + '_camera_' + str(number_image) + '.jpg'
        print("i" + Fore.RED + " finished " + Fore.RESET + name_cam + " image callback")

def frameCallback( msg ):
    global counter, br
    time = rospy.Time.now()
    br.sendTransform( (0, 0, sin(counter/140.0)*2.0), (0, 0, 0, 1.0), time, "base_link", "moving_frame" )
    counter += 1


def intrinsic_calibration_callback(feedback):
    global number_image_intrinsic
    global niic
    print("Im in intrinsic_calibration_callback")

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(5,8,0)
    objp = np.zeros((9 * 6, 3), np.float32)
    objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)
    objp = objp * 0.101
    # for name in list:
    #     print("Name")
    #     print name
    #     intrinsic_calibration(objp, name)
    if number_image_intrinsic >= niic:
        intrinsic_calibration(objp)
    else:
        print("\nPlease collect more data in order to perform the intrinsic calibration!!\n")
        print("You need more " + Fore.RED + str(niic - number_image_intrinsic - 1) + Fore.RESET + " collections!")


def intrinsic_calibration(objp):
    global folder
    global folder2
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    list = ['left', 'center', 'center2', 'center3', 'right']
    # list = ['left']
    for name in list:
        objpoints = []  # 3d point in real world space from left camera
        imgpoints = []  # 2d points in image plane from left camera.
        images = glob.glob(str(folder2) + str(name) + '_camera_*.jpg')
        # images = glob.glob(str(folder2) + str(name) + '-*.png')
        # print(images)
        number = 0
        for fname in images:
            # if number < 30:
            #     data['collections'][number]['data'][str(name) + '_camera']['data_file'] = str(name) + '_camera_' + str(number) + '.jpg'
            #     number += 1
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

            # If found, add object points, image points (after refining them)
            if ret == True:
                # print("found chessboard")
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)
        if imgpoints != []:
            print("Before calib")
            ret, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
            print("Matriz K, " + str(name))
            print('mtx')
            print(mtx)

            mtx.reshape(mtx.size, 1)
            print(mtx[0:3])
            print(mtx[3:6])
            print(mtx[6:9])
            print("Matriz D, " + str(name))
            print(dist)

            intrinsic_matrix = np.zeros((3, 3))
            intrinsic_matrix[0:3] = mtx[0:3]
            intrinsic_matrix[3:6] = mtx[3:6]
            intrinsic_matrix[6:9] = mtx[6:9]
            intrinsic_matrix.flatten()
            print('intrinsic_matrix')
            print(intrinsic_matrix)

            # data['sensors'][str(name) + '_camera']['camera_info']['K'][0:3] = np.array(mtx.reshape(mtx.size, 1))
            # data['sensors'][str(name) + '_camera']['camera_info']['K'][0:3] = intrinsic_matrix[0][0:3]
            # data['sensors'][str(name) + '_camera']['camera_info']['K'][3:6] = intrinsic_matrix[0][3:6]
            # data['sensors'][str(name) + '_camera']['camera_info']['K'][6:9] = intrinsic_matrix[0][6:9]
            # print('data[K]')
            # print(data['sensors'][str(name) + '_camera']['camera_info']['K'])

            # data['sensors'][str(name) + '_camera']['camera_info']['D'] = np.array(dist.reshape(dist.size, 1))
            for ind in range(mtx.size):
                data['sensors'][str(name) + '_camera']['camera_info']['K'][str(ind)] = mtx.reshape(mtx.size, 1)[ind][0]

            for ind1 in range(dist.size):
                data['sensors'][str(name) + '_camera']['camera_info']['D'][str(ind1)] = str(dist.reshape(dist.size, 1)[ind1][0])


        else:
            print(Fore.RED + "Couldn't find chessboard in any image of " + name + " camera!" + Fore.RESET)
        if data['sensors'][name+ '_camera']['camera_info']['K']:
            createJSONFile(str(folder) + 'test', data)
    return


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

#     if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
#         rospy.loginfo( s + ": button click" + mp + "." )
#     elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
#         rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
#     elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
#         rospy.loginfo( s + ": pose changed")
# # TODO
# #          << "\nposition = "
# #          << feedback.pose.position.x
# #          << ", " << feedback.pose.position.y
# #          << ", " << feedback.pose.position.z
# #          << "\norientation = "
# #          << feedback.pose.orientation.w
# #          << ", " << feedback.pose.orientation.x
# #          << ", " << feedback.pose.orientation.y
# #          << ", " << feedback.pose.orientation.z
# #          << "\nframe: " << feedback.header.frame_id
# #          << " time: " << feedback.header.stamp.sec << "sec, "
# #          << feedback.header.stamp.nsec << " nsec" )
#     elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
#         rospy.loginfo( s + ": mouse down" + mp + "." )
#     elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
#         rospy.loginfo( s + ": mouse up" + mp + "." )
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
    global folder
    createJSONFile(str(folder) + 'test', data)


def button_callback(feedback):
    global name_image
    global intrinsic
    global name_cam
    global number_image, handle, niic, server
    global number_image_intrinsic
    number_image += 1

    if number_image_intrinsic == niic - 1:
        server.erase("context_menu")
        menu_handler.insert("Calibrate intrinsic parameters", callback=intrinsic_calibration_callback)
        makeMenuMarker(Point(0, 0, 0))
        server.applyChanges()

    intrinsic = 0
    name_image = "left_camera"
    name_cam = "left"


def delete_intrinsic_callback(feedback):
    global folder2
    r = glob.glob(folder2 + '*')
    for i in r:
        os.remove(i)

def button_intrinsic_callback(feedback):
    global number_image_intrinsic
    global folder2
    global intrinsic
    global name_cam
    intrinsic = 1
    name_cam = "left"
    images = glob.glob(str(folder2) + 'left_camera_*.jpg')
    number_image_intrinsic = len(images)
    # images = glob.glob(str(folder2) + 'left_camera_*.jpg')
    # number_image_intrinsic += 1
    if number_image_intrinsic == niic - 1:
        server.erase("context_menu")
        menu_handler.insert("Calibrate intrinsic parameters", callback=intrinsic_calibration_callback)
        makeMenuMarker(Point(0, 0, 0))
        server.applyChanges()



def image_callback(msg):
    global number_image
    global name_image
    global name_cam

    if name_cam == "left":
        save_image(name_cam, msg)
        name_cam = "center"


def image_callback_1(msg):
    global number_image
    global name_image
    global name_cam

    if name_cam == "center":
        save_image(name_cam, msg)
        name_cam = "center2"


def image_callback_2(msg):
     
    global number_image
    global name_image
    global name_cam

    if name_cam == "center2":
        save_image(name_cam, msg)
        name_cam = "center3"

def image_callback_3(msg):
     
    global number_image
    global name_image
    global name_cam

    if name_cam == "center3":
        save_image(name_cam, msg)
        name_cam = "right"


def image_callback_4(msg):
     
    global number_image
    global name_image
    global name_cam


    if name_cam == "right":
        save_image(name_cam, msg)
        name_cam = "last"

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
    if data['sensors']['left_camera']['camera_info']['K']:
        print("Saving the json output file to " + str(output_file) + ", please wait, it could take a while ...")
        f = open(output_file, 'w')
        json.encoder.FLOAT_REPR = lambda f: ("%.6f" % f)  # to get only four decimal places on the json file
        print >> f, json.dumps(D, indent=2, sort_keys=True)
        f.close()
        print("Completed.")
    else:
        print('\nYou need to do the intrinsic calibration before you save the Json file!\nPlease collect enough data to do that sucessfully...\n')
        if number_image_intrinsic < niic - 1:
            print("You need more " + Fore.RED + str(niic - number_image_intrinsic - 1) + Fore.RESET + " collections!")



class AutoTree(dict):
    def __missing__(self, key):
        value = self[key] = type(self)()
        return value


if __name__=="__main__":

    ap = argparse.ArgumentParser()
    ap.add_argument("-niic", "--intrinsic_minimum_image", help="Minimum number of images required to perform an intrinsic calibration for all available cameras", type=int, required=True)
    args = vars(ap.parse_args())
    global niic
    global folder
    global folder2
    global number_image_intrinsic
    global intrinsic

    intrinsic = 0
    niic = args['intrinsic_minimum_image']


    # # First of all, delete all the files in dataset folder
    home = expanduser("~")
    test = home + '/catkin_ws/dataset/*'
    folder = home + '/catkin_ws/dataset/'
    folder2 = home + '/catkin_ws/dataset_intrinsic/'
    r = glob.glob(test)
    for i in r:
        os.remove(i)
    images = glob.glob(str(folder2) + 'left_camera_*.jpg')
    number_image_intrinsic = len(images)
    print number_image_intrinsic

    rospy.init_node("basic_controls")
    br = TransformBroadcaster()

    global name_cam
    name_cam = "first"

    rospy.Subscriber('left/image_raw', Image, image_callback)
    rospy.Subscriber('center/image_raw', Image, image_callback_1)
    rospy.Subscriber('center2/image_raw', Image, image_callback_2)
    rospy.Subscriber('five/image_raw', Image, image_callback_3)
    rospy.Subscriber('right/image_raw', Image, image_callback_4)
     
    global number_image
    global data
    global handle
    number_image = -1
    data = AutoTree()
    data['calibration_config']['calibration_pattern']['size'] = 0.101
    data['calibration_config']['calibration_pattern']['dimension'] = [9, 6]
    data['calibration_config']['sensor_order'] = ['left_camera', 'center_camera', 'center2_camera', 'center3_camera', 'right_camera']


    # data['sensors'] = {'left_camera': {'camera_info': {}}, 'right_camera': {'camera_info': {}}}

    # i = 0\

    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(1), frameCallback)
    server = InteractiveMarkerServer("basic_controls")

    menu_handler.insert("Collect Data", callback=button_callback)
    menu_handler.insert("Collect Data for Intrinsic Calibration", callback=button_intrinsic_callback)
    menu_handler.insert("Delete Intrinsic Calibration Data", callback=delete_intrinsic_callback)
    menu_handler.insert("Save Json File", callback=json_callback)
    # ---------------------------------------------
    # Nao esquecer de apagar linha abaixo
    # ----------------------------------------------
    if number_image_intrinsic >= niic:
        menu_handler.insert("Calibrate intrinsic parameters", callback=intrinsic_calibration_callback)

    makeMenuMarker(Point(0, 0, 0))

    server.applyChanges()

    rospy.spin()

