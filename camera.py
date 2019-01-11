#!/usr/bin/env python

import socket
import numpy as np
import cv2
import os
import time
import struct
from pyueye import ueye

class CameraInHand(object):
    def __init__(self):
        self.hCam = ueye.HIDS(1)  # 0: first available camera;  1-254: The camera with the specified camera ID
        self.sInfo = ueye.SENSORINFO()
        self.cInfo = ueye.CAMINFO()
        self.pcImageMemory = ueye.c_mem_p()
        self.MemID = ueye.int()
        self.rectAOI = ueye.IS_RECT()
        self.pitch = ueye.INT()
        self.nBitsPerPixel = ueye.INT(24)  # 24: bits per pixel for color mode; take 8 bits per pixel for monochrome
        self.channels = 3  # 3: channels for color mode(RGB); take 1 channel for monochrome
        self.m_nColorMode = ueye.INT()  # Y8/RGB16/RGB24/REG32
        self.bytes_per_pixel = int(self.nBitsPerPixel / 8)

        self.f_length = (652.7396, 653.3803)  # FocalLength
        self.p_center = (321.8104, 259.7124)  # PrincipalPoint
        self.intrinsic = np.array([[self.f_length[0], 0, self.p_center[0]],
                                    [0, self.f_length[1], self.p_center[1]],
                                    [0, 0, 1]],dtype=np.float)
        self.offset = [0,0,50]  #milimeter
        # ---------------------------------------------------------------------------------------------------------------------------------------
        print("In hand camera initilized")

    def start(self):
        # Starts the driver and establishes the connection to the camera
        nRet = ueye.is_InitCamera(self.hCam, None)
        if nRet != ueye.IS_SUCCESS:
            print("is_InitCamera error")
            exit(nRet)

        # Reads out the data hard-coded in the non-volatile camera memory and writes it to the data structure that cInfo points to
        nRet = ueye.is_GetCameraInfo(self.hCam, self.cInfo)
        if nRet != ueye.IS_SUCCESS:
            print("is_GetCameraInfo error")
            exit(nRet)

        # You can query additional information about the sensor type used in the camera
        nRet = ueye.is_GetSensorInfo(self.hCam, self.sInfo)
        if nRet != ueye.IS_SUCCESS:
            print("is_GetSensorInfo ERROR")
            exit(-1)

        nRet = ueye.is_ResetToDefault(self.hCam)
        if nRet != ueye.IS_SUCCESS:
            print("is_ResetToDefault ERROR")
            exit(nRet)

        # Set display mode to DIB
        nRet = ueye.is_SetDisplayMode(self.hCam, ueye.IS_SET_DM_DIB)
        # Set the right color mode
        if int.from_bytes(self.sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_BAYER:
            # setup the color depth to the current windows setting
            ueye.is_GetColorDepth(self.hCam, self.nBitsPerPixel, self.m_nColorMode)
            bytes_per_pixel = int(self.nBitsPerPixel / 8)
            print("IS_COLORMODE_BAYER: ", )
            print("\tm_nColorMode: \t\t", self.m_nColorMode)
            print("\tnBitsPerPixel: \t\t", self.nBitsPerPixel)
            print("\tbytes_per_pixel: \t\t", self.bytes_per_pixel)
            print()

        elif int.from_bytes(self.sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_CBYCRY:
            # for color camera models use RGB32 mode
            self.m_nColorMode = ueye.IS_CM_BGRA8_PACKED
            self.nBitsPerPixel = ueye.INT(32)
            self.bytes_per_pixel = int(self.nBitsPerPixel / 8)
            print("IS_COLORMODE_CBYCRY: ", )
            print("\tm_nColorMode: \t\t", self.m_nColorMode)
            print("\tnBitsPerPixel: \t\t", self.nBitsPerPixel)
            print("\tbytes_per_pixel: \t\t", self.bytes_per_pixel)
            print()

        elif int.from_bytes(self.sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_MONOCHROME:
            # for color camera models use RGB32 mode
            self.m_nColorMode = ueye.IS_CM_MONO8
            self.nBitsPerPixel = ueye.INT(8)
            self.bytes_per_pixel = int(self.nBitsPerPixel / 8)
            print("IS_COLORMODE_MONOCHROME: ", )
            print("\tm_nColorMode: \t\t", self.m_nColorMode)
            print("\tnBitsPerPixel: \t\t", self.nBitsPerPixel)
            print("\tbytes_per_pixel: \t\t", self.bytes_per_pixel)
            print()

        else:
            # for monochrome camera models use Y8 mode
            self.m_nColorMode = ueye.IS_CM_MONO8
            self.nBitsPerPixel = ueye.INT(8)
            self.bytes_per_pixel = int(self.nBitsPerPixel / 8)
            print("else")

        # Can be used to set the size and position of an "area of interest"(AOI) within an image
        nRet = ueye.is_AOI(self.hCam, ueye.IS_AOI_IMAGE_GET_AOI, self.rectAOI, ueye.sizeof(self.rectAOI))
        if nRet != ueye.IS_SUCCESS:
            print("is_AOI ERROR")

        self.width = self.rectAOI.s32Width
        self.height = self.rectAOI.s32Height

        # Prints out some information about the camera and the sensor
        print("Camera model:\t\t", self.sInfo.strSensorName.decode('utf-8'))
        print("Camera serial no.:\t", self.cInfo.SerNo.decode('utf-8'))
        print("Maximum image width:\t", self.width)
        print("Maximum image height:\t", self.height)
        print()

        # Allocates an image memory for an image having its dimensions defined by width and height and its color depth defined by nBitsPerPixel
        nRet = ueye.is_AllocImageMem(self.hCam, self.width, self.height, self.nBitsPerPixel, self.pcImageMemory, self.MemID)
        if nRet != ueye.IS_SUCCESS:
            print("is_AllocImageMem ERROR")
        else:
            # Makes the specified image memory the active memory
            nRet = ueye.is_SetImageMem(self.hCam, self.pcImageMemory, self.MemID)
            if nRet != ueye.IS_SUCCESS:
                print("is_SetImageMem ERROR")
                exit(nRet)
            else:
                # Set the desired color mode
                nRet = ueye.is_SetColorMode(self.hCam, self.m_nColorMode)

        # Activates the camera's live video mode (free run mode)
        nRet = ueye.is_CaptureVideo(self.hCam, ueye.IS_DONT_WAIT)
        if nRet != ueye.IS_SUCCESS:
            print("is_CaptureVideo ERROR")
            exit(nRet)

        # Enables the queue mode for existing image memory sequences
        nRet = ueye.is_InquireImageMem(self.hCam, self.pcImageMemory, self.MemID, self.width, self.height, self.nBitsPerPixel, self.pitch)
        if nRet != ueye.IS_SUCCESS:
            print("is_InquireImageMem ERROR")
            exit(nRet)
        else:
            print("Press q to leave the programm")

        return nRet

    def get_data(self):
        array = ueye.get_data(self.pcImageMemory, self.width, self.height, self.nBitsPerPixel, self.pitch, copy=False)
        frame = np.reshape(array, (self.height.value, self.width.value, self.bytes_per_pixel))
        #img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        return frame

    def resizeImage(self, image_data, marginW, marginH, sizewh_tuple):
        RGB_img = cv2.cvtColor(image_data, cv2.COLOR_BGRA2RGB)
        cut_img = RGB_img[marginH[0]:marginH[1], marginW[0]:marginW[1], :]
        resized_img = cv2.resize(cut_img, sizewh_tuple)
        return resized_img

    def __del__(self):
        # Releases an image memory that was allocated using is_AllocImageMem() and removes it from the driver management
        ueye.is_FreeImageMem(self.hCam, self.pcImageMemory, self.MemID)
        # Disables the hCam camera handle and releases the data structures and memory areas taken up by the uEye camera
        ueye.is_ExitCamera(self.hCam)

# DEPRECATED CAMERA CLASS FOR REALSENSE WITH ROS
# ----------------------------------------------

# import rospy
# from realsense_camera.msg import StreamData

# class Camera(object):


#     def __init__(self):

#         # Data options (change me)
#         self.im_height = 720
#         self.im_width = 1280

#         # RGB-D data variables
#         self.color_data = np.zeros((self.im_height,self.im_width,3))
#         self.depth_data = np.zeros((self.im_height,self.im_width))
#         self.intrinsics = np.zeros((3,3))

#         # Start ROS subscriber to fetch RealSense RGB-D data
#         rospy.init_node('listener', anonymous=True)
#         rospy.Subscriber("/realsense_camera/stream", StreamData, self.realsense_stream_callback)

#         # Recording variables
#         self.frame_idx = 0
#         self.is_recording = False
#         self.recording_directory = ''

#     # ROS subscriber callback function
#     def realsense_stream_callback(self, data):
#         tmp_color_data = np.asarray(bytearray(data.color))
#         tmp_color_data.shape = (self.im_height,self.im_width,3)
#         tmp_depth_data = np.asarray(data.depth)
#         tmp_depth_data.shape = (self.im_height,self.im_width)
#         tmp_depth_data = tmp_depth_data.astype(float)/10000
#         tmp_intrinsics = np.asarray(data.intrinsics)
#         tmp_intrinsics.shape = (3,3)

#         self.color_data = tmp_color_data
#         self.depth_data = tmp_depth_data
#         self.intrinsics = tmp_intrinsics

#         if self.is_recording:
#             tmp_color_image = cv2.cvtColor(tmp_color_data, cv2.COLOR_RGB2BGR)
#             cv2.imwrite(os.path.join(self.recording_directory, '%06d.color.png' % (self.frame_idx)), tmp_color_image)
#             tmp_depth_image = np.round(tmp_depth_data * 10000).astype(np.uint16) # Save depth in 1e-4 meters
#             cv2.imwrite(os.path.join(self.recording_directory, '%06d.depth.png' % (self.frame_idx)), tmp_depth_image)
#             self.frame_idx += 1
#         else:
#             self.frame_idx = 0

#         time.sleep(0.1)

#     # Start/stop recording RGB-D video stream
#     def start_recording(self, directory):
#         self.recording_directory = directory
#         self.is_recording = True
#     def stop_recording(self):
#         self.is_recording = False

