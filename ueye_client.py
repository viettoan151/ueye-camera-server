#!/usr/bin/env python

import socket
import numpy as np
import cv2
import os
import time
import struct
from pyueye import ueye

class CameraClient(object):

    def __init__(self):

        # Data options (change me)
        self.im_height = 480
        self.im_width = 640
        self.im_channel = 3
        self.tcp_host_ip = '127.0.0.1'
        #self.tcp_host_ip = '140.123.111.171'
        self.tcp_port = 60000
        self.buffer_size = 4098 # 4 KiB

        # Connect to server
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))

        self.camera_f = None
        self.camera_c = None
        print('Client init..')
        #self.get_data()


    def get_data(self):

        # Ping the server with anything
        self.tcp_socket.send(b'asdf')

        # Fetch TCP data:
        #     color camera intrinsics, 9 floats, number of bytes: 9 x 4
        #     depth scale for converting depth from uint16 to float, 1 float, number of bytes: 4
        #     depth image, self.im_width x self.im_height uint16, number of bytes: self.im_width x self.im_height x 2
        #     color image, self.im_width x self.im_height x 3 uint8, number of bytes: self.im_width x self.im_height x 3
        data_bytes = b''
        #get header
        data_bytes = self.tcp_socket.recv(self.buffer_size)
        (self.im_height,self.im_width,self.im_channel) = struct.unpack("<III", data_bytes[0:12])
        self.camera_f = list(struct.unpack("<dd", data_bytes[12:28]))
        self.camera_c = list(struct.unpack("<dd", data_bytes[28:44]))

        data_bytes = data_bytes[44:]
        while len(data_bytes) < (self.im_height*self.im_width*self.im_channel):
            data_bytes += self.tcp_socket.recv(self.buffer_size)

        # Reorganize TCP data into color and depth frame
        color_img = np.fromstring(data_bytes, np.uint8).reshape(self.im_height, self.im_width, self.im_channel)
        return color_img