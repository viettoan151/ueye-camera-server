# -*- coding: utf-8 -*-

# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

# -*- coding: utf-8 -*-

import socket
import numpy as np
import matplotlib.pyplot as plt
import os
import time
import struct
from ueye_server import CameraServer
from ueye_client import CameraClient

import cv2

x = [170, 1086]
y = [17, 711]
if __name__ == '__main__':
    print('init camera')
    server = CameraServer()
    server.start()
    image = server.camera.get_data()
    server.sendImage(image)
    client = None
    while(True):
        c = input('Give command:')
        if(c=='q'):
            break
        elif(c=='s'):
            print('Take and send image')
            image = server.camera.get_data()
            server.sendImage(image)
        elif(c=='r'):
            if(client is not None):
                print('Recieve:')
                color_img=client.get_data()
                print('Image size:{}'.format((client.im_height,client.im_width,client.im_channel)))
                tran_img = cv2.cvtColor(color_img, cv2.COLOR_BGRA2RGB)
                cut_img=tran_img[y[0]:y[1],x[0]:x[1],:]
                resized_image = cv2.resize(cut_img, (400, 300))
                #cv2.imwrite('%06d.png' %cnt, cv2.cvtColor(tran_img, cv2.COLOR_RGB2BGR))
                #cv2.imwrite('cut%06d.png' %cnt, cv2.cvtColor(cut_img, cv2.COLOR_RGB2BGR))
                #cv2.imwrite('resize%06d.png' %cnt, cv2.cvtColor(resized_image, cv2.COLOR_RGB2BGR))
                #noised_image = noise.noisy("gauss", resized_image)
                #print(color_img)
                plt.subplot(221)
                plt.imshow(color_img)
                plt.subplot(222)
                plt.imshow(tran_img)
                plt.subplot(223)
                plt.imshow(cut_img)
                plt.subplot(224)
                plt.imshow(resized_image)
                plt.show()
            else:
                print('No client, please create by \'c\'')
        elif(c=='c'):
            print('Start client')
            client=CameraClient()

    server.thread_alive = False
    server.disConnect()
    print('Client end!')
        
        
        
        
        
        