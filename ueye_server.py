import socket
import threading
import numpy as np
from camera import CameraInHand
import argparse
import struct
import selectors

class CameraServer(object):
    def __init__(self, ip='', port=60000, color_mode='RGB'):
        self.optflag = 1
        self.thread_alive = False
        print("[DEBUG]UEye_Ser: UEye camera server Constructor")
        self.server_ip = ""
        self.server_port = port
        self.data_bytes = bytearray()
        self.data_ready = False
        self.camera = CameraInHand()
        self.data_lock = threading.Condition()
        self.marginX=[170,1086]
        self.marginY=[17,711]
        self.img_size=(400,300)
        self.img_color_mode = color_mode

    def start(self):
        #start the camera
        self.camera.start()

        #start server
        self.halt()
        # connect to server
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.server_socket.bind(('',self.server_port))
        except socket.error as exc:
            print('[ERROR]UEye Ser: Create connection: %s' % exc)
            return -1
        print("[INFO]UEye Ser: Server started at port:%d" % self.server_port)
        # create recieve thread
        action_thread = threading.Thread(target=self.threadFunction)
        action_thread.daemon = True
        action_thread.start()
        return 1

    def halt(self):
        self.thread_alive = False
        print("[INFO]UEye Ser: Halt\n")

    def disConnect(self):
        self.server_socket.close()
        print("[INFO]UEye Ser: Connection CLOSED")

    def __writeToBuf(self, data):
        # print("Write to buffer!")
        #acquire the lock
        self.data_lock.acquire()
        self.data_ready = False
        #write data
        self.data_bytes = data.copy()
        self.data_ready = True
        #Notify another reader
        self.data_lock.notify()
        self.data_lock.release()

    def __readFromBuf(self):
        self.data_lock.acquire()
        while (not self.data_ready):
            self.data_lock.wait()
        ret = self.data_bytes.copy()
        self.data_lock.release()
        return ret

    def threadFunction(self):
        self.thread_alive = True
        while (self.thread_alive):
            self.server_socket.listen(5)
            print('[INFO]UEye Ser: Server listening...')
            conn, addr = self.server_socket.accept()
            print('Connected to client %s .' % str(addr))
            while(True):
                data = conn.recv(1024)
                if not data:
                    break
                image = self.camera.get_data()
                #Also do convert image
                if(self.img_color_mode=='RGB'):
                    image_rsz=self.camera.resizeImage(image,self.marginX, self.marginY,self.img_size)
                    self.sendImage(image_rsz)
                elif(self.img_color_mode=='BGRA'): #raw
                    self.sendImage(image)
                print('[INFO]UEye Ser: Capture and Send Image...', end='')
                send_data = self.__readFromBuf()
                conn.sendall(send_data)
                print(len(send_data))
            if not self.thread_alive:
                break
        self.disConnect()
        print("[INFO]UEye_Ser: Server thread finished")


    def sendImage(self, image_data, blocking=True):
        ret = -1
        #get image size
        image_size = image_data.shape
        if(len(image_size)!=3):
            print("[ERROR]UEye_Ser: Wrong image size!")
            return ret
        image_size = np.asarray(image_size, dtype=np.uint32)

        #pack with camera intrinsic
        cam_f = np.asarray(self.camera.f_length, dtype=np.float)
        cam_c = np.asarray(self.camera.p_center, dtype=np.float)
        #header length is 4*3 + 8*2 + 8*2 = 44
        package_header = bytearray(image_size) + bytearray(cam_f) + bytearray(cam_c)
        #get data
        package_data = bytearray(image_data)
        #write to buffer for sending
        self.__writeToBuf(package_header + package_data)
        ret = 1
        return ret

    def __del__(self):
        self.thread_alive = False
        self.disConnect()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Take camera image')
    parser.add_argument('--CM', dest='image_color_mode', action='store', default='RGB',
                        help='Color mode: RGB or BGRA')
    args = parser.parse_args()
    print('init camera')
    server = CameraServer()
    server.start()
    image = server.camera.get_data()
    server.sendImage(image)
    while(True):
        c = input('Give command:')
        if(c=='q'):
            break
        elif(c=='s'):
            print('Take and send image')
            image = server.camera.get_data()
            server.sendImage(image)

    server.thread_alive = False
    server.disConnect()
    print('Client end!')