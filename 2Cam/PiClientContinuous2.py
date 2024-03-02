import io
import socket
import struct
import time
import cv2
import numpy as np
import threading
import sys

class Client(object):
    def __init__(self,port):
        self.sock = socket.socket()
        self.conn = None
        self.sock.connect(('192.168.137.39',port))
        self.conn = self.sock.makefile('wb')

    def closeClient(self):
        self.conn.close()
        self.sock.close()

    def startImageCapturing(self):
        self.sock.send('1')

    def getImage(self):
        npImg = None
        self.sock.send('2')
        imageLen = struct.unpack('<L', self.conn.read(struct.calcsize('<L')))[0]
        if imageLen:
            stream = io.BytesIO()
            stream.write(self.conn.read(imageLen))
            stream.seek(0)
            data = np.fromstring(stream.getvalue(), dtype=np.uint8)
            npImg = cv2.imdecode(data, 0)
        return npImg

    def stopImageCapturing(self):
        self.sock.send('3')

def getClient():
    ports = range(8003, 8005)
    while True:
        for port in ports:
            try:
                myClient = Client(port)
                return myClient
            except KeyboardInterrupt:
                print 'Keyboard interrupt'
                myClient.closeClient()
                sys.exit()
            except:
                print 'Error!, port = ' + str(port)
                time.sleep(0.5)
