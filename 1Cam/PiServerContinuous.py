import io
import socket
import threading
import struct
import picamera
import time
import sys

class PiServer(object):
    def __init__(self):
        self.sock = socket.socket()
        self.camera = picamera.PiCamera()
        self.camera.resolution = (640,480)

    def connectServer(self,port):
        self.sock.bind(('172.24.1.1',port))
        self.sock.listen(2)

    def startServer(self):
        while True:
            conn,addr = self.sock.accept()
            self.createNewClient(conn,self.camera)

    def closeServer(self):
        self.sock.close()
        self.camera.close()

    def createNewClient(self,conn,camera):
        newClient = clientThread(conn,camera)
        newClient.startClientThread()

class clientThread(object):
    def __init__(self,conn,camera):
        self.conn = conn
        self.sendStream = conn.makefile('rb')
        self.camera = camera

    def startClientThread(self):
        print 'startClientThread'
        t1_send = threading.Event()
        t1_stop = threading.Event()
        while True:
            data = self.conn.recv(1024)
            if data == '1':
                print 'Starting continuous capture'
                t1 = threading.Thread(target=continuousCaptureThread,args=(self, t1_send, t1_stop))
                t1.daemon = True
                t1.start()
            if data == '2':
                t1_send.set()
            if data == '3':
                print 'Stopping continuous capture and closing connection'
                t1_stop.set()
                t1.join()
                self.conn.close()
                break


def continuousCaptureThread(clientThread, send_event, stop_event):
    stream = io.BytesIO()
    for foo in clientThread.camera.capture_continuous(stream, format='jpeg', use_video_port=True):
        if stop_event.is_set():
            break

        if send_event.is_set():
            send_event.clear()
            clientThread.sendStream.write(struct.pack('<L', stream.tell()))
            clientThread.sendStream.flush()
            stream.seek(0)
            clientThread.sendStream.write(stream.read())
        stream.seek(0)
        stream.truncate()
    print 'Done loop'

if __name__=='__main__':
    ports = range(8000,8005)
    myServer = PiServer()
    myServer.camera.start_preview()
    while True:
        for port in ports:
            try:
                myServer.connectServer(port)
                myServer.startServer()
            except KeyboardInterrupt:
                print 'Keyboard interrupt'
                myServer.closeServer()
                sys.exit(0)
            except:
                print 'Error!, port = ' + str(port)
                time.sleep(0.5)
