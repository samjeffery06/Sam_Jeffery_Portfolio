# Test of image capture and transmit times

import PiClientContinuous as pcc
import time

myClient = pcc.Client(8000)     #Create connection to server
myClient.startImageCapturing()

print 'Starting..'
sumTimes = 0
for i in range(20):
    time.sleep(0.2)
    startTime = time.time()
    img = myClient.getImage()   #Receive image from server
    transmitTime = time.time() - startTime
    sumTimes += transmitTime
    print 'Transmit time: ' + str(transmitTime) + ' s'

print 'Done transmitting..'
average = sumTimes/20
print 'Average transmit time: ' + str(average)

myClient.stopImageCapturing()
myClient.closeClient()

