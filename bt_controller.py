# -*- encoding: utf-8 -*-
'''
                                          
 _____     _        _____     _       _   
|   __|___| |___   | __  |___| |_ ___| |_ 
|   __|  _| | -_|  |    -| . | . | . |  _|
|_____|_| |_|___|  |__|__|___|___|___|_|  
                                          
@author: Víctor Mayoral Vilches <victor@erlerobot.com>
@description: A bluetooth server written in python for the android client of the robot Erle.
'''
from bluetooth import *
import binascii
import threading


class BT_Controller:
  """
  """

  def __init__(self, thrust_d = 0, pitch_d = 0, roll_d = 0, yaw_d = 0):
    self.thrust_d = thrust_d
    self.pitch_d = pitch_d
    self.roll_d = roll_d
    self.yaw_d = yaw_d
    self.t = threading.Thread(target=self.server, args = (self.thrust_d,))

  def getThrust(self):
    return self.thrust_d

  def run(self):          
    self.t.daemon = True
    self.t.start()
    # t.join()    

  def stop(self):
    self.t.exit()

  def server(self, thrust):
    """
    """
    while True:
      server_sock=BluetoothSocket( RFCOMM )
      server_sock.bind(("",PORT_ANY))
      server_sock.listen(2)

      port = server_sock.getsockname()[1]

      uuid = "00001101-0000-1000-8000-00805F9B34FB"

      advertise_service( server_sock, "SampleServer",
                         service_id = uuid,
                         service_classes = [ uuid, SERIAL_PORT_CLASS ],
                         profiles = [ SERIAL_PORT_PROFILE ],
    #                     protocols = [ OBEX_UUID ]
                          )

      print("Waiting for connection on RFCOMM channel %d" % port)

      client_sock, client_info = server_sock.accept()
      print("Accepted connection from ", client_info)

      try:
          while True:
              data = client_sock.recv(1024)
              if len(data) == 0: break              
              firstByte = data[0]
              firstByte_hexlify = binascii.hexlify(data[0])

              if firstByte == "U":
                """
                The received data follows the following pattern:
                55 [U]
                01 [left joystick "intensity"]
                05 [left joystick "angle"]
                00 [right joystick "intensity"]
                00 [right joystick "angle"]
                """
                # print "U received"
                # print "thrust: "+str(int(binascii.hexlify(data[1]),16)*10)
                self.thrust_d = int(binascii.hexlify(data[1]),16)*10
              elif firstByte == "T":
                print "T received"
              elif firstByte == "A":
                print "A received"
              elif firstByte == "B":
                print "B received"
              elif firstByte == "C":
                print "C received"
              elif firstByte == "D":
                print "D received"
              else:              
                # print data
                print "-----not recognized-----"
                for d in data:
                  print binascii.hexlify(d)
      
      except IOError:
          #pass
          print("disconnected")
          client_sock.close()
          server_sock.close()    