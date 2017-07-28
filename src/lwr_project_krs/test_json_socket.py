#! /usr/bin/python
import sys
import time
import numpy as np
import rospy

from jsonsocket import Client

host = '127.0.0.1'
port = 3333

def main():
  rospy.init_node('client_json')
  
  # Client code:
  client = Client()
  #client.connect(host, port).send({"type": "move_joint", 
                                    #"params":{
                                     #"speed": 100,
                                     #"relative": False,
                                     #"array": [1.0,0.,0.0,-1.57,0.0,1.57,0.]
                                     #}
                                   #})
                                   
  #client.connect(host, port).send({"type": "move_joint", 
                                    #"params":{
                                     #"speed": 10,
                                     #"relative": True,
                                     #"array": [0.1,0,0,0,0,0,0]
                                     #}
                                   #})
                                   
  #client.connect(host, port).send({"type": "move_tool", 
                                    #"params":{
                                      #"speed": 10,
                                      #"relative": True,
                                      #"in_tool": True,
                                      #"array": [0,0,0.1,0,0,0],
                                      #"stop_on_force": True,
                                      #"max_force": 0.5
                                     #}
                                   #})
                                   
  #client.connect(host, port).send({"type": "move_tool", 
                                    #"params":{
                                      #"speed": 10,
                                      #"relative": False,
                                      #"in_tool": False,
                                      #"array": [-0.2,-0.3,0.4,3.14,0,0],
                                      #"stop_on_force": True,
                                      #"max_force": 0.5
                                     #}
                                   #})
  
  #client.connect(host, port).send({"type": "home","params":{"speed": 100}})  
  
  #client.connect(host, port).send({"type": "stop"})  
  #client.connect(host, port).send({"type": "unstop"})
  client.connect(host, port).send({"type": "get_status"})
  
  #response = client.recv()
  
  client.close()
    
if __name__ == "__main__":
    main()