#! /usr/bin/env python
import Pyro4
from Pyro4 import naming
import gym
import threading

# Create a custom environment class that contains the functions: 
# step(action), reset(), close()
# import this class as gymclass
from my_environment import environment_class as gymclass

# This proxyID has to be the same as in the ProxyEnv on the Reinforcement Learning side. 
# You only need to change this, if you want to run multiple, different training 
# sessions at the same time
proxyID='Env1'


#-----------------------------------------------------------------------------
#---------------Init Area - run any code you want to initialize once ---------
#-----------------------------------------------------------------------------
#import rospy #for exmaple when using ros, you can start a rosnode
#import j2n6s300_test #perhaps import a file, which has to be loaded
#rospy.init_node('j2n6s300_gym', anonymous=True, log_level=rospy.WARN)






#-----------------------------------------------------------------------------
#-------DONE! You dont have to touch any of the code below--------------------
#-----------------------------------------------------------------------------

  

class NameServer(object):
    def __init__(self):
        thread = threading.Thread(target=self.start_name_server)
        thread.start()

    def start_name_server(self):
        while True:
            try: 
                print("Creating new Pyro4 name server") 
                # creates name server in own thread, so it doesnt have to be run manually in console   
                Pyro4.naming.startNSloop()    
            except  Exception as e:   
                print(e) 
                break   
        
start_name_server = NameServer() # start the Pyro4 Name Server class

#-----------------------------------------------------------------------------
#---------------------Convert our Class to ProxyClass-------------------------
#-----------------------------------------------------------------------------

ExposedClass = Pyro4.expose(gymclass) 

while True:    
    try: 
        ns = Pyro4.locateNS()
        print("Found Pyro4 name server")  # find the name server
        break
    except  Exception as e:   
        print(e)
                     
daemon = Pyro4.Daemon()     # make a Pyro daemon
uri = daemon.register(ExposedClass)    # register the exposed class rather than the library class itself
ns.register("GymEnvProxy." + proxyID, uri)   # register the object with a name in the name server

print('Proxy Gym Environment is Ready!')
daemon.requestLoop()    # start the event loop of the server to wait for calls