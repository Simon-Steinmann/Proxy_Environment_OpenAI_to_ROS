#! /usr/bin/env python
import Pyro4
from Pyro4 import naming
import gym
import threading

# Input the id of a gym Environment you wish to use. You may have to import the 
# environment file first.
from openai_ros.task_envs.cartpole_stay_up import stay_up
env_id = 'CartPole-v0'

# This proxyID has to be the same as in the ProxyEnv on the Reinforcement Learning side. 
# You only need to change this, if you want to run multiple, different training 
# sessions at the same time
proxyID='Env1'


#-----------------------------------------------------------------------------
#-------DONE! You dont have to touch any of the code below--------------------
#-----------------------------------------------------------------------------



env = gym.make(env_id)
class gymclass(gym.Env):

    def seed(self, seed=None):
        seed = env.seed()
        return seed
        

    def step(self, action):
        obs, reward, done, info = env.step(action)        
        return obs, reward, done, info
        

    def reset(self):
        obs = env.reset()
        return obs


    def close(self):
        env.close()
        
        
    def get_variable(self, var_name): # make sure only standard python types are used
        return(vars(env).get(var_name))
        
        
    def set_variable(self, var_name, value): # make sure only standard python types are used
        dic = vars(env)
        dic[var_name] = value
        
#-----------------------------------------------------------------------------
#---------------------Start Pyro4 Name Server in own thread ------------------
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