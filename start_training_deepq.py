#-----------------------------------------------------------------------------
#-------Old code -------------------------------------------------------------
#-----------------------------------------------------------------------------

import gym
import rospy
from baselines import deepq
from openai_ros.task_envs.cartpole_stay_up import stay_up

def callback(lcl, _glb):
    # stop training if reward exceeds 199
    is_solved = lcl['t'] > 100 and sum(lcl['episode_rewards'][-101:-1]) / 100 >= 199
    return is_solved

def main():
    print("starting training with deepq")      
    rospy.init_node('cartpole_training', anonymous=True, log_level=rospy.WARN)
    env = gym.make("CartPoleStayUp-v0")
    act = deepq.learn(
        env,
        network='mlp',
        lr=1e-3,
        total_timesteps=100000,
        buffer_size=50000,
        exploration_fraction=0.1,
        exploration_final_eps=0.02,
        print_freq=10,
        callback=callback
    )
    print("Saving model to cartpole_model.pkl")
    act.save("cartpole_model.pkl")

if __name__ == '__main__':
    main()
    
    
#-----------------------------------------------------------------------------
#-------New adjusted code with ProxyGymEnv------------------------------------
#-----------------------------------------------------------------------------


from baselines import deepq
# --------------new-------------------------
from ProxyEnv_RL_side import ProxyGymEnv
from gym import spaces
import numpy as np
# --------------end new---------------------

def callback(lcl, _glb):
    # stop training if reward exceeds 199
    is_solved = lcl['t'] > 100 and sum(lcl['episode_rewards'][-101:-1]) / 100 >= 199
    return is_solved

def main():
    print("starting training with deepq")      
    
    # --------------new-------------------------
    action_space = spaces.Discrete(4)
    high = np.array([
        2.5 * 2,
        np.finfo(np.float32).max,
        0.7 * 2,
        np.finfo(np.float32).max])
    observation_space = spaces.Box(-high, high)
    env = ProxyGymEnv(action_space=action_space, observation_space=observation_space)
    # --------------end new---------------------
    
    act = deepq.learn(
        env,
        network='mlp',
        lr=1e-3,
        total_timesteps=100000,
        buffer_size=50000,
        exploration_fraction=0.1,
        exploration_final_eps=0.02,
        print_freq=10,
        callback=callback
    )
    print("Saving model to cartpole_model.pkl")
    act.save("cartpole_model.pkl")

if __name__ == '__main__':
    main()    