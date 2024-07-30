from concurrent.futures import thread
import sys


sys.path.append('..')

import time
import numpy as np
import threading


from MyPetoi.petoi_kinematics import PetoiKinematics
from MyPetoi.gaits import gaits


current_gait_name = 'walk'
current_gait = gaits['walk']
current_gait_frame_idx = 0

petoi_kinematics = PetoiKinematics(render_mode='2d')

lock = threading.Lock()



def switch_gait(gait_name):
    global gaits, current_gait_name, current_gait, current_gait_frame_idx, petoi_kinematics
    if gait_name == current_gait_name: return
    current_gait_name = gait_name
    current_gait = gaits[gait_name]
    current_gait_frame_idx = 0
    

def gait_loop():
    global lock, petoi_kinematics
    lock.acquire()
    petoi_kinematics.plot_initialize()
    lock.release()

    while True:
        global gaits, current_gait_name, current_gait, current_gait_frame_idx
        lock.acquire()
        alphas, betas = petoi_kinematics.leg_ik(current_gait[current_gait_frame_idx])
        current_gait_frame_idx = (current_gait_frame_idx + 1) % len(current_gait)
        petoi_kinematics.render(alphas, betas)
        lock.release()
        time.sleep(0.05)

def temp_loop():
    while True:
        time.sleep(0.5)



def input_loop():
    """
    input format should be 
        1. "g:walk" switch gait to walk
        2. "h:1.1" make (relative) body height 1.1
        3. "a:30" make body angle 30 degrees
    """
    while True:
        global lock, gaits, current_gait_name, current_gait, current_gait_frame_idx, petoi_kinematics
        # print('input:')
        msg = input()
        msg1, msg2 = msg[0], msg[2:]

        lock.acquire()
        if msg1 == 'g':
            if msg2 not in gaits.keys():
                print("Gait not available")
            else:
                switch_gait(msg2)
        
        elif msg1 == 'h':
            petoi_kinematics.update_gamma_h(gamma=None, h=float(msg2))
        
        elif msg1 == 'a':
            petoi_kinematics.update_gamma_h(
                gamma=np.deg2rad(float(msg2)), 
                h=None
            )
        lock.release()

        

if __name__ == '__main__':

    t = threading.Thread(target=input_loop)
    t.daemon = True
    t.start()


    gait_loop()

    t.join()

    # input_loop()
