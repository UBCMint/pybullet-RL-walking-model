import random
import pybullet as p
import pybullet_data
import time
import os
import math
import numpy as np

physicsClient = p.connect(p.GUI)  
p.setRealTimeSimulation(1)

def produce_random():
    while True:
        data = random.randint(0, 1)
        print(data)
        with open("shared_data.txt", "w") as f:
            f.write(str(data))
        p.stepSimulation()
        time.sleep(1/1)


produce_random()

p.disconnect()