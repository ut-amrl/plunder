import gymnasium as gym
import panda_gym
import time
import numpy as np
import random
import sys

# Probabilistic functions
def logistic(offset, slope, x):
    return 1.0/(1.0+np.exp(-slope*(x-offset)))

def logistic2(x, offset, slope):
    return 1.0/(1.0+np.exp(-slope*(x-offset)))

def sample(p):
    return random.random()<p

def Plus(x, y):
    return x + y

def Abs(x):
    return abs(x)

def Minus(x, y):
    return x - y

def And(x, y):
    return x and y

def Or(x, y):
    return x or y

def convert_to_py():
    prog = ""
    while True:
        states = input().split(" ")
        if states[0] == "exit":
            break

        loss = input()
        asp = input()

        state1 = states[0]
        state2 = states[2]

        asp = asp.replace("Flip", "sample").replace("Logistic", "logistic2")
        asp = asp.replace("fX1=[", "").replace("]", "").replace(", true", "").replace("fX2=[", "").replace("pX1=[", "").replace("pX2=[", "")

        prog += "if ha == " + state1 + " and " + asp + ":\n"
        prog += "\treturn " + state2 + "\n"

    prog = prog.replace("FASTER", "env.action_type.actions_indexes['FASTER']")
    prog = prog.replace("SLOWER", "env.action_type.actions_indexes['SLOWER']")
    prog = prog.replace("LANE_LEFT", "env.action_type.actions_indexes['LANE_LEFT']")
    prog = prog.replace("LANE_RIGHT", "env.action_type.actions_indexes['LANE_RIGHT']")

    print(prog)

convert_to_py()