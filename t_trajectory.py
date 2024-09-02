import numpy as np
import pygame
import math
from threeR import ThreeRManipulator
from twoR import TwoRManipulator


R2 = TwoRManipulator()
R3 = ThreeRManipulator()

points = [(0,0),(1,0),(0,1),(1,1)]

for i in  range(len(points)):
    user = input("select the manipulator:")
    if user == '2R':
        q1,q2 = R2.inverse_kinematics(points[i][0],points[i][1])
        R2.draw_manipulator(q1,q2)



