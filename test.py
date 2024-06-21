import math

BASE_X =0
BASE_Y=0
LINK1_LENGTH =150
LINK2_LENGTH =120


def inverse_kinematics(x, y):
    x = x- BASE_X
    y = y - BASE_Y
    

    # Calculate the distance from the base to the target position
    distance = math.sqrt((x) ** 2 + (y) ** 2)

    # Check if the target position is within the workspace
    if distance > LINK1_LENGTH + LINK2_LENGTH or distance < abs(LINK1_LENGTH - LINK2_LENGTH):
        return (0,0)

    # Calculate the joint angles
    try:
        cos_q2 = (x ** 2 + y ** 2 - (LINK1_LENGTH ** 2 + LINK2_LENGTH ** 2)) / (2 * LINK1_LENGTH * LINK2_LENGTH)
        print(cos_q2)
        if cos_q2 < -1 or cos_q2 > 1:
            return (0,0)  # Target position is outside the valid range
        q2 = math.acos(cos_q2)
        q1 = math.atan2(y, x) - math.atan2(LINK2_LENGTH * math.sin(q2), LINK1_LENGTH + LINK2_LENGTH * math.cos(q2))
        print(q1,q2)
        
        return q1, q2
    except ValueError:
        # Handle math domain error
        return (0,0)
    

cos_q2 = ((x**2+y**2)-(l1**2+l2**2))/(2*l1*l2)