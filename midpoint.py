#!/usr/bin/env python3

import math
import time

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.sensor import INPUT_1


# --------------------------------------------------
# ROBOT PARAMETERS
# --------------------------------------------------

L1 = 14.4   # cm
L2 = 6.4    # cm

motor1 = LargeMotor(OUTPUT_A)
motor2 = LargeMotor(OUTPUT_D)

touch = TouchSensor(INPUT_1)

# Physical motor limits
MOTOR1_MIN = -135
MOTOR1_MAX = 135

MOTOR2_MIN = -155
MOTOR2_MAX = 110


# --------------------------------------------------
# FORWARD KINEMATICS
# --------------------------------------------------

def forward_kinematics(theta1_deg, theta2_deg):

    theta1 = math.radians(theta1_deg)
    theta2 = math.radians(theta2_deg)

    x = (
        L1 * math.cos(theta1)
        + L2 * math.cos(theta1 + theta2)
    )

    y = (
        L1 * math.sin(theta1)
        + L2 * math.sin(theta1 + theta2)
    )

    return x, y


# --------------------------------------------------
# ANALYTICAL INVERSE KINEMATICS
# --------------------------------------------------

def analytical_ik(x, y):

    r = math.sqrt(x**2 + y**2)

    r_min = abs(L1 - L2)
    r_max = L1 + L2

    if r < r_min or r > r_max:
        print("ERROR: Target is outside theoretical workspace.")
        return None

    # Law of cosines
    cos_theta2 = (
        x**2 + y**2 - L1**2 - L2**2
    ) / (2 * L1 * L2)

    cos_theta2 = max(-1.0, min(1.0, cos_theta2))

    theta2_a = math.acos(cos_theta2)
    theta2_b = -math.acos(cos_theta2)

    def calculate_theta1(theta2):

        return (
            math.atan2(y, x)
            -
            math.atan2(
                L2 * math.sin(theta2),
                L1 + L2 * math.cos(theta2)
            )
        )

    theta1_a = calculate_theta1(theta2_a)
    theta1_b = calculate_theta1(theta2_b)

    solution1 = (
        math.degrees(theta1_a),
        math.degrees(theta2_a)
    )

    solution2 = (
        math.degrees(theta1_b),
        math.degrees(theta2_b)
    )

    return solution1, solution2


# --------------------------------------------------
# MATH ANGLES <-> MOTOR ANGLES
# --------------------------------------------------

def math_to_motor_angles(theta1, theta2):

    # Mathematical positive = counterclockwise
    # EV3 motor positive = clockwise

    motor1_angle = -theta1
    motor2_angle = -theta2

    return motor1_angle, motor2_angle


def motor_to_math_angles(motor1_angle, motor2_angle):

    theta1 = -motor1_angle
    theta2 = -motor2_angle

    return theta1, theta2


# --------------------------------------------------
# JOINT LIMIT CHECK
# --------------------------------------------------

def valid_motor_angles(motor1_angle, motor2_angle):

    motor1_valid = (
        MOTOR1_MIN <= motor1_angle <= MOTOR1_MAX
    )

    motor2_valid = (
        MOTOR2_MIN <= motor2_angle <= MOTOR2_MAX
    )

    return motor1_valid and motor2_valid


# --------------------------------------------------
# RECORD CURRENT END-EFFECTOR POSITION
# --------------------------------------------------

def get_current_position():

    # Read current motor encoder positions
    motor1_angle = motor1.position
    motor2_angle = motor2.position

    # Convert motor angles to mathematical joint angles
    theta1, theta2 = motor_to_math_angles(
        motor1_angle,
        motor2_angle
    )

    # Calculate x,y using forward kinematics
    x, y = forward_kinematics(theta1, theta2)

    return x, y


# --------------------------------------------------
# WAIT FOR TOUCH SENSOR PRESS
# --------------------------------------------------

def wait_for_press():

    while not touch.is_pressed:
        time.sleep(0.05)

    # Wait until user releases button
    while touch.is_pressed:
        time.sleep(0.05)


# --------------------------------------------------
# MOVE TO CARTESIAN POSITION
# --------------------------------------------------

def move_to_xy(x, y):

    solutions = analytical_ik(x, y)

    if solutions is None:
        return False

    for theta1, theta2 in solutions:

        motor1_target, motor2_target = math_to_motor_angles(
            theta1,
            theta2
        )

        if valid_motor_angles(
            motor1_target,
            motor2_target
        ):

            print("\nUsing IK solution:")
            print("theta1 =", round(theta1, 2), "degrees")
            print("theta2 =", round(theta2, 2), "degrees")

            print(
                "Motor A target =",
                round(motor1_target, 2),
                "degrees"
            )

            print(
                "Motor D target =",
                round(motor2_target, 2),
                "degrees"
            )

            motor1.on_to_position(
                SpeedPercent(15),
                motor1_target,
                brake=True,
                block=True
            )

            motor2.on_to_position(
                SpeedPercent(15),
                motor2_target,
                brake=True,
                block=True
            )

            return True

    print("ERROR: No IK solution satisfies joint limits.")
    return False


# --------------------------------------------------
# MAIN PROGRAM
# --------------------------------------------------

print("MIDPOINT PROGRAM")
print("----------------")

print("\nPlace robot in theta1 = 0, theta2 = 0 position.")
print("Resetting motor encoders...")

motor1.reset()
motor2.reset()

# Allow arm to be moved manually
motor1.off(brake=False)
motor2.off(brake=False)


# --------------------------------------------------
# RECORD POINT 1
# --------------------------------------------------

print("\nMove the end effector to POINT 1.")
print("Press the touch sensor when ready.")

wait_for_press()

x1, y1 = get_current_position()

print("\nPoint 1 recorded:")
print("x1 =", round(x1, 2), "cm")
print("y1 =", round(y1, 2), "cm")


# --------------------------------------------------
# RECORD POINT 2
# --------------------------------------------------

print("\nMove the end effector to POINT 2.")
print("Press the touch sensor when ready.")

wait_for_press()

x2, y2 = get_current_position()

print("\nPoint 2 recorded:")
print("x2 =", round(x2, 2), "cm")
print("y2 =", round(y2, 2), "cm")


# --------------------------------------------------
# CALCULATE MIDPOINT
# --------------------------------------------------

mid_x = (x1 + x2) / 2
mid_y = (y1 + y2) / 2

print("\nCalculated midpoint:")

print("x =", round(mid_x, 2), "cm")
print("y =", round(mid_y, 2), "cm")


# --------------------------------------------------
# MOVE TO MIDPOINT
# --------------------------------------------------

print("\nMoving to midpoint...")

success = move_to_xy(mid_x, mid_y)

if success:

    print("\nMovement complete.")

    final_x, final_y = get_current_position()

    print("\nCalculated final position from encoders:")
    print("x =", round(final_x, 2), "cm")
    print("y =", round(final_y, 2), "cm")

else:

    print("\nCould not move to midpoint.")
