#!/usr/bin/env python3

import math

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent


# --------------------------------------------------
# ROBOT PARAMETERS
# --------------------------------------------------

L1 = 14.4   # cm
L2 = 6.4    # cm

motor1 = LargeMotor(OUTPUT_A)
motor2 = LargeMotor(OUTPUT_D)

# Physical motor limits
MOTOR1_MIN = -135
MOTOR1_MAX = 135

MOTOR2_MIN = -155
MOTOR2_MAX = 110


# --------------------------------------------------
# FORWARD KINEMATICS
# Used to verify the IK result
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

    # Distance from robot base to target
    r = math.sqrt(x**2 + y**2)

    print("Distance from base =", round(r, 3), "cm")

    # ----------------------------------------------
    # Check if target is reachable
    # ----------------------------------------------

    r_min = abs(L1 - L2)
    r_max = L1 + L2

    if r < r_min or r > r_max:
        print("ERROR: Target is outside theoretical workspace.")
        return None

    # ----------------------------------------------
    # Find theta2 using law of cosines
    #
    # cos(theta2) =
    # (x^2 + y^2 - L1^2 - L2^2)
    # ----------------------------------------------

    cos_theta2 = (
        x**2 + y**2 - L1**2 - L2**2
    ) / (2 * L1 * L2)

    # Protect against tiny floating point errors
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))

    theta2_a = math.acos(cos_theta2)
    theta2_b = -math.acos(cos_theta2)

    # ----------------------------------------------
    # Find theta1
    #
    # This form comes directly from rearranging
    # the same FK equations used in the course:
    #
    # theta1 =
    # atan2(y,x)
    # -
    # atan2(
    #   L2 sin(theta2),
    #   L1 + L2 cos(theta2)
    # )
    # ----------------------------------------------

    def calculate_theta1(theta2):

        theta1 = (
            math.atan2(y, x)
            -
            math.atan2(
                L2 * math.sin(theta2),
                L1 + L2 * math.cos(theta2)
            )
        )

        return theta1

    theta1_a = calculate_theta1(theta2_a)
    theta1_b = calculate_theta1(theta2_b)

    # Convert radians to degrees
    theta1_a = math.degrees(theta1_a)
    theta2_a = math.degrees(theta2_a)

    theta1_b = math.degrees(theta1_b)
    theta2_b = math.degrees(theta2_b)

    return (
        (theta1_a, theta2_a),
        (theta1_b, theta2_b)
    )


# --------------------------------------------------
# CONVERT MATH ANGLES TO EV3 MOTOR ANGLES
# --------------------------------------------------

def math_to_motor_angles(theta1, theta2):

    # Course/math positive angle = counterclockwise
    # Our motors:
    # positive = clockwise
    # negative = counterclockwise

    motor1_angle = -theta1
    motor2_angle = -theta2

    return motor1_angle, motor2_angle


# --------------------------------------------------
# CHECK PHYSICAL JOINT LIMITS
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
# MOVE ROBOT
# --------------------------------------------------

def move_to_xy(x, y):

    solutions = analytical_ik(x, y)

    if solutions is None:
        return

    print("\nAnalytical IK solutions:")

    for i, solution in enumerate(solutions):

        theta1, theta2 = solution

        print("\nSolution", i + 1)
        print("theta1 =", round(theta1, 2), "degrees")
        print("theta2 =", round(theta2, 2), "degrees")

        # Verify using FK
        check_x, check_y = forward_kinematics(theta1, theta2)

        print(
            "FK check = (",
            round(check_x, 2),
            ",",
            round(check_y, 2),
            ") cm"
        )

        motor1_angle, motor2_angle = math_to_motor_angles(
            theta1,
            theta2
        )

        print(
            "Motor A target =",
            round(motor1_angle, 2),
            "degrees"
        )

        print(
            "Motor D target =",
            round(motor2_angle, 2),
            "degrees"
        )

        # ------------------------------------------
        # Use first physically valid solution
        # ------------------------------------------

        if valid_motor_angles(motor1_angle, motor2_angle):

            print("Solution is within physical joint limits.")
            print("Using this solution.")

            # Move Joint 1
            motor1.on_to_position(
                SpeedPercent(15),
                motor1_angle,
                brake=True,
                block=True
            )

            # Move Joint 2
            motor2.on_to_position(
                SpeedPercent(15),
                motor2_angle,
                brake=True,
                block=True
            )

            print("\nMovement complete.")

            return

        else:
            print("Solution violates joint limits.")

    print("ERROR: Neither IK solution is physically reachable.")


# --------------------------------------------------
# MAIN
# --------------------------------------------------

print("Place robot in theta1 = 0, theta2 = 0 position.")
print("Resetting motor encoders...")

motor1.reset()
motor2.reset()

print("Motor A position:", motor1.position)
print("Motor D position:", motor2.position)

# First test target
TARGET_X = 15.0
TARGET_Y = 8.0

print("\nTarget position:")
print("x =", TARGET_X, "cm")
print("y =", TARGET_Y, "cm")

move_to_xy(TARGET_X, TARGET_Y)
