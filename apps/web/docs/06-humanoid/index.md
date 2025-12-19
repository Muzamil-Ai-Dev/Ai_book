---
id: humanoid-intro
title: "The Body: Humanoid Robotics"
sidebar_label: "Module 6: Humanoids"
description: The physics of bipedal locomotion, balance, and the challenge of underactuation.
---

# Module 6: The Body (Humanoid Robotics)

## Learning Objectives

By the end of this module, you will be able to:

1.  **Define Underactuation**, explaining why you cannot just "command" a robot to stay upright like you command an arm to move.
2.  **Analyze the Inverted Pendulum**, the fundamental physics model that describes walking.
3.  **Explain ZMP (Zero Moment Point)**, the criterion that Honda (ASIMO) used to solve walking in the 90s.
4.  **Differentiate Static vs. Dynamic Walking**, understanding why modern robots (Boston Dynamics) look organic while older robots look stiff.

## Concept Explanations

### Why Humanoids?

If wheels are more efficient (and they are), why build legs?
Because we built the world for us.
*   Stairs.
*   Doorsteps.
*   Narrow corridors.
*   Ladders.

A wheeled robot is trapped on the ground floor. A humanoid can go anywhere a firefighter can go. This is the **General Purpose** promise.

### The Problem: You are Falling

A robot arm is **Statically Stable**. If you turn off the power, it stays there (or sags due to gravity, but it doesn't "fall over").
A humanoid is **Unstable**. Its Center of Mass (CoM) is high above its small feet. It is an **Inverted Pendulum**.
Gravity wants to pull it down. To stay up, the robot must constantly fight physics.

**Walking is just "Controlled Falling".**
1.  You lean forward. Gravity accelerates you.
2.  Before your face hits the ground, you swing a leg forward to catch yourself.
3.  You push off again.

### Underactuation

This is the hardest concept in robotics.
*   **Fully Actuated**: A robot arm. You have a motor for every way it can move. If you want the hand to go to X, you solve the math and move the motors.
*   **Underactuated**: A humanoid. You have motors in the knees and hips, but you do **not** have a motor between your foot and the ground. You cannot "command" the robot to rotate around its ankle if the foot is not glued to the floor. You can only push against the ground and hope the ground pushes back (Newton's 3rd Law).

If you slip, you lose control. This is why walking on ice is hard.

## Conceptual Visualization

### The Broomstick Analogy

Try to balance a broomstick on your palm.
*   **The Broom**: The Robot.
*   **Your Hand**: The Feet.
*   **Goal**: Keep the broom upright.

To keep the broom from falling **Forward**, you must move your hand **Forward** fast.
You are constantly moving the base to keep it under the Center of Mass.
This is exactly what the ZMP algorithm does. It calculates where the "Virtual Broom" is falling and commands the feet to step there.

## Hands-on Exercises

### The "Stand Up" Challenge

If you have a simulation (or a toy robot):
1.  Set all motor positions to 0 (Stand straight).
2.  Apply gravity.
3.  The robot will fall.

Why? Because no robot is perfectly symmetrical. A tiny error of 0.001 degrees creates a torque. That torque grows.
To stand still, a robot must be active. It must run a **PID Loop** at 1000Hz:
`Target Angle = 0`.
`Error = Current IMU Pitch`.
`Motor Torque = Kp * Error`.

If the robot leans forward, the toes must push down to push the body back.

## Capstone Prep

### The Walking Controller

For our Capstone, we will not write a ZMP solver from scratch (that is a PhD thesis).
We will use **Deep Reinforcement Learning (DRL)**.
We will define a "Reward Function":
*   `+1.0` for every second the head is above 1 meter.
*   `+1.0` for moving forward velocity > 0.5 m/s.
*   `-100.0` for the torso hitting the ground.

We will let the Isaac Sim physics engine simulate millions of falls until the robot figures out how to walk. This is the "Modern" approach.