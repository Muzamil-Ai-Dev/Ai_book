---
id: capstone-index
title: "Module 8: The Capstone Project"
sidebar_label: 08 - Capstone Project
description: "Design, Build, and Simulate a Complete Physical AI System."
---

# Module 8: The Capstone Project

## Introduction

Congratulations. You have traversed the landscape of Physical AI.
*   **Module 1** gave you the philosophy.
*   **Module 2** gave you the nervous system (ROS 2).
*   **Module 3 & 4** gave you the world (Gazebo/Isaac).
*   **Module 5 & 6** gave you the body and brain (VLA/Control).
*   **Module 7** gave you the voice.

Now, it is time to build.

The **Capstone Project** is an open-ended engineering challenge. Your goal is to identify a problem, design a robotic solution, simulate it, and implement the "AI" layer that makes it intelligent.

## The Goal: A Vertical Slice

You are not expected to build a commercial product in a few weeks. You are expected to build a **Vertical Slice**.
A Vertical Slice means you implement *one* specific feature from top to bottom (User Interface -> AI -> Planner -> Controller -> Hardware/Sim), rather than implementing *many* features shallowly.

**Example**:
*   *Horizontal*: A robot that can map a room, patrol, pick up objects, and talk, but none of it works reliably.
*   *Vertical*: A robot that can purely "Pick up a red cup when asked" and does it perfectly 9/10 times.

## Tracks

Choose one of the following tracks for your project:

### Track A: The Home Assistant (Mobile Manipulation)
*   **Platform**: Fetch, Tiago, or Custom Mobile Manipulator.
*   **Challenge**: Navigate a cluttered home environment (Simulated) to retrieve specific objects requested by natural language.
*   **Key Tech**: Navigation2, MoveIt 2, VLA (RT-1) or LLM Function Calling.

### Track B: The Bipedal Walker (Locomotion)
*   **Platform**: Unitree H1, Digit, or Custom Humanoid.
*   **Challenge**: Walk across uneven terrain without falling.
*   **Key Tech**: Reinforcement Learning (Isaac Gym), MPC, Whole-Body Control.

### Track C: The Conversational Kiosk (HRI)
*   **Platform**: Static Robot Head or Tablet-face Robot.
*   **Challenge**: A receptionist robot that can answer questions about a building, guide users (via map), and handle complex dialogue states.
*   **Key Tech**: RAG, ASR/TTS, Facial Animation.

## Evaluation Criteria

Your project will be evaluated on:
1.  **System Integration**: How well do the components (ROS, AI, Sim) talk to each other?
2.  **Autonomy**: How much human intervention is required during the demo?
3.  **Code Quality**: Is the code modular, documented, and tested?
4.  **Documentation**: A clear README and a demo video.

Let's begin the final climb.