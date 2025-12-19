---
id: project-spec
title: "Project Specification & Roadmap"
sidebar_label: The Roadmap
description: "A week-by-week guide to executing your Capstone project."
---

# Project Specification & Roadmap

## Learning Objectives

By the end of this module, you will be able to:

1.  **Define a Vertical Slice**, distinguishing it from a broad but shallow horizontal prototype.
2.  **Structure a Robotics Project**, using a professional Phased approach (Design -> Sim -> MVP -> Integration).
3.  **Execute a Capstone**, integrating ROS 2, AI, and Simulation into a unified system.

## Concept Explanations

### 1. The Engineering Lifecycle

You are acting as a Systems Integrator. The lifecycle of a robotics project is distinct from pure software.

*   **Phase 1: Definition (Week 1)**: Writing the PRD (Product Requirements Document). If you don't know what "Done" looks like, you will never finish.
*   **Phase 2: Simulation (Week 2)**: Building the world. You cannot test physical AI without a physical (simulated) world.

### 2. The Vertical Slice

The most common mistake is trying to build everything at once.
A **Vertical Slice** means you implement *one* specific feature from top to bottom (User Interface -> AI -> Planner -> Controller -> Hardware/Sim), rather than implementing *many* features shallowly.

**Example**:
*   *Horizontal*: A robot that can map a room, patrol, pick up objects, and talk, but none of it works reliably.
*   *Vertical*: A robot that can purely "Pick up a red cup when asked" and does it perfectly 9/10 times.

## Conceptual Visualization

### The Cake Analogy

Imagine cutting a cake.
*   **Layer 1 (Frosting)**: The UI / Voice Command.
*   **Layer 2 (Sponge)**: The High-Level Planner / AI.
*   **Layer 3 (Filling)**: The Motion Planner (MoveIt/Nav2).
*   **Layer 4 (Base)**: The Hardware Driver / Physics Engine.

A "Horizontal Slice" is just baking the sponge (AI) but having no base (Robot) or frosting (UI). You can't eat it.
A "Vertical Slice" is a thin wedge containing all 4 layers. It is a complete bite.

**Your Capstone is that wedge.**

## Hands-on Exercises

### Exercise 1: The MVP (Week 3)

Build the "Skeleton" of the system.

**Task**: Script a fixed sequence to prove the pipeline works.
*   Script: "Drive forward 1m, turn left, open gripper."
*   **Goal**: Verify that your code *can* move the robot. If this fails, AI won't help.

### Exercise 2: Intelligence & Integration (Week 4-5)

Now, add the Brain.

**Task**: Replace the hardcoded script with your AI decision maker.
*   If using **LLM**: Connect the prompt loop.
*   **Robustness Testing**: Move the object slightly. Speak the command differently.

## Capstone Prep

### Phase 5: Documentation & Demo (Week 6)

Your final deliverable is the "Release".

**The README**:
Your GitHub repository must have a README that explains:
1.  **Installation**: How to install dependencies (`rosdep install ...`).
2.  **Usage**: Exact command to launch the demo (`ros2 launch my_pkg demo.launch.py`).

**The Video**:
Record a screen capture showing RViz, Gazebo, and the Terminal. Narrate the video explaining the system state.

### Troubleshooting Guide

*   **"The Robot is Shaking"**: Check Inertia matrices.
*   **"The LLM Hallucinates"**: Add Few-Shot Examples to your system prompt.
*   **"Navigation is Stuck"**: Check costmaps in RViz.

Good luck.
