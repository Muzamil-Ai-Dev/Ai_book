---
id: simulation-basics
title: "Deep Dive: URDF, SDF, and Physics"
sidebar_label: URDF & Physics
description: "Understanding the file formats (URDF/SDF) and the math (Rigid Body Dynamics) that make simulation possible."
---

# Deep Dive: URDF, SDF, and Physics

## Learning Objectives

By the end of this chapter, you will be able to:

1.  **Decode a URDF**, explaining the difference between Links (Inertia, Visual, Collision) and Joints (Revolute, Prismatic, Fixed).
2.  **Differentiate URDF and SDF**, knowing when to use the ROS standard vs. the Gazebo/Simulation standard.
3.  **Optimize Meshes**, understanding why visual meshes (high-poly) must be separate from collision meshes (primitives).
4.  **Debug Physics Instability**, identifying the "Exploding Robot" phenomenon caused by low solver iteration counts or bad inertia matrices.

## Prerequisites

- ROS 2 installed
- XML Editor

## Core Concepts

### 1. The Language of Shapes: URDF vs. SDF

Robots need a file format to describe their body.

#### URDF (Unified Robot Description Format)
*   **Format**: XML.
*   **Philosophy**: Tree structure. One root link (Base), branching out to children.
*   **Limitation**: A tree cannot have closed loops (cycles). You cannot describe a "four-bar linkage" or parallel robot purely in URDF.
*   **Use Case**: The ROS standard. Every ROS tool (`robot_state_publisher`, `rviz`) speaks URDF.

#### SDF (Simulation Description Format)
*   **Format**: XML.
*   **Philosophy**: World structure. Can describe the robot *and* the lighting, the ground plane, and other robots.
*   **Capability**: Supports closed kinematic chains.
*   **Use Case**: Gazebo and Ignition.

*Best Practice*: We write in **URDF** (or Xacro, a macro language for URDF) and let the simulator convert it to SDF internally if needed.

### 2. Anatomy of a Link

A "Link" is a rigid part of the robot (e.g., the forearm). In simulation, a Link is not just one thing; it is three things stacked on top of each other.

#### A. The Visual Mesh (The Skin)
*   **Purpose**: To look good for the human operator and camera sensors.
*   **Complexity**: High (10,000+ triangles).
*   **Format**: `.dae` (Collada), `.stl`, `.obj`.
*   **Physics Impact**: Zero. It is a ghost.

#### B. The Collision Mesh (The Hitbox)
*   **Purpose**: To calculate contacts. "Did the arm hit the table?"
*   **Complexity**: Extremely Low (Simple primitives like Box, Cylinder, Sphere, or Convex Hull).
*   **Physics Impact**: High. Computing collision between two 10,000-poly meshes is impossibly slow. Computing collision between two Cubes is instant.

#### C. The Inertial Matrix (The Mass)
*   **Purpose**: Newton's Second Law ($F = ma$).
*   **Data**: Mass (kg) and the Inertia Tensor (3x3 matrix describing how hard it is to rotate the object).
*   **Crucial Note**: If you set the mass of a small link to 10,000kg, the physics engine will become unstable. If you set the Inertia Matrix to zero, the robot will fly into space.

### 3. Rigid Body Dynamics: The Math of Movement

How does the simulator move the robot? It solves the **Equation of Motion**:

$$ M(q)\ddot{q} + C(q, \dot{q})\dot{q} + G(q) = \tau $$

*   **$q$**: Joint positions.
*   **$M(q)$**: Mass Matrix (Inertia).
*   **$C$**: Coriolis and Centrifugal forces (the forces felt when spinning).
*   **$G$**: Gravity.
*   **$\tau$ (Tau)**: The torque applied by motors.

Every time step (e.g., 1ms), the physics engine:
1.  Takes the current state ($q, \dot{q}$).
2.  Takes the motor torque ($\tau$).
3.  Solves this equation to find acceleration ($\ddot{q}$).
4.  Integrates acceleration to find the *new* velocity and position.

## Conceptual Visualization

### The "Exploding Robot"

One of the most common bugs in simulation is the **Explosion**. You start the sim, and the robot vibrates violently and then shoots into the sky at Mach 10.

**Why?**
Imagine a "Spring-Damper" system (like a shock absorber).
*   If the spring is too stiff ($k$ is huge) and the time step is too large ($dt$ is large), the integration fails.
*   **Frame 1**: The spring is compressed. Force is huge.
*   **Frame 2**: Based on that huge force, the object shoots *past* the equilibrium point to the other side.
*   **Frame 3**: Now the spring is stretched even more. Force is *huge-er*.
*   **Frame 4**: It shoots back even further.

**The Fix**:
1.  **Reduce Time Step**: Go from 10ms to 1ms.
2.  **Increase Solver Iterations**: Tell the physics engine (PhysX) to think harder per step.
3.  **Sanity Check Inertia**: Do not have a 1kg arm attached to a 0.001g finger. The mass ratio causes numerical errors.

## Examples / Exercises

### Writing a Clean URDF

Let's write a URDF for a "Pendulum" to see the Visual/Collision split.

```xml
<robot name="pendulum">
  
  <!-- BASE: Heavy, fixed -->
  <link name="base">
    <inertial>
      <mass value="100.0"/> <!-- Heavy base so it doesn't move -->
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <geometry><box size="0.5 0.5 0.1"/></geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry><box size="0.5 0.5 0.1"/></geometry>
    </collision>
  </link>

  <!-- JOINT: Hinge -->
  <joint name="hinge" type="continuous"> <!-- Continuous means it can spin 360 -->
    <parent link="base"/>
    <child link="rod"/>
    <origin xyz="0 0 0.2" rpy="0 1.57 0"/> <!-- Rotated 90 deg -->
    <axis xyz="1 0 0"/>
    <dynamics damping="0.1" friction="0.01"/> <!-- Physics properties! -->
  </joint>

  <!-- ROD: The moving part -->
  <link name="rod">
    <inertial>
      <mass value="1.0"/>
      <!-- Cylinder inertia approx -->
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <!-- We want it to look like a nice red pole -->
      <geometry><cylinder length="1.0" radius="0.05"/></geometry>
      <material name="red"/>
    </visual>
    <collision>
      <!-- Collision is same as visual here, which is fine for primitives -->
      <geometry><cylinder length="1.0" radius="0.05"/></geometry>
    </collision>
  </link>

</robot>
```

**Task**:
Copy this into a file named `pendulum.urdf`.
If you have ROS installed, verify it with:
`check_urdf pendulum.urdf`

## Capstone Prep

### The Robot Description Package

In a real project, we organize our files in a specific package structure.
Create this folder structure in your mind (or filesystem):

`my_robot_description/`
  `package.xml`
  `CMakeLists.txt`
  `urdf/`
    `robot.urdf.xacro` (The main file)
    `materials.xacro` (Colors)
    `macros.xacro` (Reusable components like "wheel")
  `meshes/`
    `visual/` (High res .STL files)
    `collision/` (Low res .STL or Hull files)

Separating data this way keeps the project sane as the robot grows from 2 links to 20 links.

## Summary

Simulation is more than just visualization; it is a mathematical approximation of reality. We learned that **URDF** uses a tree structure to describe robot links, each composed of a **Visual** mesh (for eyes), a **Collision** mesh (for physics), and an **Inertia** matrix (for dynamics). We also touched on the numerical instability of physics solvers (the exploding robot) and the importance of simplifying collision geometry for performance.

## References

- [ROS 2 URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [Gazebo SDF Format](http://sdformat.org/)
- [Rigid Body Dynamics Algorithms (Featherstone)](https://royfeatherstone.org/spatial/)