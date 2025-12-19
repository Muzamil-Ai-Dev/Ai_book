---
id: usd-omniverse
title: "Deep Dive: USD and the Isaac API"
sidebar_label: USD & API
description: Mastering the Universal Scene Description format and the Python API for procedural world generation.
---

# Deep Dive: USD and the Isaac API

## Learning Objectives

By the end of this chapter, you will be able to:

1.  **Deconstruct USD (Universal Scene Description)**, explaining why it is called the "Photoshop of 3D."
2.  **Manipulate the Stage Programmatically**, writing Python code to spawn, move, and modify objects in Isaac Sim.
3.  **Understand "Layering"**, separating your robot definition from the world definition using USD references.
4.  **Interface with ROS 2**, creating an Action Graph that bridges the simulation to your ROS topics.

## Concept Explanations

### 1. What is USD?

Pixar invented USD (Universal Scene Description) to solve a problem: making movies involves hundreds of artists working on the same scene.
*   The Animator moves the character.
*   The Lighter changes the sun position.
*   The Modeler fixes a crack in the wall.

In a traditional format (like `.obj` or `.fbx`), you have one monolithic file. If three people edit it, you get merge conflicts.
In **USD**, you have **Layers**.

#### The "Photoshop" Analogy
Think of a USD file not as a "file," but as a stack of transparent sheets (like layers in Photoshop).
*   **Base Layer (`robot.usd`)**: Defines the robot mesh.
*   **Opinion Layer (`simulation.usd`)**: "I think the robot should be at (0,0,0)."
*   **Override Layer (`experiment_1.usd`)**: "Actually, for this test, I want the robot to be red and at (10, 10, 0)."

When you open `experiment_1.usd`, the engine *composes* these layers in real-time. It reads the base, applies the opinion, applies the override, and renders the result. This means you can run 1,000 experiments without duplicating the original robot geometry 1,000 times. You just store the *deltas* (changes).

### 2. The Isaac Sim Python API

Isaac Sim exposes everything via Python. You can build entire worlds via code. This is essential for **Domain Randomization**.

Key Modules:
*   `omni.isaac.core`: The high-level wrapper. Easy to use.
*   `pxr`: The low-level USD library (from Pixar). Hard, but powerful.

### 3. Action Graphs (Omnigraph)

How do we simulate a Lidar? Do we write C++ code to ray-cast?
No. We use **Omnigraph**.
Omnigraph is a visual programming language (nodes and wires) inside Isaac Sim.
*   **Node A**: `On Playback Tick`
*   **Node B**: `Compute Lidar Rays`
*   **Node C**: `Publish ROS 2 Message`

You connect A -> B -> C. Every tick, the Lidar computes and publishes. You can script this graph via Python, meaning your sensor setup is reproducible.

## Conceptual Visualization

### The Composition Arc

Imagine constructing a "Kitchen" scene.

1.  **Reference**: You point to `fridge.usd` (a file on disk). You do not copy it. You reference it.
2.  **Prim (Primitive)**: The instance of the fridge in your scene.
3.  **VariantSet**: The `fridge.usd` might have variants: "Clean", "Dirty", "Dented". You just select `variant="Dirty"`.
4.  **Xform (Transform)**: You apply a translation `(x=5, y=0, z=0)`.

The final scene is just a tiny text file saying:
"Reference `fridge.usd`. Select 'Dirty'. Move to (5,0,0)."

This is extremely efficient for storage and streaming.

## Hands-on Exercises

### Procedural Table Generation

Let's write a Python script to generate a table with random objects. This is the "Hello World" of SDG (Synthetic Data Generation).

```python
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

# 1. Start the World
world = World()
world.scene.add_default_ground_plane()

# 2. Add a Table (Fixed Cuboid)
world.scene.add(
    DynamicCuboid(
        prim_path="/World/Table",
        name="table",
        position=np.array([0, 0, 0.5]),
        scale=np.array([1.0, 1.0, 1.0]),
        color=np.array([0.5, 0.3, 0.1]), # Brown
        mass=100.0
    )
)

# 3. Add Random Cubes (The "Debris")
for i in range(5):
    # Random position on top of table
    x = np.random.uniform(-0.4, 0.4)
    y = np.random.uniform(-0.4, 0.4)
    
    world.scene.add(
        DynamicCuboid(
            prim_path=f"/World/Cube_{i}",
            name=f"cube_{i}",
            position=np.array([x, y, 1.1]), # Drop from above
            scale=np.array([0.1, 0.1, 0.1]),
            color=np.random.rand(3), # Random RGB
            mass=0.1
        )
    )

# 4. Run Simulation
world.reset()
for i in range(500):
    world.step(render=True)
```

**Outcome**:
1.  A brown table appears.
2.  5 colorful cubes spawn in the air.
3.  Gravity pulls them down.
4.  They collide with the table and scatter.
5.  Every time you run this, the arrangement is unique.

## Capstone Prep

### The USD Structure for our Humanoid

We will structure our Capstone assets as follows:

*   `assets/robots/humanoid/`
    *   `humanoid.usd` (The main assembly)
    *   `materials/` (PBR Textures)
    *   `physics/` (Physics materials: friction, bounce)
*   `assets/environments/`
    *   `lab_room.usd` (The walls and floor)

In our simulation script, we will:
1.  Open `lab_room.usd`.
2.  Reference `humanoid.usd` at `(0, 0, 0)`.
3.  Add a `ROS2_Bridge` to connect the topics.