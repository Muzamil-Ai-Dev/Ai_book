---
id: transformers-robotics
title: "Deep Dive: Transformers for Robotics"
sidebar_label: Transformers & RT-X
description: Inside the architecture of RT-1, RT-2, and the math of Attention for control.
---

# Deep Dive: Transformers for Robotics

## Learning Objectives

By the end of this chapter, you will be able to:

1.  **Dissect the RT-1 Architecture**, understanding how Google encoded images and text into a unified control policy.
2.  **Implement Action Tokenization**, writing Python code to convert continuous joint angles into discrete tokens.
3.  **Calculate Self-Attention**, performing the $Q \cdot K^T$ operation to see how a robot "attends" to a coffee cup in an image.
4.  **Differentiate "Behavior Cloning" vs. "Offline RL"**, knowing the training objectives for VLA models.

## Prerequisites

- Understanding of Neural Networks
- Python (NumPy)

## Core Concepts

### 1. The Architecture of RT-1 (Robotics Transformer 1)

Published by Google DeepMind in 2022, RT-1 was a watershed moment. It proved that a single Transformer could perform 700+ different tasks with 97% success rates.

**The Inputs**:
1.  **Images**: 6 images (history) from the robot's camera. EfficientNet-B3 is used to compress these images into vector embeddings.
2.  **Text**: "Place the coke can upright." A Universal Sentence Encoder converts this to embeddings.

**The Core**:
A standard **Transformer Decoder** (like GPT). It takes the stream of Image Embeddings and Text Embeddings.

**The Output**:
Action Tokens.
RT-1 controls a 7-DoF arm + 1 Gripper.
Instead of outputting 8 continuous numbers (e.g., `0.123, -0.991...`), it outputs **Discrete Bins**.
It divides the range of motion into 256 bins.
The model outputs 8 tokens per step: `[Arm_J1, Arm_J2, ..., Arm_J7, Gripper]`.

### 2. RT-2: Vision-Language-Action

RT-2 took it a step further. RT-1 was a model *trained* for robotics. RT-2 was a **VLM (Vision Language Model)** that was *fine-tuned* for robotics.
They took a massive web-scale model (PaLI-X) that already knew about "Presidents of the USA" and "Types of Fruits" and taught it to move an arm.

**Why does this matter?**
If you ask RT-1 to "Pick up the Extinct Animal," it fails. It doesn't know what "Extinct" means.
If you ask RT-2 to "Pick up the Extinct Animal" and there is a plastic dinosaur and a plastic horse, RT-2 picks the dinosaur. It uses its **Semantic Knowledge** from the web to inform its **Physical Actions**.

### 3. The Math of Attention

Why are Transformers better than CNNs or RNNs for robotics? **Global Context**.
An RNN forgets what happened 10 seconds ago. A Transformer sees everything at once.

The core mechanism is **Scaled Dot-Product Attention**. It can be described as:

```text
Attention(Q, K, V) = softmax((Q * K_transpose) / sqrt(d_k)) * V
```
Where:
*   `Q` (Query): Represents what the model is looking for.
*   `K` (Key): Represents what describes a particular input element.
*   `V` (Value): Represents the actual content of the input element.
*   `d_k`: The dimension of the keys.
*   `softmax`: A function that converts values into probabilities.

*   **Query (Q)**: "What am I looking for?" (e.g., The concept of a "handle").
*   **Key (K)**: "What describes this patch of image?" (e.g., "I am a shiny edge").
*   **Value (V)**: "What is the content?" (e.g., The pixel data).

If $Q$ matches $K$ (high dot product), the robot "attends" to that part of the image.
In robotics, this allows the arm to "look" at the object it is trying to grasp, ignoring the distracting background.

## Conceptual Visualization

### The Action Vocabulary

Imagine a dictionary.
*   Words 0-30,000: English words ("Apple", "Run", "Blue").
*   Words 30,001-30,256: Action Bin 1 (Joint Angle -3.0 rad to -2.9 rad).
*   Words 30,257-30,512: Action Bin 2...

When the model generates a sentence, it looks like this:
*"I see a cup so I will \<Token 30100\> \<Token 30150\>..."*

We interpret the first part as thought, and the second part as motion.

## Examples / Exercises

### Implementing an Action Tokenizer

Let's write the code to convert a continuous joint angle (radians) into a discrete token ID.

```python
import numpy as np

class ActionTokenizer:
    def __init__(self, min_val=-3.14, max_val=3.14, num_bins=256):
        self.min_val = min_val
        self.max_val = max_val
        self.num_bins = num_bins
        # Create linear bins
        self.bins = np.linspace(min_val, max_val, num_bins)

    def tokenize(self, continuous_action):
        """
        Input: float (radians)
        Output: int (0 to 255)
        """
        # Clip value to bounds
        value = np.clip(continuous_action, self.min_val, self.max_val)
        # Find closest bin index
        token_id = np.digitize(value, self.bins) - 1
        return int(token_id)

    def detokenize(self, token_id):
        """
        Input: int
        Output: float
        """
        token_id = np.clip(token_id, 0, self.num_bins - 1)
        return self.bins[token_id]

# Test it
tokenizer = ActionTokenizer()
original_angle = 1.57 # 90 degrees
token = tokenizer.tokenize(original_angle)
recovered_angle = tokenizer.detokenize(token)

print(f"Original: {original_angle}")
print(f"Token ID: {token}")
print(f"Recovered: {recovered_angle}")
# You will see a small error (Quantization Error). 
# The robot jitters slightly, but the model is robust to this.
```

### The Context Window Exercise

**Scenario**: You are controlling a robot.
**Input**: Current Image + Last 10 Images.
**Task**: Catch a flying ball.

If you only use the *current* image, the ball is just a sphere in space. You don't know its velocity.
If you use the *history* (Last 10 images), the Transformer sees the trajectory. It can calculate the velocity implicitly via Attention and predict where the ball *will be*.

This is why **Sequence Length** matters in robotics.

## Capstone Prep

### Collecting Data

For our VLA, we need a dataset.
We will create a `rosbag` recorder.
*   **Topic 1**: `/camera/image_raw` (10Hz)
*   **Topic 2**: `/joint_states` (10Hz)

We will record 50 demonstrations of the robot walking to a target.
Then, we will write a script to:
1.  Read the bag.
2.  Extract the image and the *next* joint state.
3.  Tokenize the joint state.
4.  Save as `(Image, TokenID)` pairs for training.

## Summary

Transformers have revolutionized robotics by allowing us to treat control as a sequence modeling problem. We explored the architecture of **RT-1** and **RT-2**, seeing how they tokenize continuous actions into discrete bins. We implemented a simple **Action Tokenizer** to understand this quantization and discussed how the **Self-Attention** mechanism enables a robot to utilize temporal history for dynamic tasks like catching a ball.

## References

- [RT-1: Robotics Transformer](https://robotics-transformer1.github.io/)
- [RT-2: Vision-Language-Action Models](https://robotics-transformer2.github.io/)
- [Attention Is All You Need](https://arxiv.org/abs/1706.03762)