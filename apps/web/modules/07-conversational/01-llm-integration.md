---
id: llm-integration
title: "Deep Dive: Integrating LLMs with ROS 2"
sidebar_label: LLM Integration & Function Calling
description: "Building the Cognitive Layer: From Speech to Function Calling and Physical Action."
---

# Deep Dive: Integrating LLMs with ROS 2

## Learning Objectives

By the end of this chapter, you will be able to:

1.  **Architect a Cognitive Node**: Design a ROS 2 node that wraps an LLM API (OpenAI, Anthropic, or local Llama) and manages conversation state.
2.  **Implement Function Calling**: Define JSON schemas that map natural language requests to specific ROS 2 Service calls and Action Clients.
3.  **Build a Voice Pipeline**: Integrate Whisper (ASR) and a TTS engine to create a full verbal loop.
4.  **Engineer Safety Prompts**: Construct system prompts that robustly prevent the robot from executing unsafe or hallucinated commands.
5.  **Debug The Feedback Loop**: handle the "context window" problem and latency issues in real-time robotics.

## Prerequisites

- OpenAI API Key
- Python (rclpy)

## Core Concepts

### 1. The Cognitive Architecture

In traditional robotics, the "Brain" was a Finite State Machine (FSM) or a Behavior Tree. It was deterministic. State A + Event B = State C.
With an LLM, the "Brain" becomes probabilistic and semantic.

The architecture looks like a sandwich:

1.  **The Input Layer (Perception)**:
    *   **Audio**: Raw bytes from the microphone.
    *   **Vision Description**: A VLM (like GPT-4o) describes the scene: "There is a red mug on the table."
    *   **Robot State**: Odometry, battery level, current joint angles.

2.  **The Cognitive Layer (The LLM)**:
    *   This is a ROS 2 Node (often called `llm_agent`).
    *   It holds the **System Prompt** (Who am I?) and the **Context History** (What have we said?).
    *   It does **Reasoning**: "The user wants coffee. I see a mug. I should pick it up."

3.  **The Output Layer (Action)**:
    *   **Structured Output**: The LLM does not just output text; it outputs a **Function Call** (e.g., `{"tool": "pick_place", "args": {"object": "mug"}}`).
    *   **Execution**: A deterministic ROS node parses this JSON and triggers the actual MoveIt task.

### 2. Function Calling: The Bridge

The most critical concept is **Function Calling** (also known as Tool Use). An LLM is a text generator. It cannot "move" an arm. It can only "write text about moving an arm."

To bridge this, we treat the LLM as a **Router**.
We give the LLM a manual of available tools:

*   `navigate(x, y)`
*   `say(text)`
*   `find_object(class_name)`

When the user says "Go to the kitchen," the LLM analyzes the text against the manual and outputs: `navigate(location="kitchen")`.
Your Python code detects this structured output and calls the corresponding ROS Action Client.

### 3. Latency vs. Intelligence

A major challenge is **Latency**.
*   Speech-to-Text: ~1.0s
*   LLM Inference (Cloud): ~1.5s
*   Text-to-Speech: ~1.0s
*   **Total Delay**: ~3.5s

In human conversation, a 3.5s pause is awkward. In robotics, it can be dangerous.
**Strategies**:
*   **Streaming**: Process tokens as they arrive.
*   **Speculative Execution**: Start rotating the robot towards the user while processing.
*   **Local Models**: Running Llama 3 or Mistral on the robot's GPU (Jetson Orin) reduces latency but sacrifices reasoning capability.

## Conceptual Visualization

### The Function Call Flow

Imagine a relay race involving three runners: **Ear**, **Brain**, and **Muscle**.

1.  **The Handoff (User Input)**: The user yells "Grab that bottle!"
2.  **Ear (ASR)**: Catches the sound waves. It scribbles "Grab that bottle" on a piece of paper and hands it to Brain.
3.  **Brain (LLM)**: Reads the paper. It looks at a wall of buttons.
    *   Button A: `Sleep()`
    *   Button B: `Dance()`
    *   Button C: `PickObject(name)`
    *   Button D: `Navigate(coords)`
    It thinks: "Grab implies PickObject. Bottle is the name."
    It writes `PickObject("bottle")` on a robust clipboard and hands it to Muscle.
4.  **Muscle (ROS Controller)**: Reads the clipboard. It doesn't know what a "bottle" is conceptually, but it knows how to execute the `PickObject` routine. It activates the motors.

If Brain decides to hallucinate and writes `Teleport("Mars")`, Muscle looks at the clipboard, sees no such button, and throws an error flag back to Brain.

## Examples / Exercises

### Building the "Chat-Bot" Node

We will build a ROS 2 node that uses OpenAI's API to control a simulated TurtleBot.

#### Step 1: Define the Tools (The Schema)

First, we define what the robot can do. We use JSON Schema, as this is what most LLMs understand.

```python
# tools.py
TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "move_robot",
            "description": "Move the robot forward or backward, and turn.",
            "parameters": {
                "type": "object",
                "properties": {
                    "linear_x": {"type": "number", "description": "Linear velocity in m/s (max 0.5)"},
                    "angular_z": {"type": "number", "description": "Angular velocity in rad/s"}
                },
                "required": ["linear_x", "angular_z"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "stop_robot",
            "description": "Stop all motion immediately.",
            "parameters": {
                "type": "object",
                "properties": {}
            }
        }
    }
]
```

#### Step 2: The Agent Node

Create a file `llm_controller.py`. This node subscribes to string commands (simulating speech text) and publishes `Twist` messages.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import openai

class LLMAgent(Node):
    def __init__(self):
        super().__init__('llm_agent')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            String, '/speech_text', self.listener_callback, 10)
        
        # Memory
        self.conversation_history = [
            {"role": "system", "content": "You are a robot assistant. Use the provided tools to move. Do not chat, just act."}
        ]
        
        # OpenAI Client (Assume API Key is in env)
        self.client = openai.OpenAI()

    def listener_callback(self, msg):
        user_input = msg.data
        self.get_logger().info(f'User said: {user_input}')
        
        # 1. Update History
        self.conversation_history.append({"role": "user", "content": user_input})
        
        # 2. Call LLM
        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=self.conversation_history,
            tools=TOOLS,
            tool_choice="auto" 
        )
        
        tool_calls = response.choices[0].message.tool_calls
        
        # 3. Handle Tool Calls
        if tool_calls:
            for tool_call in tool_calls:
                fn_name = tool_call.function.name
                fn_args = json.loads(tool_call.function.arguments)
                
                self.get_logger().info(f'Calling tool: {fn_name} with {fn_args}')
                
                if fn_name == "move_robot":
                    self.execute_move(fn_args['linear_x'], fn_args['angular_z'])
                elif fn_name == "stop_robot":
                    self.execute_move(0.0, 0.0)
                    
            # Update history with the assistant's action
            self.conversation_history.append(response.choices[0].message)

    def execute_move(self, x, z):
        msg = Twist()
        msg.linear.x = float(x)
        msg.angular.z = float(z)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LLMAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

#### Step 3: Testing

1.  Start the Simulation (Gazebo/TurtleBot).
2.  Run the Agent Node.
3.  Publish a fake voice command:
    ```bash
    ros2 topic pub /speech_text std_msgs/msg/String "data: 'Turn around and back up'" --once
    ```
4.  **Observe**:
    *   The LLM should reason: "Turn around means angular velocity. Back up means negative linear velocity."
    *   It might call `move_robot(linear_x=-0.2, angular_z=1.57)`.
    *   The robot in Gazebo executes this.

### 4. Integration with Whisper (ASR)

Text input is easy. Audio is hard. We use `openai-whisper` (Python package).

**The ASR Node**:
1.  Captures audio from the microphone (using `PyAudio`).
2.  Buffers audio chunks (e.g., 3 seconds or silence detection).
3.  Sends the buffer to the Whisper model.
4.  Publishes the result to `/speech_text`.

*Why separate nodes?*
Separation of Concerns. The ASR node is heavy on GPU (for transcription). The LLM node waits for network I/O. The Controller node is real-time. Keeping them separate allows you to swap Whisper for Google Speech API without breaking the LLM logic.

## Capstone Prep

### Designing Your Interface

For your Capstone project, you need to decide:

1.  **Wake Word**: Will you use "Hey Robot"? (Requires a lightweight offline engine like Porcupine or OpenWakeWord). Or a Push-to-Talk button? (Easier to implement).
2.  **Safety Layer**: What if the LLM says `move_robot(100 m/s)`?
    *   **Rule**: The `execute_move` function in your code MUST clamp values.
    *   **Code**: `msg.linear.x = max(-0.5, min(0.5, x))`
    *   **Never trust the LLM.** It is a probabilistic word generator, not a safety-certified controller.

### The "System Prompt" Design

Spend time iterating on your System Prompt.
*   **Bad**: "You are a robot."
*   **Good**: "You are a generic mobile service robot. You operate in a shared home environment with humans and pets. You value safety above all. If a user asks you to do something dangerous (like run into a wall), politely refuse. Your maximum speed is 0.5m/s."

The more context you give the LLM about its physical limitations, the better it behaves.

## Advanced Topic: Retrieval Augmented Generation (RAG)

What if you want the robot to "Go to the kitchen," but it doesn't know where the kitchen is?
Standard LLMs don't know your house map.

**Solution**: RAG.
1.  **Database**: Store a list of locations: `{"kitchen": [2.0, 3.0], "bedroom": [-1.0, 5.0]}`.
2.  **Retrieval**: When the user says "Kitchen", query the database.
3.  **Augmentation**: Inject this into the prompt.
    *   *User*: "Go to kitchen."
    *   *System (Internal)*: "Context: Kitchen is at coordinates x=2.0, y=3.0."
    *   *LLM Output*: `navigate(2.0, 3.0)`.

This keeps the LLM purely as a reasoning engine, while the "Knowledge" lives in your structured database.

## Summary

We have bridged the gap between language and motion. By treating the LLM as a **Cognitive Layer** that sits between perception and action, we can use **Function Calling** to convert vague intent into structured ROS 2 commands. We addressed the challenges of latency and safety, emphasizing that the robot's lower-level controllers must always act as a safety clamp on the LLM's output.

## References

- [OpenAI Function Calling Guide](https://platform.openai.com/docs/guides/function-calling)
- [ROS 2 Python Client Library (rclpy)](https://docs.ros2.org/latest/api/rclpy/)
- [SayCan: Do As I Can, Not Just As I Say](https://say-can.github.io/)