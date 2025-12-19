---
id: nodes-topics
title: "Deep Dive: Nodes, Topics, and Architecture"
sidebar_label: Nodes & Topics
description: "Advanced architecture of ROS 2, including the RMW layer, DDS, and Quality of Service (QoS)."
---

# Deep Dive: Nodes, Topics, and the RMW

## Learning Objectives

By the end of this chapter, you will be able to:

1.  **Peel back the layers of ROS 2**, understanding the relationship between your code, `rclpy`, `rcl`, `rmw`, and the underlying DDS.
2.  **Configure Quality of Service (QoS)** profiles to optimize for lossy wireless networks vs. guaranteed delivery control loops.
3.  **Implement Managed Nodes (Lifecycle Nodes)** to create deterministic startup sequences.
4.  **Write Polyglot Code**: Create a C++ Publisher that talks to a Python Subscriber.

## Prerequisites

- Basic Python or C++ knowledge
- ROS 2 installed (Humble or Jazzy)

## Core Concepts

### 1. The Hidden Stack: What happens when you `publish()`?

When you write `node.create_publisher(...)` in Python, you are triggering a cascade of calls down a deep software stack. Understanding this stack is crucial for debugging performance issues.

**Layer 1: User Land (The Top)**
*   This is your code: `my_robot_controller.py` or `camera_driver.cpp`.

**Layer 2: Client Libraries (`rclpy` / `rclcpp`)**
*   These are language-specific wrappers. They provide the "syntactic sugar" that makes coding easy. They handle threading models (Executors) and memory management.

**Layer 3: The Robot Client Library (`rcl`)**
*   This is the core logic, written in **C**. Both the Python and C++ libraries call down into this C layer. This ensures that behavior is identical regardless of the language you use.

**Layer 4: The Middleware Interface (`rmw`)**
*   This is an abstraction layer. ROS 2 does not implement its own networking. It defines an interface (like an API contract) for *how* to send data.

**Layer 5: DDS Implementation (The Bottom)**
*   **DDS (Data Distribution Service)** is the industry standard for real-time systems (used in battleships, dams, and financial trading).
*   ROS 2 supports multiple vendors: **Eclipse Cyclone DDS**, **Fast DDS** (eProsima), **RTI Connext**.
*   You can swap these at runtime by setting an environment variable (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`).

### 2. Quality of Service (QoS)

In TCP/IP (the web), we only have one mode: "Reliable" (wait for acknowledgment). In robotics, that is often dangerous. If a robot is balancing, and a sensor packet is late, we don't want to wait for it. We want the *newest* packet. The old one is trash.

ROS 2 gives you fine-grained control via **QoS Policies**.

#### Reliability Policy
*   **Reliable**: Guarantee delivery. Retries lost packets. Used for: Parameter updates, Map data, Mission commands.
*   **Best Effort**: Fire and forget. If it drops, it drops. Used for: High-frequency sensor data (e.g., 60Hz camera feed).

#### Durability Policy
*   **Volatile**: New subscribers only receive messages sent *after* they joined. (Standard).
*   **Transient Local**: The publisher saves the *last* message (or last N messages) and sends it to new subscribers. Used for: Static data like "Robot Description" (URDF) or "Map".

### 3. Concurrency: Executors and Callback Groups

In ROS 1, threading was a nightmare. In ROS 2, it is managed by **Executors**.
An Executor is a scheduler. It holds a queue of "Callbacks" (functions that run when a message arrives).

*   **SingleThreadedExecutor**: The default. Runs one callback at a time. Safe, easy, but can block. If your `image_callback` takes 0.1s, your `motor_callback` is delayed by 0.1s.
*   **MultiThreadedExecutor**: Runs callbacks in parallel threads. High performance, but requires thread-safe code (Mutexes).

## Conceptual Visualization

### The Protocol Stack

Imagine the data flowing down:

1.  **Python Object**: `msg = String(data="Hello")`
2.  **Serialization**: Converted to binary via `rclpy`.
3.  **C Struct**: Passed to `rcl`.
4.  **DDS Sample**: `rmw` passes it to the DDS driver.
5.  **UDP Packet**: DDS wraps it in RTPS (Real-Time Publish Subscribe) protocol and blasts it over UDP.
6.  **The Network**: Cables or WiFi.
7.  **Reverse Process**: The Subscriber unpacks it up the stack.

### QoS Analogy: The Magazine Subscription vs. The Registered Letter

*   **Best Effort (Magazine)**: The publisher sends issues monthly. If the postman loses the January issue, the publisher does *not* resend it. You just get the February issue. This is fine; you want current news.
*   **Reliable (Registered Letter)**: The publisher sends a contract. They require a signature. If the postman loses it, they send it again and again until you sign. You cannot proceed until you get it.

## Examples / Exercises

### Exercise 1: The Polyglot Talk

We will write a C++ publisher and a Python subscriber.

#### Step A: The C++ Publisher (`talker.cpp`)

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node {
public:
  Talker() : Node("cpp_talker") {
    // create_publisher<MsgType>("topic_name", queue_size)
    publisher_ = this->create_publisher<std_msgs::msg::String>("chat", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&Talker::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello from C++! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_ = 0;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}
```

#### Step B: The Python Subscriber (`listener.py`)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('py_listener')
        # create_subscription(MsgType, "topic_name", callback, queue_size)
        self.subscription = self.create_subscription(
            String,
            'chat',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Observation**: Run them in separate terminals. The C++ node speaks, the Python node listens. The data structure `std_msgs/String` is the universal language they share.

### Exercise 2: Simulating Data Loss (QoS)

Modify the Python listener to use a `BestEffort` QoS profile, but keep the C++ talker on `Reliable`.

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

qos = QoSProfile(depth=10)
qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

self.subscription = self.create_subscription(
    String, 'chat', self.listener_callback, qos_profile=qos)
```

**Result**: They might NOT connect!
*   ROS 2 enforces **QoS Compatibility**.
*   Rule: A Subscriber can only offer *lower* or *equal* reliability than the Publisher.
*   If Publisher = Best Effort, Subscriber = Reliable -> **Incompatible** (Sub wants promises Pub can't make).
*   If Publisher = Reliable, Subscriber = Best Effort -> **Compatible** (Sub is easygoing).

## Capstone Prep

### Topic Namespace Design

Organizing topics is like organizing a file system. For our Humanoid, we will use this convention:

*   `/robot_id/sensor/camera/rgb` (Best Effort)
*   `/robot_id/sensor/lidar/scan` (Best Effort)
*   `/robot_id/control/cmd_vel` (Reliable - we don't want to lose stop commands!)
*   `/robot_id/debug/log` (Reliable)

This structure ensures that if we have two robots, we just change `robot_id` to `robot_1` and `robot_2`, and they never cross wires.

## Summary

In this chapter, we explored the hidden depths of ROS 2. We learned that **Nodes** are the computational units that can act as Publishers or Subscribers. **Topics** are the named pipes they use to exchange data. Crucially, the **RMW (Middleware)** layer allows ROS 2 to run on top of industrial-grade **DDS**, giving us powerful **QoS** controls for reliability and durability. Finally, we saw how **Lifecycle Nodes** provide a managed state machine for deterministic startup.

## References

- [ROS 2 Documentation: Nodes](https://docs.ros.org/en/humble/Concepts/Basic/About-Nodes.html)
- [ROS 2 Documentation: Topics](https://docs.ros.org/en/humble/Concepts/Basic/About-Topics.html)
- [DDS Foundation](https://www.dds-foundation.org/)