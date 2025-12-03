# 🤖 ROS Turtlesim Square Drawing Node

This project contains a simple **ROS (Robot Operating System) Python node** that controls the virtual turtle in the **turtlesim** simulation environment to draw a perfect square. 

---

## ⚙️ How It Works

This node uses **open-loop control** coupled with feedback from the turtle's position (`Pose`) to execute precise movements. The logic executes a cycle of moving a set distance and then turning a set angle, repeated four times.

### 1. **Key ROS Components**

| Component | Topic/Type | Role |
| :--- | :--- | :--- |
| **Publisher** | `/turtle1/cmd_vel` (`Twist`) | Sends linear (`x`) and angular (`z`) velocity commands to the turtle. |
| **Subscriber** | `/turtle1/pose` (`Pose`) | Receives the turtle's current position $(x, y)$ and orientation $(\theta)$. |
| **Node** | `draw_square` | The main execution program. |

### 2. **Parameters**

The size and speed of the square are controlled by these variables:

| Variable | Value | Unit | Description |
| :--- | :--- | :--- | :--- |
| `speed` | 2.0 | m/s | **Linear velocity** used for straight movement. |
| `angular_speed` | 1.5 | rad/s | **Angular velocity** used for turning. |
| `side_length` | 2.0 | meters | The desired length of each side of the square. |

### 3. **The Drawing Cycle Logic**

The core logic iterates four times, performing the following two steps:

#### A. Move Straight (Side Traversal)
1.  **Saves** the starting $(x, y)$ coordinates.
2.  **Commands** the turtle to move forward at `speed`.
3.  **Monitors** the distance traveled using the **Euclidean distance formula**:
    $$dist = \sqrt{(x_{current} - x_{start})^2 + (y_{current} - y_{start})^2}$$
4.  **Stops** motion when $dist \ge side\_length$.

#### B. Turn 90 Degrees (Corner Rotation)
1.  **Saves** the starting orientation ($\theta_{start}$).
2.  **Commands** the turtle to rotate at `angular_speed`.
3.  **Monitors** the angle turned. It includes a specific check to handle the $\pm\pi$ boundary wrap-around in the angle measurement, ensuring the shortest rotation is always calculated.
4.  **Stops** rotation when the angle turned is $\ge \frac{\pi}{2}$ radians (90 degrees).

After the loop completes, a final command sets all velocities to zero, ensuring the turtle stops completely. 

---

## How to Run

1.  **Start the ROS Master:**
    ```bash
    roscore
    ```
2.  **Start the Turtlesim Simulator:** Open a new terminal and run:
    ```bash
    rosrun turtlesim turtlesim_node
    ```
3.  **Run the Square Drawer Node:** Open a third terminal, save the script as `draw_square.py` within your ROS package (e.g., `src` folder), ensure it is executable, and run it:
    ```bash
    chmod +x draw_square.py
    rosrun turtle_training draw_square.py
    ```
