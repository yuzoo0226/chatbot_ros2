# Chatbot based GPT for ROS2

## Environments

- Ubuntu22.04
- ROS Humble

## How to clone

```bash
git clone https://github.com/yuzoo0226/chatbot_ros2.git
```

## How to build

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release  # all
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to chatbot_node  # this package only
```

## How to run

```bash
export OPENAI_API = "your-openai-key"
source install/setup.bash
ros2 run chatbot_node chatbot_server
```
