# ROS2 Camera Interface

## 📌 Overview

**ROS2 Camera Interface** is a ROS2 package designed to dynamically handle and manage multiple camera inputs. The number and type of cameras are specified through a configuration file, making the system flexible and scalable.

---

## Features

-  Supports multiple camera inputs
-  Uses a YAML config file to specify camera sources
-  Displays live camera feeds using `image_view`
-  Publishes camera data as ROS2 topics
-  Modular design for easy extension

---

## Requirements

- ROS2 (Humble recommended)
- OpenCV
- `image_tools` or `image_transport` (for visualization)
- Compatible camera drivers (e.g., USB or CSI)
