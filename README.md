# Dual - Motion Planning with Dual UR5e
![Dual_logo](https://github.com/user-attachments/assets/11a092f0-505f-4755-ab91-810e18b07f7a)

## 🚀 Welcome to Dual!

This project integrates dual **UR5e** robotic arms using **ROS 2 Humble** for advanced motion planning and manipulation tasks. Designed for flexibility and modularity, it provides a foundation for robotic research and industrial applications.

---

## 📌 Branch Structure

This repository consists of multiple branches, each tailored to specific configurations:

- **`main` (Standalone Mode)** – Contains only the dual **UR5e** arms without any attached tools, providing a clean foundation for motion control and trajectory planning.
- *(Additional branches include tools and specific configurations, refer to the repository for details.)*

---

## 🔧 Features

✅ Dual-arm motion planning using **MoveIt 2**  
✅ Configurable URDF and **Xacro** for modular robot setup  
✅ Supports **Gazebo** and real-world execution  
✅ **ROS 2 Humble** compatibility  
✅ Easily extendable for different robotic end-effectors  

---

## 📂 Repository Overview

This project is structured into the following main components:

```
📂 UR_SOFTHAND_DUAL
 ├── Demos/          # Demonstrations and example configurations
 ├── scripts/        # Python-based motion planning scripts
 ├── src/            # Source code and ROS 2 packages
 ├── README.md       # You're here! 👋
```

---

## 🛠️ Setup & Installation

1️⃣ Install **ROS 2 Humble** and dependencies, including:
```bash
sudo apt update && sudo apt install -y ros-humble-ur-ros2-driver
```
2️⃣ Clone the repository:
```bash
cd ~/ros2_ws/src
git clone https://github.com/yourrepo/UR_SOFTHAND_DUAL.git
cd ..
rosdep update && rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```
3️⃣ Launch the system in separate terminals with the following steps:

**Terminal 1:**
```bash
ros2 launch ur_dual_control start_robot.launch.py
```

**Terminal 2:**
```bash
ros2 launch ur_dual_moveit_config move_group.launch.py
```

**Terminal 3:**
```bash
ros2 launch ur_dual_moveit_config moveit_rviz.launch.py
```

---

## 🎯 Future Expansions

💡 Implement **machine learning** for adaptive grasping  
💡 Add **force feedback** for precision tasks  
💡 Optimize real-time **motion planning algorithms**  

Stay tuned for upcoming branches featuring **soft hands, grippers, and AI-driven control**! 🚀

---

## 🏆 Acknowledgments

Special thanks to **GII Lab at Universidade da Coruña** for supporting this research. 🎓

💬 Got questions? Feel free to open an issue or contribute!

## ✨ Contributors

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tbody>
    <tr>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/SantaCRC"><img src="https://avatars.githubusercontent.com/u/35088759?v=4?s=100" width="100px;" alt="Fabian Alvarez"/><br /><sub><b>Fabian Alvarez</b></sub></a><br /><a href="#projectManagement-SantaCRC" title="Project Management">📆</a></td>
    </tr>
  </tbody>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->

---

✨ *Empowering the future of robotics—one motion at a time!* ✨

