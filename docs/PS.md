# Problem Statement: Collaborative UAV-UGV Navigation Framework

## 1. The Ideal Context
Autonomous systems deployed in dynamic, unstructured environments—such as active construction sites or disaster response zones—must be able to navigate from point A to point B efficiently to deliver payloads or extract targets without human intervention.

## 2. The Reality (The Gap)
Current single-agent systems face mutually exclusive physical limitations:
* **Unmanned Ground Vehicles (UGV):** Possess the payload capacity and battery endurance to execute heavy-duty tasks. However, their localized sensors (2D LiDAR, ground-level cameras) suffer from horizontal line-of-sight occlusions. They cannot see over a wall or rubble to know if the path ahead is a dead-end.
* **Unmanned Aerial Vehicles (UAV):** Possess global situational awareness and bypass ground obstacles entirely. However, strict payload limitations and short battery life prevent them from executing ground-level, sustained physical tasks or heavy transport.

## 3. The Consequence
Because UGVs rely purely on localized planning, they frequently fall into "local minima" traps. They drive deep into a blocked corridor, discover the blockage too late, and must computationally struggle to backtrack. This results in severe mission delays, excessive energy consumption, and high rates of navigation failure in environments where topography changes rapidly (e.g., daily trench digging on a construction site).

## 4. The Proposed Objective
There is a critical need for a decoupled, heterogeneous robotic framework that exploits the distinct advantages of both agents. This project develops a ROS 2-based collaborative architecture where a UAV acts as an aerial reconnaissance node, processing overhead visual data to map paths versus obstacles. This mapping data is transmitted in real-time to a payload-capable UGV, which utilizes the global map to proactively route around distant macro-obstacles, dedicating its onboard processing exclusively to localized dynamic obstacle avoidance.