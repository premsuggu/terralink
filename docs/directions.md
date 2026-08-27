# Technical Directions: Path vs. Obstacle Classification

To successfully route a UGV using a UAV's overhead perspective, we must translate raw sensor data into a mathematical grid that the UGV's navigation stack can interpret. We are evaluating three primary architectural directions for this perception pipeline:

## Direction 1: The Geometric Approach (2.5D Elevation Mapping)
Instead of relying on color, this approach maps the physical height of the terrain. 
* **Mechanism:** The UAV utilizes a downward-facing RGB-D (Depth) Camera or 3D LiDAR. It projects a grid over the terrain and assigns a specific height value (Z-axis) to each cell.
* **Logic:** The UGV reads the gradient of the heights. A gradual increase is classified as a traversable ramp (Path). A sudden, sharp spike in height is classified as a wall (Obstacle).
* **Pros:** Highly accurate; immune to optical illusions (e.g., a shadow looking like a hole).
* **Cons:** Computationally heavy; requires specialized depth hardware on the UAV.

## Direction 2: The Semantic Approach (AI Vision to Costmap)
This approach abandons physical geometry and relies entirely on Neural Networks to classify the environment based on pixel context.
* **Mechanism:** The UAV uses a standard, lightweight 2D RGB camera. The video feed is processed through a Semantic Segmentation model (e.g., YOLO, Mask R-CNN).
* **Logic:** The AI colors pixels based on their real-world identity. Dirt/Asphalt pixels become traversable paths; concrete/rebar pixels become lethal obstacles; water/mud pixels become high-cost zones. The UGV Nav2 stack reads these colored masks directly.
* **Pros:** Allows for contextual navigation (e.g., telling the robot to "avoid mud", which a LiDAR cannot differentiate from solid ground).
* **Cons:** Requires a heavy GPU on the UAV (or strong offboard networking) to run machine learning models in real-time.

## Direction 3: The Vision-to-Grid Baseline (OpenCV Filtering)
A highly simplified, computationally cheap approach used for immediate simulation baselining.
* **Mechanism:** The UAV looks down with a 2D camera and uses standard OpenCV color/contrast filtering. 
* **Logic:** It assumes the ground is a uniform color (e.g., gray concrete). Anything that breaks that color threshold is instantly drawn as a black obstacle on a binary 2D Occupancy Grid. The clear areas are treated as the path, and standard Probabilistic Roadmap (PRM) algorithms generate waypoints.
* **Pros:** Extremely lightweight; easy to simulate; requires no heavy ML models.
* **Cons:** Fragile in real-world lighting. Shadows or slight color changes in the dirt will be falsely flagged as solid obstacles.