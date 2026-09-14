#!/usr/bin/env python3
"""
Convert FBX models from the LowPoly-House-Construction-Site pack
to Gazebo-compatible SDF models.

This script creates:
1. Model directories with model.config and model.sdf for each FBX
2. A world SDF file that includes all models positioned appropriately

Note: Gazebo (Ignition Fortress) can load FBX files via Assimp, but for best
compatibility, consider converting to COLLADA (.dae) or OBJ using Blender.
"""

import os
import shutil
from pathlib import Path

# Source and destination paths
SOURCE_DIR = Path("/home/prem/terralink/gazebo_world_conversion/LowPoly-House-Construction-Site-By-Majadroid/fbx files")
DEST_DIR = Path("/home/prem/terralink/gazebo_world_conversion/converted_world/models")
WORLD_DIR = Path("/home/prem/terralink/gazebo_world_conversion/converted_world/worlds")

# Model definitions: (fbx_filename, model_name, pose, scale)
# pose: x y z roll pitch yaw
# scale: x y z
MODELS = [
    ("Ground.fbx", "ground", "0 0 0 0 0 0", "1 1 1"),
    ("Building.fbx", "building", "0 0 0 0 0 0", "1 1 1"),
    ("Road.fbx", "road", "0 0 0 0 0 0", "1 1 1"),
    ("Fence.fbx", "fence", "0 0 0 0 0 0", "1 1 1"),
    ("Ramp.fbx", "ramp", "0 0 0 0 0 0", "1 1 1"),
    ("Containers.fbx", "containers", "0 0 0 0 0 0", "1 1 1"),
    ("Containers-Cargo.fbx", "containers_cargo", "0 0 0 0 0 0", "1 1 1"),
    ("Containers-Office.fbx", "containers_office", "0 0 0 0 0 0", "1 1 1"),
    ("Construction-Materials.fbx", "construction_materials", "0 0 0 0 0 0", "1 1 1"),
    ("Crane-Mounted.fbx", "crane_mounted", "0 0 0 0 0 0", "1 1 1"),
    ("Crane-On-Ground.fbx", "crane_on_ground", "0 0 0 0 0 0", "1 1 1"),
    ("Trucks.fbx", "trucks", "0 0 0 0 0 0", "1 1 1"),
    # The all-in-one object - use this as alternative if individual models don't work well
    ("_HouseConstructionSite-AllInOneObject_.fbx", "construction_site_all", "0 0 0 0 0 0", "1 1 1"),
]

def create_model_config(model_name, description=None):
    """Create model.config file for a Gazebo model."""
    if description is None:
        description = f"{model_name} from LowPoly Construction Site pack by Majadroid (CC0)"
    
    return f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Majadroid (converted for Gazebo)</name>
    <email></email>
  </author>
  <description>
    {description}
  </description>
</model>
"""

def create_model_sdf(model_name, mesh_file, pose, scale):
    """Create model.sdf file for a Gazebo model."""
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://{mesh_file}</uri>
            <scale>{scale}</scale>
          </mesh>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://{mesh_file}</uri>
            <scale>{scale}</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>0.7 0.7 0.7 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
      <pose>{pose}</pose>
    </link>
  </model>
</sdf>
"""

def create_world_sdf(use_all_in_one=False):
    """Create the main world SDF file."""
    if use_all_in_one:
        models_include = '''
    <include>
      <name>construction_site_all</name>
      <uri>model://construction_site_all</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>'''
    else:
        models_include = ""
        for fbx_file, model_name, pose, scale in MODELS:
            if model_name != "construction_site_all":
                models_include += f'''
    <include>
      <name>{model_name}</name>
      <uri>model://{model_name}</uri>
      <pose>{pose}</pose>
    </include>'''
    
    return f"""<?xml version="1.0" ?>
<!--
  LowPoly House Construction Site by Majadroid (CC0)
  Converted for Gazebo Ignition Fortress
  
  This world includes all the construction site models positioned at origin.
  The models are static and have collision/visual meshes from the FBX files.
-->
<sdf version="1.6">
  <world name="construction_site_lowpoly">
    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="gz::sim::systems::Physics">
    </plugin>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    <plugin filename="ignition-gazebo-user-commands-system" name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin filename="ignition-gazebo-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <gui fullscreen="false">
      <plugin filename="MinimalScene" name="3D View">
        <ignition-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </ignition-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.4 0.4 0.4</ambient_light>
        <background_color>0.8 0.8 0.8</background_color>
        <camera_pose>-20 0 15 0 0.5 0</camera_pose>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager">
        <ignition-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </ignition-gui>
      </plugin>
      <plugin filename="InteractiveViewControl" name="Interactive view control">
        <ignition-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </ignition-gui>
      </plugin>
      <plugin filename="CameraTracking" name="Camera Tracking">
        <ignition-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </ignition-gui>
      </plugin>
      <plugin filename="WorldControl" name="World control">
        <ignition-gui>
          <title>World control</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">72</property>
          <property type="double" key="width">121</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="left" target="left"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </ignition-gui>
        <play_pause>true</play_pause>
        <step>true</step>
        <start_paused>false</start_paused>
        <use_event>true</use_event>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <ignition-gui>
          <title>World stats</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">110</property>
          <property type="double" key="width">290</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </ignition-gui>
        <sim_time>true</sim_time>
        <real_time>true</real_time>
        <real_time_factor>true</real_time_factor>
        <iterations>true</iterations>
      </plugin>
    </gui>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 500 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane as fallback -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.33 0.42 0.18 1</ambient>
            <diffuse>0.45 0.55 0.25 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Construction site models -->
    {models_include}

    <!-- UAV (iris_quad) will be spawned by launch file -->
  </world>
</sdf>
"""

def main():
    print("Setting up Gazebo models from FBX files...")
    
    # Create destination directories
    DEST_DIR.mkdir(parents=True, exist_ok=True)
    WORLD_DIR.mkdir(parents=True, exist_ok=True)
    
    # Process each model
    for fbx_file, model_name, pose, scale in MODELS:
        src_path = SOURCE_DIR / fbx_file
        if not src_path.exists():
            print(f"WARNING: Source file not found: {src_path}")
            continue
        
        model_dir = DEST_DIR / model_name
        model_dir.mkdir(parents=True, exist_ok=True)
        
        # Copy FBX file (Gazebo can load FBX via Assimp)
        dst_fbx = model_dir / fbx_file
        shutil.copy2(src_path, dst_fbx)
        print(f"Copied {fbx_file} -> {model_dir}")
        
        # Create model.config
        config_content = create_model_config(model_name)
        (model_dir / "model.config").write_text(config_content)
        
        # Create model.sdf
        sdf_content = create_model_sdf(model_name, fbx_file, pose, scale)
        (model_dir / "model.sdf").write_text(sdf_content)
        
        print(f"Created model: {model_name}")
    
    # Create world files - two versions
    world_individual = WORLD_DIR / "construction_site_individual.world"
    world_individual.write_text(create_world_sdf(use_all_in_one=False))
    print(f"Created world: {world_individual}")
    
    world_all_in_one = WORLD_DIR / "construction_site_all_in_one.world"
    world_all_in_one.write_text(create_world_sdf(use_all_in_one=True))
    print(f"Created world: {world_all_in_one}")
    
    print("\nDone! Models are in:", DEST_DIR)
    print("World files are in:", WORLD_DIR)
    print("\nTo test:")
    print(f"  gz sim {world_individual}")
    print(f"  gz sim {world_all_in_one}")
    print("\nNote: If FBX loading fails, convert FBX to DAE/OBJ using Blender:")
    print("  blender --background --python convert_fbx_to_dae.py")

if __name__ == "__main__":
    main()