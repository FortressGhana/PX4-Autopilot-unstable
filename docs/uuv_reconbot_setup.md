# UUV Reconbot Gazebo Classic Simulation Setup

Complete documentation for setting up the Fortress Ghana Reconbot UUV simulation in PX4 with Gazebo Classic.

## Directory Structure

```
PX4-Autopilot/
├── ROMFS/px4fmu_common/init.d-posix/airframes/
│   ├── CMakeLists.txt                          # Register airframe
│   └── 22001_gazebo-classic_uuv_reconbot       # Airframe config
├── src/modules/simulation/simulator_mavlink/
│   └── sitl_targets_gazebo-classic.cmake       # Register build target
└── Tools/simulation/gazebo-classic/sitl_gazebo-classic/
    ├── models/uuv_reconbot/
    │   ├── model.config                        # Model metadata
    │   ├── uuv_reconbot.sdf                    # Model definition
    │   └── meshes/
    │       ├── fortress_reconbot.obj           # Main body mesh
    │       └── propellers.stl                  # Propeller mesh
    └── worlds/
        └── ocean.world                         # Ocean environment
```

## Naming Conventions

### Airframe File
- **Format:** `<SYS_AUTOSTART>_<simulator>_<model_name>`
- **Example:** `22001_gazebo-classic_uuv_reconbot`
- `22001` - SYS_AUTOSTART ID (22000-22999 reserved for custom models)
- `gazebo-classic` - Simulator type
- `uuv_reconbot` - Model name (must match model directory name)

### Model Directory
- Directory name must match `PX4_SIM_MODEL` in airframe config
- SDF file must be named `<model_name>.sdf` (e.g., `uuv_reconbot.sdf`)

### World File
- Referenced by `PX4_GZ_WORLD` in airframe config
- Located in `worlds/` directory with `.world` extension

---

## Step 1: Create the Gazebo Model

### 1.1 Model Directory
Create: `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/uuv_reconbot/`

### 1.2 Model Config (`model.config`)
```xml
<?xml version="1.0"?>
<model>
  <name>uuv_reconbot</name>
  <version>1.0</version>
  <sdf version="1.6">uuv_reconbot.sdf</sdf>
  <author>
    <name>Nathaniel Asiak</name>
    <email>nathaniel.asiak@example.com</email>
  </author>
  <description>
    Model of the Fortress Ghana Recon Robot with 8 thrusters.
  </description>
</model>
```

### 1.3 Model SDF (`uuv_reconbot.sdf`)

Key components:

#### Base Link with Inertial Properties
```xml
<link name="base_link">
  <inertial>
    <pose>0.13 0 0 0 0 0</pose>
    <mass>176.7</mass>
    <inertia>
      <ixx>8.01</ixx>  <ixy>0.05</ixy>  <ixz>0.01</ixz>
      <iyy>30.89</iyy> <iyz>-0.21</iyz>
      <izz>32.82</izz>
    </inertia>
  </inertial>
  <!-- Visual and collision meshes -->
</link>
```

#### IMU Sensor
```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>250</update_rate>
  <plugin name="imu_plugin" filename="libgazebo_imu_plugin.so">
    <linkName>base_link</linkName>
    <imuTopic>/imu</imuTopic>
    <!-- Noise parameters -->
  </plugin>
</sensor>
```

#### Thruster Links (8 total)
Each thruster has:
- Link with pose, inertial, visual, and collision
- Revolute joint connecting to base_link
- Motor plugin for thrust generation

**Thruster Layout:**
| # | Name | Position | Function |
|---|------|----------|----------|
| 1 | Left Front Horizontal | (-0.067, 0.141, 0.110) | Surge/Yaw |
| 2 | Right Front Horizontal | (-0.067, -0.137, 0.110) | Surge/Yaw |
| 3 | Front Vertical | (0.319, 0.082, 0.168) | Heave/Pitch |
| 4 | Rear Vertical | (-0.261, 0.082, 0.170) | Heave/Pitch |
| 5 | Front Right Angled | (0.469, 0.129, -0.1) | Surge/Heave/Roll |
| 6 | Front Left Angled | (-0.469, -0.129, -0.1) | Surge/Heave/Roll |
| 7 | Rear Left Angled | (-0.415, -0.129, -0.1) | Surge/Heave/Roll |
| 8 | Rear Right Angled | (-0.415, 0.129, -0.1) | Surge/Heave/Roll |

#### Required Plugins
```xml
<!-- Ground truth for position -->
<plugin name='groundtruth_plugin' filename='libgazebo_groundtruth_plugin.so'/>

<!-- UUV hydrodynamics and buoyancy -->
<plugin name='uuv_forces' filename='libgazebo_uuv_plugin.so'>
  <buoyancy>
    <link_name>base_link</link_name>
    <compensation>1.1</compensation>
  </buoyancy>
  <addedMassLinear>92.0 315.0 315.0</addedMassLinear>
  <dampingLinear>-80.0 -250.0 -250.0</dampingLinear>
</plugin>

<!-- Magnetometer -->
<plugin name='magnetometer_plugin' filename='libgazebo_magnetometer_plugin.so'/>

<!-- Barometer -->
<plugin name='barometer_plugin' filename='libgazebo_barometer_plugin.so'/>

<!-- MAVLink interface to PX4 -->
<plugin name="mavlink_interface" filename="libgazebo_mavlink_interface.so">
  <mavlink_udp_port>14560</mavlink_udp_port>
  <control_channels>
    <!-- 8 motor channels -->
  </control_channels>
</plugin>

<!-- Motor plugins (one per thruster) -->
<plugin name="motor_1" filename="libgazebo_motor_model.so">
  <jointName>thruster_1_joint</jointName>
  <motorNumber>0</motorNumber>
  <!-- Motor parameters -->
</plugin>
```

### 1.4 Mesh Files
Place in `meshes/` subdirectory:
- `fortress_reconbot.obj` - Main body mesh
- `propellers.stl` - Propeller mesh (shared by all thrusters)

---

## Step 2: Create the World File

Create: `Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/ocean.world`

```xml
<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="oceans">
    <scene>
      <ambient>0.01 0.01 0.01 1.0</ambient>
      <sky><clouds><speed>12</speed></clouds></sky>
    </scene>

    <!-- GPS origin: North Sea -->
    <spherical_coordinates>
      <latitude_deg>56.71897669633431</latitude_deg>
      <longitude_deg>3.515625</longitude_deg>
    </spherical_coordinates>

    <!-- Lighting -->
    <light type="directional" name="sun1">...</light>

    <!-- Ocean surface model -->
    <include>
      <uri>model://ocean</uri>
    </include>

    <!-- Physics settings -->
    <physics name='default_physics' type='ode'>
      <gravity>0 0 -9.8066</gravity>
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1</real_time_factor>
      <magnetic_field>6.0e-6 2.3e-5 -4.2e-5</magnetic_field>
    </physics>
  </world>
</sdf>
```

---

## Step 3: Create the PX4 Airframe

Create: `ROMFS/px4fmu_common/init.d-posix/airframes/22001_gazebo-classic_uuv_reconbot`

```bash
#!/bin/sh
# @name UUV Reconbot Configuration

. ${R}etc/init.d/rc.uuv_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=gazebo-classic}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=ocean}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=uuv_reconbot}

# Airframe type: UUV
param set-default CA_AIRFRAME 7

# 8 thrusters with bidirectional capability
param set-default CA_ROTOR_COUNT 8
param set-default CA_R_REV 255

# Thruster 1: Left Front Horizontal
param set-default CA_ROTOR0_AX 1.0
param set-default CA_ROTOR0_AY 0.0
param set-default CA_ROTOR0_AZ 0.0
param set-default CA_ROTOR0_KM 0
param set-default CA_ROTOR0_PX -0.0667
param set-default CA_ROTOR0_PY 0.14116
param set-default CA_ROTOR0_PZ 0.110

# ... (repeat for thrusters 2-8)

# PWM function mapping
param set-default PWM_MAIN_FUNC1 101
param set-default PWM_MAIN_FUNC2 102
param set-default PWM_MAIN_FUNC3 103
param set-default PWM_MAIN_FUNC4 104
param set-default PWM_MAIN_FUNC5 105
param set-default PWM_MAIN_FUNC6 106
param set-default PWM_MAIN_FUNC7 107
param set-default PWM_MAIN_FUNC8 108
```

### Control Allocation Parameters

| Parameter | Description |
|-----------|-------------|
| `CA_AIRFRAME` | 7 = UUV |
| `CA_ROTOR_COUNT` | Number of thrusters |
| `CA_R_REV` | Bitmask for reversible thrusters (255 = all 8) |
| `CA_ROTORn_AX/AY/AZ` | Thrust axis direction |
| `CA_ROTORn_PX/PY/PZ` | Thruster position relative to CoG |
| `CA_ROTORn_KM` | Moment coefficient (0 for thrusters) |

---

## Step 4: Register in CMakeLists

Edit: `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`

Add to `px4_add_romfs_files()`:
```cmake
px4_add_romfs_files(
    ...
    22001_gazebo-classic_uuv_reconbot
    # [22000, 22999] Reserve for custom models
)
```

---

## Step 5: Register Build Target

Edit: `src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake`

Add `uuv_reconbot` to the `models` list:
```cmake
set(models
    ...
    uuv_bluerov2_heavy
    uuv_hippocampus
    uuv_reconbot          # Add this line
)
```

---

## Step 6: Install Gazebo Classic

The `gazebo-classic` targets require Gazebo Classic 11 (not new Gazebo):

```bash
sudo apt install gazebo libgazebo-dev
```

Both can coexist:
| | Gazebo Classic | New Gazebo |
|---|---|---|
| Commands | `gazebo`, `gzserver`, `gzclient` | `gz sim` |
| Libraries | `libgazebo*` | `libgz-*` |

---

## Step 7: Build and Run

```bash
# Clean previous build (required after cmake changes)
rm -rf build/px4_sitl_default

# Build and run
make px4_sitl gazebo-classic_uuv_reconbot

# Run headless (no GUI)
HEADLESS=1 make px4_sitl gazebo-classic_uuv_reconbot

# Run with specific world
PX4_GZ_WORLD=ocean make px4_sitl gazebo-classic_uuv_reconbot
```

---

## Troubleshooting

### Unknown Build Target
```
ninja: error: unknown target 'gazebo-classic_uuv_reconbot'
```
**Fix:** Add model to `sitl_targets_gazebo-classic.cmake` models list.

### Gazebo Not Found
```
You need to have gazebo simulator installed!
```
**Fix:** Install Gazebo Classic: `sudo apt install gazebo libgazebo-dev`

### Model Not Found
```
Model uuv_reconbot not found in model path
```
**Fix:** Ensure model directory name matches `PX4_SIM_MODEL` and contains valid SDF.

### WSL2 Connection Errors
```
[Err] Unable to initialize transport
```
**Fix:**
```bash
killall -9 gzserver gzclient
export GAZEBO_IP=127.0.0.1
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
# Or run headless:
HEADLESS=1 make px4_sitl gazebo-classic_uuv_reconbot
```

---

## File Checklist

- [ ] `models/uuv_reconbot/model.config`
- [ ] `models/uuv_reconbot/uuv_reconbot.sdf`
- [ ] `models/uuv_reconbot/meshes/*.obj|*.stl|*.dae`
- [ ] `worlds/ocean.world`
- [ ] `airframes/22001_gazebo-classic_uuv_reconbot`
- [ ] `airframes/CMakeLists.txt` (model registered)
- [ ] `sitl_targets_gazebo-classic.cmake` (model in list)
- [ ] Gazebo Classic 11 installed

---

## Gazebo Sim (New Gazebo) Migration

This section covers migrating the UUV Reconbot from Gazebo Classic to Gazebo Sim (formerly Ignition Gazebo).

### Key Differences

| Aspect | Gazebo Classic | Gazebo Sim |
|--------|----------------|------------|
| Airframe ID | `22001_gazebo-classic_*` | `4xxx_gz_*` |
| Simulator var | `PX4_SIMULATOR=gazebo-classic` | `PX4_SIMULATOR=gz` |
| Enable param | N/A | `SIM_GZ_EN 1` |
| Actuator params | `PWM_MAIN_FUNCn` | `SIM_GZ_EC_FUNCn` |
| Model location | `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/` | `Tools/simulation/gz/models/` |
| World location | `worlds/*.world` | `worlds/*.sdf` |
| Motor plugin | `libgazebo_motor_model.so` | `gz-sim-multicopter-motor-model-system` |
| UUV plugin | `libgazebo_uuv_plugin.so` | `gz-sim-buoyancy-system` + `gz-sim-hydrodynamics-system` |

### Directory Structure for Gazebo Sim

```
PX4-Autopilot/
├── ROMFS/px4fmu_common/init.d-posix/airframes/
│   └── 4021_gz_uuv_reconbot              # New airframe (4000-series)
└── Tools/simulation/gz/
    ├── models/uuv_reconbot/
    │   ├── model.config
    │   ├── model.sdf                     # Gazebo Sim format
    │   └── meshes/
    │       ├── fortress_reconbot.obj
    │       └── propellers.stl
    └── worlds/
        └── ocean.sdf                     # Ocean world for Gazebo Sim
```

### Gazebo Sim Airframe (`4021_gz_uuv_reconbot`)

```bash
#!/bin/sh
#
# @name Gazebo Sim UUV Reconbot
#
# @type UUV
#

. ${R}etc/init.d/rc.uuv_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=ocean}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=uuv_reconbot}

param set-default SIM_GZ_EN 1

param set-default SENS_EN_GPSSIM 1
param set-default SENS_EN_BAROSIM 0
param set-default SENS_EN_MAGSIM 1

param set-default CA_AIRFRAME 7
param set-default CA_ROTOR_COUNT 8
param set-default CA_R_REV 255

# Thruster 1-8 configuration (same as Gazebo Classic)
# ... (CA_ROTORn_* parameters)

# Gazebo Sim ESC function mapping (instead of PWM_MAIN_FUNCn)
param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_FUNC2 102
param set-default SIM_GZ_EC_FUNC3 103
param set-default SIM_GZ_EC_FUNC4 104
param set-default SIM_GZ_EC_FUNC5 105
param set-default SIM_GZ_EC_FUNC6 106
param set-default SIM_GZ_EC_FUNC7 107
param set-default SIM_GZ_EC_FUNC8 108
```

### Gazebo Sim Model SDF Template

The model SDF for Gazebo Sim uses different plugins:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='uuv_reconbot'>
    <!-- Base link with inertial, visual, collision -->
    <link name="base_link">
      <inertial>
        <pose>0.13 0 0 0 0 0</pose>
        <mass>176.7</mass>
        <inertia>
          <ixx>8.01</ixx>  <ixy>0.05</ixy>  <ixz>0.01</ixz>
          <iyy>30.89</iyy> <iyz>-0.21</iyz>
          <izz>32.82</izz>
        </inertia>
      </inertial>
      <!-- Visual and collision elements -->
    </link>

    <!-- Thruster links and joints (8 total) -->

    <!-- Gazebo Sim Buoyancy Plugin -->
    <plugin filename="gz-sim-buoyancy-system" name="gz::sim::systems::Buoyancy">
      <graded_buoyancy>
        <default_density>1025</default_density>  <!-- Seawater -->
        <density_change>
          <above_depth>0</above_depth>
          <density>1.225</density>  <!-- Air above water -->
        </density_change>
      </graded_buoyancy>
    </plugin>

    <!-- Gazebo Sim Hydrodynamics Plugin -->
    <plugin filename="gz-sim-hydrodynamics-system" name="gz::sim::systems::Hydrodynamics">
      <link_name>base_link</link_name>
      <xDotU>-92.0</xDotU>
      <yDotV>-315.0</yDotV>
      <zDotW>-315.0</zDotW>
      <xU>-80.0</xU>
      <yV>-250.0</yV>
      <zW>-250.0</zW>
    </plugin>

    <!-- Thruster plugins (one per thruster) -->
    <plugin filename="gz-sim-thruster-system" name="gz::sim::systems::Thruster">
      <joint_name>thruster_1_joint</joint_name>
      <thrust_coefficient>0.004422</thrust_coefficient>
      <fluid_density>1025</fluid_density>
      <propeller_diameter>0.1</propeller_diameter>
      <topic>command/motor_speed</topic>
    </plugin>
    <!-- Repeat for thrusters 2-8 -->

    <!-- IMU Sensor -->
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu">
      <topic>imu</topic>
    </plugin>

    <!-- Odometry for ground truth -->
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_topic>odometry</odom_topic>
    </plugin>
  </model>
</sdf>
```

### Gazebo Sim Ocean World (`ocean.sdf`)

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="ocean">
    <physics name="1ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-buoyancy-system" name="gz::sim::systems::Buoyancy">
      <graded_buoyancy>
        <default_density>1025</default_density>
        <density_change>
          <above_depth>0</above_depth>
          <density>1.225</density>
        </density_change>
      </graded_buoyancy>
    </plugin>

    <spherical_coordinates>
      <latitude_deg>56.71897669633431</latitude_deg>
      <longitude_deg>3.515625</longitude_deg>
    </spherical_coordinates>

    <gravity>0 0 -9.8066</gravity>
    <magnetic_field>6.0e-6 2.3e-5 -4.2e-5</magnetic_field>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ocean surface plane -->
    <model name="ocean_surface">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>1000 1000</size></plane>
          </geometry>
          <material>
            <ambient>0.0 0.1 0.3 0.8</ambient>
            <diffuse>0.0 0.2 0.5 0.8</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Build and Run with Gazebo Sim

```bash
# Install Gazebo Sim (Garden or later)
sudo apt install gz-garden

# Build and run
make px4_sitl gz_uuv_reconbot

# Run with specific world
PX4_GZ_WORLD=ocean make px4_sitl gz_uuv_reconbot

# Run headless
HEADLESS=1 make px4_sitl gz_uuv_reconbot
```

### Notes

- Gazebo Sim uses `gz-sim-*-system` plugins instead of `libgazebo_*_plugin.so`
- The `gz-sim-thruster-system` plugin is purpose-built for underwater thrusters
- Both Gazebo Classic and Gazebo Sim can coexist on the same system
- Gazebo Sim requires gz-transport 13 or 14 libraries
