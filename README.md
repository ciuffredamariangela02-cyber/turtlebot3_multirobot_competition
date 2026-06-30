
# multirobot_competitive_simulation

Simulating Competitive Setpoints Navigation for Twin TurtleBots in Environment with obstacles

## Project Structure

```
.
├── docker_ws/             # Docker workspace for building the container
├── ros_ws/src             # Main ROS 2 workspace containing all packages
├── chown_me.sh            # Script to change ownership of files created as root user
├── runmac.sh              # Script to run the Docker container (for mac users)
├── execmac.sh             # Script to open a running container (for mac users)
├── run.sh                 # Script to run the Docker container 
├── exec.sh                # Script to open a running container
```

## How to Run the Simulation

### 1. **Build the Docker Image**

```bash
cd docker_ws
chmod +x build.sh
./build.sh
```

Ensure scripts are executable:
```bash
cd ..
chmod +x run.sh exec.sh chown_me.sh
```

### 2. **Run the Docker Container**

```bash
./run.sh
```

### 3. **Inside the Container**

Run the following commands:

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
cd /root/ros_workspace
colcon build
source install/setup.bash
ros2 launch ugv_competition metric.launch.py map_name:=symmetric robot1_metric:=euclidean robot2_metric:=estimated_time goal_seed:=100 goal_placement:=symmetric
```
> **Note:** It'll take about a minute to set up and start the simulation.

See also [Launching the competition environment](#launching-the-competition-environment) to see how configure it

---

## Running on macOS (Apple Silicon)

> **Note:** macOS does not natively support GUI display forwarding for Docker containers. To work around this, we recommend running the simulation inside a Linux virtual machine.

### 1. **Set Up the Virtual Machine**

1. Download and install [UTM](https://mac.getutm.app/)
2. Download the [**Ubuntu 24.04 LTS Server**](https://ubuntu.com/download/server/arm) 
3. Create a new VM in UTM using the Ubuntu ISO and complete the installation

### 2. **Install a Desktop Environment**

Once Ubuntu is installed and running, install a lightweight desktop:

```bash
sudo apt-get install -y xorg xfce4
```

### 3. **Install Git and Docker**

### 4. **Enable Display Forwarding**

In a terminal on your Mac, run:

```bash
xhost +
```

### 5. **Start the Desktop Environment**

Inside the VM, launch the XFCE desktop:

```bash
startxfce4
```

### 6. **Connect VS Code via SSH**

Use the **Remote - SSH** extension in VS Code to connect to your Ubuntu VM. This allows you to edit files and run terminals directly from your Mac.

### 7. **Clone the Repository**

```bash
git clone <repository-url>
cd multirobot_competitive_simulation
```

### 8. **Build and Run**

```bash
cd docker_ws
chmod +x build_robotics2.sh
./build_robotics2.sh
```

Ensure scripts are executable:
```bash
cd ..
chmod +x runmac.sh execmac.sh chown_me.sh
./runmac.sh
```

### 9. **Inside the Container**

Run the following commands:

```bash
colcon build
source install/setup.bash
ros2 launch ugv_competition metric.launch.py map_name:=symmetric robot1_metric:=euclidean robot2_metric:=estimated_time goal_seed:=100 goal_placement:=symmetric
```
> **Note:** It'll take about a minute to set up and start the simulation.

See also [Launching the competition environment](#launching-the-competition-environment) to see how configure it

---

<a id="launching-the-competition-environment"></a>
## Launching the Competition Environment (`metric.launch.py`)

This launch file acts as the primary entry point for the UGV competition simulation. It is highly configurable, allowing you to easily adjust the pathfinding metrics for each robot, switch between maps, and dictate how goals are generated.

### Configuration Parameters

You can customize the simulation behavior by passing the following arguments to the launch command:

* **`robot1_metric` & `robot2_metric`**: Defines the cost/distance metric used by each respective robot.
    * *Supported values:* `euclidean`, `manhattan`, `estimated_time`
    * *Default behavior:* `euclidean`
* **`map_name`**: Selects the environment map to load.
    * *Supported values:* `custom`, `symmetric`
    * *Default behavior:* `custom`
* **`goal_placement`**: Determines the strategy for spawning goals in the environment.
    * *Supported values:* `random`, `symmetric`
    * *Default behavior:* `random` (when using the `custom` map); `symmetric` (when using the `symmetric` map).
* **`goal_seed`**: An integer seed for the random number generator to ensure reproducible goal placements.

### Usage Examples

**1. Custom map with different metrics for each robot:**
```bash
ros2 launch ugv_competition metric.launch.py robot1_metric:=euclidean robot2_metric:=manhattan map_name:=custom
```

## Video Demos

Recordings of the simulation runs are available at the links below.

| Scenario| Link |
|---|---|
| Symmetric map – Euclidean vs Estimated Time | [Watch](url) |
| Custom map – Euclidean vs Manhattan | [Watch](url) |
| Custom map – Manhattan vs Estimated Time | [Watch](url) |
