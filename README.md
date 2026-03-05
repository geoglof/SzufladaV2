# Szuflada V2 -- ROS2 Autonomous Rover

Differential-drive rover built around an **ESP32** with W5500 Ethernet, quadrature encoders, HC-SR04 ultrasonic sensor, and an optional Intel RealSense D435i camera. The ROS2 stack runs inside Docker on a laptop/Chromebook and communicates with the ESP32 over MQTT through a Raspberry Pi broker.

## Hardware

| Part | Details |
|------|---------|
| MCU | ESP32-DevKit (ESP-WROOM-32) |
| Networking | W5500 Ethernet module (SPI), connected via Ethernet switch |
| Motors | 2x DC motors with BTS7960 drivers |
| Encoders | Quadrature (A/B channels), GPIO 34-39 |
| Ultrasonic | HC-SR04 (GPIO 32/33) |
| Camera | Intel RealSense D435i (USB, optional) |
| Power | Dell HP-L161NF3P (motors + encoders), USB powerbank (ESP32) |
| MQTT Broker | Mosquitto on Raspberry Pi (192.168.1.1) |

**Calibrated values:**
- Wheel diameter: 0.065 m
- Wheel base (effective): 0.264 m
- Encoder ticks per revolution: 437 (both wheels)

## Architecture

```
┌─────────────────── Docker (OrionRover) ───────────────────┐
│                                                            │
│  mqtt_bridge ──── ONE MQTT connection ──── ESP32           │
│       │                                                    │
│       ├── /encoder_raw ──► odometry_node ──► /odom + TF   │
│       ├── /distance ─────► /range                          │
│       ├── /motor_left  ◄── cmd_vel_bridge ◄── /cmd_vel     │
│       └── /motor_right ◄──┘                                │
│                                                            │
│  Behavior nodes (run ONE at a time):                       │
│    rover_teleop / obstacle_avoidance / wall_follower /     │
│    visual_tracker / move_distance  ──► /cmd_vel            │
│                                                            │
│  robot_description ──► static TF frames                    │
│  depthimage_to_laserscan ──► /scan (if camera running)     │
└────────────────────────────────────────────────────────────┘
```

Only **one MQTT connection** exists (mqtt_bridge). All other nodes are pure ROS2.

## Setup from Scratch

### 1. Flash the ESP32

Open `nowykodesp.txt` in Arduino IDE (or create a new sketch with its contents).

**Required libraries** (install via Arduino Library Manager):
- `Ethernet` (built-in)
- `PubSubClient` by Nick O'Leary

**Required board:** ESP32 Dev Module (Arduino ESP32 core 3.x)

**Important:** The ISR functions must use `REG_READ(GPIO_IN1_REG)` instead of `digitalRead()` -- see the code for details. Using `digitalRead()` inside `IRAM_ATTR` ISRs causes a boot loop crash.

Flash the code, open Serial Monitor at **115200 baud**, and verify you see:
```
Szuflada V2 starting...
Setup complete!
MQTT connected!
```

### 2. Set up the MQTT Broker

On the Raspberry Pi (192.168.1.1):

```bash
sudo apt install mosquitto mosquitto-clients
sudo systemctl enable mosquitto
sudo systemctl start mosquitto
```

Verify the broker works:
```bash
# Terminal 1: subscribe
mosquitto_sub -t "sensor/#" -v

# Terminal 2: check ESP32 is publishing
# (you should see sensor/distance and sensor/encoder messages)
```

### 3. Build the Docker Image

On the laptop/Chromebook, clone the repo and build:

```bash
git clone https://github.com/geoglof/SzufladaV2.git
cd SzufladaV2

# Build the Docker image
sudo docker build -t orionrover_img .

# Create and start the container
sudo docker run -it -d --privileged --name OrionRover \
  --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  -v /dev/bus/usb:/dev/bus/usb \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  orionrover_img
```

### 4. Add Shell Aliases (optional but recommended)

Add these to your `~/.bashrc`:

```bash
# Szuflada V2 Rover Aliases
alias rd='sudo systemctl start docker && sudo docker start OrionRover'
alias rl='sudo docker exec -it -w /root/rover_ws OrionRover bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch rover_autonomy autonomy_launch.py"'
alias rt='sudo docker exec -it -w /root/rover_ws OrionRover bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run rover_autonomy rover_teleop"'
alias rm2='sudo docker exec -it -w /root/rover_ws OrionRover bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run rover_autonomy odom_monitor"'
alias ro='sudo docker exec -it -w /root/rover_ws OrionRover bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run rover_autonomy obstacle_avoidance"'
alias re='sudo docker exec -it -w /root/rover_ws OrionRover bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 topic echo /encoder_raw"'
alias rsh='sudo docker exec -it -w /root/rover_ws OrionRover bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && bash"'
```

Then reload: `source ~/.bashrc`

## Usage

### After a reboot

```bash
rd          # Start Docker + container
rl          # Launch core stack (Terminal 1)
rt          # Run teleop (Terminal 2)
```

### Teleop Controls

| Key | Action |
|-----|--------|
| W | Drive forward (continuous) |
| S | Drive backward (continuous) |
| A | Turn left (continuous) |
| D | Turn right (continuous) |
| SPACE | Stop |
| +/- | Increase/decrease linear speed |
| ]/[ | Increase/decrease turn speed |
| L | Toggle LED |
| Q | Quit |

### Behavior Nodes (run ONE at a time in a separate terminal)

```bash
# Keyboard teleoperation
rt

# Obstacle avoidance (reverses when ultrasonic < 0.3m)
ro

# Wall following (requires depth camera for /scan)
sudo docker exec -it -w /root/rover_ws OrionRover bash -c \
  "source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
   ros2 run rover_autonomy wall_follower"

# Visual tracking (requires RealSense camera)
sudo docker exec -it -w /root/rover_ws OrionRover bash -c \
  "source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
   ros2 run rover_autonomy visual_tracker"

# Move a set distance (meters) or angle (degrees)
sudo docker exec -it -w /root/rover_ws OrionRover bash -c \
  "source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
   ros2 run rover_autonomy move_distance --ros-args -p distance:=0.5"

sudo docker exec -it -w /root/rover_ws OrionRover bash -c \
  "source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
   ros2 run rover_autonomy move_distance --ros-args -p angle:=90.0"
```

### Monitoring

```bash
# Live odometry monitor (position, heading, distance)
rm2

# Raw encoder ticks
re

# Odometry topic
sudo docker exec -it -w /root/rover_ws OrionRover bash -c \
  "source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
   ros2 topic echo /odom --field pose.pose.position"

# Shell inside container
rsh
```

### Encoder Calibration

To recalibrate ticks per revolution:

```bash
# Mark the wheel, then spin it for N ticks
sudo docker exec -it -w /root/rover_ws OrionRover bash -c \
  "source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
   ros2 run rover_autonomy encoder_calibration --ros-args \
   -p ticks:=500 -p motor:=left -p pwm:=80"
```

Count full revolutions, then: `ticks_per_rev = ticks / revolutions`

Update the values in `rover_autonomy/launch/autonomy_launch.py`.

## Network Layout

```
Laptop (Docker)                Raspberry Pi             ESP32
192.168.1.x         ◄──LAN──► 192.168.1.1    ◄──LAN──► 192.168.1.50
                               (MQTT broker)            (W5500 Ethernet)
```

All devices are on the same 192.168.1.0/24 subnet via an Ethernet switch.

## Rebuilding After Code Changes

```bash
cd ~/rover_ws/src
sudo docker build -t orionrover_img .
sudo docker rm -f OrionRover
sudo docker run -it -d --privileged --name OrionRover \
  --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  -v /dev/bus/usb:/dev/bus/usb \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  orionrover_img
```

## Troubleshooting

| Problem | Fix |
|---------|-----|
| ESP32 boot loop (`entry 0x4008059c`) | ISRs use `digitalRead()` -- must use `REG_READ(GPIO_IN1_REG)` instead |
| No MQTT data in ROS2 | Check ESP32 Serial Monitor for `MQTT connected!`. Check `ping 192.168.1.50` from the laptop |
| Odometry drifts when stationary | Noise filter is active -- encoder EMI is filtered when motors are stopped |
| Rotation angle is wrong | Recalibrate `wheel_base`: do a 360° turn and compute `real_base = current_base * (reported_deg / 360)` |
| Motors don't start at low speed | Increase `min_pwm` in the launch file (default: 60) |
| Stale ROS2 processes | Open container shell (`rsh`) and run `pkill -9 -f python3` |
