# State Sharing Demo: Multi-Agent Formation Control

This directory demonstrates the PX4 `state_sharing` module in action with a 3-UAV formation control system.

## How It Works

The demo implements distributed formation control where each UAV:

1. **Starts state sharing** via `StateSharingControl` messages to the PX4 module
2. **Receives neighbor states** through MAVLink `STATE_SHARING_MSG` broadcasts  
3. **Computes formation velocities** using received neighbor positions
4. **Maintains coordinated flight** without centralized control

### State Sharing Architecture

- **PX4 Module**: Broadcasts vehicle state via MAVLink `STATE_SHARING_MSG`
- **MAVLink Transport**: Uses broadcast/multicast for efficient network utilization
- **micro-DDS Bridge**: Forwards MAVLink messages to ROS2 topics (`/px4_N/fmu/out/incoming_state_sharing`)
- **ROS2 Controllers**: Subscribe to neighbor states for formation algorithms

### Key Integration Points

- **State Sharing Lifecycle**: Controllers send `COMMAND_START` to activate broadcasting
- **Neighbor Detection**: Waits for incoming `state_sharing_msg` from other agents
- **Formation Algorithm**: Uses relative positions for distributed velocity control

## Quick Start

1. Create the docker image with the .build.sh (optional, you can pull it from the artifactory and skip this step)
2. Use start.sh to run the container
3. From the `/app/` dir, run `make rebuild`
4. Open QGroundControl in another terminal
5. Run `./run_experiments`

## Expected Output

### Phase 1: State Sharing Initialization
```
[offboard_control] Sent StateSharingControl command=1
[offboard_control] waiting for subscriber on '/px4_1/fmu/in/incoming_state_sharing_control'
```

### Phase 2: Neighbor Discovery  
```
[offboard_control] State sharing active, transitioning to offboard mode
[VelocityController] neighbor 2: distance=5.2, effective_distance=5.2, error=0.2
[VelocityController] neighbor 3: distance=10.1, effective_distance=10.1, error=5.1
```

### Phase 3: Formation Control
```
[VelocityController] Final velocity: [0.12, -0.08, 0.00]
[offboard_control] Publishing velocity setpoint for formation maintenance
```

The UAVs will takeoff, establish state sharing links, and converge to a stable formation using only MAVLink broadcast communication bridged through micro-DDS to ROS2.