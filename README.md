# Ball Balance Control using SAC Agent

A reinforcement learning project that trains a Soft Actor-Critic (SAC) agent to balance a ball on a robotic manipulator plate using MATLAB and Simulink.

## Overview

This project implements an RL-based control system for the challenging task of balancing a ball on a plate attached to a Kinova robotic arm. The agent learns to manipulate the plate's orientation by controlling the robot's joint angles to keep the ball at a target position despite dynamic disturbances.

## Project Structure

```
├── TrainSACAgentForBallBalanceControlExample/
│   ├── TrainSACAgentForBallBalanceControlExample.mlx  # Main training script
│   ├── rlKinovaBallBalance.slx                        # Simulink environment model
│   ├── Agent1.mat, Agent2.mat, Agent3.mat             # Trained agent checkpoints
│   ├── kinova_params.m                                # Robot and ball parameters
│   ├── animatedPath.m                                 # Visualization function
│   ├── calcMOI.m                                      # Moment of inertia calculation
│   ├── cor2SpringDamperParams.m                       # Contact dynamics parameters
│   └── logs/                                          # Training episode logs
├── pp/                                                 # Additional development files
└── Train SAC Agent for Ball Balance Report.pdf        # Project documentation
```

## Key Features

- **Reinforcement Learning Algorithm**: Soft Actor-Critic (SAC) with continuous action space
- **Physics Simulation**: High-fidelity Simscape Multibody model of Kinova arm and ball dynamics
- **State Representation**: 21-dimensional observation space including:
  - Ball position and velocity (x, y, z)
  - Plate orientation and angular velocity
  - Joint positions and velocities
  - Physical parameters (ball radius, mass, MOI)
- **Control**: 7-DOF joint angle control for the Kinova manipulator

## Environment Setup

### Ball Parameters

- Radius: 0.02 m
- Mass: 0.0027 kg
- Shell thickness: 0.0002 m
- Contact friction coefficient: 0.5

### Robot Configuration

- Platform: Kinova 7-DOF robotic arm
- End-effector: Flat plate for ball balancing
- Control frequency: Real-time simulation

## Training

The SAC agent is trained using experience replay and twin critic networks to learn a robust policy for ball balancing. Training logs are saved in the `logs/` directory, allowing for analysis of learning progress and agent performance over episodes.

### Trained Agents

- `Agent1.mat`: Early training checkpoint
- `Agent2.mat`: Intermediate training checkpoint
- `Agent3.mat`: Final trained agent

## Usage

1. Open MATLAB and navigate to the project directory
2. Run `kinova_params.m` to load environment parameters
3. Open `TrainSACAgentForBallBalanceControlExample.mlx` to:
   - Train a new agent from scratch
   - Continue training from a checkpoint
   - Evaluate trained agent performance
4. Use `animatedPath()` to visualize ball trajectory on the plate

## Visualization

The `animatedPath.m` function provides real-time animation of:

- Ball position on the plate (X-Y coordinates)
- Trajectory tracking over time
- Distance from target position

## Requirements

- MATLAB R2023a or later
- Reinforcement Learning Toolbox
- Simscape Multibody
- Control System Toolbox

## Results

The trained agent successfully learns to balance the ball near the target position, demonstrating:

- Stable control under varying initial conditions
- Adaptive response to disturbances
- Smooth joint trajectories

For detailed results and analysis, refer to `Train SAC Agent for Ball Balance Report.pdf`.

## License

Copyright 2021-2022, The MathWorks, Inc.
