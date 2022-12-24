# Acc Simulation
This module is used to generate **demonstrations** - trajectories with the high-level actions missing.

## Interface:
- `gen_trajectories()` generates and returns a set of trajectories
- `print_traj()` prints the trajectory (or set of trajectories) to the console
- `write_traj()` saves the trajectory to the target file
- `execute_pure()` executes a given ASP on a demonstration to acquire the sequence of high-level actions

## File Organization
- **generate.x** - creates simulated demonstrations and contains the above interface
- **robotSets.h** - contains various sets of initial robot parameters for easy manipulation
- **out/** - contains the generated data, in csv format