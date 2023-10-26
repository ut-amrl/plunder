# Simulation
This module is used to generate **demonstrations** - trajectories with the action labels missing.

## Interface:
- `gen_trajectories()` generates and returns a set of trajectories
- `print_traj()` prints the trajectory (or set of trajectories) to the console
- `write_traj()` saves the trajectory to the target file
- `execute_pure()` executes a given ASP on a demonstration to acquire the sequence of action labels

## File Organization
- **generate.x** - creates simulated demonstrations and contains the above interface