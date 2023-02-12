# Particle Filter
This module is used to estimate the sequence of high-level actions corresponding to a demonstration, by use of a *particle filter*.

Inputs:
- **state_traj** - the observed state sequence, or demonstration
- **asp_pf** - the desired ASP to use
- **init_pf** - a desired initial high-level action distribution. Defaults to the user-provided default label.
- **obs_likelihood_pf** - the desired function to convert low-level actions to their log-likelihood. Defaults to a normal distribution around the motor model output. Editable in *system.h* in the main project folder.

Outputs:
- set of high-level action sequences, an estimate for the high-level action distribution at each time step

## Interface:
- `forwardFilter()` runs the particle filter
- `retrieveTrajectories()` traces each particle and returns the set of high-level action sequences

## File Organization
- **pf.h** - contains the core particle filter code
- **pf_runner.x** - wrapper functions to apply settings, write and read from files
- **plotter.py** - general-use python program to plot trajectories