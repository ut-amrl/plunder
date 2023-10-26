# Particle Filter
This module is used to estimate the sequence of action labels corresponding to a demonstration, by use of a *particle filter*.

Inputs:
- **state_traj** - the observed state sequence, or demonstration
- **asp_pf** - the desired ASP to use
- **init_pf** - a desired initial action label distribution. Defaults to the user-provided default (first) label.
- **obs_likelihood_pf** - the observation model. Defaults to a normal distribution around a user-provided deterministic model. Can be changed in *system.h* in the main project folder.

Outputs:
- set of action label sequences, an estimate for the action label distribution at each time step

## Interface:
- `forwardFilter()` runs the particle filter
- `retrieveTrajectories()` traces each particle and returns the set of action label sequences

## File Organization
- **pf.h** - contains the core particle filter code
- **pf_runner.x** - wrapper functions to apply settings, write and read from files
- **plotter.py** - general-use python program to plot trajectories