# 1D Target (Stop Sign)
This module is the setup for a simple vehicle driving in a straight line. The goal is to stop before some target stop sign.

## File Organization
- **domain.h** - defines the problem domain, including the spaces $H$, $L$, and $S$
- **robot.h** - defines world models, including the simulation ASP, motor model, and physics model
- **settings.h** - consolidated list of hyperparameters and settings
- **emdips_operations.json** - list of desired operations to be used during program synthesis. See *pips/* for further clarification.
- **robotSets.h** - the set of robots to use when simulating trajectories

Try running the algorithm on the setup (or see **snapshots/** for pre-acquired results). The following outputs will be generated:
- **sim/** - contains the simulated trajectories for each robot
- **out/aspx/** - contains the synthesized program at each iteration
- **out/examples/** - contains the particle filter outputs at each iteration, in csv format
- **out/states/** - contains pure ASP outputs, in csv format (i.e. the high-level actions acquired by running the given ASP at each time step)
- **plots/** - contains plots of trajectories for pretty viewing

The most useful outputs will be in **out/aspx/** and **plots/**.