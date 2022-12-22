# EM Synthesis
This module runs the actual EM Synthesis algorithm, using all the other modules as dependencies.

## File Organization
- **em.cpp** - contains the core EM Synthesis code
- **out/aspx/** - contains the generated program at each iteration
- **out/examples/** - contains the particle filter outputs at each iteration, in csv format
- **out/states/** - contains pure ASP outputs, in csv format (i.e. the high-level actions acquired by running the given ASP at each time step)
- **plots/** - contains plots of trajectories for pf outputs and pure outputs