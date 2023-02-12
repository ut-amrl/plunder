# 2D Merge
This module is the setup for a vehicle dealing with lane-merges. The goal is to make room for a vehicle merging onto the highway without crashing into other vehicles.

## File Organization
- **domain.h** - defines the problem domain, including the spaces $H$, $L$, and $O$
- **robot.h** - defines motor model
- **emdips_operations.json** - list of desired operations to be used during program synthesis. See *pips/* for further clarification.
- **settings.h** - consolidated list of hyperparameters and settings

Note that this setup differs from the 1D-target example in that the demonstrations are provided by the python programs in **python-gen** (try running **merge.py**). Demonstrations will be placed into that folder automatically. 
As a result, the setup does *not* require filling out *robotSets.h*, a simulation ASP, a physics model, or their corresponding settings.
We have provided the simulation ASP for convenience, and to generate the control plots, but it is not required.