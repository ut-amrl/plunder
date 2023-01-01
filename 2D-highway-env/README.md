# 2D Highway Env
This module is the setup for a vehicle performing lane-changing maneuvers. The goal is to travel as far as possible. Higher weight is placed on faster speeds and left lanes.

## File Organization
- **domain.h** - defines the problem domain, including the spaces $H$, $L$, and $O$
- **robot.h** - defines motor model
- **settings.h** - consolidated list of hyperparameters and settings

Note that this setup differs from the 1D-target example in that the demonstrations are provided by the python programs in **python-gen** (try running **highway2d.py**). Demonstrations will be placed into the **sim/** folder automatically. 
As a result, the setup does *not* require filling out *robotSets.h*, a simulation ASP, a physics model, or their corresponding settings.