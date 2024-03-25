# 2D Highway Env (Pass Traffic)
This module is the setup for a vehicle performing lane-changing maneuvers. The goal is to travel as far as possible without crashing.

Note that this setup differs from the 1D-target example in that the demonstrations are provided by the python programs in **python-gen** (try running **highway2d.py**). Demonstrations will be placed into that folder automatically. 
As a result, the setup does *not* require filling out *robotSets.h*, a simulation ASP, a physics model, or their corresponding settings.

Try running the algorithm on the setup (or see **snapshots/** for pre-acquired results).

The most useful/informative outputs will be:
- **out/aspx/**, which stores the synthesized policies. For example, we can see the final policy (which we have cleaned up here for presentation):
    ```
   if ha == FASTER and And(flp(lgs(Minus(r_x, l_x), -42.325607, -11.322358)), flp(lgs(Minus(x, f_x), -39.853004, 0.471738))):
       return LANE_LEFT
   if ha == FASTER and And(flp(lgs(Minus(r_x, f_x), 2.268923, 1.691306)), flp(lgs(Minus(x, f_x), -37.500061, 0.498594))):
       return LANE_RIGHT
   if ha == FASTER and flp(lgs(DividedBy(Minus(f_x, x), vx), 0.977072, -31.665253)):
       return SLOWER
   if ha == LANE_LEFT and flp(lgs(DividedBy(Minus(f_x, x), r_vx), 2.090207, 93.243362)):
       return FASTER
   if ha == LANE_LEFT and false:
       return LANE_RIGHT
   if ha == LANE_LEFT and flp(lgs(Minus(l_x, x), 30.264160, -3.164596)):
       return SLOWER
   if ha == LANE_RIGHT and flp(lgs(DividedBy(Minus(f_x, x), vx), 1.538538, 14.793274)):
       return FASTER
   if ha == LANE_RIGHT and false:
       return LANE_LEFT
   if ha == LANE_RIGHT and flp(lgs(DividedBy(Minus(x, r_x), vx), -0.984751, 182.378754)):
       return SLOWER
   if ha == SLOWER and flp(lgs(DividedBy(Minus(f_x, x), vx), 1.495547, 73.774384)):
       return FASTER
   if ha == SLOWER and flp(lgs(Minus(l_x, f_x), 5.026824, 4.910437)):
       return LANE_LEFT
   if ha == SLOWER and flp(lgs(Minus(r_x, f_x), 1.479489, 1.003578)):
       return LANE_RIGHT
   return ha
    ```

- **plots/accuracy.png** and **plots/likelihoods.png**, which shows the progress of the EM loop across iterations. Here is a (slightly prettified) version for this task:

    ![](../assets/PT_plots/accuracy-alt.png)

- **plots/testing/xx-x-graph.png**, which gives a visual representation of the action labels selected by the policy on the testing set. The first number in the file name indicates the iteration. For example:

    Iteration 1:

    ![](../assets/PT_plots/1-0-graph.png)

    Iteration 2:

    ![](../assets/PT_plots/2-0-graph.png)

    Iteration 14:

    ![](../assets/PT_plots/14-0-graph.png)
    
- **plots/testing/LA-xx-x-graph.png**, which gives a visual representation of the low-level observations predicted by the policy on the testing set. For example, here is iteration 14:

    ![](../assets/PT_plots/LA-14-0-graph.png)

We also show the behavior of the synthesized policy directly in the simulator.

Iteration 1:

![](../assets/PT_plots/asp_1.gif)

Iteration 3:

![](../assets/PT_plots/asp_3.gif)

Iteration 8:

![](../assets/PT_plots/asp_8.gif)

We provide the observation model below:
```
step(action):
    target_acc = 0
    target_heading = 0

    if (action == FASTER)
        target_acc = 40 - v
        target_heading = atan((round(y) - y) / 30)
    else if (action == SLOWER)
        target_acc = v_front  - v
        target_heading = atan((round(y) - y) / 30)
    else if (action == LANE_LEFT)
        target_acc = 30 - v
        target_heading = -0.15
    else if (action == LANE_RIGHT)
        target_acc = 30 - v
        target_heading = 0.15
    
    if (target_heading - heading > steer)
        steer = min(steer + 0.04, target_heading - heading)
    else 
        steer = max(steer - 0.04, target_heading - heading)

    if (target_acc > acc)
        acc = min(target_acc, acc + 4)
    else
        acc = max(target_acc, acc - 6)
``` 