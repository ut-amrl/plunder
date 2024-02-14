# Stack (ST)

This module is the setup for a robotic arm stacking two cubes at the desired location, in the correct order.

Try running the algorithm on the setup (or see **snapshots/** for pre-acquired results).

The most useful/informative outputs will be:
- **out/aspx/**, which stores the synthesized policies. For example, in **out/asp_iter6/asp.txt**, we can see the final policy:
    ```
    GRASP -> MOVE_TO_TARGET
    Flip(Logistic(tz2, -0.097622, -94.221039))
    LIFT -> MOVE_TO_CUBE_TOP
    Flip(Logistic(bz1, -0.142848, -96.511452))
    MOVE_TO_CUBE_BOTTOM -> MOVE_TO_TARGET
    Flip(Logistic(z, 0.024995, -43748.042969))
    ...
    ```

- **plots/accuracy.png** and **plots/likelihoods.png**, which shows the progress of the EM loop across iterations. Here is a (slightly prettified) version for this task:

    ![](snapshots/example_snapshot/plots/accuracy-alt.png)

- **plots/testing/xx-x-graph.png**, which gives a visual representation of the action labels selected by the policy on the testing set. The first number in the file name indicates the iteration. For example:

    Iteration 1:

    ![](snapshots/example_snapshot/plots/1-18-graph.png)

    Iteration 2:

    ![](snapshots/example_snapshot/plots/2-18-graph.png)

    Iteration 7:

    ![](snapshots/example_snapshot/plots/7-18-graph.png)
- **plots/testing/LA-xx-x-graph.png**, which gives a visual representation of the low-level observations predicted by the policy on the testing set. For example:

    ![](snapshots/example_snapshot/plots/LA-7-18-graph.png)

We also show the behavior of the synthesized policy directly in the simulator.

![](snapshots/example_snapshot/plunder.gif)