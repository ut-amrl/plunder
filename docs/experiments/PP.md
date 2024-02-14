<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0-beta3/css/all.min.css">
<link rel="stylesheet" type="text/css" href="../assets/style.css">

# Pick and Place (PP)

This module is the setup for a robotic arm picking up a cube and placing it at the desired location.

Try running the algorithm on the setup (or see **snapshots/** for pre-acquired results).

The most useful/informative outputs will be:
- **out/aspx/**, which stores the synthesized policies. For example, in **out/asp_iter14/asp.txt**, we can see the final policy:
    ```
    MOVE_TO_CUBE -> MOVE_TO_TARGET
    Flip(Logistic(Minus(Minus(Abs(x), Abs(bx)), Plus(z, Minus(Abs(Minus(z, Abs(bx))), Abs(x)))), -0.007982, 555.460938))
    ```

- **plots/accuracy.png** and **plots/likelihoods.png**, which shows the progress of the EM loop across iterations. Here is a (slightly prettified) version for this task:

    ![](snapshots/example_snapshot/plots/accuracy-alt.png)

- **plots/testing/xx-x-graph.png**, which gives a visual representation of the action labels selected by the policy on the testing set. The first number in the file name indicates the iteration. For example:

    Iteration 1:

    ![](snapshots/example_snapshot/plots/1-1-graph.png)

    Iteration 2:

    ![](snapshots/example_snapshot/plots/2-1-graph.png)

    Iteration 8:

    ![](snapshots/example_snapshot/plots/8-1-graph.png)

- **plots/testing/LA-xx-x-graph.png**, which gives a visual representation of the low-level observations predicted by the policy on the testing set. For example:

    ![](snapshots/example_snapshot/plots/LA-8-2-graph.png)

We also show the behavior of the synthesized policy directly in the simulator.

![](snapshots/example_snapshot/plunder.gif)