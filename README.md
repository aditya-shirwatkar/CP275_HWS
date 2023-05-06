# CP275_HWS
This course provides an end-to-end overview of different topics involved in designing or analyzing autonomous systems. It begins with different formal modeling frameworks used for autonomous systems including state-space representations (difference equations), hybrid automata, and in general labeled transition systems. It also discusses different ways of formally modeling properties of interest for such systems such as stability, invariance, reachability, and temporal logic properties. 

After this, the course covers different techniques on the verification of such systems including Lyapunov functions, reachability, barrier certificates, and potentially model checking. Finally, the course will introduce students to several techniques for designing controllers enforcing properties of interest over autonomous systems.

---

## Final Term Paper

- Rodionova, Alena, Lars Lindemann, Manfred Morari and George J. Pappas. “Time-Robust Control for STL Specifications.” 2021 60th IEEE Conference on Decision and Control (CDC) (2021): 572-579.

The code can be found in [time-robust-control](/time-robust-control)

### Extentions (Our Work)

**Problems with current literature**
- Control inputs are abrupt, leading to highly jerky state trajectories
  - [x] Solution: Perform L2 regularization of inputs
- Computational time is very high
  - [ ] Solution: Might not be possible to reduce solving time due to the nature of optimization problem (Mixed Integer Quadratic Program)

**Results after implementing above changes for a Multi Agent 3D UAV system**
- N_states = 24
- N_control = 8
- Horizon = 50
- High Level Specification: Reach goal region always within the last 10 timesteps

|     | Time Robust Control | Convex MPC |
| :-: | :-----------------: | :--------: |
|     | <img src="./time%20robust%20control/multi_3D_uav_time_rob_qp.gif" width=400> | <img src="./time%20robust%20control/multi_3D_uav_qp.gif" width=400> |
| Solver Time (secs) | 44 | 0.01 |
| Time Robustness (`dt` units) | 1 | Does not exist |
| Trajectory | Little Jerky | Smooth |
