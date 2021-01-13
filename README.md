<h1>Cao_CCTA2021</h1>

This code supplements the CCTA 2021 submission "On the Impact of the Capacity Drop Phenomenon for Freeway Traffic Flow Control" by Michael E. Cao, Gustav Nilsson, and Samuel Coogan.

<h2>Abstract</h2>
Capacity drop is an empirically observed phenomenon in vehicular traffic flow on freeways whereby, after a critical density is reached, a state of congestion sets in, but the freeway does not become decongested again until the density drops well below the critical density. This introduces a hysteresis effect so that it is easier to enter the congested state than to leave it. However, many existing first-order models of traffic flow, particularly those used for control design, ignore capacity drop, leading to suboptimal controllers. In this paper, we consider a cell transmission model of traffic flow that incorporates capacity drop to study the problem of optimal freeway ramp metering. We show that, if capacity drop is ignored in the control design, then the resulting controller, obtained via a convex program, may be significantly suboptimal. We then propose an alternative model predictive controller that accounts for capacity drop via a mixed integer linear program and show that, for sufficiently large rollout horizon, this controller is optimal. We also compare these approaches to a heuristic hand-crafted controller that is viewed as a modification of an integral feedback controller to account for capacity drop. This heuristic controller outperforms the controller that ignores capacity drop but underperforms compared to the proposed alternative model predictive controller. These results suggest that it is generally important to include capacity drop in the controller design process, and we demonstrate this insight on several case studies. 

<h2>Notes on Repository</h2>
The controller formulations outlined in the paper can be found in the `sample_controllers/` directory, though the names used for each controller during development are different from those used in the paper.
For reference:
* The Relaxed Approximate MPC (RAMPC) is implemented in `ModelPredictiveController.py`
* The Exact Hysteretic MPC (EHMPC) is implemented in `GurobiCDMPC.py`
* The Heuristic Controller (HC) is implemented in `HardCodedController.py`

Additionally, `three_cell_sim.py` and `eight_cell_sim.py` are the two case studies featured in the paper.

Note that the code requires Gurobi installed on the local machine in order to run, which is not provided in this repo.
Free academic licenses are usually accessible through the Gurobi website, though the authors cannot guarantee availability.

