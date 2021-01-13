This code supplements the CCTA 2021 submission "On the Impact of the Capacity Drop Phenomenon for Freeway Traffic Flow Control".

The controller formulations outlined in the paper can be found in the `sample_controllers/` directory, though the names used for each controller during development are different from those used in the paper.
For reference:
* The Relaxed Approximate MPC (RAMPC) is implemented in `ModelPredictiveController.py`
* The Exact Hysteretic MPC (EHMPC) is implemented in `GurobiCDMPC.py`
* The Heuristic Controller (HC) is implemented in `HardCodedController.py`

Additionally, `three_cell_sim.py` and `eight_cell_sim.py` are the two case studies featured in the paper.
