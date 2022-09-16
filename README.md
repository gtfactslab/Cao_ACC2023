<h1>Cao_ACC2023</h1>

This code supplements the ACC submission "Safe Learning-based Predictive Control from Efficient Reachability" by Michael E. Cao and Samuel Coogan.

<h2>Abstract</h2>
We consider a dynamical system subject to a disturbance input that is an unknown function of the state. Given a target goal region, we propose a control scheme that encourages exploration of the state space in order to sample the dynamics and obtain an estimate of the unknown component while avoiding unsafe regions of the state space until the goal is able to be reached with high probability. By estimating the unknown component as a Gaussian process, we efficiently obtain hyperrectangular overapproximations of the reachable set for the system using the theory of mixed monotone systems, and these sets are improved over time as measurements of the dynamics are collected. Using these reachability estimates, we propose a model predictive scheme that avoids the unsafe region and ensures the system is always within reach of a conservative, guaranteed safe region that is given a priori, thus always ensuring feasibility until the goal is reachable. We demonstrate the approach on a model of an autonomous vehicle operating on an icy road and on a planar multirotor moving in an unknown wind field.

<h2>Notes on Repository</h2>
The scripts contained within this repository can be used to reproduce the results from the paper.

For reference:

* The Autonomous Vehicle case study can be produced by running `av_mpc_example_icystate.m`
* The Planar Multirotor case study can be produced by running `sixquad_mpc_example.m`

For each case study, be sure to have the correct parameters set in `fit_params.m`
