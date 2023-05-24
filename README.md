# motion_planning

This repo contains all of the necessary code to create motion plans for a highway driving scenario using RRT.

There are four scenarios that were examined, which can be run from the four scripts described below:
- `code/motion_planners/motion_plan_geo.m`: creates a purely geometric motion plan (i.e. no kinodynamic constraints) in a static environment without state uncertainty
- `code/motion_planners/motion_plan_kino.m`: creates a kinodynamically constrained motion plan in a static environment without state uncertainty
- `code/motion_planners/motion_plan_kino_cov.m`: creates a kinodynamically constrained motion plan in a static environment with state uncertainty
- `code/motion_planners/motion_plan_kino_cov_move.m`: creates a kinodynamically constrained motion plan in a dynamic environment with state uncertainty
