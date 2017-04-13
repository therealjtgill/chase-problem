# chase-problem
Numerical solution/analytical approximation to the classic chase problem

The main matlab file is "pursuit_project.m".

The items that can be changed are

1. the target's trajectory, found inside of "target_kinematics.m"
2. the speed of the pursuer, found inside of "funcs.m" (the variable 'k')
3. and the pursuit vector, also found inside of "funcs.m" (specifically the value for 'dt', which allows the pursuer to 'lead' the target).

The classic chase problem is posed as: given a target moving with an arbitrary trajectory, design a pursuer that follows the target with the intent of 'catching' the target. The Matlab files supplied here numerically evaluate different versions of the pursuer's possible kinematics (pursuit vector and speed) given a target with a particular trajectory.

The paper elucidates an approximation of the chase problem, where it is assumed that the pursuer is extremely close to the target.
