The main matlab file is "pursuit_project.m".

The items that can be changed are

1. the target's trajectory, found inside of "target_kinematics.m"
2. the speed of the pursuer, found inside of "funcs.m" (the variable 'k')
3. and the pursuit vector, also found inside of "funcs.m" (specifically the value for 'dt', which allows the pursuer to 'lead' the target).