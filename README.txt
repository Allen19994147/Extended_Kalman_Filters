

5-1 EKF with known correspondances
MYekf_HW.m			//where I implement EKF implementation.(It replaces ekf_HW.m)

IMPORTANT : The first run of MYekf_HW.m may be chaotic sometimes,
and you may run it multiple times to check the results.

5-2 KEF with unknown correspondances
EKF_unknown.m			//EKF localization of unknown correspondence

%%% Auxiliary sub-functions:
VehicleMotionEstimation.m:	//estimation of poses based on pure model dynamics
SpecialModify.m		//Calcualte the difference of robot poses in a special manner
ThetaRegulator.m		//Confine theta between +-pi

%%% Note %%%
For visualization surposes:
"red" represents acutal pose of robot and its eclipse
"blue" represents pure estimated pose without measurement involved
"green" represents estimated pose updated by EKF


%%% Some details (work for both 5-1,5-2) %%%
1.To better exaime the EKF localization, you may increase targets by simply decomment
from line 26 ~ 29. 

2.To enlarge the map, I extend the scale of the map by a factor of 2 on each sides.
If you want to have a original map, decomment line 34.

3.To avoid "while loop" in line 57, I replace it with a for loop.
If you want the while loop back, decomment it and comment line 58.