# FollowGapMethod 
- Author: Volkan Sezer, Metin Gokasan
- [Paper: A novel obstacle avoidance algorithm: Follow the Gap Method][https://www.sciencedirect.com/science/article/pii/S0921889012000838]
### C++ 11 Environment
### This algorithm was experimented in simulation and realworld (using 2D Lidar)
- [Video][https://robonote.tistory.com/44]
- [Image]


<img src="Log.png" width="40%" height="30%" alt="log"></img>


### for example(main.cpp) 
<pre><code>
//yaw - vehicle heading in reference coordinates
//desired yaw - radian from Vehicle Position to Goal Position 
//distance array - distance data array from lidar
//min, max angle - start angle, end angle on horizontal of view >= 0
//Clearance - blue Point
//Obstacle - red point

//final Distance, returnAngle: green line
//final Dist1, PI1: black line
//final Dist2, PI2: red line

//finalAngle: GapAngle between obstacles
#include "FollowGap.h"
FollowGapMethod followGap;
result = followGap.Solve(yaw, DesiredYaw,Distance array, minimum angle, maximum angle);

double finalX=followGap.finalDistance/1000.0*cosf(followGap.returnAngle*M_PI/180.0);
double finalY=followGap.finalDistance/1000.0*sinf(followGap.returnAngle*M_PI/180.0);
double obs1X=followGap.finalDist1*cosf(followGap.finalPI1*M_PI/180.0);
double obs1Y=followGap.finalDist1*sinf(followGap.finalPI1*M_PI/180.0);
double obs2X=followGap.finalDist2*cosf(followGap.finalPI2*M_PI/180.0);
double obs2Y=followGap.finalDist2*sinf(followGap.finalPI2*M_PI/180.0);

yaw error = followGap.finalAngle - yaw;

//ex result=followGap.Solve(yaw,desiredYaw,inputRanges,40,140); my horizontal of view is 100.
//if(result) control
</code></pre>
