# FollowGapMethod Author: Volkan Sezer, Metin Gokasan
[Paper: A novel obstacle avoidance algorithm: Follow the Gap Method][https://www.sciencedirect.com/science/article/pii/S0921889012000838]
### C++ 11 Environment
### This algorithm was experimented in simulation and realworld (using 2D Lidar)
- - -
### for example(main.cpp) 
<pre><code>
#include "FollowGap.h"
FollowGapMethod followGap 
result = followGap.Solve(yaw, DesiredYaw(from Vehicle Position to Goal Position), Distance array(2D lidar data), minimum angle(lidar coordinates), maximum angle(lidar coordinates))
</code></pre>

- ex result=followGap.Solve(yaw,desiredYaw,inputRanges,40,140); my horizontal of view is 100.
- if(result) control

