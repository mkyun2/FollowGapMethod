#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
using namespace std;


class FollowGapMethod
{
    public:
    FollowGapMethod();
    ~FollowGapMethod();
    double returnAngle = 0.0;
    double finalAngle = 0.0;
    double finalPI1=0.0;
    double finalPI2=0.0;
    double finalDist1=0.0;
    double finalDist2=0.0;
    int finalDistance = 0;
    int finalBetween = 0;
    bool Solve(double HeadingAngle,double& GoalAngle,int* ranges,int minAngle,int maxAngle);



};