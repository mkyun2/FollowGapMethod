#include "FollowGap.h"


FollowGapMethod::FollowGapMethod(){}
FollowGapMethod::~FollowGapMethod(){}

double PI1,PI2;
int D1,D2,gapDistance;
int  gapBetween;
bool FollowGapMethod::Solve(double HeadingAngle, double& GoalAngle,int* ranges,int minAngle,int maxAngle)
{
//Initialiization
    FollowGapMethod::finalAngle = 0;
    FollowGapMethod::finalDistance = 0;
    FollowGapMethod::finalBetween = 0;
    FollowGapMethod::finalPI1 = 0;
    FollowGapMethod::finalPI2 = 0;
    FollowGapMethod::finalDist1 = 0;
    FollowGapMethod::finalDist2 = 0;
    int num_readings = 180;
    const int Max_distance=15000;
    const int smallestDistanceSaturation=1500;
    float Final_Angle=0;

//  0 : no obstacle
//  1 :    obstacle
// 00 : no obstacle area
// 10 : Starting point
// 01 : Ending point
// 11 : obstacle area
    vector<int> Clearance_StartAngle;
    vector<int> Clearance_EndAngle;
    vector<int> Obstacle_StartAngle;
    vector<int> Obstacle_EndAngle;
    uint8_t Obstacle_flag = 0;
    uint8_t Obstacle_flag_pre = 0;
    uint8_t Num_of_Obstacle=0;
    int Obstacle_distance_num=0;
    int Obstacle_distance[num_readings]={0};

    uint8_t Clearance_flag=0;
    uint8_t Clearance_flag_pre=0;
    uint8_t Num_of_Clearance=0;

    int Obstacle_point_start[180]  = {0};	
    int Obstacle_point_end[180]    = {0};
    int Num_of_obstacle_point[180] = {0};
    double Distance_of_Obstacle[180]  = {0};

    // flag를 이용해 Obstacles와 clearances를 구분하는 function.
    for(int i = minAngle; i<maxAngle; i++){
        Distance_of_Obstacle[i]=Max_distance;

        if((ranges[i])<1000)
            ranges[i]=Max_distance;
        else if((ranges[i])>(Max_distance))
            ranges[i]=Max_distance;
        else
            ranges[i]=ranges[i];

    }
    // for(int i=minAngle; i<maxAngle; i++)
    // {
    //     //(yaw+180)-(minAngle+maxAngle)/2+(i-minAngle) ~ (yaw+180)+(minAngle+maxAngle)/2+(i-minAngle)  
    //     //ranges[i]=ranges[i]-int(*ranges+i)%10;
        
    //     //////printf("%f",ranges[i]);
    //     if(ranges[i] < Max_distance)
    //     {
    //         Obstacle_flag = 1;
    //         Clearance_flag = 0;

    //         //Obstacle_distance[Obstacle_distance_num]=ranges[i];
    //         //Obstacle_distance_num++;// Obstacle 개수 
    //     }
    //     else
    //     {
    //         Obstacle_flag = 0;
    //         Clearance_flag = 1;
    //     }
        
    //     //======================Obstacle=============================
    //     if(Obstacle_flag==1 && Obstacle_flag_pre==0)
    //     {
    //         Num_of_Obstacle++;
    //     }
        
    //     Obstacle_flag_pre=Obstacle_flag;
    //     //======================Clearance=============================
    //     //첫 시작이 clearance라면 
    //     if(Clearance_flag == 1 && Clearance_flag_pre == 0)
    //         Num_of_Clearance++;
    //     Clearance_flag_pre=Clearance_flag;
        
    // }
    Obstacle_flag=0;
    Obstacle_flag_pre=0;
    Clearance_flag=0;
    Clearance_flag_pre=0;
    //========================Obstacle,Clearance info =============================
    for(int i=minAngle; i<maxAngle; i++)
    {
        
        
        if(ranges[i] < Max_distance)
        {
            Obstacle_flag = 1;
            Clearance_flag = 0;
        }
        else if(ranges[i] == Max_distance)
        {
            Obstacle_flag = 0;
            Clearance_flag = 1;
        }
        if((Obstacle_flag == 1) && (Obstacle_flag_pre == 0))
        {
            //printf("Obstacle Start, %d\n", i);
            // ////printf("Obstacle StartPoint: %d",i);
            if(i<maxAngle-1){
            Obstacle_StartAngle.push_back(i);
            //Obstacle_point_start[i] = i; // start point pos
            
            }
            Distance_of_Obstacle[i]=ranges[i]; // distance to start point
            Num_of_obstacle_point[i] = Num_of_obstacle_point[i] + 1; //

        }
        else if((Obstacle_flag == 1) && (Obstacle_flag_pre == 1))
        {
            // if(ranges[i]<500)
            // ////printf("Obstacle %dth DISTANCE: %d",i,ranges[i]);
            
            Num_of_obstacle_point[i] = Num_of_obstacle_point[i] + 1; // Obstacle을 구성하는 Points의 개수를 counting
            Distance_of_Obstacle[i] = ranges[i];
            if(i > maxAngle-2)
            {
                Obstacle_EndAngle.push_back(i);
            }
            
        }
        else if((Obstacle_flag == 0) && (Obstacle_flag_pre == 1))
        {
            Obstacle_EndAngle.push_back(i);
            Distance_of_Obstacle[i] = ranges[i];///////////////////////////////////////////////////////
        }
        // ////printf("Distance Obstacle, %d, %d, %f",Distance_of_Obstacle[i],i,ranges[i]);

        if((Clearance_flag == 1) && (Clearance_flag_pre == 0))
        {
            
            //printf("Clearance Start, %d\n", i);
            // ////printf("Clear startPoint: %d",i);
            //Clearance_point_start[i] = i;
            
            if(i<maxAngle-1){
            Clearance_StartAngle.push_back(i);
            }
        }
        else if((Clearance_flag == 1) && (Clearance_flag_pre == 1))
        {
            //printf("Clearance ing\n");
        //	Obstacle_Area[Num_of_obstacle][Num_of_obstacle_point[Num_of_obstacle]] = ds_data.ranges[i];
        //	Obstacle_Area[1][Num_of_obstacle][Num_of_obstacle_point[Num_of_obstacle]] = i;
            if(i > maxAngle-2)
            {
                // ////printf("Clearance End1, %d, Ranges:%f", i,ranges[i]);
                //printf("Clear EndPoint: %d",i);
                Clearance_EndAngle.push_back(i);
                //Clearance_point_end[i] = i;// * 0.25 - 60;	// Obstacle Ending Point
                

            }
        //	Num_of_obstacle_point[Num_of_obstacle] = Num_of_obstacle_point[Num_of_obstacle] + 1;
        }
        else if((Clearance_flag == 0) && (Clearance_flag_pre == 1))
        {
            // ////printf("Clearance End2, %d", i);
            // //Clearance_point_end[i] = i;
            //printf("Clear EndPoint: %d \n",i);
            Clearance_EndAngle.push_back(i);

        }

        Obstacle_flag_pre = Obstacle_flag;
        Clearance_flag_pre = Clearance_flag;
    //Known Obstacle Num, Angle, distance, points
    // ////printf("Clear: %d",Num_of_Clearance);
        
    }

    //start num --> Vector(angle num)
    //Angle of obstacle and cleanrance
    //Clearance num + Obstacle num
    vector<double> MaxPI1;
    vector<double> MaxPI2;
    vector<double> MaxDist1;
    vector<double> MaxDist2;
    vector<int> GapDistance;
    vector<int> GapBetween;  
    vector<double> GapAngle;
    vector<double> FinalAngle;
    vector<double> MinDistance;

    double Now_FinalAngle=0.0;
    double Max_FinalAngle=0.0;
    double Now_GapAngle=0.0;
    double Max_GapAngle = 0.0;
    int Max_GapAngle_Num=0;
    double Min_Distance=20000.0;
    double Now_Distance=0;
    float Gain_GapAngle = 8;
    float Gain_GoalAngle= 1;

    HeadingAngle = (HeadingAngle*180.0/M_PI);

    if(Clearance_StartAngle.size()>0)
    {
        for(int i=0; i<Clearance_StartAngle.size(); i++)
        {  
            ////printf("Distance Error Check\n");
            ////printf("Original Start Angle: %d, End Angle: %d",Clearance_StartAngle[i],Clearance_EndAngle[i]);
            ////printf("Original Start Angle: %d\n",Clearance_StartAngle[i]);
            ////printf("End Angle: %d\n",Clearance_EndAngle[i]);
            ////printf("Start Angle: %f, End Angle: %f",Clearance_StartAngle[i]-60.0+(yaw*180.0/M_PI),Clearance_EndAngle[i]-60.0+(yaw*180.0/M_PI));
            ////printf("CHecking Clearance_EndDistance: %f, StartDistance: %f \n",Distance_of_Obstacle[Clearance_EndAngle[i]],Distance_of_Obstacle[Clearance_StartAngle[i]]);
                
            if(Distance_of_Obstacle[Clearance_EndAngle[i]] < Distance_of_Obstacle[Clearance_StartAngle[i]])
            {
                // if(Clearance_EndAngle[i]==num_readings-1)
                //         Min_Distance = ranges2[Clearance_EndAngle[i]];
                // else
                MinDistance.push_back(Distance_of_Obstacle[Clearance_EndAngle[i]+1]);
                //Min_Distance = Distance_of_Obstacle[Clearance_EndAngle[i]];
                //////printf("CHecking2 Clearance_EndAngle: %d, StartAngle: %d",Clearance_EndAngle[i],Clearance_StartAngle[i]);
                //////printf("CHecking2 Clearance_EndDistance: %d, StartDistance: %d",Distance_of_Obstacle[Clearance_EndAngle[i]],Distance_of_Obstacle[Clearance_StartAngle[i]]);
            
            }
            else
            {
                // if(Clearance_StartAngle[i]==0)
                //     Min_Distance = ranges2[Clearance_StartAngle[i]]; // how do robot when approach to zero 
                // else
                MinDistance.push_back(Distance_of_Obstacle[Clearance_StartAngle[i]]);
                //Min_Distance = Distance_of_Obstacle[Clearance_StartAngle[i]]; // how do robot when approach to zero 
                //////printf("CHecking1 Clearance_EndAngle: %d, StartAngle: %d",Clearance_EndAngle[i],Clearance_StartAngle[i]);
                //////printf("CHecking1 Clearance_EndDistance: %d, StartDistance: %d",Distance_of_Obstacle[Clearance_EndAngle[i]],Distance_of_Obstacle[Clearance_StartAngle[i]]);
            
            }
            
            PI1 = abs((float)Clearance_StartAngle[i])/180.0*M_PI;//-((maxAngle-minAngle)/2+minAngle))/180.0*M_PI;
            PI2 = abs((float)Clearance_EndAngle[i])/180.0*M_PI;//-((maxAngle-minAngle)/2+minAngle))/180.0*M_PI;
            
            //printf("HtoC_Start: %f HtoC_End: %f \n",PI1*180.0/M_PI,PI2*180.0/M_PI);
            if(Clearance_StartAngle[i]!=minAngle)
            D1 = Distance_of_Obstacle[Clearance_StartAngle[i]-1]; // Obstacle Distance
            else
            D1 = Distance_of_Obstacle[Clearance_StartAngle[i]];
            if(Clearance_EndAngle[i]!=maxAngle)
            D2 = Distance_of_Obstacle[Clearance_EndAngle[i]];
            else
            D2 = Distance_of_Obstacle[Clearance_EndAngle[i]];
            
            gapDistance = sqrt(D1*D1+D2*D2+2*D1*D2*cosf(PI2-PI1))/2;
            
            
            gapBetween = sqrt(D1*D1+D2*D2-2*D1*D2*cosf((PI2-PI1)));
            
            //Gap_CenterAngle
            //Now_GapAngle = (((Clearance_EndAngle[i]) - (Clearance_StartAngle[i]))/2);//+Clearance_StartAngle[i];
            
            Now_GapAngle = acos((D1+D2*cosf((PI2-PI1)))/sqrt(D1*D1+D2*D2+2*D1*D2*cosf((PI2-PI1))))*180.0/M_PI;//+minAngle-(maxAngle+minAngle)/2;//-((PI1)))*180.0/M_PI;
            //printf("GapBw: %d Gap Angle: %f GapDistance: %d Start_Distance: %d , End_Distance: %d \n",gapBetween,Now_GapAngle,gapDistance,D1,D2);
            //printf("StartANgle %d, EndAngle %d \n",Clearance_StartAngle[i],Clearance_EndAngle[i]);
            MaxPI1.push_back(PI1);
            MaxPI2.push_back(PI2);
            MaxDist1.push_back((double)D1/1000.0);
            MaxDist2.push_back((double)D2/1000.0);
            GapAngle.push_back(Now_GapAngle);
            GapDistance.push_back(gapDistance);
            GapBetween.push_back(gapBetween);

            // FinalAngle.push_back(Now_FinalAngle);
            if(abs(Max_GapAngle)<abs(Now_GapAngle))
            {
                Max_GapAngle = Now_GapAngle;
                
            }
            //////printf("Error Check2\n");
            
        }
        //printf("MaxGap %f, NowGap %f GapAngle size %d \n",Max_GapAngle,Now_GapAngle,GapAngle.size());
        if(std::find(GapAngle.begin(),GapAngle.end(), Max_GapAngle) != GapAngle.end())
        {
            Max_GapAngle_Num = std::find(GapAngle.begin(), GapAngle.end(), Max_GapAngle) - GapAngle.begin();
            //printf("GapAngle: %f Max_NUM:%d \n",GapAngle[Max_GapAngle_Num],Max_GapAngle_Num);
        }
        ////printf("Error Check3\n");

        if(Obstacle_StartAngle.size()>0)
        {
            for(int i=0; i<Obstacle_StartAngle.size(); i++)
            {
                ////printf("Error Check4 size: %d",Obstacle_StartAngle.size());
                for(int j=Obstacle_StartAngle[i]+1; j<Obstacle_EndAngle[i]; j++)
                {
                    //////printf("Error Check5 : %f", );
                    ////printf("Error Check7 angle: %d, obs_dist:%f \n",j,Distance_of_Obstacle[j]);
                    ////printf("Error Check7 angle: %d, min_dist:%f \n",j,Min_Distance);
                    if(Min_Distance>Distance_of_Obstacle[j] )
                    {
                        ////printf("Error Check6\n");
                    Min_Distance=Distance_of_Obstacle[j];
                    
                    }
                }
            }
        }
        //Min_Distance = MinDistance[Max_GapAngle_Num];
        if(Min_Distance<20000.0)
            Min_Distance=Min_Distance;//-1000.0;
        Min_Distance=(Min_Distance)/1000.0;
        
        ///////////////////////////////////
        //Max_GapAngle: location of Angle
        ///////////////////////////////////

        Max_GapAngle = ((float)GapAngle[Max_GapAngle_Num]+Clearance_StartAngle[Max_GapAngle_Num]-90+HeadingAngle)*M_PI/180.0;
        FollowGapMethod::returnAngle = (float)GapAngle[Max_GapAngle_Num]+Clearance_StartAngle[Max_GapAngle_Num];
        if(Max_GapAngle>M_PI)
            Max_GapAngle = Max_GapAngle-2*M_PI;
        else if(Max_GapAngle<-M_PI)
            Max_GapAngle = Max_GapAngle +2*M_PI;
        //printf("2 HeadingAngle: %f MaxGap: %f DesiredYaw: %f \n",HeadingAngle,Max_GapAngle,GoalAngle*180.0/M_PI);
        //printf("3 Distance: %d",GapDistance[Max_GapAngle_Num]);

        Final_Angle = ((Gain_GapAngle/Min_Distance*Max_GapAngle)+(Gain_GoalAngle*GoalAngle)) / ((Gain_GapAngle/Min_Distance)+Gain_GoalAngle);
        
        if(Final_Angle>M_PI)
            Final_Angle = Final_Angle-2*M_PI;
        else if(Final_Angle<-M_PI)
            Final_Angle = Final_Angle +2*M_PI;
        // if(find(FinalAngle.begin(),FinalAngle.end(), Max_FinalAngle) != FinalAngle.end())
        //     Max_GapAngle_Num = find(FinalAngle.begin(), FinalAngle.end(), Max_FinalAngle) - FinalAngle.begin();
        // // //}or SORT -> 0th index 
        //Final_Angle = FinalAngle[Max_GapAngle_Num];
        /////////////////////////////////////
        //}//Obsta size if end
        ////////////////////////////////////////
    //else
        //Final_Angle=GoalAngle;
    //////printf("Gap Angle: %f, Min_Distance: %f",Max_GapAngle*180.0/M_PI,Min_Distance);
    //////printf("GoalAngle: %f Final_Angle: %f Heading: %f", GoalAngle*180.0/M_PI,Final_Angle*180.0/M_PI, yaw*180.0/M_PI);
    //////printf("LocalPosition x:%f y:%f z:%f Angle:%f",LocalPosition.pose.position.x,LocalPosition.pose.position.y,LocalPosition.pose.position.z,atan2f(LocalPosition.pose.position.y,LocalPosition.pose.position.x)*180.0/M_PI);
    //////printf("DesiredPosition x:%f y:%f z:%f Angle:%f",Desired_Position(0),Desired_Position(1),Desired_Position(2),atan2f(Desired_Position(1),Desired_Position(0))*180.0/M_PI);

    //Known GapAngle Num, Max_GapAngle, obstacle distance
    
    }
    //printf("Max GapAngle: %f \n",Max_GapAngle*180.0/M_PI);
    
    if(Clearance_StartAngle.size()>0)
    {
        FollowGapMethod::finalAngle = Final_Angle;
        FollowGapMethod::finalDistance = GapDistance[Max_GapAngle_Num];
        FollowGapMethod::finalBetween = GapBetween[Max_GapAngle_Num];
        FollowGapMethod::finalPI1 = MaxPI1[Max_GapAngle_Num]*180.0/M_PI;
        FollowGapMethod::finalPI2 = MaxPI2[Max_GapAngle_Num]*180.0/M_PI;
        FollowGapMethod::finalDist1 = MaxDist1[Max_GapAngle_Num];
        FollowGapMethod::finalDist2 = MaxDist2[Max_GapAngle_Num];
        if(finalBetween > 1000)
            return true;
        else 
            return false;
    }
    else
        return false;
    
}