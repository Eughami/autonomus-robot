/**
 *    Desc:
 *
 *    (C) 2017, Author: EMRE OZBILGE
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License along
 *    with this program; if not, see <http://www.gnu.org/licenses/>
 */


#include <libplayerc++/playerc++.h>
#include <iostream>
#include <cstdio>
#include <cmath>
#include <iomanip>
#include <vector> // std::vector
#include <algorithm> // min_element, max_element
#include <numeric> // accumulate
#include <cstdlib>
#include <ctime>
#include <unistd.h>

using namespace PlayerCc;


/* number of pixels away from the image centre a blob can be, to be in front of the robot.
 * This is essentially the margin of error.*/
const int margin = 5;
// define motion constants
const double LV_MAX=0.5; // maximum linear speed
const double AV_MAX=0.5; // maximum angular speed
// how close the robot can approach to the blob
const double CLOSE=0.30; //cm
// define finite state machine
int state=-1;
// constants for moving to target position
const double CLOSE_TARGET=1; // 1m
const double ANG_ERR=dtor(1); //1 degree
int COLLECTED_BLOB_NO = 0;


void goToTarget(double& lv,double& av,const std::vector<double>& tar_pos,std::vector<double>& rob_pos)
{

    double dist2target;
    double delta_diff, net_diff;

    // calculate the distance to the target
    dist2target = std::sqrt((tar_pos[0]-rob_pos[0])*(tar_pos[0]-rob_pos[0])+(tar_pos[1]-rob_pos[1])*(tar_pos[1]-rob_pos[1]));

    // calculate angle between target position and the robot heading
    delta_diff = std::atan2(tar_pos[1]-rob_pos[1],tar_pos[0]-rob_pos[0])-rob_pos[2];

    // if delta_diff is greater/less than 180 degree adjust the net angle
    if (delta_diff>M_PI)
        net_diff=delta_diff-2*M_PI;
    else if (delta_diff<-M_PI)
        net_diff=delta_diff+2*M_PI;
    else
        net_diff=delta_diff;

    // set angular and linear velocities
    av=AV_MAX*net_diff;
    if (std::fabs(net_diff)>ANG_ERR)
        lv=0.0; // stop
    else
        lv=LV_MAX*(1.0-std::exp(-dist2target)); // faster when away

    // display some paramters
    std::cout<<"target:"<<"("<<tar_pos[0]<<","<<tar_pos[1]<<")"<<" dist:"<<dist2target<<" deg_err:"<<PlayerCc::rtod(net_diff)<<std::endl;

    // change the state if you reach the one
    if (dist2target<CLOSE_TARGET)
    {
        lv=0.0;
        av=0.0;
        state=3;
    }
}


void moveToBox(double& lv,double& av,const int& centre,const playerc_blobfinder_blob_t& blob,const Position2dProxy &pp,BlobfinderProxy &bfp,const std::vector<double>& tar_pos)
{
    // How far is the detected blob
    std::cout<<"range:"<<blob.range<<std::endl;

    if(blob.range < CLOSE_TARGET && blob.range >  CLOSE+0.10) {
        /*
        * If robot is close to collection Area it means
        * the blob is already in the collection area
        * Turn right/ left and go back to wander
        */

        double d = pow((pp.GetXPos() - tar_pos[0]), 2) + pow((pp.GetYPos()- tar_pos[1]), 2);
        //make a check if blob is inside the collection Area go back (Collection Area raduis is 1m + 0.5m for the robot length and error margin)

        std::cout<<"distance from collection area: "<<d<<std::endl;
        if (d <=  pow(1.5, 2))
        {
            // if no blob is seen go back to wander
            // set state to 0 again
            if (bfp.GetCount() == 0)
            {
                state = 0;
            }
            else {

                // turn right or left

                av = 0.5;
                lv = 0;
            }
        }
    }
    //change the state to grip the object
    else if (blob.range<CLOSE)
    {

        lv=0.0;
        av=0.0;
        state=1;

    }
    else {
        // the closer to the object the robot moves slower
        lv = LV_MAX * (1.05-std::exp( -std::sqrt((blob.range-CLOSE)*(blob.range-CLOSE))) );

        if(blob.x < centre-margin) {
            std::cout<<"blob is on my left\n";
            av=0.1;
        }
        else if(blob.x > centre+margin) {
            std::cout<<"blob is on my right\n";
            av=-0.1;
        }
        else {
            std::cout<<"blob is at front\n";
            av=0.0;
        }
    }// end of else for range>close

}


void Wander(double& av, double& lv) {

    int max_turn=30;
    int min_turn=-30;
    double max_speed=1;

    av = dtor((rand()%(max_turn-min_turn+1))+min_turn);
    lv = ((rand()%11)/10.0)*max_speed;

    std::cout<<"---------------------------Wandering behaviour-------------------"<<std::endl;
}

void ObstacleAvoidance(double& av, double& lv, RangerProxy &lp) {
    std::vector<double> temp_readings(180);
    const int SIZE_LP=lp.GetRangeCount();
    const double MAX_LV=0.6; //cm
    const double MAX_AV=30;
    double l_min, r_min;
    double l_field,r_field;

    for (int i=0; i<SIZE_LP; ++i)
        temp_readings[i]=lp[i];

    r_min=( 0.8 * *std::min_element(temp_readings.begin(),temp_readings.begin()+90) + 0.2* std::accumulate(temp_readings.begin(), temp_readings.begin()+90, 0.0)/90 );

    l_min=( 0.8* *std::min_element(temp_readings.begin()+90,temp_readings.begin()+180) + 0.2 * std::accumulate(temp_readings.begin()+90, temp_readings.begin()+180, 0.0)/90 );

    l_field=(100000*r_min)/500-100;
    r_field=(100000*l_min)/500-100;


    if (l_field>100)
        l_field=100;

    if (r_field>100)
        r_field=100;

    lv=(r_field+l_field)/500;
    av=(r_field-l_field);
    std::cout<<"Obstale Avoidance : reading lv "<< lv<< " av : " << av<<std::endl;


    if ((r_field+l_field)<0.05)
        av=( (std::rand() %2) ? 1:-1 ) * MAX_AV;

    lv=limit(lv,0.0,MAX_LV);
    av=limit(av,-MAX_AV,MAX_AV);
    av=dtor(av);

    std::cout<<"Obstale Avoidance : reading lv "<< lv<< " av : " << av<<std::endl;
}

int main() {
    srand(time(NULL));
    // we throw exceptions on creation if we fail
    try
    {
        // create robot client to communicate with robot
        PlayerClient robot("localhost");

        // creating instance for robot position data
        Position2dProxy pp(&robot, 0);

        // define a BlobfinderProxy proxy to communicate with robot's gripper
        BlobfinderProxy bfp(&robot, 0);

        // define a gripper proxy to communicate with robot's gripper
        GripperProxy gp(&robot, 0);

        // creating proxy for laser device
        RangerProxy lp(&robot,1);

        //change client data mode to receive always latest data
        robot.SetDataMode(PLAYER_DATAMODE_PULL);
        robot.SetReplaceRule(true);

        // make sure we get data from laser
        std::puts ( "Waiting Laser sensors...\n" );
        while (lp.GetRangeCount()==0) {
            robot.Read();
        }
        std::puts ( "Laser is done!\n" );

        // enabling robot motors
        pp.SetMotorEnable (true);

        // only print two decimals after the point
        std::cout << std::setprecision(2) << std::fixed;

        //find the largest blob
        playerc_blobfinder_blob_t blob;
        int biggestBlobArea = 0;
        int biggestBlob = 0;
        int centre;
        // current status of the gripper
        bool gripopen = true;
        // robot current velocities
        double lv=0.0, av=0.0;
        // define collection area coordinates
        std::vector<double> coll_pos(2);
        coll_pos[0]=5;
        coll_pos[1]=-3;
        //define charging area coordinates
        std::vector<double> char_pos(2);
        char_pos[0]=-7;
        char_pos[1]=-7;
        // robot current position
        std::vector<double> rob_pos(3);
        // position of the current released disc
        std::vector<double> curr_pos(3);
        double dist;
        double curr_ang_err;

        // initial state
        state=0;

        // go into sense-think-act loop
        for(;;)
        {
            // this blocks until new data comes; 10Hz by default
            robot.Read();

            // TODO: (1) implement wandering (searching discs) behaviour
            // TODO: (2) implement obstacle avoidance
            // TODO: (3) implement go to charge area behaviour
            // TODO: (4) Add new states in order controller works accurately

            if (state==0) {
                if(bfp.GetCount()!=0) {

                    for(int i=0; i<bfp.GetCount(); i++)
                    {
                        //get blob from proxy
                        playerc_blobfinder_blob_t currBlob = bfp[i];
                        // here we are checking which blob is bigger because it is possible there is two blob next to each other
                        if( std::abs((int)currBlob.area) > biggestBlobArea)
                        {
                            biggestBlob = i;
                            biggestBlobArea = currBlob.area;
                        }
                    }// end of for

                    blob = bfp[biggestBlob];
                    // find centre of image
                    centre = bfp.GetWidth()/2;

                    moveToBox(lv,av,centre,blob,pp,bfp,coll_pos);
                }
                else
                {
                    // TODO: incomplete part
                    Wander(av, lv);
                    //Avoid obstacle
                    ObstacleAvoidance(av, lv, lp);


                }
            }
            else if (state==1) {
                std::cout << "Closing gripper\n";
                gripopen=false; // change gripper status
                gp.Close();
                lv=0.0;
                av=0.0;
                state=2; // move to collection area
            }
            else if (state ==2) // move to the collection area
            {
                rob_pos[0]=pp.GetXPos();
                rob_pos[1]=pp.GetYPos();
                rob_pos[2]=pp.GetYaw();
                //Avoid obstacle
                ObstacleAvoidance(av, lv, lp);
                goToTarget(lv,av,coll_pos,rob_pos);
            }
            else if (state==3)
            {
                // you have to release the box now, then backward to leave the collection area
                std::cout << "Opening gripper\n";
                gripopen=true; // change gripper status
                gp.Open();
                lv=0.0;
                av=0.0;
                state = 4;
                // increase the flag for number of collected blob
                COLLECTED_BLOB_NO++;
                curr_pos.clear();
                curr_pos[0]=pp.GetXPos();
                curr_pos[1] = pp.GetYPos();
                curr_pos[2] = pp.GetYaw();
            } else if (state == 4) {

                dist = std::sqrt((pp.GetXPos()- curr_pos[0])*(pp.GetXPos()- curr_pos[0]) + (pp.GetYPos() - curr_pos[1])*(pp.GetYPos() - curr_pos[1]));

                curr_ang_err = std::fabs(curr_pos[2]+M_PI)- pp.GetYaw();
                if(curr_ang_err> M_PI)
                    curr_ang_err = curr_ang_err-2*M_PI;

                else if(curr_ang_err <-M_PI)
                    curr_ang_err = curr_ang_err+2*M_PI;
                std::cout<<"Distance to collection area:"<<dist<<" Current angle error:"<<curr_ang_err<<std::endl;

                if(dist< 0.3) {

                    // go backward
                    lv= -0.5;
                    av=0;
                }
                else if( curr_ang_err> dtor(10)) {
                    // turn left
                    lv=0;
                    av=0.5;

                } else {
                    // make a check if 3 blob are collected
                    // then go back to charge station

                    lv=0;
                    av=0;
                    state = 0;
                }
            }


            // set the new lv and av
            pp.SetSpeed(lv,av);
            sleep(1);
            std::cout<<"state:"<<state<<"  lv:"<<lv<<"  av:"<<av<<" collected blob:"<<COLLECTED_BLOB_NO<<std::endl;
            if(COLLECTED_BLOB_NO == 3) {
                std::cout<<"All 3 blob are collected going back to chargin area"<<std::endl;
                return -1;

            }
        } //end of sense-think-act loop

    }
    catch (PlayerCc::PlayerError & e)
    {
        std::cerr << e << std::endl;
        return -1;
    }

    return 0;

}
