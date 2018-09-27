#include "obstacles.h"


/*
void getLaserScan (const sensor_msgs::LaserScan &sens_msg)
{
    angle_min = sens_msg.angle_min;
    angle_max = sens_msg.angle_max;
    angle_increment = sens_msg.angle_increment;
    bool obst_exist = categorizeLaserScan(angle_min, angle_max, angle_increment, sens_msg.ranges);
}*/

bool categorizeLaserScan(float angle_min,float angle_max,float angle_increment,float *ranges)
{
    //convert radians to angles
    angle_increment = angle_increment*180/M_PI;
    angle_min = angle_min * 180/M_PI;
    angle_max = angle_max * 180/M_PI;

    int point_number = (angle_max - angle_min)/angle_increment;
    int range_treshold = 1000;

    //vagues - should sum to 100
    int REAR_VAGUE = 20;
    int SIDE_VAGUE = 15;
    int FRONT_VAGUE = 10;
    int MIDDLE_VAGUE = 10;

    //jeden procent kątowy
    float percent =  (angle_max - angle_min)/100;

    //right sectors
    RR[0] = angle_min;
    RR[1] = angle_min + REAR_VAGUE * percent;

    RS[0] = RR[1];
    RS[1] = RS[0] + SIDE_VAGUE * percent;

    RF[0] = RS[1];
    RF[1] = RF[0] + FRONT_VAGUE * percent;

    //middle sector

    MF[0] = RF[1];
    MF[1] = MF[0] + MIDDLE_VAGUE * percent;

    //left sectors

    LF[0] = MF[1];
    LF[1] = LF[0] + FRONT_VAGUE * percent;

    LS[0] = LF[1];
    LS[1] = LS[0] + SIDE_VAGUE * percent;

    LR[0] = LS[1];
    LR[1] = LR[0] + REAR_VAGUE * percent;

    //filtrowanie po odległosci
    for(int i = 0;i<point_number;i++)
    {
        float range = ranges[i];
        float angle = i*angle_increment;

        if(range<range_treshold)
        {
           //jest git - idź dalej
        }
        else
        {
            continue; //jeżeli punkt za daleko przejdź do kolejnej iteracji
        }

        //połowa zakresu 0-135
        if(angle<(angle_max-angle_min)/2)
        {
            if(angle>RR[0] && angle<RR[1])
                RR_count++;
            if(angle>RS[0] && angle<RS[1])
                RS_count++;
            if(angle>RF[0] && angle<RF[1])
                RF_count++;
            if(angle>MF[0] && angle<MF[1])
                MF_count++;
        }
        else //zakres 135-270
        {
            if(angle>LR[0] && angle<LR[1])
                LR_count++;
            if(angle>LS[0] && angle<LS[1])
                LS_count++;
            if(angle>LF[0] && angle<LF[1])
                LF_count++;
            if(angle>MF[0] && angle<MF[1])
                MF_count++;
        }
    }

    //przeszkoda z przodu stop
    if(LF_count>3 && MF_count>3 && RF_count>3)
    {
        return true;
    }

    //przeszkoda z lewej - stop
    if(LS_count>3 && LF_count>3)
    {
        return true;
    }
    //przeszkoda z prawej - stop
    else if(RF_count>3 && RS_count>3)
    {
        return true;
    }

    //nie ma przeszkody
    return false;
}