#include "deadreckoning.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "deadreckoning");
    Deadreckoning drc;
    ros::spin();
    return 0;
}