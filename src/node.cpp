# include "GlobalLocalizer.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global localizer");
    GlobalLocalizer global_localizer;
    ros::spin();
    return 0;
}
