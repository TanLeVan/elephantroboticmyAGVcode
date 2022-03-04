#include "aibot_hw/aibot_base.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "aibot_base_node");
    AIbotBase aibot;
    ros::spin();
    return 0;
}