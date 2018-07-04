#include <actions/goto_point.h>
#include <actions/choose_pose.h>
#include <behavior_tree.h>
#include <iostream>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "VizzyBehaviorTree");
    ros::NodeHandle nh;

    WorkingMemory *wm = new WorkingMemory();
    
    try
    {
        int TickPeriod_milliseconds = 1000;

        BT::ActionGotoPointNode* action_goto =
            new BT::ActionGotoPointNode("Act:GotoPoint", wm);
        
        BT::ActionChoosePoseNode* action_choose_pose =
            new BT::ActionChoosePoseNode("Act:ChoosePose", wm);
        
        BT::SequenceNodeWithMemory* sequence_navigation =
            new BT::SequenceNodeWithMemory("Seq:Navigation");

        sequence_navigation->AddChild(action_choose_pose);
        sequence_navigation->AddChild(action_goto);

        Execute(sequence_navigation, TickPeriod_milliseconds);


    }
    catch(BT::BehaviorTreeException& Exception)
    {
        std::cerr << Exception.what() << '\n';
    }
    
    delete wm;
    return 0;
}
