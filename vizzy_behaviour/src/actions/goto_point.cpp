/* Copyright (C) (2018) Joao Avelino - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <actions/goto_point.h>
#include <string>




BT::ActionGotoPointNode::ActionGotoPointNode(std::string name,  WorkingMemory *wm_PTR) : ActionNode::ActionNode(name), 
    working_memory_PTR_(wm_PTR)
{
    type_ = BT::ACTION_NODE;

    //Create move base action client
    move_client_ = new MoveBaseClient("move_base", true);
    move_client_->waitForServer(ros::Duration(10));

    goal_msg_.target_pose.header.frame_id = "map";

    thread_ = std::thread(&ActionGotoPointNode::WaitForTick, this);

}

BT::ActionGotoPointNode::~ActionGotoPointNode() {
    delete move_client_;
}

void BT::ActionGotoPointNode::WaitForTick()
{
    while (true)
    {
        // Waiting for the first tick to come
        DEBUG_STDOUT(get_name() << " WAIT FOR TICK");

        tick_engine.Wait();
        DEBUG_STDOUT(get_name() << " TICK RECEIVED");

        // Running state
        set_status(BT::RUNNING);
        
        // Perform action...
        goal_msg_.target_pose.header.stamp = ros::Time::now();
        goal_msg_.target_pose.pose = working_memory_PTR_->poseGoal_.pose;
        move_client_->sendGoal(goal_msg_);

        while (get_status() != BT::HALTED)
        {
            actionlib::SimpleClientGoalState state = move_client_->getState();

            
            if(state.SUCCEEDED == state.state_)
            {
                set_status(BT::SUCCESS);
                DEBUG_STDOUT(" Action " << get_name() << " Done!");
            }else if (state.PREEMPTED == state.state_ || state.LOST == state.state_ || 
                state.ABORTED == state.state_ || state.REJECTED == state.state_ || 
                state.RECALLED == state.state_ || state.LOST == state.state_)
            {
                set_status(BT::FAILURE);
                DEBUG_STDOUT(" Action " << get_name() << " FAILED!");
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(500));

        }
    }
}

void BT::ActionGotoPointNode::Halt()
{
    set_status(BT::HALTED);
    move_client_->cancelAllGoals();
    DEBUG_STDOUT("HALTED state set!");
}


