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


#include <actions/choose_pose.h>




BT::ActionChoosePoseNode::ActionChoosePoseNode(std::string name,  WorkingMemory *wm_PTR, std::string waypoint_path) : 
    ActionNode::ActionNode(name), working_memory_PTR_(wm_PTR)
{
    type_ = BT::ACTION_NODE;

    //Parse YAML file and make database of points

    YAML::Node waypoint_yaml = YAML::LoadFile(waypoint_path);
    const YAML::Node& points = waypoint_yaml["points"];

    for (auto it = points.begin(); it != points.end(); ++it)
    {
        auto point = *it;

        std::string speech_goal;

        WayPoint waypoint;

        waypoint.goal_name_ = point["name"].as<std::string>();
        waypoint.speech_str_ = point["speechString"].as<std::string>();

        const YAML::Node& pose_node = point["pose"];
        const YAML::Node& gaze_node = point["fixation_point"];

        waypoint.pose_goal_.position.x = pose_node["x_cord"].as<double>();
        waypoint.pose_goal_.position.y = pose_node["y_cord"].as<double>();
        waypoint.pose_goal_.position.z = pose_node["z_cord"].as<double>();

        waypoint.pose_goal_.orientation.x = pose_node["quartinion_a"].as<double>();
        waypoint.pose_goal_.orientation.y = pose_node["quartinion_b"].as<double>();
        waypoint.pose_goal_.orientation.z = pose_node["quartinion_c"].as<double>();
        waypoint.pose_goal_.orientation.w = pose_node["quartinion_d"].as<double>();

        waypoint.gaze_goal_.fixation_point.point.x = gaze_node["x_cord"].as<double>();
        waypoint.gaze_goal_.fixation_point.point.y = gaze_node["y_cord"].as<double>();
        waypoint.gaze_goal_.fixation_point.point.z = gaze_node["z_cord"].as<double>();

        wm_PTR->waypoint_list_.push_back(waypoint);

    }


    //Create move base action client
    thread_ = std::thread(&ActionChoosePoseNode::WaitForTick, this);

}

BT::ActionChoosePoseNode::~ActionChoosePoseNode() {
}

void BT::ActionChoosePoseNode::WaitForTick()
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
        
        while (get_status() != BT::HALTED)
        {

            if(true)
            {
                set_status(BT::SUCCESS);
                DEBUG_STDOUT(" Action " << get_name() << " Done!");
            }else if (true)
            {
                set_status(BT::FAILURE);
                DEBUG_STDOUT(" Action " << get_name() << " FAILED!");
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(500));

        }
    }
}

void BT::ActionChoosePoseNode::Halt()
{
    set_status(BT::HALTED);

    DEBUG_STDOUT("HALTED state set!");
}


