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

#ifndef WORKING_MEMORY_H
#define WORKING_MEMORY_H

#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <vizzy_msgs/GazeAction.h>


class WayPoint{

    public:
    std::string goal_name_;
    std::string speech_str_;
    geometry_msgs::Pose pose_goal_;
    vizzy_msgs::GazeGoal gaze_goal_;
  
};




class WorkingMemory{

    public:

    //Speech
    std::string speechGoal_;

    //Navigation
    geometry_msgs::PoseStamped poseGoal_;

    //WayPoint Goal
    WayPoint waypoint_goal_;

    //List of waypoints
    std::vector<WayPoint> waypoint_list_;

    int numFailures = 0;

    //Gaze goal
    vizzy_msgs::GazeGoal fixationGoal;


    

};



#endif //WORKING_MEMORY_