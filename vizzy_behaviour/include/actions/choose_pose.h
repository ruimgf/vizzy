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


#ifndef ACTIONS_CHOOSE_POSE_NODE_H
#define ACTIONS_CHOOSE_POSE_NODE_H

#include <action_node.h>
#include <string>
#include <working_memory/working_memory.h>
#include <yaml-cpp/yaml.h>



namespace BT
{
class ActionChoosePoseNode : public ActionNode
{
public:
    // Constructor
    explicit ActionChoosePoseNode(std::string Name, WorkingMemory *wm_PTR, std::string waypoint_path);
    ~ActionChoosePoseNode();

    void WaitForTick();
    void set_time(int time);

    void Halt();
    
private:

    WorkingMemory *working_memory_PTR_;
};
}  // namespace BT

#endif  // ACTIONS_CHOOSE_POSE_NODE_H
