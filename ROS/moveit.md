# MoveIt!

## from Indigo to Kinetic


```C++
// Indigo
#include <moveit/move_group_interface/move_group.h>
moveit::planning_interface::MoveGroup::Plan
bool suc = group_->plan(temp_plan)
    if (suc)
    ...

// Kinetic
#include <moveit/move_group_interface/move_group_interface.h>
moveit::planing_interface::MoveGroupInterface::Plan
moveit::planning_interface::MoveItErrorCode suc = group_->plan(temp_plan);
		if (suc)
        ...
```