// group_motion.h
#pragma once
#include <string>
#include <vector>
bool runGroupPoseServos(const std::vector<std::string>& servos,
                        const std::vector<int>& targets);
// New declaration
bool moveArmsVertically(float mm);