#pragma once

#include <vector>
#include <string>

std::vector<std::string> getActuatedJointNamesOfReducedModel() {
    return {"L_UPPERARM_P",
            "R_UPPERARM_P",
            "L_UPPERLEG_Y",
            "L_UPPERLEG_R",
            "L_UPPERLEG_P",
            "L_LOWERLEG_P",
            "L_FOOT_P",
            "L_FOOT_R",
            "R_UPPERLEG_Y",
            "R_UPPERLEG_R",
            "R_UPPERLEG_P",
            "R_LOWERLEG_P",
            "R_FOOT_P",
            "R_FOOT_R"};
}

std::vector<std::string> getActuatedJointNamesOfFixedUpperBodyModel() {
    return {"L_UPPERLEG_Y",
            "L_UPPERLEG_R",
            "L_UPPERLEG_P",
            "L_LOWERLEG_P",
            "L_FOOT_P",
            "L_FOOT_R",
            "R_UPPERLEG_Y",
            "R_UPPERLEG_R",
            "R_UPPERLEG_P",
            "R_LOWERLEG_P",
            "R_FOOT_P",
            "R_FOOT_R"};
}