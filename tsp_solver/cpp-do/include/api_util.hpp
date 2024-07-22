#ifndef MOSIPP_API_UTIL_HPP
#define MOSIPP_API_UTIL_HPP

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
using namespace std;

namespace api {
    enum Actions { MOVE, WAIT, SERVE, FINISH };
    static const map<int, string> desc = {
            {MOVE, "MOVE"}, {WAIT, "WAIT"}, {SERVE, "SERVE"}, {FINISH, "FINISH"}};

    struct State {
        // the workspace is a gridmap
        // row, col
        int r, c;
        double t; // time
        Actions a;
    };

    struct APIResult {
        vector<State> plan;
    };
}

#endif //MOSIPP_API_UTIL_HPP
