//
// Created by 13027 on 2024-03-09.
//

#ifndef MOSIPP_ALGO_UTIL_HPP
#define MOSIPP_ALGO_UTIL_HPP

// #include <unordered_map>
// #include <vector>
// #include <iostream>
#include "graph.hpp"

namespace rzq{
namespace search{

/**
 * @brief Epsilon-dominance.
 */
        template<typename IterData>
        bool EpsDominance(IterData v1, IterData v2, IterData b1,
                          IterData b2, double eps=0.0, bool less=true) {
            //判断g是否支配.
            auto i2 = v2.begin();
            for (auto i1 = v1.begin(); i1 != v1.end(); i1++){
                if (less) {
                    if (*i1 > (1.0+eps)*(*i2)) { return false; }
                }else{
                    if (*i1 < (1.0+eps)*(*i2)) { return false; }
                }
                i2++;
            }
            //判断b是否支配.
            auto bi2 = b2.begin();
            for (auto bi1 = b1.begin(); bi1 != b1.end(); bi1++){
                    if (*bi1 < (1.0+eps)*(*i2)) { return false; }
                bi2++;
            }

            return true;
        };

/**
 * @brief result data structure.
 */
        struct ALGOResult {
            bool success = false;
            std::unordered_map< long, std::vector<long> > paths;
            std::unordered_map< long, std::vector<long> > times;
            std::unordered_map< long, basic::CostVector > costs; //需要solution label id来获取。
            long totalCost;

            std::vector<long> targets;
            long n_generated = 0;
            long n_expanded = 0;
            double rt_initHeu = 0.0;
            double rt_search = 0.0;
            double rt_tot = 0.0;
            void writeHeader(std::ostream& out) ;
            void writeRow(std::ostream& out) ;
        };

/**
 *
 */
        std::ostream& operator<<(std::ostream& os, const ALGOResult& res) ;


/**
 * @brief A search label, which identifies a partial solution path.
 */
        //TODO: 更改ServiceCost0以满足实际需求.
        struct b_Label {
            b_Label() {};
            b_Label(long id0, long v0, basic::CostVector g0, basic::CostVector f0, long t0, long tb0,
                    basic::CostVector b_vector0, long ServiceCost0 = 1, bool serveFlag = false) {
                id = id0; v = v0; g = g0; f = f0; t = t0; tb = tb0;
                b_vector = std::move(b_vector0); ServiceCost = ServiceCost0; //这个move是改良的。
            };
            long id; // label's id, make it easy to look up.
            long v;
            basic::CostVector g;
            basic::CostVector f;
            basic::CostVector b_vector;
            basic::CostVector mstWeight; //用LKH以后就存储的是LKH给的cost_min.
            long t; // the arrival time at a state.
            long tb; // the ending time of the safe interval in a state.
            bool TaskDone(); // 验证b_vector中的元素是不是都为1.
            long ServiceCost;
            bool serveFlag;
            long mstConectedTarget;
        };
/**
 * @brief For storing the cost brought by executing the task.
 */


/**
 * @brief for cout.
 */
        std::ostream& operator<<(std::ostream& os, const b_Label& l) ;

struct DataEntry {
    long source;
    std::vector<long> targetSet;
    std::vector<std::vector<long>> node_constraints;
};

void load_and_parse_json(const std::string& file_name, std::vector<DataEntry>& data_entries);
} // end namespace search
} // end namespace rzq




#endif //MOSIPP_ALGO_UTIL_HPP
