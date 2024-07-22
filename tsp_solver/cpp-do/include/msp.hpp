#ifndef RZQ_SEARCH_MINIMUMSPANNINGTREEHEURISTIC_HPP
#define RZQ_SEARCH_MINIMUMSPANNINGTREEHEURISTIC_HPP

#include "graph.hpp"
#include "dijkstra.hpp"
#include "Algo_util.hpp"
#include <set>
#include <vector>
#include <unordered_map>
#include <limits>
#include <queue>
#include <algorithm>


#pragma once
namespace rzq {
    namespace search {

//        class MinimumSpanningTreeHeuristic {
//        public:
//            MinimumSpanningTreeHeuristic();
//            virtual ~MinimumSpanningTreeHeuristic();
//
//            // 设置图指针，允许对不同的图使用该启发式类
//            virtual void SetGraphPtr(basic::Graph* g);
//
//            // 生成从目标节点出发的最小生成树，并计算启发式值
//            virtual int GenerateMST(long target);
////            int GenerateMST(long startVertex, const std::vector<long>& targets, const basic::CostVector& b_vec);
//
//            // 获取特定节点到目标节点的启发式值
//            virtual long GetCost(long u);
//
//            // 获取所有节点到目标节点的启发式值的映射
//            virtual std::unordered_map<long, long> GetHeuristicMap();
//
//        protected:
//            basic::Graph* _graph; // 图的指针
//            std::unordered_map<long, long> _u2h; // 存储启发式值的映射
//        };

// MinimumSpanningTreeHeuristic.hpp

        std::string b2str(const std::vector<long>& b_vec);
        std::pair<long, long> CalculateMSTWithPrim(b_Label& l, const std::vector<long>& unfinishedTargets,
                                                   const std::unordered_map<long, std::unordered_map<long, long>>& allDistMaps);

        class MinimumSpanningTreeHeuristic {
        public:
            // 默认构造函数
            MinimumSpanningTreeHeuristic();

            // 析构函数
            virtual ~MinimumSpanningTreeHeuristic();

            // 设置GridkConn实例
            void SetGraphPtr(rzq::basic::GridkConn* g);

            // 生成最小生成树，并计算启发式值
            int CalculateMSTSum(const std::vector<long>& targets, std::vector<long> b_vec);


            // 获取特定节点到目标节点的启发式值
            long GetCost(long u, std::vector<long> b_vec);

            // 获取所有节点到目标节点的启发式值的映射
            std::unordered_map<long, long> GetHeuristicMap(std::vector<long> b_vec);

            // 将b_vec转化为字符串，用于对应u2h.


        protected:
            rzq::basic::GridkConn* _grid; // GridkConn实例的指针
            std::unordered_map< std::string, std::unordered_map<long, long> > _u2h; // 存储启发式值的映射
            std::vector<DijkstraScan> _dijks;
        };



    }
}

#endif // RZQ_SEARCH_MINIMUMSPANNINGTREEHEURISTIC_HPP



