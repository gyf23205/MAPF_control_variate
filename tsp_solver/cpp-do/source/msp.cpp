#include <chrono>
#include "msp.hpp"


namespace rzq {
    namespace search {

        MinimumSpanningTreeHeuristic::MinimumSpanningTreeHeuristic() : _grid(nullptr) {}

        MinimumSpanningTreeHeuristic::~MinimumSpanningTreeHeuristic() {}

        void MinimumSpanningTreeHeuristic::SetGraphPtr(rzq::basic::GridkConn* g) {
            _grid = g;
        }


//        int MinimumSpanningTreeHeuristic::CalculateMSTSum(const std::vector<long>& targets, std::vector<long> b_vec) {
//            std::string str = b2str(b_vec);
//            std::vector<long> unfinishedTargets;
//            for (int i = 0; i < targets.size(); ++i) {
//                if(b_vec[i] == 0) {
//                    unfinishedTargets.push_back(targets[i]);
//                }
//            }
//            std::vector<long> allVertexIds = _grid->GetAllVertexId();
//            std::unordered_map<long, std::unordered_map<long, long>> allDistMaps;
//
//            //TODO: pre-compute.      FINISHED!
//            // 对targets中的每个顶点执行Dijkstra搜索，填充allDistMaps
//            for (long target : unfinishedTargets) {
//                DijkstraScan dijks;
//                dijks.SetGraphPtr(_grid);
//                dijks.Search(target, 0); // Assume 0 is some default parameter
//                allDistMaps[target] = dijks.GetDistMap();
//            }
//
//            //TODO:compute when expand, not storing.
//            // 对每个顶点计算其到targets的MST的总权重
//            for (long vertexId : allVertexIds) {
//                // 使用Prim算法计算MST的总权重
//                std::pair<long,long> mst_res = CalculateMSTWithPrim(vertexId, unfinishedTargets, allDistMaps);
//                long totalWeight = mst_res.first;
//                _u2h[str][vertexId] = totalWeight;
//            }
//
//            ////////////////////////////
////            _u2h.clear();
////            for(int i = 0; i < 9000000; i++) {
////                _u2h[i] = 0;
////            }
//            ////////////////////////////////
//            return 1;
//        }

        std::pair<long, long> CalculateMSTWithPrim(b_Label& l, const std::vector<long>& unfinishedTargets,
                                                   const std::unordered_map<long, std::unordered_map<long, long>>& allDistMaps) {
            std::set<long> inMST;
            std::unordered_map<long, long> parentMap;
            std::priority_queue<std::pair<long, long>, std::vector<std::pair<long, long>>, std::greater<std::pair<long, long>>> pq;

            pq.push({0, l.v});
            long totalWeight = 0;
            long directlyConnectedTarget = -1; // 初始化为-1，表示未找到

            while (!pq.empty() && inMST.size() < unfinishedTargets.size()) {
                auto [cost, u] = pq.top();
                pq.pop();

                if (inMST.insert(u).second) {
                    totalWeight += cost;
                    if (u != l.v && parentMap[u] == l.v) { // 如果u不是初始顶点且其父顶点是vertexId
                        directlyConnectedTarget = u; // 找到了与vertexId直接相连的顶点
                    }
                    for (long target : unfinishedTargets) {
                        if (inMST.find(target) == inMST.end()) {
                            long weightToTarget = (allDistMaps.find(target) != allDistMaps.end() && allDistMaps.at(target).find(u) != allDistMaps.at(target).end()) ? allDistMaps.at(target).at(u) : std::numeric_limits<long>::max();
                            pq.push({weightToTarget, target});
                            parentMap[target] = u;
                        }
                    }
                }
            }

            return {totalWeight, directlyConnectedTarget};
        }

        long MinimumSpanningTreeHeuristic::GetCost(long u, std::vector<long> b_vec) {
            std::string str = b2str(b_vec);
            auto it = _u2h[str].find(u);
            return it != _u2h[str].end() ? it->second : std::numeric_limits<long>::max();
        }

        std::unordered_map<long, long> MinimumSpanningTreeHeuristic::GetHeuristicMap(std::vector<long> b_vec) {
            std::string str = b2str(b_vec);
            return _u2h[str];
        }

        std::string b2str(const std::vector<long>& b_vec) {
            std::string str;
            for (long i : b_vec) {
                str += static_cast<char>(i);
            }
            return str;
        }

    } // namespace search
} // namespace rzq
