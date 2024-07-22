#include "avltree.hpp"
#include "Algo.hpp"
#include <set>
#include <memory>
#include <chrono>
#include <utility>

namespace rzq{
namespace search{
            SIPPStateSpace::SIPPStateSpace() {
                return;
            };

            SIPPStateSpace::~SIPPStateSpace() {
                return;
            };

            void SIPPStateSpace::SetGraphPtr(basic::Graph* g) {
                _graph = g;
                return;
            };

            void SIPPStateSpace::AddNodeCstr(long nid, long t) {
                if ( _avl_node.find(nid) == _avl_node.end() ) {
                    _avl_node[nid] = basic::AVLTree<long>();
                }
                _avl_node[nid].Add(t); // add this unsafe interval.
                // std::cout << " add node cstr = " << nid << " t = " << t << std::endl;
                return;
            };

            void SIPPStateSpace::AddEdgeCstr(long u, long v, long t) {
                if ( _avl_edge.find(u) == _avl_edge.end() ) {
                    _avl_edge[u] = std::unordered_map<long, basic::AVLTree<long> >();
                }
                if ( _avl_edge[u].find(v) == _avl_edge[u].end() ) {
                    _avl_edge[u][v] = basic::AVLTree<long>();
                }
                _avl_edge[u][v].Add(t);
                return;
            };

            bool SIPPStateSpace::FindSafeItv(long nid, long t, long* ta, long* tb) {
                *ta = 0;
                *tb = MOSIPP_MAX_TIME;

                //Find if the node has constraints.
                if (_avl_node.find(nid) == _avl_node.end()) {
                    return true; // [0, Tmax]
                }
                if (_avl_node[nid].Find(t).h != 0) {
                    // the input t overlaps with exact a node constraint.
                    *tb = 0;
                    return false;
                }
                long ub = MOSIPP_MAX_TIME;
                if ( _avl_node[nid].FindMinMore(t, &ub) ) {
                    *tb = ub - 1;
                }
                long lb = 0;
                if ( _avl_node[nid].FindMaxLess(t, &lb) ) {
                    *ta = lb + 1;
                }
                return true;
            };

            long SIPPStateSpace::GetDuration(long u, long v) {
                // the first component of the graph must be the traversal time.
                return _graph->GetCost(u,v)[0];
            };

//Just simply emplace safe intervals gained by _GetSuccItv into a new std::vector<Label>.
    std::vector<b_Label> SIPPStateSpace::GetSuccBLabels(long u, long v, long ta, long tb, basic::CostVector& B) {
        if (DEBUG_MOSIPP >= 3) {
            std::cout << "[DEBUG] >>>>>>> SIPPStateSpace::_GetSuccItv with input " << u
                      << ", " << v << ", " << ta << ", " << tb <<  std::endl;
        }
        std::vector<b_Label> out;
        std::vector<long> tas, tbs;
        _GetSuccItv(u,v,ta,tb, &tas, &tbs);
        for (size_t i = 0; i < tas.size(); i++) {
            if (DEBUG_MOSIPP >= 3) {
                std::cout << "[DEBUG] >>>>>>> SIPPStateSpace::_GetSuccItv " << tas[i] << ", " << tbs[i] << std::endl;
            }
            if (ENABLE_SAFE_CHECK_MOSIPP) {
                if (tas[i] <= ta) {
                    std::cout << "[ERROR] SIPPStateSpace::GetSuccLabels tas[i]=" << tas[i] << " < ta=" << ta << std::endl;
                    throw std::runtime_error( "[ERROR] SIPPStateSpace::GetSuccLabels wrong interval !" );
                }
                long d = GetDuration(u,v);
                if (tas[i] > tb + d) {
                    std::cout << "[ERROR] SIPPStateSpace::GetSuccLabels tas[i]=" << tas[i] << " > tb+d=" << tb+d << std::endl;
                    throw std::runtime_error( "[ERROR] SIPPStateSpace::GetSuccLabels wrong interval !" );
                }
            }
            //这里缺少一个b_Label构造函数中的b_vector.
            out.emplace_back(-1, v, basic::CostVector(), basic::CostVector(), tas[i], tbs[i], B);
        }
        return out;
    };

            bool SIPPStateSpace::ViolateNC(long u, long t) {
                if (_avl_node[u].Find(t).h != 0) {
                    // the input t overlaps with exact a node constraint.
                    return true;
                }
                return false;
            };

            bool SIPPStateSpace::ViolateEC(long u, long v, long t) {
                if (_avl_edge.find(u) == _avl_edge.end() ) {
                    return false;
                }
                if (_avl_edge[u].find(v) == _avl_edge[u].end() ) {
                    return false;
                }
                if (_avl_edge[u][v].Find(t).h == 0 ) {
                    return false;
                }
                return true;
            };


// ta and tb are the bound of the safe interval of vertex u.
            void SIPPStateSpace::_GetSuccItv(long u, long v, long ta, long tb,
                                             std::vector<long>* out_tas, std::vector<long>* out_tbs)
            {
                if ( (ENABLE_SAFE_CHECK_MOSIPP) && (tb < ta) ) {
                    std::cout << "[ERROR] SIPPStateSpace::_GetSuccItv tb !< ta, " << tb << " !< " << ta << std::endl;
                    throw std::runtime_error( "[ERROR] SIPPStateSpace::_GetSuccItv tb !< ta !?" );
                }
                long d = GetDuration(u,v);
                std::vector<long> unsafe_points;
                _FindAllUnsafeTimeStep(u, v, &unsafe_points);

                if (DEBUG_MOSIPP >= 3) {
                    for (auto k: unsafe_points) {std::cout << "[DEBUG] >>>>>>>>>>> unsafe k = " << k << std::endl;}
                }

                long t0 = ta + d;
                long tub = (tb >= MOSIPP_MAX_TIME-d) ? MOSIPP_MAX_TIME : tb + d; // upper bound, this is impl in a way to avoid overflow.

                if (unsafe_points.size() == 0) {
                    out_tas->push_back(t0);
                    out_tbs->push_back(MOSIPP_MAX_TIME);
                    return;
                }

                // Note that unsafe_points have been sorted. The indexes of out_tas and out_tbs should be matched when used.
                for (size_t ii = 0; ii < unsafe_points.size(); ii++) {
                    const long& tk = unsafe_points[ii];
                    if (tk <= t0) {
                        if (tk == t0) {
                            t0++;

                            // t0 > tub means the mission can be finished within the safe interval of vertex u.
                            if (t0 > tub) {break;} // everytime when t0 changes, need to check
                        }
                        continue;
                    }

                    out_tas->push_back(t0);
                    out_tbs->push_back(tk-1); // right before the next unsafe point

                    // This "+1" is for searching another safe interval.
                    t0 = tk + 1; // note, if MOSIPP_MAX_TIME is not set properly, this may OVERFLOW !!!
                    if (t0 > tub) {break;} // everytime when t0 changes, need to check
                }

                // After all the unsafe points considered, if to is still within the safe interval of vertex u.
                if (t0 <= tub) {
                    out_tas->push_back(t0);
                    out_tbs->push_back(MOSIPP_MAX_TIME); //
                }else{
                    // nothing.
                }

                if (ENABLE_SAFE_CHECK_MOSIPP) {
                    for (size_t ii = 0; ii < unsafe_points.size(); ii++) {
                        for (size_t jj = 0; jj < out_tas->size(); jj++) {
                            if (unsafe_points[ii] >= out_tas->at(jj) && unsafe_points[ii] <= out_tbs->at(jj)) {
                                std::cout << "[ERROR] SIPPStateSpace::_GetSuccItv unsafe point, " << unsafe_points[ii]
                                          << " is within [" << out_tas->at(jj) << "," << out_tbs->at(jj) << "]" << std::endl;
                                throw std::runtime_error( "[ERROR] SIPPStateSpace::_GetSuccItv computed interval is not safe !?" );
                            }
                        }
                    }
                }
                return;
            };


            void SIPPStateSpace::_FindAllUnsafeTimeStep(long u, long v, std::vector<long> *out) {

                long d = GetDuration(u,v);
                std::vector<long> unsafe_points1;
                if ( _avl_node.find(v) != _avl_node.end() ) {
                    _avl_node[v].ToSortedVector(&unsafe_points1);
                }
                std::vector<long> unsafe_points2;
                if (_avl_edge.find(u) != _avl_edge.end() ) {
                    if (_avl_edge[u].find(v) != _avl_edge[u].end() ){
                        _avl_edge[u][v].ToSortedVector(&unsafe_points2);
                    }
                }
                size_t i1=0, i2=0;
                std::vector<long>& unsafe_points = *out;
                if (unsafe_points1.size() == 0) {
                    for (auto kk : unsafe_points2) {
                        unsafe_points.push_back(kk+d); // don't forget this d !!
                    }
                }else if (unsafe_points2.size() == 0){
                    unsafe_points = unsafe_points1;
                }else{
                    while ( i1 != unsafe_points1.size() || i2 != unsafe_points2.size() ) {
                        if (i1 == unsafe_points1.size() ) {
                            unsafe_points.push_back(unsafe_points2[i2]+d);
                            i2++;
                            continue;
                        }else if (i2 == unsafe_points2.size()) {
                            unsafe_points.push_back(unsafe_points1[i1]);
                            i1++;
                            continue;
                        }else{
                            long usfp2 = unsafe_points2[i2]+d;
                            if ( unsafe_points1[i1] < usfp2 ) {
                                unsafe_points.push_back(unsafe_points1[i1]);
                                i1++;
                                continue;
                            }else if ( unsafe_points1[i1] > usfp2 ) {
                                unsafe_points.push_back(usfp2);
                                i2++;
                                continue;
                            }else { // unsafe_points1[i1] === usfp2
                                unsafe_points.push_back(usfp2);
                                i1++;
                                i2++;
                                continue;
                            }
                        }
                    }
                }
                return;
            };


/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////


            FrontierLinear::FrontierLinear() {
                return;
            };

            FrontierLinear::~FrontierLinear() {
                return;
            };

            bool FrontierLinear::Check(basic::CostVector& g, basic::CostVector& b) {
                if (c_wait.size() == 0) {
                    throw std::runtime_error("[ERROR], FrontierLinear::Check, wait_cost not set!");
                }
                for (auto iter : b_labels) {
                    if (_LabelDom(iter.second.g, g, iter.second.b_vector, b)) {return true;}
                }
                return false;
            };

            void FrontierLinear::Update(b_Label& l, std::unordered_set<long> *deleted) {
                Filter(l, deleted);
                Add(l);
                return;
            };

            bool FrontierLinear::Remove(const b_Label& l) {

                for (auto iter: b_labels) {
                    if (l.g==iter.second.g && l.b_vector==iter.second.b_vector) {
                        b_labels.erase(iter.first);
                        return true;
                    }
                }
                return false;
            };

            void FrontierLinear::Add(const b_Label& l) {
                this->b_labels[l.id] = l;
                return;
            };


            void FrontierLinear::Filter(b_Label& l, std::unordered_set<long> *deleted) {
                auto newLabels = b_labels;
                for (auto iter : b_labels) {
                    if (_LabelDom(l.g, iter.second.g, l.b_vector, iter.second.b_vector)) {
                        newLabels.erase(iter.first);
                        if (deleted) {
                            deleted->insert(iter.first);
                        }
                    }
                }
                b_labels = newLabels;
                return;
            };


            void FrontierLinear::SetWaitCost(const basic::CostVector& c) {
                c_wait = c;
            }

            //
            bool FrontierLinear::_LabelDom(basic::CostVector& g1, basic::CostVector& g2,
                                           basic::CostVector& b1, basic::CostVector& b2) {
                if (g2[0] < g1[0]) {return false;}
                long dt = (g2[0] - g1[0]);
                if (EpsDominance(g1 + c_wait*dt, g2, b1, b2)) {return true;}
                return false;
            }

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

            ALGO::ALGO() {};

            ALGO::~ALGO() {};

            void ALGO::SetGraphPtr(basic::Graph* g) {
                _graph = g;
                _sss.SetGraphPtr(g);
            };

        bool ALGO::writeTSPFile(std::unordered_map<long, std::unordered_map<long, long>> allDistMaps, const long cur,
                                basic::CostVector& b_vec, const std::string& filename) {
            std::ofstream outfile(filename);
            if (!outfile.is_open()) {
                std::cerr << "Error: Could not open the file!" << std::endl;
                return false;
            }

            long dummy_id = 999999; // To represent dummy node.
//            long cur_id = cur;
            std::unordered_map<long, long> cur_dist_map;
            std::unordered_map<long, long> dummy_dist_map;

            std::vector<long> unfinishedTargets;
            for (int i = 0; i < _target.size(); ++i) {
                if(b_vec[i] == 0) {
                    unfinishedTargets.push_back(_target[i]);
                }
            }

            // Construct the map of current node.
            for(int i = 0; i < unfinishedTargets.size(); i++) {
                cur_dist_map.insert(std::make_pair(unfinishedTargets[i], allDistMaps[unfinishedTargets[i]][cur]));
            }
            cur_dist_map.insert(std::make_pair(dummy_id, 0));
            cur_dist_map.insert(std::make_pair(cur, 0));

            // Construct the map of dummy node.
            for(int i = 0; i < unfinishedTargets.size(); i++) {
                dummy_dist_map.insert(std::make_pair(unfinishedTargets[i], 99999));
            }
            dummy_dist_map.insert(std::make_pair(cur, 0));
            dummy_dist_map.insert(std::make_pair(dummy_id, 0));

            // Add dummy node to the map of the targets.
            for(int i = 0; i < unfinishedTargets.size(); i++) {
                allDistMaps[unfinishedTargets[i]][dummy_id] = 0;
            }

            allDistMaps[cur] = cur_dist_map;
            allDistMaps[dummy_id] = dummy_dist_map;


            // Writing header information
            outfile << "NAME : test_tsp_file\n";
            outfile << "COMMENT : file for LKH test\n";
            outfile << "TYPE : ATSP\n";

            // Determining the dimension
            // +2 means adding current node and dummy node.
            int dimension = unfinishedTargets.size() + 2;

            outfile << "DIMENSION : " << dimension << "\n";
            outfile << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
            outfile << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
            outfile << "EDGE_WEIGHT_SECTION\n";

            unfinishedTargets.push_back(cur);
            unfinishedTargets.push_back(dummy_id);
            std::cout << std::endl;

            // Writing the distance matrix
            for (int i = 0; i < dimension; ++i) {
                    for (int j = 0; j < dimension; ++j) {
                        //TODO: track the directly connected point here.
                            outfile << allDistMaps.at(unfinishedTargets[i]).at(unfinishedTargets[j]) << " ";
                        }
                    outfile << "\n";
                }
            outfile.close();
            std::cout << std::endl;
            return true;
        }

        // InitHeu只负责计算好targets的dijksMap，作为algo的成员变量存储起来。
        std::unordered_map<long, std::unordered_map<long, long>> ALGO::InitHeu(const std::vector<long>& targets) {
                auto tstart = std::chrono::steady_clock::now(); // 开始计时
            std::unordered_map<long, std::unordered_map<long, long>> allDistMaps;
                for (long target : _target) {
                    DijkstraScan dijks;
                    dijks.SetGraphPtr(_graph);
                    dijks.Search(target, 0); // Assume 0 is some default parameter
                    allDistMaps[target] = dijks.GetDistMap();
                }
                auto tend = std::chrono::steady_clock::now(); // 结束计时
                _res.rt_initHeu += std::chrono::duration<double>(tend - tstart).count(); // 计算运行时间
                _dijks4Targets = allDistMaps;

                // FOR TESTING.
//            rzq::basic::CostVector test_b_vec(0,17);
//            writeTSPFile(allDistMaps, 1293, test_b_vec, "test_tsp_file");
                return allDistMaps;
            }

            int ALGO::Search(long vo, long vo_ta, long vo_tb, double time_limit) {
                // ### init heu ###
                // 初始没有任务被完成，因此传入全为0的b_vec.
                InitHeu( rzq::basic::CostVector(0,_target.size()) );
                // ### init ###
                auto tstart = std::chrono::steady_clock::now(); // heuristic computation time is not counted.
                int retFlag = _InitSearch(vo, vo_ta, vo_tb);
                if (retFlag == 0) {
                    // this can happen for MO-CBS-TSD usage.
                    _res = search::ALGOResult();
                    api_res = api::APIResult();
                    return 0;
                }

                if (DEBUG_MOSIPP > 0) {
                    std::cout << "[DEBUG] Init, _lo = " << _lo << std::endl;
                }

                bool timeout = false;

                //从这里开始改！
                // ### main search loop ###
                while ( !_open.empty() ) {
                    // ## select label l, lexicographic order ##

                    // pseudocode line5. Main search loop.
                    b_Label l = b_label[ _open.begin()->second ];
                    _open.erase(_open.begin());


                    //pseudo line7-8.
                    if (l.TaskDone()) {
                        std::cout << "Task Done!" << std::endl;
                        _res.success = true;
                        _AddSols(l);
                        // ### post-process the results ###
                        auto tend = std::chrono::steady_clock::now();
                        _res.rt_search = std::chrono::duration<double>(tend-tstart).count();
                        _PostProcRes();
                        return true;
                    }
                    std::cout << "[DEBUG] ### Pop l = " << l << std::endl;
                    if (DEBUG_MOSIPP > 0) {
                        std::cout << "[DEBUG] ### Pop l = " << l << std::endl;
                    }

                    // ## lazy dominance check ##

                    // pseudocode line10. Pruned by label dominance.
                    if ( _FrontierCheck(l) ) {
                        if (DEBUG_MOSIPP > 1) {
                            std::cout << "[DEBUG] Frontier-check, dom, cont..." << std::endl;
                        }
                        continue;
                    }

                    //pseudocode line18. To filter other labels.
                    _UpdateFrontier(l);
                    if (DEBUG_MOSIPP > 1) {
                        std::cout << "[DEBUG] ### Exp. " << l << std::endl;
                    }

                    auto tnow = std::chrono::steady_clock::now();
                    auto time_passed = std::chrono::duration<double>(tnow-tstart).count();
                    if (time_passed > time_limit) {
                        if (DEBUG_MOSIPP > 1) {
                            std::cout << "[DEBUG] ### MOSIPP - TIMEOUT !! " << l << std::endl;
                        }
                        timeout = true;
                        break;
                    }

                    // ## expand label l ##
                    _Expand(l);

                } // end while

                if (DEBUG_MOSIPP > 1) {
                    std::cout << "[DEBUG] ### MOSIPP - Exit while loop" << std::endl;
                }

                // timeout, no success.
                _res.success = ! timeout;
                auto tend = std::chrono::steady_clock::now();
                _res.rt_search = std::chrono::duration<double>(tend-tstart).count();

                // ### post-process the results ###
                _PostProcRes();

                if (DEBUG_MOSIPP > 1) {
                    std::cout << "[DEBUG] ### MOSIPP - Exit Search()" << std::endl;
                }

                return int(_res.success); // TODO, extend to more return flags.
            };


            basic::CostVector ALGO::_Heuristic(b_Label& l, basic::CostVector& b_vec, std::unordered_map<long, std::unordered_map<long, long>>& allDistMaps) {
//            return basic::CostVector(0,_graph->GetCostDim());

                ////////////////////////////////////////////////////////////////////////////////
                auto tstart = std::chrono::steady_clock::now(); // 开始计时
                writeTSPFile(allDistMaps, l.id, b_vec, "test_tsp_file.tsp");
                std::cout << "Here is label " << l.id << " " << "before system call"<< std::endl; 
                system("./LKH ./test_tsp_file.par");
                long cost = getTourLength("test_tsp_file.tour");
                auto out = basic::CostVector(0, _graph->GetCostDim());
                for (size_t cdim = 0; cdim < out.size(); cdim++) {
                    if (out[cdim] < 0) {
                        throw std::runtime_error( "[ERROR], unavailable heuristic !?" );
                    }
                }
                l.mstWeight.push_back(cost);
                out[0] = cost;
                auto tend = std::chrono::steady_clock::now(); // 结束计时
                _res.rt_initHeu += std::chrono::duration<double>(tend - tstart).count(); // 计算运行时间
                return out;
                ///////////////////////////////////////////////////////////////////////////////

                std::string str = b2str(l.b_vector);
                std::vector<long> unfinishedTargets;
                for (int i = 0; i < _target.size(); ++i) {
                    if(b_vec[i] == 0) {
                        unfinishedTargets.push_back(_target[i]);
                    }
                }
                    // 使用Prim算法计算MST的总权重
                std::pair<long,long> mst_res = CalculateMSTWithPrim(l, unfinishedTargets, allDistMaps);
                long totalWeight = mst_res.first;
                long directTarget = mst_res.second;
                out[0] = totalWeight;
                l.mstWeight.push_back(totalWeight);
                l.mstConectedTarget = mst_res.second;
                return out;
            };

            ALGOResult ALGO::GetResult() const {
                return _res;
            };

            api::APIResult ALGO::GetAPIResult() const {
                return api_res;
            }

            void ALGO::SetGrid(basic::GridkConn& g) {
                temp_g = g;
                SetGraphPtr(&temp_g);
            };


            void ALGO::AddNodeCstr(long nid, long t) {
                _sss.AddNodeCstr(nid, t);
            };

            void ALGO::AddEdgeCstr(long u, long v, long t) {
                _sss.AddEdgeCstr(u, v, t);
            };

            void ALGO::SetWaitCost(const std::vector<long>& wait_cost) {
                _wait_cvec.resize(wait_cost.size());
                for (size_t ii = 0; ii < wait_cost.size(); ii++){
                    _wait_cvec[ii] = wait_cost[ii];
                    if ((0 == ii) && (wait_cost[0] != 1)) {
                        std::cout << "[WARNING] MOSIPP::SetWaitCost, the 1st dim in the cost "
                                  << "vec is transition time, wait cost is set to "
                                  << wait_cost[0] << " (non-unit), be careful!" << std::endl;
                    }
                }

                return;
            };

            void ALGO::SetTarget(std::vector<long>& target_vec) {
                _target.resize(target_vec.size());
                numt = target_vec.size();
                for (size_t ii = 0; ii < target_vec.size(); ii++){
                    _target[ii] = target_vec[ii];
                }
                if (_target.size() == 0) {
                    std::cout << "[WARNING] ALGO::SetTarget, No target!" << std::endl;
                }

                return;
            };

////////////// Protected //////////////

            long ALGO::_GenLabelId() {return _label_id_gen++;
            };

            bool ALGO::_FrontierCheck(b_Label& l) {

                // if l.v does not reach the goal v_d, need to augment the vector to consider time step.

                //ss is of the format of : [0] is the long type of vertex, [1] is ta, and [2] is tb.
                auto ss = _L2S(l); // _State2String(_Label2State(l));
                if (b_alpha.find( ss ) == b_alpha.end())                  {return false;}
                // return _alpha[ ss ].Check(AugVec(l));
                return b_alpha[ ss ].Check(l.g, l.b_vector);
            };

            bool ALGO::_BuildPath(long lid, std::vector<long>* path, std::vector<long>* times) {
                std::vector<long> out, out2;
                if (DEBUG_MOSIPP) {std::cout << "## _BuildPath first label = " << b_label[lid] << std::endl;}
                out.push_back(b_label[lid].v);
                out2.push_back(b_label[lid].t);
                while( _parent.find(lid) != _parent.end() ) {
                    if (DEBUG_MOSIPP) {std::cout << "## _BuildPath label = " << b_label[_parent[lid]] << std::endl;}
                    out.push_back(b_label[_parent[lid]].v);
                    out2.push_back(b_label[_parent[lid]].t);
                    lid = _parent[lid];
                }
                path->clear();
                times->clear();
                for (size_t i = 0; i < out.size(); i++) {
                    path->push_back(out[out.size()-1-i]);
                    times->push_back(out2[out2.size()-1-i]);
                }
                return true;
            };

            bool ALGO::_BuildState(long lid, vector<api::State>& plan) {
                while( _parent.find(lid) != _parent.end() ) {
                    api::State state;
                    state.t = b_label[lid].g[0];
                    state.c = temp_g.x(b_label[lid].v);
                    state.r = temp_g.y(b_label[lid].v);
                    //判断位置是否发生变化
                    if ( (temp_g.x(b_label[lid].v) != temp_g.x(b_label[_parent[lid]].v) ||
                         temp_g.y(b_label[lid].v) != temp_g.y(b_label[_parent[lid]].v)) && b_label[lid].b_vector == b_label[_parent[lid]].b_vector) {
                        state.a = api::MOVE;
                    }
                    else if ( b_label[lid].b_vector != b_label[_parent[lid]].b_vector ) {
                        state.a = api::SERVE;
                    }
                    else if ( b_label[lid].g[0] != b_label[_parent[lid]].g[0]) {
                        state.a = api::WAIT;
                    }
                    else if (b_label[lid].TaskDone()) {
                        state.a = api::FINISH;
                    }
                    plan.push_back(state);
                    lid = _parent[lid];
                }
                return true;
            }


            void ALGO::_Expand(b_Label& l) {
//                std::cout << "expand!" << std::endl;
                _res.n_expanded++;
                std::unordered_set<long> succs = _graph->GetSuccs(l.v);

                for (const auto& u : succs) { // loop over vertices
                    if (DEBUG_MOSIPP > 0) {
                        std::cout << "[DEBUG] >>>> Loop auto u : succs, u = " << u << std::endl;
                    }
                    std::vector<long> tas, tbs;
                    auto l2all = _sss.GetSuccBLabels(l.v, u, l.t, l.tb, l.b_vector);
                    for (auto l2 : l2all) {

                        if (DEBUG_MOSIPP > 0) {
                            std::cout << "[DEBUG] >>>> Loop v= " << u << " for loop l' = " << l2 << std::endl;
                        }

                        // get wait time and cost, generate new labels
                        long waitTime = l2.t - l.t - _sss.GetDuration(l.v, l2.v); // wait time
                        l2.id = _GenLabelId();
                        l2.g = l.g + _graph->GetCost(l.v, l2.v) + (_GetWaitCost(l.v) * waitTime);

                        //lazy heuristic的判断条件
                        if (_dijks4Targets[l.mstConectedTarget][l2.v] == _dijks4Targets[l.mstConectedTarget][l.v]
                                                                            - _graph->GetCost(l.v, l2.v)[0]) {
                            l2.f = l2.g + l.mstWeight - _graph->GetCost(l.v, l2.v);
                        }
                        else {
                            l2.f = l2.g + _Heuristic(l2, l2.b_vector, _dijks4Targets);
                        }

                        b_label[l2.id] = l2;
                        _parent[l2.id] = l.id;
                        if (DEBUG_MOSIPP > 0) {
                            std::cout << "[DEBUG] >>>> Loop v= " << u << " gen l' = " << l2 << std::endl;
                        }
                        if (_FrontierCheck(l2)) {
                            if (DEBUG_MOSIPP > 1) {
                                std::cout << "[DEBUG] ----- Frontier-Check, dom, cont..." << std::endl;
                            }
                            continue;
                        }
                        if (DEBUG_MOSIPP > 0) {
                            std::cout << "[DEBUG] >>>>  Add to open..." << std::endl;
                        }
                        _res.n_generated++;
                        _open.insert( std::make_pair(l2.f, l2.id) );
                        //以上为不执行任务的label，这是一定可以产生的.
                        //以下是若时间允许且l2为target时，则执行任务的label.
                        //g[0]表示时间.

                        //判断l2对应节点是否为target.
                        auto it = std::find(_target.begin(), _target.end(), l2.v);
                        if(it != _target.end()){
                            long index = std::distance(_target.begin(), it);
                            if (l2.g[0] + l2.ServiceCost < l2.tb && l2.b_vector[index] == 0) {
                                b_Label l3 = l2;
                                l3.id = _GenLabelId();
                                l3.g[0] = l2.g[0] + l2.ServiceCost;
                                l3.b_vector[index] = 1;//此处改过！
                                l3.f = l3.g + _Heuristic(l3, l3.b_vector, _dijks4Targets);
                                l3.t += l3.ServiceCost;
                                l3.serveFlag = true;
                                b_label[l3.id] = l3;
                                //pseudocode line27. _parent is used to restore the path.
                                _parent[l3.id] = l.id;
                                //以下检验l3.
                                if (DEBUG_MOSIPP > 0) {
                                    std::cout << "[DEBUG] >>>> Loop v= " << u << " gen l' = " << l3 << std::endl;
                                }
                                if (_FrontierCheck(l3)) {
                                    if (DEBUG_MOSIPP > 1) {
                                        std::cout << "[DEBUG] ----- Frontier-Check, dom, cont..." << std::endl;
                                    }
                                    continue;
                                }
                                if (DEBUG_MOSIPP > 0) {
                                    std::cout << "[DEBUG] >>>>  Add to open..." << std::endl;
                                }
                                _res.n_generated++;
                                _open.insert( std::make_pair(l3.f, l3.id) );
                            }
                        }
//                        std::cout << "execution++" << std::endl;
                    } // end for l2,l3.
                } // end for u
                return ;
            };

            int ALGO::_InitSearch(long vo, long vo_ta, long vo_tb) {

                _vo = vo;

                basic::CostVector zero_vec, init_b_vector;
                zero_vec.resize(_graph->GetCostDim(), 0);

                init_b_vector.resize(numt, 0);
//                init_b_vector[vo] = 1; //默认出发点的任务已完成.
                // about starting label.
                if ((vo_ta == -1) && (vo_tb == -1)) {
                    long tta, ttb;
                    bool ret_flag = _sss.FindSafeItv(vo, 0, &tta, &ttb);
                    if (!ret_flag) {
                        std::cout << "[ERROR] MOSIPP::_InitSearch, fail to find start interval for input, vo=" << vo
                                  << "itv[" << vo_ta << "," << vo_tb << "]" << std::endl;
                        throw std::runtime_error("[ERROR] MOSIPP::_InitSearch, fail to find start interval for input !");
                    }

                    // pseudocode line1.
                    _lo = b_Label(_GenLabelId(), vo, zero_vec, _Heuristic(_lo, init_b_vector, _dijks4Targets), tta, ttb, init_b_vector); // initial label.
                }
                else {
                    std::cout << "[ERROR] MOSIPP::_InitSearch, unacceptable input, vo=" << vo
                              << "itv[" << vo_ta << "," << vo_tb << "]" << std::endl;
                    throw std::runtime_error("[ERROR] MOSIPP::_InitSearch, unacceptable input!");
                }
                b_label[_lo.id] = _lo;
                _res.n_generated++;

                // pseudocode line2.
                _open.insert( std::make_pair(_lo.f, _lo.id) );
                return 1;
            };

//State is a CostVector type where [0] is the long type of vertex, [1] is ta, and [2] is tb.
            basic::CostVector ALGO::_Label2State(const b_Label& l) {
                auto out = basic::CostVector(0,3+l.b_vector.size());
                long ta=-1, tb=-1;
                auto ret_flag = _sss.FindSafeItv(l.v, l.t, &ta, &tb);

                if (!ret_flag) {
                    std::cout << "[ERROR] MOSIPP::_Label2State, input label " << l << " overlaps with obstacles." << std::endl;
                    throw std::runtime_error( "[ERROR] MOSIPP::_Label2State, input label overlaps with obstacles !?" );
                }

                if (ENABLE_SAFE_CHECK_MOSIPP && tb < ta) {
                    std::cout << "[ERROR] MOSIPP::_Label2State, FindSafeItv " << ta << tb << ", tb < ta !?" << std::endl;
                    throw std::runtime_error( "[ERROR] MOSIPP::_Label2State, FindSafeItv, tb < ta !? !?" );
                }

                out[0] = l.v;
                out[1] = ta;
                out[2] = tb;
                /**
                 *  此处循环次数应等于target的数量.
                 *  增加以下out[i]是为了区分b_vec不同的label.
                 */
                for(int j = 0, i = 3; i < 3+l.b_vector.size(); i++, j++) {
                    out[i] = l.b_vector[j];
                }
                return out;
            };

            std::string ALGO::_State2String(const basic::CostVector& g) const{
                std::string out;
                for (int i = 0; i < g.size(); i++) {
                    out += std::to_string(g[i]);
                    if (i != g.size()-1) {
                        out += ",";
                    }
                }
                return out;
            };

            basic::CostVector ALGO::_GetWaitCost(long u) const{
                return _wait_cvec;
            };
//TODO: _UpdateFrontier_LinearSearch. _UpdateFrontier_LinearMap.
            void ALGO::_UpdateFrontier(b_Label& l) {
                std::string s = _L2S(l);
                /**
                 * 把下面的if判断去掉了，意义在于可以重复访问同一个节点。
                 */
                if (b_alpha.find(s) == b_alpha.end()) {
                    if (DEBUG_MOSIPP > 2) {
                        std::cout << "[DEBUG] new Frontier for label " << l << std::endl;
                    }
                    // _alpha[s] = new Frontier;
                    b_alpha[s] = FrontierLinear();
                    b_alpha[s].SetWaitCost(_wait_cvec);
                }

                if (DEBUG_MOSIPP > 2) {
                    std::cout << "[DEBUG] new Frontier for label " << l << std::endl;
                }
                // _alpha[s] = new Frontier;
//                b_alpha[s] = FrontierLinear();
//                b_alpha[s].SetWaitCost(_wait_cvec);
                b_alpha[s].Update(l); //AugVec(l));

                if (DEBUG_MOSIPP > 1) {
                    std::cout << "[DEBUG] ----->> UpdateF. label ids = {";
                    for (auto iter : b_alpha[s].b_labels) {
                        std::cout << iter.first << ",";
                    }
                    std::cout << "} " << std::endl;
                }
            };

            void ALGO::_AddSols(const b_Label& l) {

                // filter sol, not necessary in this impl.
                std::vector<long> new_sol_id;
                for (auto lid : _sol_label_ids){
                    // EpsDominance true => l dominates all labels in sol_label_ids.
                    if (EpsDominance(l.g, b_label[lid].g, l.b_vector, b_label[lid].b_vector)) {
                        continue;
                    }else{
                        new_sol_id.push_back(lid);
                    }
                }

                // if at least one of the solutions in original sol_label_ids pruned by l, add l into the sol_labels_ids.
                if (new_sol_id.size() < _sol_label_ids.size()) {
                    _sol_label_ids = new_sol_id;
                }

                // add sol
                _sol_label_ids.push_back(l.id);
                return ;
            };


            void ALGO::_PostProcRes() {
                // ### post-process the results ###
                if (_sol_label_ids.size() > 0) {
                    for (auto lid : _sol_label_ids) {
                        _res.paths[lid] = std::vector<long>();
                        _res.times[lid] = std::vector<long>();
                        bool ret_flag = _BuildPath(lid, &(_res.paths[lid]), &(_res.times[lid]) );
                        // when this flag is used, remember to check here for safety.
                        _res.costs[lid] = b_label[lid].g;
                        _res.totalCost = _res.costs[lid][0];
                        _BuildState(lid, api_res.plan);
                    }
                }
                return ;
            }

        long ALGO::getTourLength(const std::string &filename) {
            std::ifstream file(filename);
            if (!file.is_open()) {
                std::cerr << "Unable to open file: " << filename << std::endl;
                return -1;  // 表示错误
            }

            std::string line;
            long length = -1;  // 初始化为-1，表示尚未找到长度

            while (std::getline(file, line)) {
                if (line.find("Length =") != std::string::npos) {
                    // 找到包含 "Length =" 的行
                    size_t pos = line.find("=");
                    if (pos != std::string::npos) {
                        // 提取等号后面的值，并转换为整数
                        std::string lengthStr = line.substr(pos + 1);
                        length = std::stol(lengthStr);
                        break;  // 找到长度后可以退出循环
                    }
                }
            }

            file.close();
            return length;
        }

/////////////////////////////////////////////////////////////////
////////////////// RunALGO /////////////////////
/////////////////////////////////////////////////////////////////

            std::vector< std::vector<long> > RunALGOGrid(
                basic::GridkConn& g, long vo, double time_limit,
                basic::CostVector& wait_cost, std::vector< std::vector<long> >& ncs,
                std::vector< std::vector<long> >& ecs, ALGOResult* res, api::APIResult* api_res, std::vector<long>& targetVec)
            {
                ALGO algo;
                algo.SetTarget(targetVec);
                algo.SetGrid(g); // polymorphism
                algo.SetWaitCost(wait_cost);
                for (auto nc: ncs) {
                    algo.AddNodeCstr(nc[0], nc[1]);
                }
                for (auto ec: ecs) {
                    algo.AddEdgeCstr(ec[0], ec[1], ec[2]);
                }
                int outFlag = algo.Search(vo, -1, -1, time_limit);
                if (!outFlag) {std::cout << "[ERROR] Search failed!" << std::endl; assert(0);}
                *res = algo.GetResult();
                *api_res = algo.GetAPIResult();
                //TODO: Fix the following codes.
                for (auto iter : res->paths) {
                    long k = iter.first; // id of a Pareto-optipmal solution
                    // path nodes
                    std::vector<long> nodes;
                    std::vector<long> times;
                    for (auto xx : res->paths[k]) {
                        nodes.push_back(xx);
                    }
                    // times
                    for (auto xx : res->times[k]) {
                        times.push_back(xx);
                    }

                    for(int i = 0; i < nodes.size(); i++) {
                        std::vector<long> coming_ncs({nodes[i],times[i]});
                        if (std::find(ncs.begin(), ncs.end(),coming_ncs) == ncs.end()) {
                            ncs.emplace_back(coming_ncs);
                        }
                        else {
                            continue;
                        }
                    }
                }
                for (auto ii : targetVec) {
                    res->targets.push_back(ii);
                }
                return ncs;
            };

        } // end namespace search
} // end namespace rzq