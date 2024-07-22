#ifndef MOSIPP_ALGO_HPP
#define MOSIPP_ALGO_HPP

#include <set>
#include <unordered_map>
#include <limits>
#include <list>
#include <algorithm>
#include <cassert>
#include "graph.hpp"
#include "dijkstra.hpp"
#include "avltree.hpp"
#include "Algo_util.hpp"
#include "msp.hpp"
#include "api_util.hpp"



namespace rzq{
namespace search{

#define DEBUG_MOSIPP 0
#define MOSIPP_MAX_TIME (std::numeric_limits<long>::max() / 2) // make it smaller to avoid overflow.
#define ENABLE_SAFE_CHECK_MOSIPP 1


/**
 * @brief (NEW for MO-SIPP) put time as the (M+1)-th component in the cost vector.
 * Augmented Vector.
 */
// basic::CostVector AugVec(const Label& l) ;

/**
 * @brief The search space for SIPP/ALGO.
 */
        class SIPPStateSpace {
        public:
            SIPPStateSpace();
            virtual ~SIPPStateSpace();
            virtual void SetGraphPtr(basic::Graph* g) ;
            virtual void AddNodeCstr(long nid, long t) ;
            virtual void AddEdgeCstr(long u, long v, long t) ;
            /**
             * @brief
             */
            virtual bool FindSafeItv(long nid, long t, long* ta, long* tb) ;
            /**
             * @brief Compute the transition time from u to v.
             */
            virtual long GetDuration(long u, long v) ;
            /**
             * @brief Get Successors.
             */
            virtual std::vector<b_Label> GetSuccBLabels(long u, long v, long ta, long tb, basic::CostVector& B) ;
            /**
             * @brief If violate node constraint.
             */
            virtual bool ViolateNC(long u, long t) ;
            /**
             * @brief If violate edge constraint.
             */
            virtual bool ViolateEC(long u, long v, long t) ;
        protected:
            /**
             * Return the successor intervals about moving from  (u,[ta,tb]) to v.
             * The output are two vectors of longs, the first one contains all starting time steps of intervals,
             * and the second one contains all ending time steps of intervals
             */
            virtual void _GetSuccItv(long u, long v, long ta, long tb, std::vector<long>*, std::vector<long>*) ;
            /**
             *
             */
            void _FindAllUnsafeTimeStep(long u, long v, std::vector<long> *out) ;

            basic::Graph* _graph;
            std::unordered_map<long, basic::AVLTree<long> > _avl_node; // store obstacles, unsafe time steps at a node.
            // for fast lookup unsafe time steps at a node.
            std::unordered_map<long, std::unordered_map<long, basic::AVLTree<long> > > _avl_edge; // store obstacles, unsafe time steps at an edge.
            // for fast lookup unsafe time steps at an edge.
        };

/**
 *
 */
        std::ostream& operator<<(std::ostream& os, const ALGOResult& res) ;

/**
 * @brief store all non-dominated vectors. Do linear scan for checking and updating.
 */
        class FrontierLinear{  //搞定！
        public:
            /**
             * @brief
             */
            FrontierLinear() ;
            virtual ~FrontierLinear() ;
            virtual bool Check(basic::CostVector& g, basic::CostVector& b) ;
            virtual bool Remove(const b_Label& l) ;
            virtual void Add(const b_Label& l) ; // this directly add without doing dominance check (filtering).
            virtual void Filter(b_Label& l, std::unordered_set<long> *deleted=NULL); // use l to filter existing labels.
            virtual void Update(b_Label& l, std::unordered_set<long> *deleted=NULL) ; // Filter + Add

            virtual void SetWaitCost(const basic::CostVector& c_wait) ; // must do this!
            virtual bool _LabelDom(basic::CostVector& g1, basic::CostVector& g2,
                                   basic::CostVector& b1, basic::CostVector& b2) ; // 1st dim must be transition time.

// protected: // make ostream easier.
            // std::list<basic::CostVector> _nondom; // these are all augmented cost vectors.
            // std::unordered_set<long> label_ids;

            std::unordered_map<long, b_Label> b_labels; // just g-vec is needed, TODO, this can be improved.
            basic::CostVector c_wait;
        };

/**
 * @brief As name.
 */
        class ALGO {
        public:
            ALGO() ;
            virtual ~ALGO() ;
            /**
             * @brief Set graph as pointer, can leverage polymorphism here if I want.
             */
            virtual void SetGraphPtr(basic::Graph* g) ;
            /**
             * @brief This vd must be the same as the vd in Search().
             */
            virtual std::unordered_map<long, std::unordered_map<long, long>> InitHeu(const std::vector<long>& targets);
            /**
             * @brief search for all cost-unique Pareto-optimal paths from vo to vd.
             * Arguments vo_ta, vo_tb, vd_ta, vd_tb are useless, just set to -1.
             */
            virtual int Search(long vo, long vo_ta, long vo_tb, double time_limit) ;

            virtual ALGOResult GetResult() const ;
            virtual api::APIResult GetAPIResult() const;
            /**
             * @brief
             */
            virtual void SetGrid(basic::GridkConn& g) ;

            virtual void AddNodeCstr(long nid, long t) ;

            virtual void AddEdgeCstr(long u, long v, long t) ;
            /**
             * @brief Same for all states.
             */
            virtual void SetWaitCost(const std::vector<long>& wait_cost) ;

            virtual void SetTarget(std::vector<long>& target_vec) ;

        protected:

            // return the heuristic vector from v to vd.
            virtual basic::CostVector _Heuristic(b_Label& l, basic::CostVector& b_vec, std::unordered_map<long, std::unordered_map<long, long>>& allDistMaps) ;

            virtual void _UpdateFrontier(b_Label& l) ;

            virtual long _GenLabelId() ;

            virtual bool _FrontierCheck(b_Label& l) ;

            virtual bool _BuildPath(long lid, std::vector<long>* path, std::vector<long>* times) ;

            virtual bool _BuildState(long lid, vector<api::State>& plan) ;

            /**
             * @brief Verify if a label identifies a solution path.
             */
            virtual void _Expand(b_Label& l);
            /**
             * @brief (NEW for MO-SIPP) Initialize the search.
             */
            virtual int _InitSearch(long vo, long vo_ta, long vo_tb) ;
            /**
             * @brief (NEW for MO-SIPP) Find the state (v, ta, tb) for the label.
             * Frontiers are defined at states. To simplify impl, use a vector to encode state.
             */
            virtual basic::CostVector _Label2State(const b_Label& l) ;
            virtual std::string _State2String(const basic::CostVector& g) const;
            inline std::string _L2S(const b_Label& l) { return _State2String(_Label2State(l)); };
            /**
             *
             */
            basic::CostVector _GetWaitCost(long u) const;
//            virtual bool _SolDomCheck(basic::CostVector& f) ;
            virtual void _AddSols(const b_Label& l) ;
            virtual void _PostProcRes();

            basic::Graph* _graph;
            std::unordered_map< std::string, FrontierLinear > b_alpha;
            std::vector<long> _sol_label_ids; // sol set, store all labels that arrive at goal.

            long _label_id_gen = 0;
            long _vo = -1, _vd = -1;
            std::set< std::pair< basic::CostVector, long> > _open;
            std::unordered_map<long, b_Label> b_label;
            std::unordered_map<long, long> _parent;
            ALGOResult _res;
            api::APIResult api_res;
            std::vector<MinimumSpanningTreeHeuristic> _msp;

            basic::GridkConn temp_g;

            SIPPStateSpace _sss;
            b_Label _lo;
            basic::CostVector _wait_cvec;

            long numt;
            std::vector<long> _target;

            std::unordered_map<long, std::unordered_map<long, long>> _dijks4Targets;

            bool writeTSPFile(unordered_map<long, std::unordered_map<long, long>> allDistMaps, const long cur,
                              basic::CostVector& b_vec, const string &filename);

            long getTourLength(const std::string& filename) ;
        };

/**
 * @brief a function wrapper of MOSIPP class. Make it easy to call from other C++ classes or functions.
 */
        std::vector< std::vector<long> > RunALGOGrid(
                basic::GridkConn& g, long vo, double time_limit,
                basic::CostVector& wait_cost, std::vector< std::vector<long> >& ncs,
                std::vector< std::vector<long> >& ecs, ALGOResult* res, api::APIResult* api_res, std::vector<long>& targetVec) ;

    } // end namespace search
} // end namespace rzq

#endif //MOSIPP_ALGO_HPP
