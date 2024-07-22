//#include "mosipp.hpp"
#include "Algo.hpp"
#include "data_load.hpp"
#include <ctime>
#include <sstream>

void MovingAI();
int SimpleTest();
void MovingAI();
void TargetGenerator(std::vector<long>& init_target, long numTarget, long maxVal);
bool DataBaseOutput(std::vector<std::vector<long>> ncs, std::string file);
bool LoadNodeConstraints(std::vector<std::vector<long>>& node_constraints, const std::string& file);

//int main() {
////    Test2by2Grid();
////    SimpleTest();
//    MovingAI();
//    return 0;
//}


/*
 * /////////////////////////////////////////////////////
 */
bool LoadNodeConstraints(std::vector<std::vector<long>>& node_constraints, const std::string& file) {
    std::ifstream f(file);
    if (!f.is_open()) {
        std::cerr << "[Error]: Unable to open file " << file << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(f, line)) {
        std::vector<long> addncs;
        std::stringstream ss(line);
        std::string value;

        while (std::getline(ss, value, ',')) {
            try {
                long num = std::stol(value);
                addncs.push_back(num);
            } catch (const std::exception& e) {
                std::cerr << "[Error]: Invalid number in the file." << std::endl;
                return false;
            }
        }

        node_constraints.push_back(addncs);
    }

    f.close();
    return true;
}
/*
 * /////////////////////////////////////////////////////
 */

bool DataBaseOutput(std::vector<std::vector<long>> ncs, std::string file) {
    /*
     * 注释掉的为无洗数据版本.
     */
//    // 打开一个文件流来写入文件
//    std::ofstream f(file);
//
//    // 检查文件是否成功打开
//    if (!f.is_open()) {
//        std::cerr << "Error: Unable to open file." << std::endl;
//        return false;
//    }
//
//    // 遍历数据并写入文件
//    for (const auto& row : ncs) {
//        for (size_t i = 0; i < row.size(); ++i) {
//            f << row[i];
//            if (i < row.size() - 1)
//                f << ","; // 在每个值后面添加逗号，除了行的最后一个值
//        }
//        f << "\n"; // 每一行数据后添加换行符
//    }
//
//    // 关闭文件
//    f.close();
//
//    return true;

    // 使用集合来消除重复的行
    std::set<std::string> uniqueRows;

    // 将每行转换为字符串并添加到集合中
    for (const auto& row : ncs) {
        std::ostringstream oss;
        for (size_t i = 0; i < row.size(); ++i) {
            oss << row[i];
            if (i < row.size() - 1)
                oss << ",";
        }
        uniqueRows.insert(oss.str());
    }

    // 打开一个文件流来写入文件
    std::ofstream f(file);
    if (!f.is_open()) {
        std::cerr << "Error: Unable to open file." << std::endl;
        return false;
    }

    // 遍历集合并写入文件
    for (const auto& row : uniqueRows) {
        f << row << "\n";
    }

    // 关闭文件
    f.close();

    return true;
};

void TargetGenerator(rzq::basic::GridkConn& g, std::vector<long>& init_target, long numTarget, long maxVal) {

    // Initialize random seed
    std::srand(std::time(nullptr));

    for (int i = 0; i < numTarget; ++i) {
        long randomVal = std::rand() % maxVal; // Generate a random number between 0 and 2400
        if (g.GetGridValue(randomVal) == 0) {
            init_target.push_back(randomVal);
        }
        else {
            i--;
        }
    }

    return ;
}


void MovingAI() {
    std::string mapfile = "../maps/arena.map";
    std::string scenfile = "../scens/arena.map.scen";

    rzq::basic::GridkConn g(mapfile);
    movingai::scenario_manager scenmgr;
    scenmgr.load_scenario(scenfile);

    rzq::basic::CostVector wait_cost(1,1); // all values=1, length=1.

    // no constraint, traditional shortest path
    std::vector< std::vector<long> > node_constraints = {};
    std::vector< std::vector<long> > edge_constraints = {};

    double time_limit = 999999; // seconds
    for (int i=0; i<scenmgr.num_experiments(); i++) {
        auto exp = scenmgr.get_experiment(i);
        long sx, sy, gx, gy, vo, vd;
//        vo = g.v(0,0);
//        vd = g.v(0,0);
        sx = exp->startx();
        sy = exp->starty();
        vo = g.v(sy,sx);
        gx = exp->goalx();
        gy = exp->goaly();
        vd = g.v(gy,gx);
        std::vector<long> init_target;
        rzq::search::ALGOResult res;
        api::APIResult api_res;
        TargetGenerator(g,init_target, 5, 49*49);
        node_constraints = rzq::search::RunALGOGrid(g, vo, time_limit, wait_cost, node_constraints,
                                                    edge_constraints, &res, &api_res, init_target);
        std::cout << res << std::endl;
        DataBaseOutput(node_constraints, "C:\\Users\\13027\\Desktop\\newAlgorithm\\data_ncs.csv");
        LoadNodeConstraints(node_constraints, "C:\\Users\\13027\\Desktop\\newAlgorithm\\data_ncs.csv");
    }
};

int SimpleTest() {
    // static environment
    rzq::basic::Grid static_world; // static obstacles appears in this 2d grid.
    int r = 3; // rows (y)
    int c = 3; // columns (x)
    static_world.Resize(r,c);
    // 下面是静态障碍物的！
    // static_world.Set(1,1,1); // set grid[y=1,x=1] = 1, a static obstacle.

    // declare cost structure.
    rzq::basic::Grid cost1;
    cost1.Resize(r,c,1); // each action takes unit time.
    std::vector<rzq::basic::Grid> cost_grids; // cost vectors (implemented at nodes rather than edges), arrival cost at a node.
    cost_grids.push_back(cost1); // cost_grids[0] = e.g. traversal time cost,

    // workspace graph (static)
    rzq::basic::GridkConn g; // this is the graph (impl as a grid) that represents the workspace
    g.Init(static_world, cost_grids);

    // dynamic obstacles along known trajectories are represented as node/edge constraints
    std::vector< std::vector<long> > node_constraints;
    // each element must be of length two in format: [node_id, time_step].
    // node_id = y*num_cols + x; time_step = the time step when the obstacle appears.
    node_constraints.emplace_back( std::vector<long>({6,1}) ); // node id = 6 (i.e. y=2,x=0), t=1
    node_constraints.emplace_back( std::vector<long>({6,2}) ); // t=2
    node_constraints.emplace_back( std::vector<long>({6,3}) ); //
    // node_constraints.emplace_back( std::vector<long>({6,4}) ); // node id =6 is blocked for a few consecutive time steps.

    std::vector< std::vector<long> > edge_constraints; // This is not massively tested yet... place holder...

    std::vector<long> test_target;
    test_target.push_back(1);
    test_target.push_back(2);
    test_target.push_back(3);
    test_target.push_back(7);
    //TODO: Modify the followings.

    long vo = 0; // start node id.
    long vd = 0; // goal node id.
    double time_limit = 999999; // seconds
    rzq::basic::CostVector wait_cost(1,1); // all values=1, length=1. 这里只考虑时间一个cost维度。
    long timeSum = 0; //每次都加上当前次序搜索用的时间。
    rzq::search::ALGOResult res;
    api::APIResult api_res;
    rzq::search::RunALGOGrid(g, vo, time_limit, wait_cost, node_constraints, edge_constraints, &res, &api_res, test_target);
    std::cout << res << std::endl;

    // print paths, times and costs
    std::cout << " reprint solutions for more clarity:" << std::endl;
    for (auto iter : res.paths) {
        long k = iter.first; // id of a Pareto-optipmal solution
        // path nodes
        std::cout << " path nodes = ";
        for (auto xx : res.paths[k]) {
            std::cout << xx << ", ";
        }
        std::cout << std::endl;
        // times
        std::cout << " times = ";
        for (auto xx : res.times[k]) {
            std::cout << xx << ", ";
        }
        std::cout << std::endl;
        // cost
        std::cout << " cost = " << res.costs[k] << std::endl;
    }
    return 1;
}

int Test2by2Grid() {
    // static environment
    rzq::basic::Grid static_world; // static obstacles appears in this 2d grid.
    int r = 2; // rows (y)
    int c = 2; // columns (x)
    static_world.Resize(r,c);
    // 下面是静态障碍物的！
    // static_world.Set(1,1,1); // set grid[y=1,x=1] = 1, a static obstacle.

    // declare cost structure.
    rzq::basic::Grid cost1;
    cost1.Resize(r,c,1); // each action takes unit time.
    std::vector<rzq::basic::Grid> cost_grids; // cost vectors (implemented at nodes rather than edges), arrival cost at a node.
    cost_grids.push_back(cost1); // cost_grids[0] = e.g. traversal time cost,

    // workspace graph (static)
    rzq::basic::GridkConn g; // this is the graph (impl as a grid) that represents the workspace
    g.Init(static_world, cost_grids);

    // dynamic obstacles along known trajectories are represented as node/edge constraints
    std::vector< std::vector<long> > node_constraints;
    // each element must be of length two in format: [node_id, time_step].
    // node_id = y*num_cols + x; time_step = the time step when the obstacle appears.
    node_constraints.emplace_back( std::vector<long>({1,1}) ); // node id = 1 (i.e. y=1,x=0), t=1
    node_constraints.emplace_back( std::vector<long>({1,2}) ); // t=2
    node_constraints.emplace_back( std::vector<long>({1,3}) ); //
    // node_constraints.emplace_back( std::vector<long>({6,4}) ); // node id =6 is blocked for a few consecutive time steps.

    std::vector< std::vector<long> > edge_constraints; // This is not massively tested yet... place holder...

    std::vector<long> test_target;
    test_target.push_back(1);
    test_target.push_back(2);
    test_target.push_back(3);
    test_target.push_back(7);

    //TODO: Modify the followings.

    long vo = 0; // start node id.
    long vd = 0; // goal node id.
    double time_limit = 999999; // seconds
    rzq::basic::CostVector wait_cost(1,1); // all values=1, length=1. 这里只考虑时间一个cost维度。
    long timeSum = 0; //每次都加上当前次序搜索用的时间。
    rzq::search::ALGOResult res;
    api::APIResult api_res;
    rzq::search::RunALGOGrid(g, vo, time_limit, wait_cost, node_constraints, edge_constraints, &res, &api_res, test_target);
    std::cout << res << std::endl;

    // print paths, times and costs
    std::cout << " reprint solutions for more clarity:" << std::endl;
    for (auto iter : res.paths) {
        long k = iter.first; // id of a Pareto-optipmal solution
        // path nodes
        std::cout << " path nodes = ";
        for (auto xx : res.paths[k]) {
            std::cout << xx << ", ";
        }
        std::cout << std::endl;
        // times
        std::cout << " times = ";
        for (auto xx : res.times[k]) {
            std::cout << xx << ", ";
        }
        std::cout << std::endl;
        // cost
        std::cout << " cost = " << res.costs[k] << std::endl;
    }
    return 1;
}

int main() {
 return 0;
}
