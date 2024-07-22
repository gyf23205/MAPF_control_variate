#include <chrono>
#include "Algo.hpp"
#include "Algo_util.hpp"
#include "api_util.hpp"
//#include <fstream>
//#include <iostream>
//#include <map>
//#include <string>
//#include <vector>
using namespace std;

namespace api {

ostream &operator<<(ostream &os, const APIResult &res) {
  for (const auto &state : res.plan) {
    os << state.r << " " << state.c << " " << state.t << " "
       << desc.find(state.a)->second << endl;
  }
  return os;
}

void solve(const string &mapfile, const string &infile, const string &outfile) {
    auto t_tot_start = std::chrono::steady_clock::now();
  ofstream fout(outfile);
  APIResult API_res;
  // res.plan = {
  //     {1, 1, 0, WAIT},     // start at (1, 1) at time 0, wait
  //     {1, 1, 10, MOVE},    // wait for 10s, then move
  //     {2, 1, 11, SERVE},   // reach (2, 1), then execute task
  //     {2, 1, 12.5, MOVE},  // finish task, then move
  //     {2, 2, 13.5, SERVE}, // reach (2, 2), then execute task
  //     {2, 2, 15.5, FINISH} // finish task, all done
  // };
  // fout << res << endl;
  // fout.close();
  rzq::basic::GridkConn g(mapfile);
  rzq::basic::CostVector wait_cost(1, 1);
  std::vector<std::vector<long>> edge_constraints = {};
  double time_limit = 999999;
  std::vector<rzq::search::DataEntry> data_entries;
  rzq::search::load_and_parse_json(infile, data_entries);
  for (int i = 0; i < data_entries.size(); i++) {
    std::vector<std::vector<long>> node_constraints =
            data_entries[i].node_constraints;
    std::vector<long> target = data_entries[i].targetSet;
    long vo = data_entries[i].source;
    rzq::search::ALGOResult res;
    api::APIResult api_res;
    rzq::search::RunALGOGrid(g, vo, time_limit, wait_cost, node_constraints,
                             edge_constraints, &res, &api_res, target);
      auto t_tot_tend = std::chrono::steady_clock::now();
      res.rt_tot = std::chrono::duration<double>(t_tot_tend - t_tot_start).count();
//      fout << "General Result:" << endl;
      // 下面注释的一行是旧版本的result输出.
      //fout << res << endl;
      res.writeHeader(fout);
      res.writeRow(fout);
//      fout << "Detailed Actions:" << endl;
      fout << api_res << endl;
  }
}
} // namespace api

int main(int argc, char **argvs) {
  // call ./api <mapfile> <instancefile> <outfile>
  string mapfile = string(argvs[1]);
  string infile = string(argvs[2]);
  string outfile = string(argvs[3]);
  api::solve(mapfile, infile, outfile);
  return 0;
}
