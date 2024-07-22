#include <algorithm>
#include <fstream>
#include "json/json.h"
#include "Algo_util.hpp"
namespace rzq{
    namespace search{
        bool b_Label::TaskDone() {
            return std::all_of(b_vector.begin(), b_vector.end(), [](long val) { return val == 1; });
        };

        std::ostream& operator<<(std::ostream& os, const ALGOResult& res) {
            os << "Targets:{";
            for (auto ii : res.targets) {
                os << ii << ",";
            }
            os << "}" << std::endl;
            os << "ALGO-Result:{success:" << int(res.success) << ", n_solutions:" << res.costs.size()
               << ", n_generated:" << res.n_generated
               << ", n_expanded:" << res.n_expanded << ", rt_initHeu:" << res.rt_initHeu << ", rt_search:"
               << res.rt_search << "; solutions:{";
            for (auto ii : res.paths) {
                os << ",id(" << ii.first << "),cost=" << res.costs.at(ii.first) << ",nodes{";
                for (auto k : ii.second) {
                    os << k << ",";
                }
                os << "},time{";
                for (auto k : res.times.at(ii.first)) {
                    os << k << ",";
                }
                os << "}";
            }
            os << "}}";
            os << std::endl;
            return os;
        };


        std::ostream& operator<<(std::ostream& os, const b_Label& l)
        {
            std::string s;
            s = "{id:" + std::to_string(l.id) + ",v:" + std::to_string(l.v) + ",t:"
                + std::to_string(l.t) + ",tb:" + std::to_string(l.tb) + ",g:"
                + l.g.ToStr() + ",f:" + l.f.ToStr() + "}";
            os << s;
            return os;
        };

        void ALGOResult::writeRow(std::ostream& out) {
            // write the value of fields in a format like: "100,100,0.2,109,..."
            out << n_generated << "," << n_expanded << "," << rt_initHeu << "," << rt_search << "," << rt_tot << "," <<
                totalCost << std::endl;
        }

        void ALGOResult::writeHeader(std::ostream& out) {
            out << "n_gen,n_expd,rt_heur,rt_search,rt_tot,cost" << std::endl;
        }

void load_and_parse_json(const std::string &file_name, std::vector<DataEntry> &data_entries) {
    std::ifstream file(file_name);
    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(file, root, false)) {
        std::cerr << "Failed to parse JSON\n" << reader.getFormattedErrorMessages();
        return;
    }

    const Json::Value& data = root["data"];
    for (const auto& entry : data) {
        DataEntry data_entry;
        data_entry.source = entry["source"].asInt64(); // 解析source
        const Json::Value& targetSet = entry["targetSet"]; // 解析targetSet
        for (const auto& target : targetSet) {
            data_entry.targetSet.push_back(target.asInt64());
        }

        const Json::Value& nodeConstraints = entry["node_constraints"]; // 解析node_constraints
        for (Json::ValueConstIterator it = nodeConstraints.begin(); it != nodeConstraints.end(); ++it) {
            long nodeId = std::stol(it.key().asString()); // 节点ID
            for (const auto& val : *it) {
                // 对每个约束值，创建一个包含节点ID和该值的向量，并推入node_constraints中
                std::vector<long> constraintPair = {nodeId, static_cast<long>(val.asInt64())};
                data_entry.node_constraints.push_back(constraintPair);
            }
        }

        data_entries.push_back(data_entry);
    }
}
    } // end namespace search
} // end namespace rzq
