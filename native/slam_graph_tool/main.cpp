// SPDX-License-Identifier: MulanPSL-2.0
// Cold-load verifier and renderer. No ROS publishers or optimizer side effects.
#include <karto_sdk/Mapper.h>
#include "padded_grid.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <queue>
#include <set>
#include <cmath>
#include <stdexcept>
#include <sys/resource.h>

int main(int argc, char **argv) {
  if (argc != 4) { std::cerr << "usage: slam_graph_tool STEM OUTPUT_DIR RESOLUTION\n"; return 2; }
  try {
    rlimit stack; getrlimit(RLIMIT_STACK, &stack);
    if (stack.rlim_cur < 40000000) { stack.rlim_cur = 40000000; setrlimit(RLIMIT_STACK, &stack); }
    const std::string stem=argv[1], out=argv[2];
    const double resolution=std::stod(argv[3]);
    if (!std::isfinite(resolution) || resolution<=0) throw std::runtime_error("invalid resolution");
    karto::Mapper mapper;
    karto::Dataset dataset;
    mapper.LoadFromFile(stem+".posegraph");
    dataset.LoadFromFile(stem+".data");
    if (dataset.GetLasers().empty()) throw std::runtime_error("no laser metadata");
    // Continuing a graph can retain several laser objects with the same
    // sensor name in Dataset. The upstream loader uses the first one. Do not
    // register the duplicate names again when cold-validating a resumed graph.
    std::set<karto::Name> registered;
    for (auto laser:dataset.GetLasers()) {
      auto sensor=dynamic_cast<karto::Sensor *>(laser);
      if (!sensor) throw std::runtime_error("invalid laser object");
      if (registered.insert(sensor->GetName()).second)
        karto::SensorManager::GetInstance()->RegisterSensor(sensor);
    }
    std::map<int,std::set<int>> adjacency;
    std::set<int> ids;
    std::ofstream nodes(out+"/nodes.jsonl");
    nodes << std::setprecision(17);
    for (auto &sensor:mapper.GetGraph()->GetVertices()) for (auto &entry:sensor.second) {
      if (!entry.second) throw std::runtime_error("null graph vertex");
      auto scan=entry.second->GetObject();
      const auto p=scan->GetCorrectedPose();
      if (!std::isfinite(p.GetX()) || !std::isfinite(p.GetY()) || !std::isfinite(p.GetHeading()))
        throw std::runtime_error("nonfinite graph pose");
      if (!ids.insert(scan->GetUniqueId()).second) throw std::runtime_error("duplicate node id");
      nodes << "{\"id\":" << scan->GetUniqueId() << ",\"pose\":[" << p.GetX() << ',' << p.GetY() << ',' << p.GetHeading() << "]}\n";
    }
    if (ids.empty()) throw std::runtime_error("empty graph");
    for (auto edge:mapper.GetGraph()->GetEdges()) {
      if (!edge) throw std::runtime_error("null graph edge");
      int a=edge->GetSource()->GetObject()->GetUniqueId(), b=edge->GetTarget()->GetObject()->GetUniqueId();
      if (!ids.count(a) || !ids.count(b)) throw std::runtime_error("dangling edge");
      adjacency[a].insert(b); adjacency[b].insert(a);
    }
    size_t components=0; auto remaining=ids;
    while (!remaining.empty()) {
      ++components; std::queue<int> q; q.push(*remaining.begin()); remaining.erase(q.front());
      while (!q.empty()) {int a=q.front();q.pop();for(int b:adjacency[a]) if(remaining.erase(b)) q.push(b);}
    }
    std::unique_ptr<karto::OccupancyGrid> grid(PaddedGrid::Render(
      mapper.GetAllProcessedScans(), resolution,
      static_cast<kt_int32u>(mapper.getParamMinPassThrough()), mapper.getParamOccupancyThreshold()));
    if (!grid || grid->GetWidth()<=0 || grid->GetHeight()<=0) throw std::runtime_error("empty grid");
    std::ofstream pgm(out+"/occupancy.pgm",std::ios::binary);
    pgm << "P5\n" << grid->GetWidth() << ' ' << grid->GetHeight() << "\n255\n";
    size_t known=0;
    for(int y=grid->GetHeight()-1;y>=0;--y) for(int x=0;x<grid->GetWidth();++x) {
      auto value=grid->GetValue(karto::Vector2<kt_int32s>(x,y));
      unsigned char pixel=value==karto::GridStates_Occupied?0:value==karto::GridStates_Free?254:205;
      if(value!=karto::GridStates_Unknown) ++known;
      pgm.write(reinterpret_cast<const char *>(&pixel),1);
    }
    const auto offset=grid->GetCoordinateConverter()->GetOffset();
    std::ofstream yaml(out+"/occupancy.yaml");
    yaml << std::setprecision(17) << "image: occupancy.pgm\nresolution: " << resolution
         << "\norigin: [" << offset.GetX() << ", " << offset.GetY()
         << ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
    std::ofstream stats(out+"/graph.json");
    stats << std::setprecision(17) << "{\"nodes\":" << ids.size() << ",\"edges\":"
          << mapper.GetGraph()->GetEdges().size() << ",\"components\":" << components
          << ",\"known_area_m2\":" << known*resolution*resolution << ",\"resolution\":" << resolution << "}\n";
    if (!nodes || !pgm || !yaml || !stats) throw std::runtime_error("output write failed");
    return 0;
  } catch(const karto::Exception &e) {std::cerr << e.GetErrorMessage() << '\n';return 1;}
    catch(const std::exception &e) {std::cerr << e.what() << '\n';return 1;}
    catch(...) {std::cerr << "graph decode failed\n";return 1;}
}
