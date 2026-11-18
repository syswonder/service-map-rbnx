// SPDX-License-Identifier: MulanPSL-2.0
// Read-only per-node observation audit for local rebuilding experiments.
#include <karto_sdk/Mapper.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <cmath>
#include <sys/resource.h>
int main(int argc,char**argv){
 if(argc!=7)return 2;
 try{
  rlimit stack;getrlimit(RLIMIT_STACK,&stack);stack.rlim_cur=std::max(stack.rlim_cur,rlim_t(40000000));setrlimit(RLIMIT_STACK,&stack);
  std::string stem=argv[1];double x0=std::stod(argv[3]),y0=std::stod(argv[4]),x1=std::stod(argv[5]),y1=std::stod(argv[6]);
  karto::Mapper mapper;karto::Dataset dataset;mapper.LoadFromFile(stem+".posegraph");dataset.LoadFromFile(stem+".data");
  std::set<karto::Name> registered;for(auto laser:dataset.GetLasers()){auto s=dynamic_cast<karto::Sensor*>(laser);if(registered.insert(s->GetName()).second)karto::SensorManager::GetInstance()->RegisterSensor(s);}
  std::ofstream out(argv[2]);out<<std::setprecision(17);
  for(auto scan:mapper.GetAllProcessedScans()){
   auto ranges=scan->GetRangeReadingsVector();auto points=scan->GetPointReadings(false);auto laser=scan->GetLaserRangeFinder();auto p=scan->GetCorrectedPose();
   out<<"{\"id\":"<<scan->GetUniqueId()<<",\"pose\":["<<p.GetX()<<","<<p.GetY()<<","<<p.GetHeading()<<"],\"nan_indices\":[";
   bool first=true;for(size_t i=0;i<ranges.size();++i)if(std::isnan(ranges[i])){if(!first)out<<",";first=false;out<<i;}
   out<<"],\"region_endpoints\":[";first=true;
   for(size_t i=0;i<ranges.size();++i){double r=ranges[i];if(!std::isfinite(r)||r<=laser->GetMinimumRange()||r>=laser->GetMaximumRange()||r>=laser->GetRangeThreshold()-karto::KT_TOLERANCE)continue;
    auto pt=points[i];if(pt.GetX()<x0||pt.GetX()>x1||pt.GetY()<y0||pt.GetY()>y1)continue;
    if(!first)out<<",";first=false;out<<"["<<pt.GetX()<<","<<pt.GetY()<<"]";
   }out<<"]}\n";
  }return out?0:1;
 }catch(const karto::Exception&e){std::cerr<<e.GetErrorMessage()<<"\n";return 1;}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}
}
