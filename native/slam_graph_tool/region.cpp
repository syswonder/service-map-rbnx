// SPDX-License-Identifier: MulanPSL-2.0
// Edit historical observations on a private map branch; retain graph topology.
#include <karto_sdk/Mapper.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <sys/resource.h>

int main(int argc,char** argv) {
 const bool verify=argc==3&&std::string(argv[1])=="--verify-pair";
 if(argc!=8&&!verify){std::cerr<<"usage: slam_region_tool INPUT_STEM OUTPUT_DIR XMIN YMIN XMAX YMAX MARGIN\n";return 2;}
 try {
  rlimit stack;getrlimit(RLIMIT_STACK,&stack);stack.rlim_cur=std::max(stack.rlim_cur,rlim_t(40000000));setrlimit(RLIMIT_STACK,&stack);
  if(verify){
   std::string stem=argv[2];karto::Mapper mapper;karto::Dataset dataset;
   mapper.LoadFromFile(stem+".posegraph");dataset.LoadFromFile(stem+".data");
   std::set<karto::Name> registered;
   for(auto laser:dataset.GetLasers()){auto sensor=dynamic_cast<karto::Sensor*>(laser);if(registered.insert(sensor->GetName()).second)karto::SensorManager::GetInstance()->RegisterSensor(sensor);}
   for(auto scan:mapper.GetAllProcessedScans()){
    auto found=dataset.GetData().find(scan->GetUniqueId());
    if(found==dataset.GetData().end())throw std::runtime_error("saved scan missing from dataset");
    auto stored=dynamic_cast<karto::LocalizedRangeScan*>(found->second);
    if(!stored)throw std::runtime_error("invalid saved dataset scan");
    auto a=scan->GetRangeReadingsVector(),b=stored->GetRangeReadingsVector();
    if(a.size()!=b.size())throw std::runtime_error("saved scan lengths disagree");
    for(size_t i=0;i<a.size();++i)if(!(a[i]==b[i]||(std::isnan(a[i])&&std::isnan(b[i]))))throw std::runtime_error("saved scan representations disagree");
   }
   std::cout<<"PASS: Mapper and Dataset range masks agree\n";return 0;
  }
  std::string stem=argv[1],out=argv[2];double x0=std::stod(argv[3]),y0=std::stod(argv[4]),x1=std::stod(argv[5]),y1=std::stod(argv[6]),margin=std::stod(argv[7]);
  for(double v:{x0,y0,x1,y1,margin})if(!std::isfinite(v))throw std::runtime_error("region must be finite");
  if(x0>=x1||y0>=y1||margin<0||margin>1)throw std::runtime_error("invalid region or margin");
  karto::Mapper mapper;karto::Dataset dataset;mapper.LoadFromFile(stem+".posegraph");dataset.LoadFromFile(stem+".data");
  std::set<karto::Name> registered;
  for(auto laser:dataset.GetLasers()){
   auto sensor=dynamic_cast<karto::Sensor*>(laser);if(!sensor)throw std::runtime_error("invalid laser metadata");
   if(registered.insert(sensor->GetName()).second)karto::SensorManager::GetInstance()->RegisterSensor(sensor);
  }
  auto intersects=[&](double ox,double oy,double ex,double ey){
   double lo=0.,hi=1.;
   for(int axis=0;axis<2;++axis){double a=axis?oy:ox,b=axis?ey:ex,mn=(axis?y0:x0)-margin,mx=(axis?y1:x1)+margin,d=b-a;
    if(std::abs(d)<1e-12){if(a<mn||a>mx)return false;}
    else{double t0=(mn-a)/d,t1=(mx-a)/d;if(t0>t1)std::swap(t0,t1);lo=std::max(lo,t0);hi=std::min(hi,t1);if(lo>hi)return false;}
   }return true;
  };
  size_t masked=0,affected=0,empty=0,valid_before=0,valid_after=0;
  std::ofstream changes(out+"/region-scans.jsonl");
  // Buffers can hold references to scans that become empty. The first mapping
  // scan must be re-anchored against verified retained observations on restart.
  for(auto name:mapper.GetMapperSensorManager()->GetSensorNames()){
   mapper.GetMapperSensorManager()->ClearRunningScans(name);mapper.GetMapperSensorManager()->ClearLastScan(name);
  }
  for(auto scan:mapper.GetAllProcessedScans()){
   auto ranges=scan->GetRangeReadingsVector();auto points=scan->GetPointReadings(false);auto origin=scan->GetSensorPose().GetPosition();auto laser=scan->GetLaserRangeFinder();size_t changed=0,valid=0;
   if(ranges.size()!=points.size())throw std::runtime_error("inconsistent scan cache");
   for(size_t i=0;i<ranges.size();++i){double r=ranges[i];if(!std::isfinite(r)||r<=laser->GetMinimumRange()||r>=laser->GetMaximumRange())continue;
    ++valid_before;double ex=points[i].GetX(),ey=points[i].GetY();if(!std::isfinite(ex)||!std::isfinite(ey))throw std::runtime_error("nonfinite endpoint");
    if(r>laser->GetRangeThreshold()){double scale=laser->GetRangeThreshold()/r;ex=origin.GetX()+(ex-origin.GetX())*scale;ey=origin.GetY()+(ey-origin.GetY())*scale;}
    if(intersects(origin.GetX(),origin.GetY(),ex,ey)){ranges[i]=std::numeric_limits<double>::quiet_NaN();++changed;}else ++valid;
   }
   if(changed){
    scan->SetRangeReadings(ranges);scan->SetCorrectedPoseAndUpdate(scan->GetCorrectedPose());
    // The two archives are loaded independently, so Dataset can own a second
    // scan object with the same ID. Edit BOTH serialized representations.
    auto found=dataset.GetData().find(scan->GetUniqueId());
    if(found==dataset.GetData().end())throw std::runtime_error("scan missing from dataset");
    auto stored=dynamic_cast<karto::LocalizedRangeScan*>(found->second);
    if(!stored)throw std::runtime_error("dataset entry is not a localized scan");
    if(stored!=scan){stored->SetRangeReadings(ranges);stored->SetCorrectedPoseAndUpdate(scan->GetCorrectedPose());}
    masked+=changed;++affected;
   }
   if(valid==0)++empty;valid_after+=valid;
   changes<<"{\"id\":"<<scan->GetUniqueId()<<",\"masked\":"<<changed<<",\"remaining_valid\":"<<valid<<"}\n";
  }
  if(masked==0)throw std::runtime_error("region does not intersect any retained ray");
  if(valid_after<100)throw std::runtime_error("too little observation remains for localization");
  mapper.SaveToFile(out+"/posegraph.posegraph");dataset.SaveToFile(out+"/posegraph.data");
  std::ofstream report(out+"/region-native.json");report<<std::setprecision(17)<<"{\"bounds\":["<<x0<<","<<y0<<","<<x1<<","<<y1<<"],\"margin_m\":"<<margin<<",\"masked_rays\":"<<masked<<",\"affected_scans\":"<<affected<<",\"empty_scans\":"<<empty<<",\"valid_before\":"<<valid_before<<",\"valid_after\":"<<valid_after<<",\"policy\":\"invalidate_intersecting_ray\"}\n";
  if(!changes||!report)throw std::runtime_error("write failed");return 0;
 }catch(const karto::Exception&e){std::cerr<<e.GetErrorMessage()<<"\n";return 1;}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}
}
