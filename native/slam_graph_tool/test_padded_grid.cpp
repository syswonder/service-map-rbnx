#include "padded_grid.h"
#include <memory>
#include <iostream>
int main() {
  if (PaddedGrid::Render(karto::LocalizedRangeScanVector{},.05,2,.1)!=nullptr) return 2;
  auto* laser=karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom,karto::Name(std::string("endpoint-regression")));
  laser->SetMinimumRange(.1);laser->SetMaximumRange(10.);laser->SetRangeThreshold(10.);
  laser->SetMinimumAngle(-karto::KT_PI);laser->SetMaximumAngle(karto::KT_PI/2.);
  laser->SetAngularResolution(karto::KT_PI/2.);
  karto::SensorManager::GetInstance()->RegisterSensor(laser);
  karto::LocalizedRangeScanVector scans;
  for(int i=0;i<4;++i){
    auto* scan=new karto::LocalizedRangeScan(laser->GetName(),std::vector<kt_double>{2.,2.,2.,2.});
    scan->SetOdometricPose(karto::Pose2(0.,0.,0.));scan->SetCorrectedPose(karto::Pose2(0.,0.,0.));
    scans.push_back(scan);
  }
  std::unique_ptr<karto::OccupancyGrid> grid(PaddedGrid::Render(scans,.05,2,.1));
  for(auto point:{karto::Vector2<kt_double>(-2.,0.),karto::Vector2<kt_double>(2.,0.),karto::Vector2<kt_double>(0.,-2.),karto::Vector2<kt_double>(0.,2.)}){
    auto cell=grid->WorldToGrid(point);
    if(!grid->IsValidGridIndex(cell)||grid->GetValue(cell)!=karto::GridStates_Occupied){std::cerr<<"wall endpoint lost at boundary\n";return 1;}
  }
  for(auto* scan:scans)delete scan;
  std::cout<<"PASS: all four observed boundary walls retained\n";
  return 0;
}
