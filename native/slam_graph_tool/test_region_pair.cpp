// SPDX-License-Identifier: MulanPSL-2.0
#include <karto_sdk/Mapper.h>
#include <filesystem>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <sys/wait.h>
#include <unistd.h>
class NoopSolver:public karto::ScanSolver{
 IdPoseVector corrections;
 public:void Configure(rclcpp::Node::SharedPtr)override{} void Compute()override{}
 const IdPoseVector& GetCorrections()const override{return corrections;}
};
int call(const std::vector<std::string>& args){
 pid_t pid=fork();if(pid<0)throw std::runtime_error("fork failed");
 if(pid==0){std::vector<char*> argv;for(auto&s:args)argv.push_back(const_cast<char*>(s.c_str()));argv.push_back(nullptr);execv(argv[0],argv.data());_exit(127);}
 int status;waitpid(pid,&status,0);return WIFEXITED(status)?WEXITSTATUS(status):128;
}
int main(int argc,char**argv){
 if(argc!=3)return 2;
 try{
  std::string tool=argv[1];std::filesystem::path root=argv[2];std::filesystem::remove_all(root);std::filesystem::create_directories(root/"edited");
  std::string source=(root/"original").string(),edited=(root/"edited/posegraph").string();
  {
   NoopSolver solver;karto::Mapper mapper;karto::Dataset dataset;mapper.SetScanSolver(&solver);
   auto* laser=karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom,karto::Name(std::string("region-fixture")));
   laser->SetMinimumRange(.1);laser->SetMaximumRange(10.);laser->SetRangeThreshold(10.);
   laser->SetMinimumAngle(-karto::KT_PI);laser->SetMaximumAngle(karto::KT_PI-karto::KT_PI/180.);laser->SetAngularResolution(karto::KT_PI/180.);
   dataset.Add(laser);
   auto* scan=new karto::LocalizedRangeScan(laser->GetName(),std::vector<kt_double>(360,2.));
   scan->SetOdometricPose(karto::Pose2(0,0,0));scan->SetCorrectedPose(karto::Pose2(0,0,0));
   if(!mapper.Process(scan))throw std::runtime_error("fixture scan rejected");dataset.Add(scan);
   mapper.SaveToFile(source+".posegraph");dataset.SaveToFile(source+".data");
  }
  if(call({tool,source,(root/"edited").string(),"1.5","-.5","2.5",".5",".05"}))throw std::runtime_error("region edit failed");
  if(call({tool,"--verify-pair",edited}))throw std::runtime_error("edited archives disagree");
  {
   karto::Mapper mapper;mapper.LoadFromFile(edited+".posegraph");auto scans=mapper.GetAllProcessedScans();
   if(scans.size()!=1||scans[0]->GetUniqueId()!=0)throw std::runtime_error("node identity changed");
   auto ranges=scans[0]->GetRangeReadingsVector();
   if(!std::isnan(ranges[180])||ranges[0]!=2.)throw std::runtime_error("wrong ray mask");
  }
  // Regression: a correct preview must not hide an unedited Dataset archive.
  std::filesystem::copy_file(source+".data",edited+".data",std::filesystem::copy_options::overwrite_existing);
  if(call({tool,"--verify-pair",edited})==0)throw std::runtime_error("stale dataset was accepted");
  std::cout<<"PASS: masks persisted in both archives; stale dataset rejected\n";return 0;
 }catch(const karto::Exception&e){std::cerr<<e.GetErrorMessage()<<"\n";return 1;}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}
}
