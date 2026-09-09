#include <cmath>
#include <csignal>
#include <thread>
#include <deque>
#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include "kiss_icp/pipeline/KissICP.hpp"

using Cloud = sensor_msgs::msg::PointCloud2;
using Odom = nav_msgs::msg::Odometry;
using JSON = nlohmann::json;
class Mapping : public rclcpp::Node {
 public:
  Mapping() : Node("alpine_kiss_mapping") {
    config_.deskew=false; config_.voxel_size=.25; config_.max_range=60.; config_.min_range=.2;
    config_.max_num_iterations=30; config_.max_num_threads=2; config_.max_points_per_voxel=5;
    weight_=declare_parameter("prior_icp_weight",.2);
    prior_tolerance_=declare_parameter("prior_tolerance_s",.12);
    const auto offset=declare_parameter<std::vector<double>>("lidar_position_ros",{.335,0.,2.55});
    if(offset.size()!=3)throw std::runtime_error("lidar_position_ros needs 3 values");
    extrinsic_=Sophus::SE3d(Eigen::Quaterniond::Identity(),Eigen::Vector3d(offset[0],offset[1],offset[2]));
    configured_=true;
    icp_=std::make_unique<kiss_icp::pipeline::KissICP>(config_);
    odom_pub_=create_publisher<Odom>("/sim/slam/odom",10);
    cloud_pub_=create_publisher<Cloud>("/sim/slam/registered_points",rclcpp::QoS(1));
    map_pub_=create_publisher<Cloud>("/sim/slam/local_map",rclcpp::QoS(1).transient_local());
    path_pub_=create_publisher<nav_msgs::msg::Path>("/sim/slam/path",1);
    status_sub_=create_subscription<std_msgs::msg::String>("/sim/status",10,[this](std_msgs::msg::String::ConstSharedPtr m){
      auto j=JSON::parse(m->data,nullptr,false);
      if(j.is_discarded() || !j.contains("epoch") || !j.contains("sim_time"))return;
      const std::string id=j["epoch"].dump();const double t=j.value("sim_time",0.0);
      if(id!=reset_ || t<last_sim_time_-.01){scene_cutoff_ms_=t*1000.;icp_->Reset();odoms_.clear();pending_.reset();path_.poses.clear();reset_=id;
        std_msgs::msg::Header h;h.frame_id="map";h.stamp=now();map_pub_->publish(cloud({},h));cloud_pub_->publish(cloud({},h));path_.header=h;path_pub_->publish(path_);}
      last_sim_time_=t;
    });
    odom_sub_=create_subscription<Odom>("/sim/localization/odom",100,[this](Odom::ConstSharedPtr m){
      if(m->header.frame_id!="map" || m->child_frame_id!="base_link") return;
      odoms_.push_back(m);while(odoms_.size()>200)odoms_.pop_front();process();
    });
    cloud_sub_=create_subscription<Cloud>("/points_lio",rclcpp::SensorDataQoS(),[this](Cloud::ConstSharedPtr m){pending_=m;process();});
    RCLCPP_INFO(get_logger(),"KISS-ICP + simulated odometry prior (not GPS/independent SLAM); snapshot deskew disabled; outputs in map; no loop closure");
  }
 private:
  Cloud cloud(const std::vector<Eigen::Vector3d>& points,const std_msgs::msg::Header& header) {
    Cloud out;out.header=header;
    sensor_msgs::PointCloud2Modifier mod(out);mod.setPointCloud2FieldsByString(1,"xyz");mod.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(out,"x"),y(out,"y"),z(out,"z");
    for(auto& p:points){*x=p.x();*y=p.y();*z=p.z();++x;++y;++z;}out.is_dense=true;return out;
  }
  void process(){
    if(!configured_ || !pending_ || odoms_.empty())return;
    const auto stamp=rclcpp::Time(pending_->header.stamp);
    if(stamp.seconds()*1000. < scene_cutoff_ms_){pending_.reset();return;}
    Odom::ConstSharedPtr prior;double best=1.e9;
    for(auto& o:odoms_){double d=std::abs((stamp-rclcpp::Time(o->header.stamp)).seconds());if(d<best){best=d;prior=o;}}
    if(best>prior_tolerance_ || std::abs((now()-stamp).seconds())>2.0)return;
    auto scan=pending_;pending_.reset();
    if(scan->header.frame_id!="lidar36_link" || scan->is_bigendian)return;
    std::vector<Eigen::Vector3d> points;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*scan,"x"),y(*scan,"y"),z(*scan,"z");
      for(;x!=x.end();++x,++y,++z){Eigen::Vector3d p(*x,*y,*z);if(p.allFinite())points.push_back(extrinsic_*p);}
    }catch(const std::runtime_error& e){RCLCPP_WARN(get_logger(),"%s",e.what());return;}
    if(points.size()<30)return;
    auto& p=prior->pose.pose;Eigen::Quaterniond q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
    if(!q.coeffs().allFinite() || q.norm()<.1)return;
    kiss_icp::pipeline::PosePrior hint;
    hint.pose=Sophus::SE3d(q.normalized(),Eigen::Vector3d(p.position.x,p.position.y,p.position.z));
    hint.translation_weight=weight_;hint.yaw_weight=weight_;hint.max_translation_correction=.3;hint.max_yaw_correction=.035;
    auto [frame,keypoints]=icp_->RegisterFrame(points,{},hint);
    const auto pose=icp_->pose();auto rot=pose.unit_quaternion();auto pos=pose.translation();
    Odom out;out.header=scan->header;out.header.frame_id="map";out.child_frame_id="slam_base_link";
    out.pose.pose.position.x=pos.x();out.pose.pose.position.y=pos.y();out.pose.pose.position.z=pos.z();
    out.pose.pose.orientation.x=rot.x();out.pose.pose.orientation.y=rot.y();out.pose.pose.orientation.z=rot.z();out.pose.pose.orientation.w=rot.w();
    for(int i:{0,7,14,21,28,35}){out.pose.covariance[i]=.02;out.twist.covariance[i]=1.e3;}
    odom_pub_->publish(out);
    for(auto& point:frame)point=pose*point;
    cloud_pub_->publish(cloud(frame,out.header));
    geometry_msgs::msg::PoseStamped ps;ps.header=out.header;ps.pose=out.pose.pose;
    path_.header=out.header;path_.poses.push_back(ps);if(path_.poses.size()>5000)path_.poses.erase(path_.poses.begin());
    path_pub_->publish(path_);
    if(++frames_%10==0)map_pub_->publish(cloud(icp_->LocalMap(),out.header));
  }
  kiss_icp::pipeline::KISSConfig config_;
  std::unique_ptr<kiss_icp::pipeline::KissICP> icp_;
  Sophus::SE3d extrinsic_;
  double scene_cutoff_ms_=0.,last_sim_time_=-1.,prior_tolerance_=.12;
  bool configured_=false;double weight_=.2;int frames_=0;std::string reset_;
  Cloud::ConstSharedPtr pending_;std::deque<Odom::ConstSharedPtr> odoms_;nav_msgs::msg::Path path_;
  rclcpp::Publisher<Odom>::SharedPtr odom_pub_;
  rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_,map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_sub_,status_sub_;
};
static volatile std::sig_atomic_t stopping=0;
int main(int argc,char** argv){
  rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,[](int){stopping=1;});std::signal(SIGTERM,[](int){stopping=1;});
  auto node=std::make_shared<Mapping>();
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  while(!stopping && rclcpp::ok()){
    executor.spin_some(std::chrono::milliseconds(5));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::signal(SIGINT,SIG_IGN);std::signal(SIGTERM,SIG_IGN);
  rclcpp::shutdown();
}
