// Standalone synthetic IMU transport benchmark: no control/actuator topics.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <thread>
#include <vector>
using Clock=std::chrono::steady_clock;
double now() {return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
double quantile(std::vector<double> values,double fraction) {
  if(values.empty()) return 0;
  std::sort(values.begin(),values.end());
  return values[std::min(values.size()-1,size_t(fraction*(values.size()-1)))];
}
int main(int argc,char** argv) {
  std::map<std::string,std::string> args;
  for(int i=1;i+1<argc;i+=2) args[argv[i]]=argv[i+1];
  std::string role=args.count("--role")?args["--role"]:"publisher";
  double duration=args.count("--duration")?std::stod(args["--duration"]):12;
  double stall=args.count("--stall-ms")?std::stod(args["--stall-ms"])/1000:0;
  rclcpp::init(0,nullptr);
  auto node=std::make_shared<rclcpp::Node>("cpp_imu_transport_"+role);
  auto qos=rclcpp::QoS(rclcpp::KeepLast(role=="publisher"?10:1));
  if(args["--reliability"]=="best_effort") qos.best_effort(); else qos.reliable();
  std::vector<double> gaps,ages,calls,beats;
  std::atomic<bool> stop{false};
  std::thread heartbeat([&]{while(!stop){beats.push_back(now());std::this_thread::sleep_for(std::chrono::milliseconds(1));}});
  size_t count=0;
  double start=0,last=0;
  if(role=="publisher") {
    auto pub=node->create_publisher<sensor_msgs::msg::Imu>("/research/imu",qos);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    start=now(); double deadline=start;
    while(rclcpp::ok() && now()-start<duration) {
      auto t=now();
      if(last) gaps.push_back((t-last)*1000); last=t;
      sensor_msgs::msg::Imu msg;
      msg.header.stamp=node->now(); msg.header.frame_id="imu_link";
      msg.orientation.w=1; msg.angular_velocity.x=0.001*count; msg.linear_acceleration.z=9.81;
      auto before=now(); pub->publish(msg); calls.push_back((now()-before)*1000); ++count;
      deadline+=.01;
      std::this_thread::sleep_until(Clock::time_point(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(deadline))));
      if(deadline<now()-.01) deadline=now();
    }
  } else {
    auto sub=node->create_subscription<sensor_msgs::msg::Imu>("/research/imu",qos,
      [&](sensor_msgs::msg::Imu::ConstSharedPtr msg){
        auto t=now(); if(last) gaps.push_back((t-last)*1000); last=t;
        ages.push_back((node->now()-rclcpp::Time(msg->header.stamp)).seconds()*1000); ++count;
        if(stall>0) std::this_thread::sleep_for(std::chrono::duration<double>(stall));
      });
    start=now();
    while(rclcpp::ok() && now()-start<duration) {
      rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  auto elapsed=now()-start; stop=true; heartbeat.join();
  std::vector<double> beat_gaps;
  for(size_t i=1;i<beats.size();++i) beat_gaps.push_back((beats[i]-beats[i-1])*1000);
  std::ofstream out(args["--output"]);
  out<<"{\"language\":\"cpp\",\"count\":"<<count<<",\"elapsed_s\":"<<elapsed
     <<",\"publish_max_ms\":"<<quantile(calls,1)<<",\"publish_p99_ms\":"<<quantile(calls,.99)
     <<",\"source_or_receipt_gap_max_ms\":"<<quantile(gaps,1)<<",\"source_age_max_ms\":"<<quantile(ages,1)
     <<",\"source_age_p99_ms\":"<<quantile(ages,.99)<<",\"heartbeat_gap_max_ms\":"<<quantile(beat_gaps,1)<<"}\n";
  rclcpp::shutdown();
}
