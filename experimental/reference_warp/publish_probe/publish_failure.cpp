// Explicit held-simulation fault injection only. Failed calls return ERROR, never fake success.
#include <rcl/publisher.h>
#include <rcl/error_handling.h>
#include <dlfcn.h>
#include <unistd.h>
#include <array>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

extern "C" rcl_ret_t rcl_publish(const rcl_publisher_t* pub, const void* msg,
                                rmw_publisher_allocation_t* allocation) {
  using Fn = rcl_ret_t(*)(const rcl_publisher_t*,const void*,rmw_publisher_allocation_t*);
  static Fn real = []() {
    void *lib=dlopen("librcl.so",RTLD_NOW|RTLD_NOLOAD);
    auto fn = lib ? reinterpret_cast<Fn>(dlsym(lib,"rcl_publish")) : nullptr;
    if (!fn) _exit(96);
    return fn;
  }();
  const char *enabled=getenv("HUMANOID_DDS_FAULT_INJECTION");
  if (!enabled || strcmp(enabled,"held_sim")!=0) return real(pub,msg,allocation);
  static const char *topics[]={"/imu/data_raw","/joint_states","/motor_commands",
                                "/ball_chase/cmd_vel","/safety/estop",
                                "/head/servo_target_rad","/head/tilt_target_rad"};
  static std::array<std::atomic<unsigned long>,7> counts{};
  const char *topic=rcl_publisher_get_topic_name(pub);
  const char *selected=getenv("HUMANOID_FAIL_PUBLISH_TOPICS");
  if (topic && selected) {
    const std::string selection=std::string(",")+selected+",";
    for (size_t i=0;i<7;++i) if (strcmp(topic,topics[i])==0 &&
          selection.find(std::string(",")+topic+",")!=std::string::npos) {
      // Exercise the initial estop delivery; the high-rate streams fail only after readiness.
      const unsigned long trigger=i==4 ? 1 : (i>=5 ? 10 : 100);
      if (++counts[i]==trigger) {
        fprintf(stderr,"[dds-fault-injection] ERROR topic=%s count=%lu pid=%d\n",topic,trigger,getpid());
        RCL_SET_ERROR_MSG("held-simulation injected DDS publication failure");
        return RCL_RET_ERROR;
      }
    }
  }
  return real(pub,msg,allocation);
}
