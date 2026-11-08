// Opt-in timing only: forward every real call, preserve data/QoS/return codes.
// No signals, delay injection, or library modifications. Optional mutex tracing
// unwinds slow-call stacks and can perturb timing; never used in normal launches.
#include <rcl/publisher.h>
#include <rmw/rmw.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#ifdef ENABLE_MUTEX_TRACE
#include <pthread.h>
#include <execinfo.h>
#endif
#ifdef ENABLE_WAIT_STATS
#include <sys/resource.h>
#endif

namespace {
int log_fd = -1;
#ifdef ENABLE_MUTEX_TRACE
int lock_fd = -1;
thread_local const char* active_publish_topic = nullptr;
template<typename Fn, typename... Args>
int trace_lock(const char* name, Fn real, const void* object, Args... args) {
  if (!active_publish_topic || lock_fd<0) return real(args...);
  timespec b{},e{};
  clock_gettime(CLOCK_MONOTONIC,&b);
  const int result=real(args...);
  clock_gettime(CLOCK_MONOTONIC,&e);
  const int64_t begin=int64_t(b.tv_sec)*1000000000+b.tv_nsec;
  const int64_t end=int64_t(e.tv_sec)*1000000000+e.tv_nsec;
  if (end-begin>=20000000) {
    const char* topic=active_publish_topic;
    active_publish_topic=nullptr;
    char line[1024];
    int size=snprintf(line,sizeof(line),"LOCK %s pid=%d tid=%ld topic=%s object=%p begin_ns=%lld end_ns=%lld wall_ms=%.3f result=%d\n",
      name,getpid(),long(gettid()),topic,object,(long long)begin,(long long)end,(end-begin)/1e6,result);
    if(size>0 && size<int(sizeof(line))) {auto written=write(lock_fd,line,size);(void)written;}
    void* frames[32];
    int count=backtrace(frames,32);
    backtrace_symbols_fd(frames,count,lock_fd);
    active_publish_topic=topic;
  }
  return result;
}
#endif
#ifdef ENABLE_WAIT_STATS
int wait_fd = -1;
void wait_record(const char* layer,const char* topic,int64_t begin,int64_t end,int64_t cpu,
                 const rusage& before) {
  if (wait_fd<0 || end-begin<20000000) return;
  rusage after{}; getrusage(RUSAGE_THREAD,&after);
  char line[1024];
  int size=snprintf(line,sizeof(line),
    "{\"layer\":\"%s\",\"pid\":%d,\"tid\":%ld,\"topic\":\"%s\",\"begin_ns\":%lld,\"end_ns\":%lld,\"cpu_ns\":%lld,\"voluntary_switches\":%ld,\"involuntary_switches\":%ld,\"minor_faults\":%ld,\"major_faults\":%ld}\n",
    layer,getpid(),static_cast<long>(gettid()),topic?topic:"",
    static_cast<long long>(begin),static_cast<long long>(end),static_cast<long long>(cpu),
    after.ru_nvcsw-before.ru_nvcsw,after.ru_nivcsw-before.ru_nivcsw,
    after.ru_minflt-before.ru_minflt,after.ru_majflt-before.ru_majflt);
  if(size>0 && size<int(sizeof(line))) {const auto written=write(wait_fd,line,size);(void)written;}
}
#endif
thread_local unsigned long rcl_count = 0, rmw_count = 0;
int64_t ns(clockid_t clock) {
  timespec t{}; clock_gettime(clock, &t);
  return int64_t(t.tv_sec)*1000000000LL+t.tv_nsec;
}
void* resolve(const char* library, const char* symbol) {
  void* handle = dlopen(library, RTLD_NOW|RTLD_NOLOAD);
  void* function = handle ? dlsym(handle, symbol) : nullptr;
  if (!function) {
    fprintf(stderr,"[publish-boundary] failed resolving %s:%s\n",library,symbol);
    _exit(96);
  }
  return function;
}
__attribute__((constructor)) void open_log() {
  const char* dir = getenv("HUMANOID_PUBLISH_TRACE_DIR");
  if (!dir) return;
  char path[4096];
  snprintf(path,sizeof(path),"%s/%d.csv",dir,getpid());
  log_fd = open(path,O_CREAT|O_WRONLY|O_APPEND|O_CLOEXEC,0644);
  if (log_fd < 0) { perror("publish-boundary log"); _exit(97); }
#ifdef ENABLE_MUTEX_TRACE
  snprintf(path,sizeof(path),"%s/%d.locks.txt",dir,getpid());
  lock_fd=open(path,O_CREAT|O_WRONLY|O_APPEND|O_CLOEXEC,0644);
  if(lock_fd<0) {perror("publish-boundary locks");_exit(97);}
#endif
#ifdef ENABLE_WAIT_STATS
  snprintf(path,sizeof(path),"%s/%d.wait.jsonl",dir,getpid());
  wait_fd=open(path,O_CREAT|O_WRONLY|O_APPEND|O_CLOEXEC,0644);
  if(wait_fd<0) {perror("publish-boundary wait log");_exit(97);}
#endif
}
void record(const char* layer,const char* topic,int64_t begin,int64_t end,int64_t cpu,
            int result,unsigned long count) {
  if (log_fd < 0 || (end-begin < 2000000 && count>5 && count%100)) return;
  static auto gil_check = reinterpret_cast<int(*)(void)>(dlsym(RTLD_DEFAULT,"PyGILState_Check"));
  char line[1024];
  int size = snprintf(line,sizeof(line),"%s,%d,%ld,%lld,%lld,%lld,%d,%d,%lu,%s\n",
      layer,getpid(),static_cast<long>(gettid()),static_cast<long long>(begin),
      static_cast<long long>(end),static_cast<long long>(cpu),result,
      gil_check ? gil_check() : -1,count,topic ? topic : "");
  if (size>0 && size<int(sizeof(line))) {
    const auto written=write(log_fd,line,size);
    (void)written; // Missing trace records are diagnosed from retained logs.
  }
}
}

extern "C" rcl_ret_t rcl_publish(const rcl_publisher_t* pub,const void* msg,
                                  rmw_publisher_allocation_t* allocation) {
  using Fn = rcl_ret_t(*)(const rcl_publisher_t*,const void*,rmw_publisher_allocation_t*);
  static Fn real = reinterpret_cast<Fn>(resolve("librcl.so","rcl_publish"));
  if (log_fd<0) return real(pub,msg,allocation);
#ifdef ENABLE_WAIT_STATS
  rusage before{}; getrusage(RUSAGE_THREAD,&before);
#endif
  auto begin=ns(CLOCK_MONOTONIC), cpu=ns(CLOCK_THREAD_CPUTIME_ID);
  auto result=real(pub,msg,allocation);
  auto end=ns(CLOCK_MONOTONIC); cpu=ns(CLOCK_THREAD_CPUTIME_ID)-cpu;
#ifdef ENABLE_WAIT_STATS
  wait_record("rcl",rcl_publisher_get_topic_name(pub),begin,end,cpu,before);
#endif
  record("rcl",rcl_publisher_get_topic_name(pub),begin,end,cpu,result,++rcl_count);
  return result;
}

extern "C" rmw_ret_t rmw_publish(const rmw_publisher_t* pub,const void* msg,
                                  rmw_publisher_allocation_t* allocation) {
  using Fn = rmw_ret_t(*)(const rmw_publisher_t*,const void*,rmw_publisher_allocation_t*);
  static Fn real = reinterpret_cast<Fn>(resolve("librmw_implementation.so","rmw_publish"));
  if (log_fd<0) return real(pub,msg,allocation);
#ifdef ENABLE_WAIT_STATS
  rusage before{}; getrusage(RUSAGE_THREAD,&before);
#endif
  auto begin=ns(CLOCK_MONOTONIC), cpu=ns(CLOCK_THREAD_CPUTIME_ID);
#ifdef ENABLE_MUTEX_TRACE
  const char* previous_topic=active_publish_topic;
  active_publish_topic=pub ? pub->topic_name : "";
#endif
  auto result=real(pub,msg,allocation);
#ifdef ENABLE_MUTEX_TRACE
  active_publish_topic=previous_topic;
#endif
  auto end=ns(CLOCK_MONOTONIC); cpu=ns(CLOCK_THREAD_CPUTIME_ID)-cpu;
#ifdef ENABLE_WAIT_STATS
  wait_record("rmw",pub ? pub->topic_name : "",begin,end,cpu,before);
#endif
  record("rmw",pub ? pub->topic_name : "",begin,end,cpu,result,++rmw_count);
  return result;
}

#ifdef ENABLE_MUTEX_TRACE
extern "C" int pthread_mutex_lock(pthread_mutex_t* mutex) noexcept {
  using Fn=int(*)(pthread_mutex_t*);
  static Fn real=reinterpret_cast<Fn>(dlsym(RTLD_NEXT,"pthread_mutex_lock"));
  return trace_lock("pthread_mutex_lock",real,mutex,mutex);
}
extern "C" int pthread_mutex_timedlock(pthread_mutex_t* mutex,const timespec* deadline) noexcept {
  using Fn=int(*)(pthread_mutex_t*,const timespec*);
  static Fn real=reinterpret_cast<Fn>(dlsym(RTLD_NEXT,"pthread_mutex_timedlock"));
  return trace_lock("pthread_mutex_timedlock",real,mutex,mutex,deadline);
}
extern "C" int pthread_mutex_clocklock(pthread_mutex_t* mutex,clockid_t clock,const timespec* deadline) noexcept {
  using Fn=int(*)(pthread_mutex_t*,clockid_t,const timespec*);
  static Fn real=reinterpret_cast<Fn>(dlsym(RTLD_NEXT,"pthread_mutex_clocklock"));
  return trace_lock("pthread_mutex_clocklock",real,mutex,mutex,clock,deadline);
}
#endif
