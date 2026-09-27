// Direct Magewell V4L2 NV12 capture. Explicitly selected with capture = magewell.
#include "src/platform/common.h"
#include "src/logging.h"
#include "magewell_encode.h"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include "magewell_sdk/mw-linux.h"

namespace platf {
namespace {
using clock_type = std::chrono::steady_clock;
long long now_ns() { return std::chrono::duration_cast<std::chrono::nanoseconds>(clock_type::now().time_since_epoch()).count(); }
const char *device_path() { const char *p = std::getenv("MAGEWELL_DEVICE"); return p ? p : "/dev/video0"; }
int ctl(int fd, unsigned long req, void *arg) { int r; do { r=ioctl(fd, req, arg); } while(r<0 && errno==EINTR); return r; }
struct nv12_image : img_t {
  std::vector<uint8_t> storage;
  nv12_image(int w,int h,int pitch) : storage(size_t(pitch)*h*3/2) {
    width=w; height=h; row_pitch=pitch; pixel_pitch=1; data=storage.data();
  }
};
class magewell_display_t : public display_t {
  struct mapping { void *data; size_t size; };
  int fd=-1, pitch=0;
  bool streaming=false, warned_timestamp=false;
  std::vector<mapping> buffers;
  FILE *trace=nullptr;
public:
  ~magewell_display_t() override {
    if(streaming) { int type=V4L2_BUF_TYPE_VIDEO_CAPTURE; ctl(fd,VIDIOC_STREAMOFF,&type); }
    for(auto &b:buffers) munmap(b.data,b.size);
    if(fd>=0) close(fd);
    if(trace) fclose(trace);
  }
  bool init() {
    fd=open(device_path(),O_RDWR|O_NONBLOCK|O_CLOEXEC);
    if(fd<0) return false;
    MWCAP_VIDEO_SIGNAL_STATUS signal{};
    if(ctl(fd,MWCAP_IOCTL_GET_VIDEO_SIGNAL_STATUS,&signal)<0 || signal.cx<=0 || signal.cy<=0 || signal.bInterlaced) return false;
    width=env_width=signal.cx; height=env_height=signal.cy;
    if(width<48 || height<42 || width%2 || height%2) return false;
    v4l2_format fmt{}; fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width=width; fmt.fmt.pix.height=height; fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_NV12; fmt.fmt.pix.field=V4L2_FIELD_NONE;
    if(ctl(fd,VIDIOC_S_FMT,&fmt)<0 || fmt.fmt.pix.pixelformat!=V4L2_PIX_FMT_NV12 || int(fmt.fmt.pix.width)!=width || int(fmt.fmt.pix.height)!=height) return false;
    pitch=fmt.fmt.pix.bytesperline;
    if(pitch<width || fmt.fmt.pix.sizeimage<size_t(pitch)*height*3/2) return false;
    v4l2_streamparm parm{}; parm.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator=signal.dwFrameDuration;
    parm.parm.capture.timeperframe.denominator=10000000;
    if(ctl(fd,VIDIOC_S_PARM,&parm)<0) return false;
    MWCAP_VIDEO_PROCESS_SETTINGS settings{};
    if(ctl(fd,MWCAP_IOCTL_GET_VIDEO_PROCESS_SETTINGS,&settings)<0) return false;
    settings.bLowLatency=1;
    settings.colorFormat=MWCAP_VIDEO_COLOR_FORMAT_YUV709;
    settings.quantRange=MWCAP_VIDEO_QUANTIZATION_LIMITED;
    if(ctl(fd,MWCAP_IOCTL_SET_VIDEO_PROCESS_SETTINGS,&settings)<0 || ctl(fd,MWCAP_IOCTL_GET_VIDEO_PROCESS_SETTINGS,&settings)<0 || !settings.bLowLatency ||
       settings.colorFormat!=MWCAP_VIDEO_COLOR_FORMAT_YUV709 || settings.quantRange!=MWCAP_VIDEO_QUANTIZATION_LIMITED) return false;
    if(const char *path=std::getenv("MAGEWELL_TRACE")) {
      trace=fopen(path,"a");
      if(trace) setvbuf(trace,nullptr,_IOLBF,0);
    }
    BOOST_LOG(info) << "Magewell direct NV12 " << width << 'x' << height << " pitch " << pitch << ", low latency enabled";
    return true;
  }
  std::shared_ptr<img_t> alloc_img() override { return std::make_shared<nv12_image>(width,height,pitch); }
  int dummy_img(img_t *img) override {
    memset(img->data,16,size_t(pitch)*height); memset(img->data+size_t(pitch)*height,128,size_t(pitch)*height/2); return 0;
  }
  std::unique_ptr<avcodec_encode_device_t> make_avcodec_encode_device(pix_fmt_e fmt) override {
    if(fmt!=pix_fmt_e::nv12) return nullptr;
    return magewell::make_encode_device(width,height);
  }
  capture_e capture(const push_captured_image_cb_t &push,const pull_free_image_cb_t &pull,bool *) override {
    v4l2_requestbuffers req{}; req.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; req.memory=V4L2_MEMORY_MMAP; req.count=4;
    if(ctl(fd,VIDIOC_REQBUFS,&req)<0 || req.count<2) return capture_e::error;
    for(unsigned i=0;i<req.count;++i) {
      v4l2_buffer b{}; b.type=req.type; b.memory=req.memory; b.index=i;
      if(ctl(fd,VIDIOC_QUERYBUF,&b)<0 || b.length<size_t(pitch)*height*3/2) return capture_e::error;
      void *p=mmap(nullptr,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd,b.m.offset);
      if(p==MAP_FAILED) return capture_e::error;
      buffers.push_back({p,b.length});
      if(ctl(fd,VIDIOC_QBUF,&b)<0) return capture_e::error;
    }
    int type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(ctl(fd,VIDIOC_STREAMON,&type)<0) return capture_e::error;
    streaming=true;
    while(true) {
      pollfd p{fd,POLLIN,0}; int ready=poll(&p,1,100);
      if(ready<0) { if(errno==EINTR) continue; return capture_e::error; }
      if(!ready) { if(!push(nullptr,false)) return capture_e::ok; continue; }
      if(p.revents&(POLLERR|POLLHUP|POLLNVAL)) return capture_e::reinit;
      std::shared_ptr<img_t> img;
      if(!pull(img)) return capture_e::ok;
      v4l2_buffer newest{}; bool have=false; unsigned dropped=0;
      // After waiting for a free output image, retain only the newest completed capture.
      for(unsigned n=0;n<req.count;++n) {
        v4l2_buffer b{}; b.type=req.type; b.memory=req.memory;
        if(ctl(fd,VIDIOC_DQBUF,&b)<0) { if(errno==EAGAIN) break; return capture_e::error; }
        if(have) { if(ctl(fd,VIDIOC_QBUF,&newest)<0) return capture_e::error; ++dropped; }
        newest=b; have=true;
      }
      if(!have) continue;
      auto dequeued=now_ns();
      if(newest.index>=buffers.size() || newest.bytesused<size_t(pitch)*height*3/2 || (newest.flags&V4L2_BUF_FLAG_ERROR)) return capture_e::reinit;
      // ProCapture 1.3.0.4458 stamps ktime_get_ns() after DMA completion,
      // but omits V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC. Validate its clock domain
      // instead of discarding the useful driver timestamp based on that flag.
      auto timestamp=std::chrono::seconds(newest.timestamp.tv_sec)+std::chrono::microseconds(newest.timestamp.tv_usec);
      auto driver_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp).count();
      bool driver_timestamp=driver_ns>0 && driver_ns<=dequeued && dequeued-driver_ns<=1000000000LL;
      auto selected_ns=driver_timestamp ? driver_ns : dequeued;
      img->frame_timestamp=clock_type::time_point(std::chrono::duration_cast<clock_type::duration>(std::chrono::nanoseconds(selected_ns)));
      if(!driver_timestamp && !warned_timestamp) {
        BOOST_LOG(warning) << "Magewell invalid/stale driver timestamp " << driver_ns
                           << " at dequeue " << dequeued << ", flags " << newest.flags
                           << "; using dequeue-time proxy (not DMA completion)";
        warned_timestamp=true;
      }
      memcpy(img->data,buffers[newest.index].data,size_t(pitch)*height*3/2);
      if(ctl(fd,VIDIOC_QBUF,&newest)<0) return capture_e::error;
      if(trace) {
        unsigned sum=0,count=0;
        for(int y=height/2-21;y<height/2+21;++y) for(int x=width/2-24;x<width/2+24;++x) { sum+=img->data[size_t(y)*pitch+x]; ++count; }
        auto frame_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(img->frame_timestamp->time_since_epoch()).count();
        fprintf(trace,"capture,%lld,%lld,%lld,%u,%u,%.3f,%lld,%u,%u\n",dequeued,now_ns(),(long long)frame_ns,newest.sequence,dropped,double(sum)/count,(long long)driver_ns,newest.flags,unsigned(driver_timestamp));
      }
      if(!push(std::move(img),true)) return capture_e::ok;
    }
  }
};
}
std::vector<std::string> magewell_display_names() {
  int fd=open(device_path(),O_RDWR|O_NONBLOCK|O_CLOEXEC);
  if(fd<0) return {};
  MWCAP_VIDEO_SIGNAL_STATUS signal{};
  bool valid=ctl(fd,MWCAP_IOCTL_GET_VIDEO_SIGNAL_STATUS,&signal)==0 && signal.cx>0 && signal.cy>0;
  close(fd); return valid ? std::vector<std::string>{device_path()} : std::vector<std::string>{};
}
std::shared_ptr<display_t> magewell_display(mem_type_e type,const std::string &,const video::config_t &) {
  if(type!=mem_type_e::vaapi) return nullptr;
  auto d=std::make_shared<magewell_display_t>();
  if(!d->init()) { BOOST_LOG(error) << "Magewell direct capture initialization failed: " << strerror(errno); return nullptr; }
  return d;
}
}
