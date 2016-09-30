#include <librealsense/rs.hpp>
#include <pangolin/video/drivers/realsense.h>

namespace pangolin {

RealSenseVideo::RealSenseVideo(ImageDim dim, int fps, bool registerRGBD)
  : dim_(dim), fps_(fps), registerRGBD_(registerRGBD) {

  ctx_ = new rs::context();
  sizeBytes = 0;
  for (int32_t i=0; i<ctx_->get_device_count(); ++i) {
    devs_.push_back(ctx_->get_device(i));

    devs_[i]->enable_stream(rs::stream::depth, dim_.x, dim_.y, rs::format::z16, fps_);
    StreamInfo streamD(VideoFormatFromString("GRAY16LE"), dim_.x, dim_.y, dim_.x*2, 0);
    streams.push_back(streamD);

    sizeBytes += streamD.SizeBytes();
    devs_[i]->enable_stream(rs::stream::color, dim_.x, dim_.y, rs::format::rgb8, fps_);
    StreamInfo streamRGB(VideoFormatFromString("RGB24"), dim_.x, dim_.y, dim_.x*3, (uint8_t*)0+sizeBytes);
    streams.push_back(streamRGB);
    sizeBytes += streamRGB.SizeBytes();

    devs_[i]->start();
  }
  total_frames = std::numeric_limits<int>::max();
}

RealSenseVideo::~RealSenseVideo() {
  delete ctx_;
}

void RealSenseVideo::Start() {
  for (int32_t i=0; i<ctx_->get_device_count(); ++i) {
    devs_[i]->stop();
    devs_[i]->start();
  }
  current_frame_index = 0;
}

void RealSenseVideo::Stop() {
  for (int32_t i=0; i<ctx_->get_device_count(); ++i) {
    devs_[i]->stop();
  }
}

size_t RealSenseVideo::SizeBytes() const {
  return sizeBytes;
}

const std::vector<StreamInfo>& RealSenseVideo::Streams() const {
  return streams;
}

bool RealSenseVideo::GrabNext(unsigned char* image, bool wait) {

  unsigned char* out_img = image;
  for (int32_t i=0; i<ctx_->get_device_count(); ++i) {
    if (wait) {
      devs_[i]->wait_for_frames();
    }
    if (registerRGBD_) {
        memcpy(out_img, devs_[i]->get_frame_data(rs::stream::depth_aligned_to_color), streams[i*2].SizeBytes());
    } else {
        memcpy(out_img, devs_[i]->get_frame_data(rs::stream::depth), streams[i*2].SizeBytes());
    }
    out_img += streams[i*2].SizeBytes();
    memcpy(out_img, devs_[i]->get_frame_data(rs::stream::color), streams[i*2+1].SizeBytes());
    out_img += streams[i*2+1].SizeBytes();
  }
  return true;
}

bool RealSenseVideo::GrabNewest(unsigned char* image, bool wait) {
  return GrabNext(image, wait);
}

int RealSenseVideo::GetCurrentFrameId() const {
  return current_frame_index;
}

int RealSenseVideo::GetTotalFrames() const {
  return total_frames;
}

int RealSenseVideo::Seek(int frameid) {
  // TODO
  return -1;
}

void RealSenseVideo::GetCurrentPower(int idx, double *power) {
  rs::option opt[] = {rs::option::f200_laser_power};
  devs_[idx]->get_options(opt,1,power);
  // printf("current power %f\n", power[0]);
}

void RealSenseVideo::SetPower(int idx, double power){
  double power_[] = {power};
  rs::option opt[] = {rs::option::f200_laser_power};
  devs_[idx]->set_options(opt,1,power_);
  printf("set the power!\n");
}

void RealSenseVideo::SetPowers(double power) {
  for (int32_t i=0; i<ctx_->get_device_count(); ++i) {
    SetPower(i,power);
  }
  printf("set all the power!\n");
}

}
