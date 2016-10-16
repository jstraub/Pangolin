#include <librealsense/rs.hpp>
#include <pangolin/video/drivers/realsense.h>
#include <pangolin/utils/timer.h>
#include <string>
#include <iostream>

namespace pangolin {

RealSenseVideo::RealSenseVideo(ImageDim dim, int fps, bool registerRGBD)
  : dim_(dim), fps_(fps), registerRGBD_(registerRGBD) {

  ctx_ = new rs::context();
  sizeBytes = 0;
  for (int32_t i=0; i<ctx_->get_device_count(); ++i) {
    devs_.push_back(ctx_->get_device(i));

    devs_[i]->enable_stream(rs::stream::depth, dim_.x, dim_.y,
        rs::format::z16, fps_);

    StreamInfo streamD(VideoFormatFromString("GRAY16LE"), dim_.x,
        dim_.y, dim_.x*2, (uint8_t*)0+sizeBytes);
    streams.push_back(streamD);

    sizeBytes += streamD.SizeBytes();

    devs_[i]->enable_stream(rs::stream::color, dim_.x, dim_.y,
        rs::format::rgb8, fps_);

    StreamInfo streamRGB(VideoFormatFromString("RGB24"), dim_.x,
        dim_.y, dim_.x*3, (uint8_t*)0+sizeBytes);
    streams.push_back(streamRGB);
    
    sizeBytes += streamRGB.SizeBytes();

    devs_[i]->start();
  }
  total_frames = std::numeric_limits<int>::max();
  UpdateProperties();
}

void RealSenseVideo::UpdateProperties() {
  size_t numDevices = ctx_->get_device_count();

  json::value& jsrealsense = device_properties["realsense"];
  json::value& jsdevices = jsrealsense["devices"];
  jsdevices = json::value(json::array_type,false);
  jsdevices.get<json::array>().resize(numDevices);
  for (size_t i=0; i<numDevices; ++i) {
    json::value& jsdevice = jsdevices[i];
    jsdevice["serial_number"] = GetSerial(i);
    jsdevice["projector_power"] = GetCurrentPower(i);
  }
  json::value& jsstreams = jsrealsense["streams"];
  jsstreams = json::value(json::array_type,false);
  jsstreams.get<json::array>().resize(streams.size());
  for (size_t i=0; i<streams.size(); ++i) {
    json::value& jsstream = jsstreams[i];
    jsstream["type"] = streams[i].PixFormat().format;
  }

  streams_properties = &frame_properties["streams"];
  *streams_properties = json::value(json::array_type,false);
  streams_properties->get<json::array>().resize(streams.size());
}

RealSenseVideo::~RealSenseVideo() {
  delete ctx_;
}

void RealSenseVideo::Start() {
  for (int32_t i=0; i<ctx_->get_device_count(); ++i) {
    //devs_[i]->stop();
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
    } else {
      devs_[i]->poll_for_frames();
    }

    if (registerRGBD_) {
        memcpy(out_img, devs_[i]->get_frame_data(rs::stream::depth_aligned_to_color), streams[i*2].SizeBytes());
    } else {
        memcpy(out_img, devs_[i]->get_frame_data(rs::stream::depth), streams[i*2].SizeBytes());
    }
    out_img += streams[i*2].SizeBytes();
    (*streams_properties)[2*i]["hosttime_us"] = pangolin::Time_us(pangolin::TimeNow());

    memcpy(out_img, devs_[i]->get_frame_data(rs::stream::color), streams[i*2+1].SizeBytes());
    out_img += streams[i*2+1].SizeBytes();
    (*streams_properties)[2*i+1]["hosttime_us"] = pangolin::Time_us(pangolin::TimeNow());
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

double RealSenseVideo::GetCurrentPower(int idx) {
  double power = 0;
  rs::option opt[] = {rs::option::f200_laser_power};
  devs_[idx]->get_options(opt,1,&power);
  return power;
}

void RealSenseVideo::SetPower(int idx, double power){
  double power_[] = {power};
  rs::option opt[] = {rs::option::f200_laser_power};
  devs_[idx]->set_options(opt,1,power_);
  if (GetCurrentPower(idx) != power) {
    std::cerr << "Problem with setting power of realsense device to " << power 
      << std::endl;
  }
}

void RealSenseVideo::SetPowers(double power) {
  for (int32_t i=0; i<ctx_->get_device_count(); ++i) {
    SetPower(i,power);
  }
}

std::string RealSenseVideo::GetSerial(int idx){
  if (idx < (int)devs_.size()){
    const char* serialNumber = devs_[idx]->get_serial();
    return std::string(serialNumber);
  } 
  return "";
}

}
