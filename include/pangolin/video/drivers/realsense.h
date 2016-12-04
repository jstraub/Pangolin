

#ifndef PANGOLIN_REALSENSE_H
#define PANGOLIN_REALSENSE_H

#include <pangolin/pangolin.h>

#include <pangolin/video/video.h>

namespace rs {
class context;
class device;
}

namespace pangolin
{

//! Interface to video capture sources
struct RealSenseVideo : public VideoInterface, public VideoPropertiesInterface, public VideoPlaybackInterface
{
 public:

    // Open all RGB and Depth streams from all devices
    RealSenseVideo(ImageDim dim=ImageDim(640,480), int fps=30, bool registerRGBD=false);

    // Open streams specified
    // TODO
    //RealSenseVideo(std::vector<OpenNiStreamMode>& stream_modes);

    ~RealSenseVideo();

    //! Implement VideoInput::Start()
    void Start();

    //! Implement VideoInput::Stop()
    void Stop();

    //! Implement VideoInput::SizeBytes()
    size_t SizeBytes() const;

    //! Implement VideoInput::Streams()
    const std::vector<StreamInfo>& Streams() const;

    //! Implement VideoInput::GrabNext()
    bool GrabNext( unsigned char* image, bool wait = true );

    //! Implement VideoInput::GrabNewest()
    bool GrabNewest( unsigned char* image, bool wait = true );

    //! Grab turn on projector grab one frame and turn it off
    bool GrabOne(int idx, unsigned char* image);

    //! Implement VideoPropertiesInterface::Properties()
    const json::value& DeviceProperties() const {
        return device_properties;
    }

    //! Implement VideoPropertiesInterface::Properties()
    const json::value& FrameProperties() const {
        return frame_properties;
    }

    //! Implement VideoPlaybackInterface::GetCurrentFrameId
    int GetCurrentFrameId() const;

    //! Implement VideoPlaybackInterface::GetTotalFrames
    int GetTotalFrames() const ;

    //! Implement VideoPlaybackInterface::Seek
    int Seek(int frameid);

    /// Get the current laser power level of ith sensor
    double GetCurrentPower(int idx);

    /// Set ith device's laser power to power
    void SetPower(int idx, double power);

    /// Set all devices' laser power to power
    void SetPowers(double power);

    /// Get ith device's serial number
    std::string GetSerial(int idx);

    /// Update Sensors Properties
    void UpdateProperties();

protected:
    size_t sizeBytes;

    std::vector<StreamInfo> streams;

    json::value device_properties;
    json::value frame_properties;
    json::value* streams_properties;

    int current_frame_index;
    int total_frames;

    rs::context* ctx_;
    std::vector<rs::device*> devs_;

    ImageDim  dim_;
    int fps_;
    bool registerRGBD_;
};

}

#endif // PANGOLIN_REALSENSE_H
