/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// ROS communication
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse_ini.h>
#include <polled_camera/publication_server.h>

// Diagnostics
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <self_test/self_test.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include "prosilica_camera/ProsilicaCameraConfig.h"

// Standard libs
#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <sstream>

#include "prosilica/prosilica.h"
#include "prosilica/rolling_sum.h"

/// @todo Only stream when subscribed to
class ProsilicaNode
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher streaming_pub_;
  polled_camera::PublicationServer poll_srv_;
  ros::ServiceServer set_camera_info_srv_;
  ros::Subscriber trigger_sub_;
  
  // Camera
  boost::scoped_ptr<prosilica::Camera> cam_;
  prosilica::FrameStartTriggerMode trigger_mode_; /// @todo Make this property of Camera
  bool running_;
  unsigned long max_data_rate_;
  tPvUint32 sensor_width_, sensor_height_; // full resolution dimensions (maybe should be in lib)
  bool auto_adjust_stream_bytes_per_second_;

  // Hardware triggering
  std::string trig_timestamp_topic_;
  ros::Time trig_time_;

  // ROS messages
  sensor_msgs::Image img_;
  sensor_msgs::CameraInfo cam_info_;

  // Dynamic reconfigure
  typedef prosilica_camera::ProsilicaCameraConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  ReconfigureServer reconfigure_server_;
  
  // Diagnostics
  ros::Timer diagnostic_timer_;
  boost::shared_ptr<self_test::TestRunner> self_test_;
  diagnostic_updater::Updater diagnostic_;
  std::string hw_id_;
  int count_;
  double desired_freq_;
  static const int WINDOW_SIZE = 5; // remember previous 5s
  unsigned long frames_dropped_total_, frames_completed_total_;
  RollingSum<unsigned long> frames_dropped_acc_, frames_completed_acc_;
  unsigned long packets_missed_total_, packets_received_total_;
  RollingSum<unsigned long> packets_missed_acc_, packets_received_acc_;

  // So we don't get burned by auto-exposure
  unsigned long last_exposure_value_;
  int consecutive_stable_exposures_;

public:
  ProsilicaNode(const ros::NodeHandle& node_handle)
    : nh_(node_handle),
      it_(nh_),
      cam_(NULL), running_(false), auto_adjust_stream_bytes_per_second_(false),
      count_(0),
      frames_dropped_total_(0), frames_completed_total_(0),
      frames_dropped_acc_(WINDOW_SIZE),
      frames_completed_acc_(WINDOW_SIZE),
      packets_missed_total_(0), packets_received_total_(0),
      packets_missed_acc_(WINDOW_SIZE),
      packets_received_acc_(WINDOW_SIZE)
  {
    // Two-stage initialization: in the constructor we open the requested camera. Most
    // parameters controlling capture are set and streaming started in configure(), the
    // callback to dynamic_reconfig.
    prosilica::init();

    if (prosilica::numCameras() == 0)
      ROS_WARN("Found no cameras on local subnet");

    // Determine which camera to use. Opening by IP address is preferred, then guid. If both
    // parameters are set we open by IP and verify the guid. If neither are set we default
    // to opening the first available camera.
    ros::NodeHandle local_nh("~");
    unsigned long guid = 0;
    std::string guid_str;
    if (local_nh.getParam("guid", guid_str) && !guid_str.empty())
      guid = strtol(guid_str.c_str(), NULL, 0);

    std::string ip_str;
    if (local_nh.getParam("ip_address", ip_str) && !ip_str.empty()) {
      cam_.reset( new prosilica::Camera(ip_str.c_str()) );
      
      // Verify guid is the one expected
      unsigned long cam_guid = cam_->guid();
      if (guid != 0 && guid != cam_guid)
        throw prosilica::ProsilicaException(ePvErrBadParameter,
                                            "guid does not match expected");
      guid = cam_guid;
    }
    else {
      if (guid == 0) guid = prosilica::getGuid(0);
      cam_.reset( new prosilica::Camera(guid) );
    }
    hw_id_ = boost::lexical_cast<std::string>(guid);
    ROS_INFO("Found camera, guid = %s", hw_id_.c_str());
    diagnostic_.setHardwareID(hw_id_);

    // Record some attributes of the camera
    tPvUint32 dummy;
    PvAttrRangeUint32(cam_->handle(), "Width", &dummy, &sensor_width_);
    PvAttrRangeUint32(cam_->handle(), "Height", &dummy, &sensor_height_);
    
    // Try to load intrinsics from on-camera memory.
    loadIntrinsics();

    // Set up self tests and diagnostics.
    // NB: Need to wait until here to construct self_test_, otherwise an exception
    // above from failing to find the camera gives bizarre backtraces
    // (http://answers.ros.org/question/430/trouble-with-prosilica_camera-pvapi).
    self_test_.reset(new self_test::TestRunner);
    self_test_->add( "Info Test", this, &ProsilicaNode::infoTest );
    self_test_->add( "Attribute Test", this, &ProsilicaNode::attributeTest );
    self_test_->add( "Image Test", this, &ProsilicaNode::imageTest );
    
    diagnostic_.add( "Frequency Status", this, &ProsilicaNode::freqStatus );
    diagnostic_.add( "Frame Statistics", this, &ProsilicaNode::frameStatistics );
    diagnostic_.add( "Packet Statistics", this, &ProsilicaNode::packetStatistics );
    diagnostic_.add( "Packet Error Status", this, &ProsilicaNode::packetErrorStatus );

    diagnostic_timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&ProsilicaNode::runDiagnostics, this));

    // Service call for setting calibration.
    set_camera_info_srv_ = nh_.advertiseService("set_camera_info", &ProsilicaNode::setCameraInfo, this);

    // Start dynamic_reconfigure
    reconfigure_server_.setCallback(boost::bind(&ProsilicaNode::configure, this, _1, _2));
  }

  void configure(Config& config, uint32_t level)
  {
    ROS_DEBUG("Reconfigure request received");

    if (level >= (uint32_t)driver_base::SensorLevels::RECONFIGURE_STOP)
      stop();

    // Trigger mode
    if (config.trigger_mode == "streaming") {
      trigger_mode_ = prosilica::Freerun;
      /// @todo Tighter bound than this minimal check
      desired_freq_ = 1; // make sure we get _something_
    }
    else if (config.trigger_mode == "syncin1") {
      trigger_mode_ = prosilica::SyncIn1;
      desired_freq_ = config.trig_rate;
    }
    else if (config.trigger_mode == "syncin2") {
      trigger_mode_ = prosilica::SyncIn2;
      desired_freq_ = config.trig_rate;
    }
#if 0
    else if (config.trigger_mode == "fixedrate") {
      ROS_DEBUG("Fixed rate not supported yet implementing software");
      trigger_mode_ = prosilica::Software;
      ///@todo add the fixed rate implementation
      desired_freq_ = 0;
    }
#endif
    else if (config.trigger_mode == "polled") {
      trigger_mode_ = prosilica::Software;
      desired_freq_ = 0;
    }
    else {
      ROS_ERROR("Invalid trigger mode '%s' in reconfigure request", config.trigger_mode.c_str());
    }
    trig_timestamp_topic_ = config.trig_timestamp_topic;

    // Exposure
    if (config.auto_exposure)
    {
      cam_->setExposure(0, prosilica::Auto);
      if (cam_->hasAttribute("ExposureAutoMax"))
      {
        tPvUint32 us = config.exposure_auto_max*1000000. + 0.5;
        cam_->setAttribute("ExposureAutoMax", us);
      }
      if (cam_->hasAttribute("ExposureAutoTarget"))
        cam_->setAttribute("ExposureAutoTarget", (tPvUint32)config.exposure_auto_target);
    }
    else {
      unsigned us = config.exposure*1000000. + 0.5;
      cam_->setExposure(us, prosilica::Manual);
    }

    // Gain
    if (config.auto_gain) {
      if (cam_->hasAttribute("GainAutoMax"))
      {
        cam_->setGain(0, prosilica::Auto);
        cam_->setAttribute("GainAutoMax", (tPvUint32)config.gain_auto_max);
        cam_->setAttribute("GainAutoTarget", (tPvUint32)config.gain_auto_target);
      }
      else {
        tPvUint32 major, minor;
        cam_->getAttribute("FirmwareVerMajor", major);
        cam_->getAttribute("FirmwareVerMinor", minor);
        ROS_WARN("Auto gain not available for this camera. Auto gain is available "
                 "on firmware versions 1.36 and above. You are running version %u.%u.",
                 (unsigned)major, (unsigned)minor);
        config.auto_gain = false;
      }
    }
    else
      cam_->setGain(config.gain, prosilica::Manual);
    
    // White balance
    if (config.auto_whitebalance) {
      if (cam_->hasAttribute("WhitebalMode"))
        cam_->setWhiteBalance(0, 0, prosilica::Auto);
      else {
        ROS_WARN("Auto white balance not available for this camera.");
        config.auto_whitebalance = false;
      }
    }
    else
      cam_->setWhiteBalance(config.whitebalance_blue, config.whitebalance_red, prosilica::Manual);

    // Binning configuration
    if (cam_->hasAttribute("BinningX")) {
      tPvUint32 max_binning_x, max_binning_y, dummy;
      PvAttrRangeUint32(cam_->handle(), "BinningX", &dummy, &max_binning_x);
      PvAttrRangeUint32(cam_->handle(), "BinningY", &dummy, &max_binning_y);
      config.binning_x = std::min(config.binning_x, (int)max_binning_x);
      config.binning_y = std::min(config.binning_y, (int)max_binning_y);
      
      cam_->setBinning(config.binning_x, config.binning_y);
    }
    else if (config.binning_x > 1 || config.binning_y > 1)
    {
      ROS_WARN("Binning not available for this camera.");
      config.binning_x = config.binning_y = 1;
    }

    // Region of interest configuration
    // Make sure ROI fits in image
    config.x_offset = std::min(config.x_offset, (int)sensor_width_ - 1);
    config.y_offset = std::min(config.y_offset, (int)sensor_height_ - 1);
    config.width  = std::min(config.width, (int)sensor_width_ - config.x_offset);
    config.height = std::min(config.height, (int)sensor_height_ - config.y_offset);
    // If width or height is 0, set it as large as possible
    int width  = config.width  ? config.width  : sensor_width_  - config.x_offset;
    int height = config.height ? config.height : sensor_height_ - config.y_offset;

    // Adjust full-res ROI to binning ROI
    /// @todo Replicating logic from polledCallback
    int x_offset = config.x_offset / config.binning_x;
    int y_offset = config.y_offset / config.binning_y;
    unsigned int right_x  = (config.x_offset + width  + config.binning_x - 1) / config.binning_x;
    unsigned int bottom_y = (config.y_offset + height + config.binning_y - 1) / config.binning_y;
    // Rounding up is bad when at max resolution which is not divisible by the amount of binning
    right_x = std::min(right_x, (unsigned)(sensor_width_ / config.binning_x));
    bottom_y = std::min(bottom_y, (unsigned)(sensor_height_ / config.binning_y));
    width = right_x - x_offset;
    height = bottom_y - y_offset;

    cam_->setRoi(x_offset, y_offset, width, height);
    
    // TF frame
    img_.header.frame_id = cam_info_.header.frame_id = config.frame_id;

    // Normally the node adjusts the bandwidth used by the camera during diagnostics, to use as
    // much as possible without dropping packets. But this can create interference if two
    // cameras are on the same switch, e.g. for stereo. So we allow the user to set the bandwidth
    // directly.
    auto_adjust_stream_bytes_per_second_ = config.auto_adjust_stream_bytes_per_second;
    if (!auto_adjust_stream_bytes_per_second_)
      cam_->setAttribute("StreamBytesPerSecond", (tPvUint32)config.stream_bytes_per_second);

    /// @todo If exception thrown due to bad settings, will fail to start camera
    if (level >= (uint32_t)driver_base::SensorLevels::RECONFIGURE_STOP)
      start();
  }

  ~ProsilicaNode()
  {
    stop();
    cam_.reset(); // must destroy Camera before calling prosilica::fini
    prosilica::fini();
  }

  void syncInCallback (const std_msgs::HeaderConstPtr& msg)
  {
    /// @todo Replace this with robust trigger matching
    trig_time_ = msg->stamp;
  }
  
  ///@todo add the setting of output sync1 and sync2?

  void start()
  {
    if (running_) return;

    if (trigger_mode_ == prosilica::Software) {
      poll_srv_ = polled_camera::advertise(nh_, "request_image", &ProsilicaNode::pollCallback, this);
      // Auto-exposure tends to go wild the first few frames after startup
      // if (auto_expose) normalizeExposure();
    }
    else {
      if ((trigger_mode_ == prosilica::SyncIn1) || (trigger_mode_ == prosilica::SyncIn2)) {
        if (!trig_timestamp_topic_.empty())
          trigger_sub_ = nh_.subscribe(trig_timestamp_topic_, 1, &ProsilicaNode::syncInCallback, this);
      }
      else {
        assert(trigger_mode_ == prosilica::Freerun);
      }
      cam_->setFrameCallback(boost::bind(&ProsilicaNode::publishImage, this, _1));
      streaming_pub_ = it_.advertiseCamera("image_raw", 1);
    }
    cam_->start(trigger_mode_, prosilica::Continuous);
    running_ = true;
  }

  void stop()
  {
    if (!running_) return;

    cam_->stop(); // Must stop camera before streaming_pub_.
    poll_srv_.shutdown();
    trigger_sub_.shutdown();
    streaming_pub_.shutdown();
    
    running_ = false;
  }

  void pollCallback(polled_camera::GetPolledImage::Request& req,
                    polled_camera::GetPolledImage::Response& rsp,
                    sensor_msgs::Image& image, sensor_msgs::CameraInfo& info)
  {
    if (trigger_mode_ != prosilica::Software) {
      rsp.success = false;
      rsp.status_message = "Camera is not in software triggered mode";
      return;
    }

    tPvFrame* frame = NULL;

    try {
      cam_->setBinning(req.binning_x, req.binning_y);

      if (req.roi.x_offset || req.roi.y_offset || req.roi.width || req.roi.height) {
        // GigE cameras use ROI in binned coordinates, so scale the values down.
        // The ROI is expanded if necessary to land on binned coordinates.
        unsigned int left_x = req.roi.x_offset / req.binning_x;
        unsigned int top_y  = req.roi.y_offset / req.binning_y;
        unsigned int right_x  = (req.roi.x_offset + req.roi.width  + req.binning_x - 1) / req.binning_x;
        unsigned int bottom_y = (req.roi.y_offset + req.roi.height + req.binning_y - 1) / req.binning_y;
        unsigned int width = right_x - left_x;
        unsigned int height = bottom_y - top_y;
        cam_->setRoi(left_x, top_y, width, height);
      } else {
        cam_->setRoiToWholeFrame();
      }

      // Zero duration means "no timeout"
      unsigned long timeout = req.timeout.isZero() ? PVINFINITE : req.timeout.toNSec() / 1e6;
      frame = cam_->grab(timeout);
    }
    catch (prosilica::ProsilicaException &e) {
      if (e.error_code == ePvErrBadSequence)
        throw; // not easily recoverable

      rsp.success = false;
      rsp.status_message = (boost::format("Prosilica exception: %s") % e.what()).str();
      return;
    }

    if (!frame) {
      /// @todo Would be nice if grab() gave more info
      rsp.success = false;
      rsp.status_message = "Failed to capture frame, may have timed out";
      return;
    }

    info = cam_info_;
    image.header.frame_id = img_.header.frame_id;
    if (!processFrame(frame, image, info)) {
      rsp.success = false;
      rsp.status_message = "Captured frame but failed to process it";
      return;
    }
    info.roi.do_rectify = req.roi.do_rectify; // do_rectify should be preserved from request

    rsp.success = true;
  }

  static bool frameToImage(tPvFrame* frame, sensor_msgs::Image &image)
  {
    // NOTE: 16-bit and Yuv formats not supported
    static const char* BAYER_ENCODINGS[] = { "bayer_rggb8", "bayer_gbrg8", "bayer_grbg8", "bayer_bggr8" };

    std::string encoding;
    if (frame->Format == ePvFmtMono8)       encoding = sensor_msgs::image_encodings::MONO8;
    else if (frame->Format == ePvFmtBayer8) 
    {
#if 1
      encoding = BAYER_ENCODINGS[frame->BayerPattern];
#else
      image.encoding = sensor_msgs::image_encodings::BGR8;
      image.height = frame->Height;
      image.width = frame->Width;
      image.step = frame->Width * 3;
      image.data.resize(frame->Height * (frame->Width * 3));
      PvUtilityColorInterpolate(frame, &image.data[2], &image.data[1], &image.data[0], 2, 0);
      return true;
#endif
    }
    else if (frame->Format == ePvFmtRgb24)  encoding = sensor_msgs::image_encodings::RGB8;
    else if (frame->Format == ePvFmtBgr24)  encoding = sensor_msgs::image_encodings::BGR8;
    else if (frame->Format == ePvFmtRgba32) encoding = sensor_msgs::image_encodings::RGBA8;
    else if (frame->Format == ePvFmtBgra32) encoding = sensor_msgs::image_encodings::BGRA8;
    else {
      ROS_WARN("Received frame with unsupported pixel format %d", frame->Format);
      return false;
    }

    uint32_t step = frame->ImageSize / frame->Height;
    return sensor_msgs::fillImage(image, encoding, frame->Height, frame->Width, step, frame->ImageBuffer);
  }
  
  bool processFrame(tPvFrame* frame, sensor_msgs::Image &img, sensor_msgs::CameraInfo &cam_info)
  {
    /// @todo Better trigger timestamp matching
    if ((trigger_mode_ == prosilica::SyncIn1 || trigger_mode_ == prosilica::SyncIn2) && !trig_time_.isZero()) {
      img.header.stamp = cam_info.header.stamp = trig_time_;
      trig_time_ = ros::Time(); // Zero
    }
    else {
      /// @todo Match time stamp from frame to ROS time?
      img.header.stamp = cam_info.header.stamp = ros::Time::now();
    }
    
    /// @todo Binning values retrieved here may differ from the ones used to actually
    /// capture the frame! Maybe need to clear queue when changing binning and/or
    /// stuff binning values into context?
    tPvUint32 binning_x = 1, binning_y = 1;
    if (cam_->hasAttribute("BinningX")) {
      cam_->getAttribute("BinningX", binning_x);
      cam_->getAttribute("BinningY", binning_y);
    }
    // Binning averages bayer samples, so just call it mono8 in that case
    if (frame->Format == ePvFmtBayer8 && (binning_x > 1 || binning_y > 1))
      frame->Format = ePvFmtMono8;

    if (!frameToImage(frame, img))
      return false;

    // Set the operational parameters in CameraInfo (binning, ROI)
    cam_info.binning_x = binning_x;
    cam_info.binning_y = binning_y;
    // ROI in CameraInfo is in unbinned coordinates, need to scale up
    cam_info.roi.x_offset = frame->RegionX * binning_x;
    cam_info.roi.y_offset = frame->RegionY * binning_y;
    cam_info.roi.height = frame->Height * binning_y;
    cam_info.roi.width = frame->Width * binning_x;
    cam_info.roi.do_rectify = (frame->Height != sensor_height_ / binning_y) ||
                              (frame->Width  != sensor_width_  / binning_x);

    count_++;
    return true;
  }
  
  void publishImage(tPvFrame* frame)
  {
    static ros::Time timer = ros::Time::now();
    if (processFrame(frame, img_, cam_info_))
      streaming_pub_.publish(img_, cam_info_);
    ros::Duration duration = ros::Time::now() - timer;
    ROS_INFO("Actual Fps: %d",int(1.0/duration.toSec()));
    timer = ros::Time::now();
  }

  void loadIntrinsics()
  {
    // Retrieve contents of user memory
    std::string buffer(prosilica::Camera::USER_MEMORY_SIZE, '\0');
    cam_->readUserMemory(&buffer[0], prosilica::Camera::USER_MEMORY_SIZE);

    // Parse calibration file
    std::string camera_name;
    if (camera_calibration_parsers::parseCalibrationIni(buffer, camera_name, cam_info_))
      ROS_INFO("Loaded calibration for camera '%s'", camera_name.c_str());
    else
      ROS_WARN("Failed to load intrinsics from camera");
  }

  bool setCameraInfo(sensor_msgs::SetCameraInfo::Request& req,
                     sensor_msgs::SetCameraInfo::Response& rsp)
  {
    ROS_INFO("New camera info received");
    sensor_msgs::CameraInfo &info = req.camera_info;
    
    // Sanity check: the image dimensions should match the max resolution of the sensor.
    tPvUint32 width, height, dummy;
    PvAttrRangeUint32(cam_->handle(), "Width", &dummy, &width);
    PvAttrRangeUint32(cam_->handle(), "Height", &dummy, &height);
    if (info.width != width || info.height != height) {
      rsp.success = false;
      rsp.status_message = (boost::format("Camera_info resolution %ix%i does not match current video "
                                          "setting, camera running at resolution %ix%i.")
                            % info.width % info.height % width % height).str();
      ROS_ERROR("%s", rsp.status_message.c_str());
      return true;
    }
    
    stop();

    std::string cam_name = "prosilica";
    cam_name += hw_id_;
    std::stringstream ini_stream;
    if (!camera_calibration_parsers::writeCalibrationIni(ini_stream, cam_name, info)) {
      rsp.status_message = "Error formatting camera_info for storage.";
      rsp.success = false;
    }
    else {
      std::string ini = ini_stream.str();
      if (ini.size() > prosilica::Camera::USER_MEMORY_SIZE) {
        rsp.success = false;
        rsp.status_message = "Unable to write camera_info to camera memory, exceeded storage capacity.";
      }
      else {
        try {
          cam_->writeUserMemory(ini.c_str(), ini.size());
          cam_info_ = info;
          rsp.success = true;
        }
        catch (prosilica::ProsilicaException &e) {
          rsp.success = false;
          rsp.status_message = e.what();
        }
      }
    }
    if (!rsp.success)
      ROS_ERROR("%s", rsp.status_message.c_str());
    
    start();

    return true;
  }

  void normalizeCallback(tPvFrame* frame)
  {
    unsigned long exposure;
    cam_->getAttribute("ExposureValue", exposure);
    //ROS_WARN("Exposure value = %u", exposure);

    if (exposure == last_exposure_value_)
      consecutive_stable_exposures_++;
    else {
      last_exposure_value_ = exposure;
      consecutive_stable_exposures_ = 0;
    }
  }
  
  void normalizeExposure()
  {
    ROS_INFO("Normalizing exposure");
    /// @todo assert(stopped)

    last_exposure_value_ = 0;
    consecutive_stable_exposures_ = 0;
    cam_->setFrameCallback(boost::bind(&ProsilicaNode::normalizeCallback, this, _1));
    cam_->start(prosilica::Freerun);

    /// @todo thread safety
    while (consecutive_stable_exposures_ < 3)
      boost::this_thread::sleep(boost::posix_time::millisec(250));

    cam_->stop();
  }

  /////////////////
  // Diagnostics //
  /////////////////
  
  void runDiagnostics()
  {
    self_test_->checkTest();
    diagnostic_.update();
  }
  
  void freqStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    double freq = (double)(count_)/diagnostic_.getPeriod();

    if (freq < (.9*desired_freq_))
    {
      status.summary(2, "Desired frequency not met");
    }
    else
    {
      status.summary(0, "Desired frequency met");
    }

    status.add("Images in interval", count_);
    status.add("Desired frequency", desired_freq_);
    status.add("Actual frequency", freq);
    
    count_ = 0;
  }

  void frameStatistics(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    // Get stats from camera driver
    float frame_rate;
    unsigned long completed, dropped;
    cam_->getAttribute("StatFrameRate", frame_rate);
    cam_->getAttribute("StatFramesCompleted", completed);
    cam_->getAttribute("StatFramesDropped", dropped);

    // Compute rolling totals, percentages
    frames_completed_acc_.add(completed - frames_completed_total_);
    frames_completed_total_ = completed;
    unsigned long completed_recent = frames_completed_acc_.sum();
    
    frames_dropped_acc_.add(dropped - frames_dropped_total_);
    frames_dropped_total_ = dropped;
    unsigned long dropped_recent = frames_dropped_acc_.sum();

    float recent_ratio = float(completed_recent) / (completed_recent + dropped_recent);
    float total_ratio = float(completed) / (completed + dropped);

    // Set level based on recent % completed frames
    if (dropped_recent == 0) {
      status.summary(0, "No dropped frames");
    }
    else if (recent_ratio > 0.8f) {
      status.summary(1, "Some dropped frames");
    }
    else {
      status.summary(2, "Excessive proportion of dropped frames");
    }

    status.add("Camera Frame Rate", frame_rate);
    status.add("Recent % Frames Completed", recent_ratio * 100.0f);
    status.add("Overall % Frames Completed", total_ratio * 100.0f);
    status.add("Frames Completed", completed);
    status.add("Frames Dropped", dropped);
  }

  void packetStatistics(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    // Get stats from camera driver
    unsigned long received, missed, requested, resent;
    cam_->getAttribute("StatPacketsReceived", received);
    cam_->getAttribute("StatPacketsMissed", missed);
    cam_->getAttribute("StatPacketsRequested", requested);
    cam_->getAttribute("StatPacketsResent", resent);

    // Compute rolling totals, percentages
    packets_received_acc_.add(received - packets_received_total_);
    packets_received_total_ = received;
    unsigned long received_recent = packets_received_acc_.sum();
    
    packets_missed_acc_.add(missed - packets_missed_total_);
    packets_missed_total_ = missed;
    unsigned long missed_recent = packets_missed_acc_.sum();

    float recent_ratio = float(received_recent) / (received_recent + missed_recent);
    float total_ratio = float(received) / (received + missed);

    if (missed_recent == 0) {
      status.summary(0, "No missed packets");
    }
    else if (recent_ratio > 0.99f) {
      status.summary(1, "Some missed packets");
    }
    else {
      status.summary(2, "Excessive proportion of missed packets");
    }

    // Adjust data rate
    unsigned long data_rate = 0;
    cam_->getAttribute("StreamBytesPerSecond", data_rate);
    unsigned long max_data_rate = cam_->getMaxDataRate();
    
    if (auto_adjust_stream_bytes_per_second_)
    {
      if (max_data_rate < prosilica::Camera::GIGE_MAX_DATA_RATE)
        status.mergeSummary(1, "Max data rate is lower than expected for a GigE port");

      try
      {
        /// @todo Something that doesn't oscillate
        float multiplier = 1.0f;
        if (recent_ratio == 1.0f)
        {
          multiplier = 1.1f;
	    }
        else if (recent_ratio < 0.99f)
        {
          multiplier = 0.9f;
        }
        if (multiplier != 1.0f)
        {
          unsigned long new_data_rate = std::min((unsigned long)(multiplier * data_rate + 0.5), max_data_rate);
          new_data_rate = std::max(new_data_rate, max_data_rate/1000);
          if (data_rate != new_data_rate)
          {
            data_rate = new_data_rate;
            cam_->setAttribute("StreamBytesPerSecond", data_rate);
            ROS_DEBUG("Changed data rate to %lu bytes per second", data_rate);
          }
        }
      }
      catch (prosilica::ProsilicaException &e)
      {
        if (e.error_code == ePvErrUnplugged)
          throw;
        ROS_ERROR("Exception occurred: '%s'\n"
                  "Possible network issue. Attempting to reset data rate to the current maximum.",
                  e.what());
        data_rate = max_data_rate;
        cam_->setAttribute("StreamBytesPerSecond", data_rate);
      }
    }
    
    status.add("Recent % Packets Received", recent_ratio * 100.0f);
    status.add("Overall % Packets Received", total_ratio * 100.0f);
    status.add("Received Packets", received);
    status.add("Missed Packets", missed);
    status.add("Requested Packets", requested);
    status.add("Resent Packets", resent);
    status.add("Data Rate (bytes/s)", data_rate);
    status.add("Max Data Rate (bytes/s)", max_data_rate);
  }

  void packetErrorStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    unsigned long erroneous;
    cam_->getAttribute("StatPacketsErroneous", erroneous);

    if (erroneous == 0) {
      status.summary(0, "No erroneous packets");
    } else {
      status.summary(2, "Possible camera hardware failure");
    }

    status.add("Erroneous Packets", erroneous);
  }

  ////////////////
  // Self tests //
  ////////////////

  // Try to load camera name, etc. Should catch gross communication failure.
  void infoTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.name = "Info Test";

    self_test_->setID(hw_id_);

    std::string camera_name;
    try {
      cam_->getAttribute("CameraName", camera_name);
      status.summary(0, "Connected to Camera");
      status.add("Camera Name", camera_name);
    }
    catch (prosilica::ProsilicaException& e) {
      status.summary(2, e.what());
    }
  }

  // Test validity of all attribute values.
  void attributeTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.name = "Attribute Test";

    tPvAttrListPtr list_ptr;
    unsigned long list_length;

    if (PvAttrList(cam_->handle(), &list_ptr, &list_length) == ePvErrSuccess) {
      status.summary(0, "All attributes in valid range");
      for (unsigned int i = 0; i < list_length; ++i) {
        const char* attribute = list_ptr[i];
        tPvErr e = PvAttrIsValid(cam_->handle(), attribute);
        if (e != ePvErrSuccess) {
          status.summary(2, "One or more invalid attributes");
          if (e == ePvErrOutOfRange)
            status.add(attribute, "Out of range");
          else if (e == ePvErrNotFound)
            status.add(attribute, "Does not exist");
          else
            status.addf(attribute, "Unexpected error code %u", e);
        }
      }
    }
    else {
      status.summary(2, "Unable to retrieve attribute list");
    }
  }

  // Try to capture an image.
  void imageTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.name = "Image Capture Test";

    try {
      if (trigger_mode_ != prosilica::Software) {
        tPvUint32 start_completed, current_completed;
        cam_->getAttribute("StatFramesCompleted", start_completed);
        for (int i = 0; i < 6; ++i) {
          boost::this_thread::sleep(boost::posix_time::millisec(500));
          cam_->getAttribute("StatFramesCompleted", current_completed);
          if (current_completed > start_completed) {
            status.summary(0, "Captured a frame, see diagnostics for detailed stats");
            return;
          }
        }

        // Give up after 3s
        status.summary(2, "No frames captured over 3s in freerun mode");
        return;
      }

      // In software triggered mode, try to grab a frame
      cam_->setRoiToWholeFrame();
      tPvFrame* frame = cam_->grab(500);
      if (frame)
	{
	  status.summary(0, "Successfully triggered a frame capture");
	}
      else
	{
	  status.summary(2, "Attempted to grab a frame, but received NULL");
	}
	
    }
    catch (prosilica::ProsilicaException &e) {
      status.summary(2, e.what());
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "prosilica_driver");

  ros::NodeHandle nh("camera");
  ProsilicaNode pn(nh);
  ros::spin();

  return 0;
}
