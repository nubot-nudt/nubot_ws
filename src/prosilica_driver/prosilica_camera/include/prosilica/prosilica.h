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

#ifndef PROSILICA_H
#define PROSILICA_H

#include <stdexcept>
#include <string>
#include <boost/function.hpp>
#include <boost/thread.hpp>

// PvApi.h isn't aware of the usual detection macros
// TODO: support systems other than x86 linux
#define _LINUX
#define _x86
#include <PvApi.h>
#undef _LINUX
#undef _x86

namespace prosilica {

struct ProsilicaException : public std::runtime_error
{
  tPvErr error_code;
  
  ProsilicaException(tPvErr code, const char* msg)
    : std::runtime_error(msg), error_code(code)
  {}
};

void init();                // initializes API
void fini();                // releases internal resources
size_t numCameras();        // number of cameras found
uint64_t getGuid(size_t i); // camera ids

/// According to FrameStartTriggerMode Enum - AVT GigE Camera and Driver Attributes
/// Firmware 1.38 April 7,2010
enum FrameStartTriggerMode
{
  Freerun, 
  SyncIn1, 
  SyncIn2, 
  FixedRate, 
  Software,
  None
};

enum AcquisitionMode
{
  Continuous,
  SingleFrame,
  MultiFrame,
  Recorder
};

enum AutoSetting
{
  Manual,
  Auto,
  AutoOnce
};

class Camera
{
public:
  static const size_t DEFAULT_BUFFER_SIZE = 4;
  
  Camera(unsigned long guid, size_t bufferSize = DEFAULT_BUFFER_SIZE);
  Camera(const char* ip_address, size_t bufferSize = DEFAULT_BUFFER_SIZE);

  ~Camera();

  //! Must be used before calling start() in a non-triggered mode.
  void setFrameCallback(boost::function<void (tPvFrame*)> callback);
  //! Start capture.
  void start(FrameStartTriggerMode = Freerun, AcquisitionMode = Continuous);
  //! Stop capture.
  void stop();
  //! Capture a single frame from the camera. Must be called after
  //! start(Software Triggered).
  tPvFrame* grab(unsigned long timeout_ms = PVINFINITE);

  void setExposure(unsigned int val, AutoSetting isauto = Manual);
  void setGain(unsigned int val, AutoSetting isauto = Manual);
  void setWhiteBalance(unsigned int blue, unsigned int red,
                       AutoSetting isauto = Manual);

  void setRoi(unsigned int x, unsigned int y,
              unsigned int width, unsigned int height);
  void setRoiToWholeFrame();
  void setBinning(unsigned int binning_x = 1, unsigned int binning_y = 1);

  //! Returns true if camera supports the attribute.
  bool hasAttribute(const std::string &name);

  //! General get/set attribute functions.
  void getAttributeEnum(const std::string &name, std::string &value);
  void getAttribute(const std::string &name, tPvUint32 &value);
  void getAttribute(const std::string &name, tPvFloat32 &value);
  void getAttribute(const std::string &name, std::string &value);
  
  void setAttributeEnum(const std::string &name, const std::string &value);
  void setAttribute(const std::string &name, tPvUint32 value);
  void setAttribute(const std::string &name, tPvFloat32 value);
  void setAttribute(const std::string &name, const std::string &value);

  void runCommand(const std::string& name);
  
  unsigned long guid();

  unsigned long getMaxDataRate();
  static const unsigned long GIGE_MAX_DATA_RATE = 115000000;
  
  //! Data must have size <= USER_MEMORY_SIZE bytes.
  static const size_t USER_MEMORY_SIZE = 512;
  void writeUserMemory(const char* data, size_t size);
  void readUserMemory(char* data, size_t size);

  //! Get raw PvApi camera handle.
  tPvHandle handle();
  
private:
  tPvHandle handle_; // handle to open camera
  tPvFrame* frames_; // array of frame buffers
  tPvUint32 frameSize_; // bytes per frame
  size_t bufferSize_; // number of frame buffers
  FrameStartTriggerMode FSTmode_;
  AcquisitionMode Amode_;
  boost::function<void (tPvFrame*)> userCallback_;
  boost::mutex frameMutex_;

  void setup();
  
  static void frameDone(tPvFrame* frame);
};

} // namespace prosilica

#endif
