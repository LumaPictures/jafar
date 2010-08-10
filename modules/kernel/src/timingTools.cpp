/* $Id$ */

#include "kernel/timingTools.hpp"

using namespace boost::posix_time;



namespace jafar {
namespace kernel {


std::ostream& operator<<(std::ostream& os, const jafar::kernel::Timestamp &t)
{
	const std::streamsize oldprecision = os.precision();
	os << std::setprecision(19) << t.data;
	os.precision(oldprecision);
	return os;
}



/*
 * class FrameRate
 */

FrameRate::FrameRate(FrameRate::TypeUpdate upd_, int updTimeInt_, int updFrameInt_) {
  upd = upd_;
  lastUpdate = microsec_clock::local_time();
  fpsUpdateInterval = millisec(updTimeInt_);
  numFramesInterval = updFrameInt_;
  numFrames = 0;
  fps = 0;
}


void FrameRate::updateFps() {
  numFrames++;
  ptime currentUpdate = microsec_clock::local_time();
  time_duration currentInt;
  switch (upd) {
  case FrameRate::ontime:
    if( (currentInt = currentUpdate - lastUpdate) > fpsUpdateInterval )
      {
	fps = numFrames / static_cast<float>(currentInt.total_seconds() + 
					     currentInt.fractional_seconds() / 1000000000.0) ;
	lastUpdate = currentUpdate;
	numFrames = 0;
      }
    break;
  case FrameRate::onframe:
    if( numFrames > numFramesInterval )
      {
	time_duration currentInt  = currentUpdate - lastUpdate;
	fps = numFrames / static_cast<float>(currentInt.total_seconds() + 
					     currentInt.fractional_seconds() / 1000000000.0) ;
	lastUpdate = currentUpdate;
	numFrames = 0;
      }
    break;
  } 
}
  
float FrameRate::getFps() {
  return fps;
}

}}
