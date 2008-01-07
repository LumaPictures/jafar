/* $Id$ */

#if (defined(__MACH__) ||  defined(__APPLE__) || defined(__NetBSD__) \
		|| defined(__OpenBSD__) || defined(__FreeBSD__))
#include <libgen.h>
#endif

#include <cstring>

#include "kernel/jafarDebug.hpp"

using namespace jafar::debug;

/*
 * class Debug
 */

DebugStream::DebugStream() : 
  debugStream(&std::cerr),
  fileStream(),
  modulesLevel(),
  defaultLevel(Debug),
  debugging()
{
#ifndef JFR_NDEBUG
  //  (*debugStream) << "D:kernel/DebugStream: debug stream is created." << std::endl;
#endif
}

DebugStream::~DebugStream()
{
#ifndef JFR_NDEBUG
  //  (*debugStream) << "D:kernel/DebugStream: debug stream is destroyed." << std::endl;
#endif
}

void DebugStream::setup(std::string const& module_, Level level_)
{
  DebugStream& dbg = instance();

  std::map<std::string, Level>::const_iterator it = dbg.modulesLevel.find(module_);
  if (it != dbg.modulesLevel.end()) 
    dbg.debugging = level_ <= it->second;
  else
    dbg.debugging = level_ <= dbg.defaultLevel;

  if (dbg.isDebugging()) {
    switch (level_) {
    case Trace:
      dbg << "E:";
      break;
    case Warning:
      dbg << "** WARNING:";
      break;
    default:
      dbg << "D:";
      break;
    }
  }
}

void DebugStream::sendLocation(std::string const& module_, char const* file_, int line_)
{
  DebugStream& dbg = instance();
  if (dbg.isDebugging()) {
#ifndef JFR_DEBUG_FULL_PATH
#ifdef __NetBSD__
    dbg << module_ << "/" << basename(const_cast<char*> (file_)) << ":" << line_ << ": ";
#else
    dbg << module_ << "/" << basename(file_) << ":" << line_ << ": ";
#endif
#else
    dbg << file_ << ":" << line_ << ": ";
#endif
  }
}

void DebugStream::setOutputFile(std::string const& filename_) 
{
  DebugStream& dbg = instance();
  dbg.fileStream.close();
  dbg.fileStream.open(filename_.c_str());
  JFR_IO_STREAM(dbg.fileStream, "DebugStream::setOutputFile: impossible to send debug to file " << filename_);
  dbg.debugStream = &dbg.fileStream;
}

void DebugStream::setDefaultStream() {
  DebugStream& dbg = instance();
  dbg.fileStream.close();
  dbg.debugStream = &std::cerr;
}
