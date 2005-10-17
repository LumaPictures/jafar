/* $Id$ */

#if (defined(__MACH__) && defined(__APPLE__))
#include <libgen.h>
#endif
#include "kernel/jafarDebug.hpp"

using namespace jafar::debug;

/*
 * class Debug
 */

DebugStream::DebugStream() : 
  debugStream(&std::cerr),
  fileStream(),
  modulesLevel(),
  level(Debug),
  debugging()
{
#ifndef JFR_NDEBUG
  (*debugStream) << "D:kernel/DebugStream: debug stream is created." << std::endl;
#endif
}

DebugStream::~DebugStream()
{
#ifndef JFR_NDEBUG
  (*debugStream) << "D:kernel/DebugStream: debug stream is destroyed." << std::endl;
#endif
}

void DebugStream::setup(std::string const& module_, Level level_)
{
  DebugStream& dbg = instance();
  dbg.debugging = level_ <= dbg.level;
  if (dbg.debugging) {
    std::map<std::string, Level>::const_iterator it = dbg.modulesLevel.find(module_);
    if (it != dbg.modulesLevel.end()) 
      dbg.debugging = level_ <= it->second;
  }
}

void DebugStream::sendLocation(std::string const& module_, char const* file_, int line_)
{
  DebugStream& dbg = instance();
  if (dbg.isDebugging()) {
#ifndef JFR_DEBUG_FULL_PATH
    dbg << module_ << "/" << basename(file_) << ":" << line_ << ": ";
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
