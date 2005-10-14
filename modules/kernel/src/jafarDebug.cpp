/* $Id$ */

#include "kernel/jafarDebug.hpp"

using namespace jafar::kernel;

/*
 * class Debug
 */

DebugStream::DebugStream() : 
  debugStream(&std::cerr),
  p_on(true),
  modulesLevel(),
  level(Debug),
  debugging()
{}

void DebugStream::setup(std::string const& module_, Level level_)
{
  debugging = p_on && (level_ <= level);
  if (debugging) {
    std::map<std::string, Level>::const_iterator it = modulesLevel.find(module_);
    if (it != modulesLevel.end()) 
      debugging = level_ <= it->second;
  }
}
