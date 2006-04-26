/* $Id$ */

#include <fstream>

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/tokenizer.hpp"

#include "kernel/keyValueFile.hpp"

using namespace jafar::kernel;

/*
 * class KeyValueFileReader
 */

KeyValueFile::KeyValueFile(std::string const& keyValueSeparator_, char commentPrefix_) :
  keyValueSeparator(keyValueSeparator_),
  commentPrefix(commentPrefix_)
{}

void KeyValueFile::readFile(std::string const& filename) {
  using namespace std;

  ifstream file(filename.c_str());

  JFR_IO_STREAM(file, "KeyValueFile:readFile: failed to open file: " << filename);

  int lineNumber = 0;
  
  // reading the lines
  string line;
  string key;
  string value;

  // tokenizer
  typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
  boost::char_separator<char> sep(keyValueSeparator.c_str());

  while(getline(file,line)) {
    lineNumber++;

    // this is an empty line
    // or a comment
    if (line.size() == 0 || line.at(0) == commentPrefix)
      continue;
	  
    tokenizer tokens(line, sep);
    tokenizer::iterator it = tokens.begin();

    JFR_PRED_ERROR(it != tokens.end(),
		   KernelException,
		   KernelException::KEYVALUEFILE_INVALID_LINE,
		   "KeyValueFile:readFile: invalid line at " << filename << ":" << lineNumber);

    key = *it;
    it++;
    
    JFR_PRED_ERROR(it != tokens.end(),
		   KernelException,
		   KernelException::KEYVALUEFILE_INVALID_LINE,
		   "KeyValueFile:readFile: invalid line at " << filename << ":" << lineNumber);

    value = *it;
    it++;

    JFR_PRED_ERROR(it == tokens.end(),
		   KernelException,
		   KernelException::KEYVALUEFILE_INVALID_LINE,
		   "KeyValueFile:readFile: invalid line at " << filename << ":" << lineNumber);
    
    KeyValueMap::const_iterator itKeyValue = keyValue.find(key);
    JFR_PRECOND(itKeyValue == keyValue.end(), "KeyValueMap::readFile: item (" 
		<< key << keyValueSeparator << value << ") "
		<< " is already defined (" << itKeyValue->first << keyValueSeparator << itKeyValue->second << ") " << endl
		<<  "Redefined at " << filename << ":" << lineNumber);

    //    JFR_DEBUG(key << ": " << value);
    keyValue[key] = value;
  }

}

void KeyValueFile::writeFile(std::string const& filename) {

  std::ofstream file(filename.c_str());
  JFR_IO_STREAM(file, "KeyValueFile:writeFile: failed to open file: " << filename);

  file << commentPrefix <<" Generated by KeyValueFile::writeFile" << std::endl;
  file << commentPrefix <<" " << boost::posix_time::to_simple_string(boost::posix_time::second_clock::local_time()) << std::endl;
  file << std::endl;

  for (KeyValueMap::iterator it = keyValue.begin() ; it !=  keyValue.end() ; ++it) {
    file << it->first << keyValueSeparator << " " << it->second << std::endl;
  }
}

/*
 * class KeyValueFileLoad
 */

void KeyValueFileLoad::load(std::string const& filename,
			    std::string const& keyValueSeparator_, char commentPrefix_)
{
  JFR_TRACE_BEGIN;
  KeyValueFile kvf(keyValueSeparator_, commentPrefix_);
  kvf.readFile(filename);
  loadKeyValueFile(kvf);
  JFR_TRACE_END("KeyValueFile:load: loading file " << filename);
}

/*
 * class KeyValueFileSave
 */

void KeyValueFileSave::save(std::string const& filename,
			    std::string const& keyValueSeparator_, char commentPrefix_)
{
  JFR_TRACE_BEGIN;
  KeyValueFile kvf(keyValueSeparator_, commentPrefix_);
  saveKeyValueFile(kvf);
  kvf.writeFile(filename);
  JFR_TRACE_END("KeyValueFile:load: loading file " << filename);
}
