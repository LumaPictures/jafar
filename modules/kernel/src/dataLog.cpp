/* $Id$ */

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/tokenizer.hpp"

#include "kernel/jafarException.hpp"
#include "kernel/dataLog.hpp"

using namespace jafar::kernel;

/*
 * DataLogger
 */

DataLogger::DataLogger(std::string const& logFilename_, 
		       std::string const& separator_, 
		       std::string const& commentPrefix_) :
  logStream(logFilename_.c_str()),
  separator(separator_),
  commentPrefix(commentPrefix_),
  nbColumns(0),
  loggables() 
{
  JFR_IO_STREAM(logStream,
		"DataLogger: error while opening file " << logFilename_);
}

void DataLogger::writeComment(std::string const& comment_)
{
  logStream << commentPrefix << commentPrefix << " " << comment_ << " " << std::endl;
  JFR_IO_STREAM(logStream,
		"DataLogger::writeComment: error while writting :\n" << comment_);
}

void DataLogger::writeCurrentDate()
{
//   JFR_TRACE_BEGIN;
  writeComment(boost::posix_time::to_simple_string(boost::posix_time::second_clock::local_time()));
//   JFR_TRACE_END("DataLogger::writeCurrentDate");
}

void DataLogger::addLoggable(DataLoggable const& loggable_)
{
  loggables.push_back(&loggable_);
  loggable_.writeLogHeader(*this);
}

void DataLogger::log()
{
  for (LoggablesList::const_iterator it = loggables.begin() ; it != loggables.end() ; ++it) {
    (**it).writeLogData(*this);
  }
  logStream << std::endl;
    JFR_IO_STREAM(logStream,
		  "DataLogger::log");
}


void DataLogger::writeLegend(std::string const& legend_)
{
  nbColumns++;
  logStream << commentPrefix << " " << nbColumns  << ":" << legend_ << std::endl;
  JFR_IO_STREAM(logStream,
		"DataLogger::writeLegend: error while writting :\n" << legend_);
}

void DataLogger::writeLegendTokens(std::string const& legendTokens_, std::string const& separator_)
{
  typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
  boost::char_separator<char> sep(separator_.c_str());
  tokenizer tokens(legendTokens_, sep);
  logStream << commentPrefix << " ";
  for (tokenizer::iterator it = tokens.begin(); it != tokens.end(); ++it) {
    nbColumns++;
    logStream << nbColumns << ":" << *it << separator_;
  }
  logStream << std::endl;
  JFR_IO_STREAM(logStream,
		"DataLogger::writeLegendTokens: error while writting :\n" << legendTokens_);
}

