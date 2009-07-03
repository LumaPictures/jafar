/* $Id$ */

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/tokenizer.hpp"

#include "kernel/jafarException.hpp"
#include "kernel/dataLog.hpp"

using namespace jafar::kernel;

/*
 * DataLoggable
 */

DataLoggable::DataLoggable() : p_logger(0) {}

DataLoggable::~DataLoggable() {
  if (p_logger)
    p_logger->removeLoggable(*this);
}

void DataLoggable::setLogger(DataLogger& logger) {
  JFR_PRECOND(!p_logger,
	      "DataLoggable: only one logger is supported");
  p_logger = &logger;
}

/*
 * DataLogger
 */

DataLogger::DataLogger(std::string const& logFilename_, 
		       char separator_, 
		       char commentPrefix_) :
  logStream(logFilename_.c_str()),
  logStarted(false),
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

void DataLogger::addLoggable(DataLoggable& loggable_)
{
  loggables.push_back(&loggable_);
  loggable_.setLogger(*this);
  loggable_.writeLogHeader(*this);
  loggable_.addMembersToLog(*this);
}

void DataLogger::removeLoggable(DataLoggable const& loggable_)
{
  loggables.remove(&loggable_);
}

void DataLogger::addSlaveLogger(DataLogger& logger_)
{
  slaves.push_back(&logger_);
}

void DataLogger::log()
{
	if (!logStarted)
	{
		logStream << logHeaderLine.str() << std::endl;
		logStarted = true;
	}

  // ask loggables to log their data in turn
  for (LoggablesList::const_iterator it = loggables.begin() ; it != loggables.end() ; ++it) {
    (**it).writeLogData(*this);
  }
  logStream << std::endl;
  JFR_IO_STREAM(logStream, "DataLogger::log");

  // dispatch the log() event to the slaves
  for (LoggersList::iterator it = slaves.begin() ; it != slaves.end() ; ++it) {
    (**it).log();
  }
}


void DataLogger::logStats()
{
  // ask loggables to log their data in turn
  for (LoggablesList::const_iterator it = loggables.begin() ; it != loggables.end() ; ++it) {
    (**it).writeLogStats(*this);
  }
  logStream << std::endl;
  JFR_IO_STREAM(logStream, "DataLogger::log");

  // dispatch the log() event to the slaves
  for (LoggersList::iterator it = slaves.begin() ; it != slaves.end() ; ++it) {
    (**it).logStats();
  }
}


void DataLogger::writeLegend(std::string const& legend_)
{
  nbColumns++;
  logStream << commentPrefix << " " << nbColumns  << ":" << legend_ << std::endl;
  logHeaderLine << legend_ << separator;
  JFR_IO_STREAM(logStream,
		"DataLogger::writeLegend: error while writting :\n" << legend_);
}

void DataLogger::writeLegendTokens(std::string const& legendTokens_, std::string const& tokenSep_)
{
  typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
  boost::char_separator<char> sep(tokenSep_.c_str());
  tokenizer tokens(legendTokens_, sep);
  logStream << commentPrefix << " ";
  for (tokenizer::iterator it = tokens.begin(); it != tokens.end(); ++it) {
    nbColumns++;
    logStream << nbColumns << ":" << *it << separator;
    logHeaderLine << *it << separator;
  }
  logStream << std::endl;
  JFR_IO_STREAM(logStream,
		"DataLogger::writeLegendTokens: error while writting :\n" << legendTokens_);
}

