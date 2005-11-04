/* $Id$ */

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/tokenizer.hpp"

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
{}

void DataLogger::writeComment(std::string const& comment_)
{
  logStream << commentPrefix << " " << comment_ << std::endl;
}

void DataLogger::writeCurrentDate()
{
  writeComment(boost::posix_time::to_simple_string(boost::posix_time::second_clock::local_time())); 
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
}


void DataLogger::writeLegend(std::string const& legend_)
{
  nbColumns++;
  logStream << commentPrefix << " " << nbColumns  << ":" << legend_ << std::endl;
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
}

/*
 * DataLog
 */

const std::string DataLog::_dev_null = "/dev/null";

DataLog::DataLog() : filename(_dev_null),
                     commentPrefix("# "),
                     logging(false),
                     log(_dev_null.c_str())
{}

DataLog::DataLog(const std::string& filename_, bool startLogging_) :
  filename(filename_),
  commentPrefix("# "),
  logging(false),
  log(_dev_null.c_str())
{
  if (startLogging_) {
    startLogging();
  }
}

DataLog::~DataLog() {
  log.close();
}

void DataLog::setFileName(const std::string& filename_) {
  filename = filename_;
}

void DataLog::setCommentPrefix(const std::string& commentPrefix_) {
  commentPrefix = commentPrefix_;
}

void DataLog::startLogging() {
  log.close();
  log.open(filename.c_str(), std::ios_base::out); // FIXME
  logging = true;
}

void DataLog::stopLogging() {
  log.close();
  logging = false;
  log.open(_dev_null.c_str());
}

void DataLog::breakLogging() {
  stopLogging();
}

void DataLog::resumeLogging() {
  log.close();
  log.open(filename.c_str(), std::ios_base::app); // FIXME
  logging = true;
}

void DataLog::writeComment(const std::string& comment_) {
  if (logging)
    log << commentPrefix << comment_ << std::endl;
}

void DataLog::writeTime() {
  if (logging)
    log << commentPrefix << boost::posix_time::second_clock::local_time() << std::endl;
}

void DataLog::write(const std::string& s_) {
  if (logging)
    log << s_ << std::endl;
}

void DataLog::endl() {
  if (logging)
    log << std::endl;
}

void DataLog::flush() {
  if (logging)
    log << std::flush;
}

std::ostream& jafar::kernel::operator <<(std::ostream& s, const DataLog& l_) {
  s << "file: " << l_.filename << std::endl;
  s << "logging: " << l_.logging << std::endl;
  s << "comment: " << l_.commentPrefix << std::endl;
  return s;
}
