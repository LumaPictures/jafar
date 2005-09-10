/* $Id$ */

#include "kernel/dataLog.hpp"

#include "boost/date_time/posix_time/posix_time.hpp"

using namespace jafar::kernel;

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
