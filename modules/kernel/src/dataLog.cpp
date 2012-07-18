/* $Id$ */

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/tokenizer.hpp"

#include "kernel/jafarException.hpp"
#include <kernel/jafarMacro.hpp>
#include "kernel/dataLog.hpp"

using namespace jafar::kernel;



/*
 * LoggerTask
 */

void LoggerTask::task()
{ JFR_GLOBAL_TRY
  // this thread should never be real time
  kernel::setCurrentThreadScheduler(SCHED_OTHER, 0);
  kernel::setCurrentThreadPriority(niceness);

  const int warning_queue_size = 500;
  int remain = 0, prev_remain = 0;

  Loggable *object;

  while (!stopping || remain)
  {
    // wait for and get next data to save
    cond.wait(boost::lambda::_1 != 0, false);
    object = queue.back();
    queue.pop_back();
    cond.var--;
    remain = cond.var;
    cond.unlock();

    object->log();
    delete object;

    bool remain_cut = (remain > warning_queue_size ? remain : 0);
    if (remain_cut > prev_remain || (remain_cut == 0 && prev_remain != 0))
      std::cout << "LoggerTask: " << remain << " objects in queue." << std::endl;
    prev_remain = remain_cut;
  }
  JFR_GLOBAL_CATCH
}

LoggerTask::LoggerTask(int niceness): cond(0), stopping(false), niceness(niceness)
{
  task_thread = new boost::thread(boost::bind(&LoggerTask::task,this));
}

void LoggerTask::push(Loggable *loggable)
{
  cond.lock();
  queue.push_front(loggable);
  cond.var++;
  cond.unlock();
  cond.notify();
}

void LoggerTask::stop(bool wait)
{
  stopping = true;
  if (wait) join();
}

void LoggerTask::join()
{
  task_thread->join();
}


/*
 * LoggableString
 */

void LoggableString::log()
{
  logStream << content;
  JFR_IO_STREAM(logStream, "LoggableString: write error");
}


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
  loggerTask(NULL),
  loggables() 
{
  logHeaderLine << commentPrefix_;
  JFR_IO_STREAM(logStream,
		"DataLogger: error while opening file " << logFilename_);
}

DataLogger::~DataLogger()
{
	LoggablesList::iterator end = loggables.end();
	for(LoggablesList::iterator current = loggables.begin();
			current != end; ++current)
	{
		// Set the p_logger for all loggables to NULL, to indicate that it is
		// no longer valid... this way, things should work whether all the
		// loggables are deleted first, or the logger
		DataLoggable* curP = *current;
		curP->p_logger = NULL;
	}
}


void DataLogger::write(std::ostringstream & content)
{
  if (loggerTask)
    loggerTask->push(new LoggableString(logStream, content.str()));
  else
  {
    logStream << content.str();
    JFR_IO_STREAM(logStream, "DataLogger: write error");
  }
  content.str("");
}


void DataLogger::writeComment(std::string const& comment_)
{
  logContent << commentPrefix << commentPrefix << " " << comment_ << " " << std::endl;
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
  write(logContent);
}

void DataLogger::removeLoggable(DataLoggable & loggable_)
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
		write(logContent); // flush
		logHeaderLine << std::endl;
		write(logHeaderLine);
		logStarted = true;
	}

  // ask loggables to log their data in turn
  for (LoggablesList::const_iterator it = loggables.begin() ; it != loggables.end() ; ++it)
    (**it).writeLogData(*this);
  logContent << std::endl;
  write(logContent);

  // dispatch the log() event to the slaves
  for (LoggersList::iterator it = slaves.begin() ; it != slaves.end() ; ++it)
    (**it).log();
}


void DataLogger::logStats()
{
  // ask loggables to log their data in turn
  for (LoggablesList::const_iterator it = loggables.begin() ; it != loggables.end() ; ++it)
    (**it).writeLogStats(*this);
  logContent << std::endl;
  write(logContent);

  // dispatch the log() event to the slaves
  for (LoggersList::iterator it = slaves.begin() ; it != slaves.end() ; ++it) {
    (**it).logStats();
  }
}


void DataLogger::writeLegend(std::string const& legend_)
{
  nbColumns++;
  logContent << commentPrefix << " " << nbColumns  << ":" << legend_ << std::endl;
  logHeaderLine << legend_ << separator;
}

void DataLogger::writeLegendTokens(std::string const& legendTokens_, std::string const& tokenSep_)
{
  typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
  boost::char_separator<char> sep(tokenSep_.c_str());
  tokenizer tokens(legendTokens_, sep);
  logContent << commentPrefix << " ";
  for (tokenizer::iterator it = tokens.begin(); it != tokens.end(); ++it) {
    nbColumns++;
    logContent << nbColumns << ":" << *it << separator;
    logHeaderLine << *it << separator;
  }
  logContent << std::endl;
}

