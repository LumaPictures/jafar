/* $Id$ */

#ifndef KERNEL_DATA_LOG_HPP
#define KERNEL_DATA_LOG_HPP

#include <string>
#include <sstream>
#include <fstream>
#include <list>
#include <iomanip>

#include <kernel/threads.hpp>

namespace jafar {

  namespace kernel {


    /** Interface Loggable. Objects that aim at being logged
     * by LoggerTask have to implement this interface and contain
     * everything that is required to log the object (file names, file
     * descriptors, content...)
     *
     * \ingroup kernel
     */
    class Loggable
    {
     public:
      virtual void log() = 0;
      virtual ~Loggable() {}
    };


    /** This object creates a thread that log objects.
     *
     * \ingroup kernel
     */
    class LoggerTask
    {
     private:
      kernel::VariableCondition<size_t> cond;
      std::list<Loggable*> queue;
      bool stopping;
      boost::thread *task_thread;
      int niceness;

     private:
      void task();

     public:
      LoggerTask(int niceness = 0);
      void push(Loggable *loggable);
      void stop(bool wait = true);
      void join();
    };


    /** Loggable object for DataLogger
     *
     * \ingroup kernel
     */
    class LoggableString: public Loggable
    {
     private:
      std::ofstream &logStream;
      std::string content;
     public:
       LoggableString(std::ofstream & logStream, std::string const & content):
        logStream(logStream), content(content) {}
       virtual void log();
    };


    class DataLogger;
    class DataLoggable;

    /** Interface DataLoggable. Objects which send data to a
     * DataLogger have to implement this interface.
     *
     * \todo fuse writeLogHeader() and writeLogData(), this would ensure that
     * header and data does actually match.
     *
     * \ingroup kernel
     */
    class DataLoggable {

    private:

      /// My logger
      DataLogger* p_logger;

      void setLogger(DataLogger& logger);

    protected:

      DataLoggable();
      virtual ~DataLoggable();

      /** Implements this method calling repeatidly \a log
       * methods. You should use writeComment(), writeLegend() or
       * writeLegendTokens().
       */
      virtual void writeLogHeader(DataLogger& log) const = 0;

      /** Once the loggable has been added, this method is called so
       * that the loggable can add some of its members to the
       * log. Implements this method calling addLoggable(). By default
       * this method is empty.
       */
      virtual void addMembersToLog(DataLogger& log) const {}

      /** Implements this method calling repeatidly \a log
       * methods. You should use writeData() or writeDataVector().
       */
      virtual void writeLogData(DataLogger& log) const = 0;

      /** write stats at the end of the log
       */
      virtual void writeLogStats(DataLogger& log) const {}


      friend class DataLogger;

    }; // class Loggable



    /** This object logs data in a single log file.
     *
     * \ingroup kernel
     */
    class DataLogger {

    private:

      std::ofstream logStream;
      std::ostringstream logHeaderLine;
      std::ostringstream logContent;
      bool logStarted;

      /// default separator is whitespace
      char separator;

      /// default comment prefix is '# '
      char commentPrefix;

      unsigned int nbColumns;

      LoggerTask *loggerTask;

      typedef std::list<DataLoggable *> LoggablesList;
      LoggablesList loggables;

      typedef std::list<DataLogger*> LoggersList;
      LoggersList slaves;

      void write(std::ostringstream & content);

    public:

      /**
       * @param logFilename_ filename of the data log.
       * @param separator_ string used to separate the data
       * @param commentPrefix_ string which starts a comment line
       */
      DataLogger(std::string const& logFilename_,
        char separator_ = '\t',
        char commentPrefix_ = '#');

      ~DataLogger();

      /// setting a logger task defers actual file write to a separate thread
      void setLoggerTask(LoggerTask *loggerTask) { this->loggerTask = loggerTask; }

      /// write \a comment_
      void writeComment(std::string const& comment_);

      /// write current date as a comment
      void writeCurrentDate();

      /// add \a loggable_ to be logged by this logger. The log header is printed at this moment, so you should use this function when all objects have been created.
      void addLoggable(DataLoggable& loggable_);

      /// remove \a loggable_
      void removeLoggable(DataLoggable & loggable_);

      /** Add slave \a logger_. log() events are dispatched to the
       * slaves.
       */
      void addSlaveLogger(DataLogger& logger_);

      /// this method triggers a log.
      void log();
      /// this method triggers a log of stats.
      void logStats();

      /** This method writes a line of legend. Example:
       *
       * \code
       *  log.writeLegend("x");
       * \endcode
       */
      void writeLegend(std::string const& legend_);

      /** This method writes lines of legend based on the tokens found
       * in \a legendTokens_ and seperated by \a separator_. It uses
       * boost::tokenizer. Example:
       *
       * \code
       *  log.writeLegendTokens("x y z");
       * \endcode
       */
      void writeLegendTokens(std::string const& legendTokens_, std::string const& tokenSep_ = " ");

      /// this method logs any data
      template <typename T>
      void writeData(T const& d) {
        logContent << d << separator;
      }
      void writeNewLine() {
        logContent << std::endl;
      }

      /// write a NaN to the log
      void writeNaN() {
        logContent << "NaN" << separator;
      }

      /// this method logs any data vector
      template <class Vec>
      void writeDataVector(Vec const& v) {
        for (std::size_t i = 0 ; i < v.size() ; ++i)
          writeData(v[i]);
      }

    }; // class DataLogger

  }
}

#endif // KERNEL_DATA_LOG_HPP
