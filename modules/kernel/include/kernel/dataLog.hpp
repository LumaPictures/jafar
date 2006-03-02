/* $Id$ */

#ifndef KERNEL_DATA_LOG_HPP
#define KERNEL_DATA_LOG_HPP

#include <string>
#include <fstream>
#include <list>

namespace jafar {

  namespace kernel {


    class DataLogger;
    class DataLoggable;

    /** Interface DataLoggable. Objects which send data to a
     * DataLogger have to implement this interface.
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

      friend class DataLogger;

    }; // class Loggable

    /** This object logs data.
     *
     * \ingroup kernel
     */
    class DataLogger {

    private:

      std::ofstream logStream;

      /// default separator is whitespace
      char separator;

      /// default comment prefix is '# '
      char commentPrefix;

      unsigned int nbColumns;

      typedef std::list<DataLoggable const*> LoggablesList;
      LoggablesList loggables;

      typedef std::list<DataLogger*> LoggersList;
      LoggersList slaves;

    public:

      /**
       * @param logFilename_ filename of the data log.
       * @param separator_ string used to separate the data
       * @param commentPrefix_ string which starts a comment line
       */
      DataLogger(std::string const& logFilename_, 
		 char separator_ = ' ', 
		 char commentPrefix_ = '#');


      /// write \a comment_
      void writeComment(std::string const& comment_);

      /// write current date as a comment
      void writeCurrentDate();

      /// add \a loggable_ to be logged by this logger.
      void addLoggable(DataLoggable& loggable_);

      /// remove \a loggable_
      void removeLoggable(DataLoggable const& loggable_);

      /** Add slave \a logger_. log() events are dispatched to the
       * slaves.
       */
      void addSlaveLogger(DataLogger& logger_);

      /// this method triggers a log.
      void log();

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
	logStream << d << separator;
      }

      /// write a NaN to the log
      void writeNaN() {
	logStream << "NaN" << separator;
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
