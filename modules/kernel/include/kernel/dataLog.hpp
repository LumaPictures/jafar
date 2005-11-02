/* $Id$ */

#ifndef KERNEL_DATA_LOG_HPP
#define KERNEL_DATA_LOG_HPP

#include <string>
#include <fstream>

namespace jafar {

  namespace kernel {

    /** This class helps you log data into files. You can use it in 3
     * ways: 
     *   - in C++ use this class in your code,
     *   - in C++ specialize this class to add logging facility to one of your class,
     *   - in tcl use the functions wrapped in the module \c kernel.
     *
     * An example using tcl and the kernel module (doc/demoDataLog.tcl):
     * \include demoDataLog.tcl
     *
     * Usage in C++ is very similar except that you have direct access
     * to the stream \c log and you can use operator << to send data
     * to the stream (instead of method writeData()):
     * \code
     * using namespace jafar::kernel;
     * DataLog myDataLog("log.dat");
     * double d1 = 1.2;
     * myDataLog.log << d1 << endl << flush; 
     * \endcode
     *
     * \todo rework the code so as to define a nice stream. 
     *
     * \ingroup kernel
     */
    class DataLog {

    private :

      static const std::string _dev_null;
      
      std::string filename;

      /// default comment prefix is '# '
      std::string commentPrefix;

      bool logging;

    public :

#ifndef SWIG // we do not want swig to wrap this attribute

      /** The output stream used to log data.
       * When logging is "off", the stream is redirected to /dev/null.
       */
      std::ofstream log;

#endif

      DataLog();

      DataLog(const std::string& filename_, bool startLogging_ = true);

      ~DataLog();

      /** Change the filename that will be used the next time you
       *  startLogging() or resumeLogging(). It does not affect the
       *  current log stream.
       */
      void setFileName(const std::string& filename_);

      void setCommentPrefix(const std::string& commentPrefix_ = "#");

      /// start logging data to the file (erase the file if it already exists).
      void startLogging();

      /// stop logging data to the file (all data is sent to /dev/null).
      void stopLogging();

      /// same as stopLogging().
      void breakLogging();

      /// start logging appending data to the file.
      void resumeLogging();

      inline bool isLogging() {
        return logging;
      };

      void writeComment(const std::string& comment_);

      /** write current time prefixed with \c commentPrefix to the
       * log. Example: 
       * \code 
       * # 2004-Nov-25 10:12:51 
       * \endcode
       */
      void writeTime();

      template<class Vec>
      void writeVec(Vec v) {
	for (std::size_t i = 0 ; i < v.size() ; ++i) {
	  log << v(i) << " ";
	}
      }
      
      /** This function writes string \c s_ into the log and ends the
       * line. It is intended to be wrapped by swig and used with
       * tcl. In C++ prefer the operator <<.
       */
      void write(const std::string& s_);

      /** This function ends the current line of the log. It is
       * intended to be wrapped by swig and used with tcl. In C++
       * prefer log << std::endl.
       */
      void endl();

      /** This function flushes the log. It is intended to be wrapped
       * by swig and used with tcl. In C++ prefer log << std::flush.
       */
      void flush();

      friend std::ostream& operator <<(std::ostream& s, const DataLog& l_);

    };

    std::ostream& operator <<(std::ostream& s, const DataLog& l_);

  }
}

#endif // KERNEL_DATA_LOG_HPP
