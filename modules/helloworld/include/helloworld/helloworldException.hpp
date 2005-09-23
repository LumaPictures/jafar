/* $Id$ */

#ifndef HELLOWORLD_EXCEPTION_HPP
#define HELLOWORLD_EXCEPTION_HPP

#include "kernel/jafarException.hpp"

namespace jafar {

  namespace helloworld {

    /** Base class for all exceptions defined in the module
     * helloworld.
     *
     * @ingroup helloworld
     */
    class HelloworldException : public jafar::kernel::Exception {

    public:

      /** This enumeration defines exceptions id for the module
       * helloworld.
       */
      enum ExceptionId {
        EMPTY_HELLO,  /**< The hello string is empty */
        BAD_FORMAT    /**< bad format for string hello */
      };

      /** Constructor. You should not use this constructor directly,
       * prefer macros jfrThrowEx or jfrCreateEx which fill for you
       * parameters \c file_ and \c line_.
       *
       * @param id_ exception id
       * @param message_ message used for debug
       * @param file_ where the exception was thrown
       * @param line_ where the exception was thrown
       */
      HelloworldException(ExceptionId id_, 
                          const std::string& message_, 
                          const std::string& file_, int line_) throw();

      virtual ~HelloworldException() throw();

      ExceptionId getExceptionId() const throw();

    protected:

      ExceptionId id;

      static std::string exceptionIdToString(ExceptionId id_) throw();
      
    };

    /** hello attribute not initialized. This is an over simple
     * example on how to add your own exception.  
     *
     * @ingroup helloworld
     */
    class HelloworldFormatException : public HelloworldException {

    public:

      HelloworldFormatException(const std::string& hello_, 
                                const std::string& message_, 
                                const std::string& file_, int line_) throw();

      virtual ~HelloworldFormatException() throw();

      const std::string& getHello() const throw();

    private:

      /// the proposed hello string
      std::string hello;

    };

  }
}

#endif // HELLOWORLD_EXCEPTION_HPP

