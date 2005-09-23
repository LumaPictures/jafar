/* $Id$ */

#ifndef HELLOWORLD_HELLO_WORLD_HPP
#define HELLOWORLD_HELLO_WORLD_HPP

#include <string>
#include <ostream>

#include "kernel/jafarException.hpp"
using jafar::kernel::JafarException;

#include "helloworld/helloworldException.hpp"


namespace jafar {

  namespace helloworld {

    /** HelloWorld class. This class is a simple example.
     *
     * \ingroup helloworld
     */
    class HelloWorld {

    private :
  
      /// contains the string to be displayed
      std::string hello;

      static bool checkHello(const std::string& hello_);

    public :

      /// Default constructor.
      HelloWorld();

      /** Constructor.
       * @param hello_ the initial string
       * @pre \a hello_ is not empty
       */
      HelloWorld(const std::string& hello_);

      /// Destructor.
      ~HelloWorld();

      /** Returns \a hello 
       */
      const std::string& getHello() const;

      /** Set the value of the string to \a hello_
       * @pre \a hello_ not empty
       * @throw jafar::kernel::JafarException precondition
       * @throw jafar::helloworld::HelloworldFormatException \a hello_
       * must start with "Hello"
       */
      void setHello(const std::string& hello_);

      /// Set hello to the empty string
      void clearHello();

      /** Print the string on the standard output.
       * @throw HelloworldException try to print an empty string
       */
      void printHello();

      friend std::ostream& operator <<(std::ostream& s, const HelloWorld& h_);

    }; // class HelloWorld

    std::ostream& operator <<(std::ostream& s, const HelloWorld& h_);

  } // namespace helloworld
} // namespace jafar

#endif // HELLOWORLD_HELLO_WORLD_HPP
