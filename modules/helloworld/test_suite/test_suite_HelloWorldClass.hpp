/* $Id$ */

// test suite specific includes
#include "helloworld/helloWorld.hpp"
#include "helloworld/helloworldException.hpp"

namespace jafar {

  namespace helloworld {

    /* 
     * class HelloWorld test suite 
     */

    class test_HelloWorld {

    public:

      HelloWorld hw;
      HelloWorld hw2;

      test_HelloWorld() : hw(), hw2("Hello world") {}

      void test_init() {
        BOOST_CHECK_EQUAL(hw.getHello().size(), 0);
        BOOST_CHECK_EQUAL(hw2.getHello(), "Hello world");
      }

      void test_setHello() {
        hw.setHello("Hello world");
        BOOST_CHECK_EQUAL(hw.getHello(), "Hello world");

        BOOST_CHECK_THROW(hw.setHello(""), jafar::kernel::JafarException);

        BOOST_CHECK_THROW(hw.setHello("what ever"), HelloworldFormatException);
      }

      void test_clearHello() {
        hw.clearHello();
        BOOST_CHECK_EQUAL(hw.getHello().size(), 0);
      }
    };
  } // namespace helloworld

} // namespace jafar
