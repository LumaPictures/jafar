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

    class test_suite_HelloWorld : public test_suite {

    public:

      test_suite_HelloWorld() : test_suite("test_suite_HelloWorld") {
        // add member function test cases to a test suite
        boost::shared_ptr<test_HelloWorld> instance( new test_HelloWorld() );

        test_case* test_case_init       = 
          BOOST_CLASS_TEST_CASE( &test_HelloWorld::test_init, instance );
        test_case* test_case_setHello   = 
          BOOST_CLASS_TEST_CASE( &test_HelloWorld::test_setHello, instance );
        test_case* test_case_clearHello = 
          BOOST_CLASS_TEST_CASE( &test_HelloWorld::test_clearHello, instance );

        test_case_setHello->depends_on( test_case_init );
        test_case_clearHello->depends_on( test_case_init );

        add(test_case_init);
        add(test_case_setHello);
        add(test_case_clearHello); 
      }

    };

  } // namespace helloworld

} // namespace jafar
