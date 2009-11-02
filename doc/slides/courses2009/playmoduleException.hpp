#ifndef PLAYMODULE_PLAYMODULE_EXCEPTION_HPP
#define PLAYMODULE_PLAYMODULE_EXCEPTION_HPP

#include "kernel/jafarException.hpp"

namespace jafar {
  namespace playmodule {
    class PlaymoduleException : public ::jafar::kernel::Exception {

    public:

      enum ExceptionId {
        //        MY_ERROR /**< my error */
      };
      
    }; // class PlaymoduleException
  } // namespace playmodule
} // namespace jafar
#endif // PLAYMODULE_PLAYMODULE_EXCEPTION_HPP

