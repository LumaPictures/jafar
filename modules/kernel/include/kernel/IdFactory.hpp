/* $Id$ */

#ifndef _IDMAKER_HPP_
#define _IDMAKER_HPP_

namespace jafar {
  namespace kernel {
    /**
     * @ingroup kernel
     * This class allows to create an unique id.
     * The current implementation increment a counter, and will fail when getId() > MAX_UINT
     */
    class IdFactory {
      public:
        IdFactory();
        ~IdFactory();
      public:
        /// Call this to get an unique id
        unsigned int getId();
        /// Call this when you want stop using an id
        void releaseId(unsigned int);
      private:
        unsigned int m_lastId;
    };
  }
}

#endif
