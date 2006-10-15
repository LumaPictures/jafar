/* $Id$ */

#ifndef KERNEL_PROGRESS_BAR_HPP
#define KERNEL_PROGRESS_BAR_HPP

#include "kernel/timingTools.hpp"

namespace jafar {
  namespace kernel {

    /** A simple text based progress bar.
     *
     * \ingroup kernel
     */
    class ProgressBar {

    public:

      ProgressBar(std::string title, unsigned int cptMax, unsigned int barWidth = 60);
      ~ProgressBar();

      void operator++();

    private:

      unsigned int m_barWidth;
      unsigned int m_cpt, m_cptMax;

      Chrono chrono;
      
      void bar();

    }; // class ProgressBar

  }
}


#endif // KERNEL_PROGRESS_BAR_HPP
