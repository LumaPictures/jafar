/* $Id$ */

/** Tools for module slamseg.
 *
 * \file slamsegTools.i
 * \ingroup slamseg
 */   

%{

#include <sstream>
#include <dseg/SegmentsSet.hpp>
#include <dseg/SegmentHypothesis.hpp>

%}

%inline %{

namespace jafar {
  namespace slamseg {


    /** Template print function which calls the output operator<< of A
     * and returns the resulting string.
     */
    template<class A>
    std::string print(const A& a_) {
      std::ostringstream os;
      os << a_ << std::endl;
      return os.str();
    };
    
    std::string segmentsStorageToDisplayList(const dseg::SegmentsSet& _currentLines)
    {
      std::ostringstream os;
      for( int i = 0; i < _currentLines.count(); ++i)
      {
        const dseg::SegmentHypothesis* seg = _currentLines.segmentAt(i);
        os << "{ " << seg->id() << " "
           << seg->x1() << " " << seg->y1() << " "
           << seg->x2() << " " << seg->y2() << " } ";
      }
      return os.str();
    }

  } // namespace slamseg
} // namespace jafar

%}


