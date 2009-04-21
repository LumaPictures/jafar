/* $Id$ */

/** Tools for module slamlines.
 *
 * \file slamlinesTools.i
 * \ingroup slamlines
 */   

%{

#include <sstream>

#include "slamlines/imageSegmentManager.hpp"


%}

%inline %{

namespace jafar {
  namespace slamlines {


    /** Template print function which calls the output operator<< of A
     * and returns the resulting string.
     */
    template<class A>
    std::string print(const A& a_) {
      std::ostringstream os;
      os << a_ << std::endl;
      return os.str();
    };

    
    /*
     * segments
     */

    jafar::slamlines::VecMatches* new_VecMatches() {
      return new VecMatches();
    }

    jafar::slamlines::VecImageSegments* new_VecImageSegments() {
      return new VecImageSegments();
    }

    std::string imageSegmentsVecToDisplayList(jafar::slamlines::VecImageSegments const& vecImageSegments)
    {
      std::ostringstream os;
      for (VecImageSegments::const_iterator it = vecImageSegments.begin() ; it != vecImageSegments.end() ; ++it) {
        os << "{ " << it->id << " "
           << it->ext1(0) << " " << it->ext1(1) << " "
           << it->ext2(0) << " " << it->ext2(1) << " } ";
      }
      return os.str();
    }
    
  } // namespace slamlines
} // namespace jafar

%}


