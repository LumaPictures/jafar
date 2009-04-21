/* $Id$ */

/** Tools for module slampt.
 *
 * \file slamptTools.i
 * \ingroup slampt
 */   

%{

#include <sstream>

#include "slampt/imagePointManager.hpp"

%}

%inline %{

namespace jafar {
  namespace slampt {


    /** Template print function which calls the output operator<< of A
     * and returns the resulting string.
     */
    template<class A>
    std::string print(const A& a_) {
      std::ostringstream os;
      os << a_ << std::endl;
      return os.str();
    };
    
    std::string getZones(jafar::slampt::ImagePointManager const& m) {
      using namespace jafar::slam;
      std::ostringstream ss;
      double stepU = m.imageWidth / m.nbZonesU;
      double stepV = m.imageHeight / m.nbZonesV;
      for (unsigned int i=0 ; i<m.nbZonesU ; ++i) {
        for (unsigned int j=0 ; j<m.nbZonesU ; ++j) {
          ss << "{" << int(round(i*stepU)) << " " << int(round(j*stepV)) << " "
             << int(round((i+1)*stepU)) << " " << int(round((j+1)*stepV)) << "} ";
        }
      }
      return ss.str();
    };

  } // namespace slampt
} // namespace jafar

%}


