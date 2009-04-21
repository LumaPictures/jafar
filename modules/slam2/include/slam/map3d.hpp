/* $Id$ */

#ifndef SLAM_MAP3D_HPP
#define SLAM_MAP3D_HPP

#ifdef HAVE_TTL

#include <list>

// #define DEBUG_TTL

#include "HeTriang.h"
#include "HeDart.h"
#include "HeTraits.h"

#include "jmath/jblas.hpp"
#include "slam/slamEkf.hpp"
#include "slam/feature.hpp"

namespace jafar {
  namespace slam {

#ifndef SWIG

    class Point3D : public hed::Node {

    private:

      BaseFeature::IdType id;
     
      jblas::vec3 px;
      jblas::sym_mat pxCov;

      int firstDataFrameId;
      int lastDataFrameId;

    public:
      
      Point3D();
      Point3D(double x_, double y_, double z_);
      Point3D(BaseFeature const& feature_);

      friend std::ostream& operator <<(std::ostream& s, Point3D const& p_);
      friend std::istream& operator >>(std::istream& s, Point3D& p_);
      friend class Map;

    };

    std::ostream& operator <<(std::ostream& s, Point3D const& p_);

    std::istream& operator >>(std::istream& s, Point3D& p_);

#endif // SWIG

    class Map {
    public:

      Map(double xmin_, double ymin_, 
	  double xmax_, double ymax_,
	  double zref_=0.0);

      ~Map();

      typedef std::list<Point3D*>::iterator map_it;
      typedef std::list<Point3D*>::const_iterator map_const_it;

      std::list<Point3D*> pointsMap;

      hed::Triangulation triangulation;

      void add(BaseFeature const& feature);
      void removeBoundary();
      void write(std::string const& filename_) const;
      void writeCalife(std::string const& filename_) const;

      void read(std::string& filename_);

      //      friend std::ostream& operator <<(std::ostream& s, Map const& m_);

    private:

      hed::Dart boundaryDart;

    };

    std::ostream& operator <<(std::ostream& s, Map const& m_);

    std::istream& operator >>(std::istream& s, Map& m_);

  } // namespace slam
} // namespace jafar

#endif // HAVE_TTL
#endif // SLAM_MAP3D_HPP
