/* $Id$ */

#ifdef HAVE_TTL

#include <cmath>
#include <fstream>

#include "kernel/jafarException.hpp"

#include "slam/map3d.hpp"

using namespace jafar::slam;

/*
 * class Point3D
 */

Point3D::Point3D() : pxCov(3,3) {}

Point3D::Point3D(double x_, double y_, double z_) :
  hed::Node(x_, y_, z_),
  id(),
  px(), pxCov(3,3),
  firstDataFrameId(),
  lastDataFrameId()
{
  px(0)=x_;
  px(1)=y_;
  px(2)=z_;
  pxCov.clear();
}

Point3D::Point3D(BaseFeature const& feature_) :
  hed::Node(feature_.getX()(0),feature_.getX()(1),feature_.getX()(2)),
  id(feature_.getId()),
  px(feature_.getX()),
  pxCov(feature_.getP()),
  firstDataFrameId(feature_.dataFramesId.front()),
  lastDataFrameId(feature_.dataFramesId.back())
{}

std::ostream& jafar::slam::operator <<(std::ostream& s, Point3D const& p_) 
{
  s << p_.id << " " 
    << p_.px << " "
    << p_.pxCov << " "
    << p_.firstDataFrameId << " "
    << p_.lastDataFrameId;
  return s;
}

std::istream& jafar::slam::operator >>(std::istream& s, Point3D& p_)
{
  JFR_IO_STREAM(s >> p_.id >> p_.px >> p_.pxCov >> p_.firstDataFrameId >> p_.lastDataFrameId,
		"while reading Point3D");
  return s;
}

/*
 * Map
 */

Map::Map(double xmin_, double ymin_, 
	 double xmax_, double ymax_,
	 double zref_) 
{
  // initialize triangulation
  pointsMap.push_back(new Point3D(xmin_,ymin_,zref_));
  pointsMap.push_back(new Point3D(xmin_,ymax_,zref_));
  pointsMap.push_back(new Point3D(xmax_,ymin_,zref_));
  pointsMap.push_back(new Point3D(xmax_,ymax_,zref_));

  triangulation.createDelaunay(pointsMap.begin(), pointsMap.end());
}

Map::~Map() 
{
  // class Point3D derives Node wich has a built-in ref counter 
  //   for (map_it it = pointsMap.begin() ; it != pointsMap.end() ; ++it) {
  //     delete *it;
  //   }
}

void Map::add(BaseFeature const& feature) 
{
  pointsMap.push_back(new Point3D(feature));
  
  hed::Dart dart = triangulation.createDart();
  bool status = ttl::insertNode<hed::TTLtraits>(dart, *(static_cast<hed::Node*>(pointsMap.back())));
  if (!status) {
    JFR_DEBUG("node: " << *pointsMap.back());
    JFR_POSTCOND(ttl::inTriangle<hed::TTLtraits>(*(hed::Node*)(pointsMap.back()), dart), 
		 "Map::add: node outside");
  }
  JFR_POSTCOND(status, "Map::add: delaunay triangulation failed");
}

void Map::removeBoundary() {
  hed::Edge* be = triangulation.getBoundaryEdge();
  hed::Dart bd(be);
  ttl::removeRectangularBoundary<hed::TTLtraits>(bd);
}

void Map::write(std::string const& filename_) const
{
  std::ofstream fileStream(filename_.c_str(), std::ios_base::out);
  fileStream << *this;
  fileStream.close();
}

void Map::writeCalife(std::string const& filename_) const
{
  std::ofstream fs(filename_.c_str(), std::ios_base::out);
  fs << pointsMap.size() << std::endl;
  for (map_const_it it = pointsMap.begin() ; it !=  pointsMap.end() ; ++it)
    {
      Point3D& p = **it;
      fs << p.id << " " 
	 << p.px(0) << " "
	 << p.px(1) << " "
	 << p.px(2) << " "
	 << sqrt(p.pxCov(0,0)) << " "
	 << sqrt(p.pxCov(1,1)) << " "
	 << sqrt(p.pxCov(2,2)) << " "
	 << p.firstDataFrameId << " "
	 << p.lastDataFrameId << std::endl;
    }
  fs.close();
}

void Map::read(std::string& filename_) 
{
  std::ifstream fileStream(filename_.c_str(), std::ios_base::in);
  JFR_IO_STREAM(fileStream, "error while opening file " + filename_);
  try {
    fileStream >> *this;
  }
  catch(jafar::kernel::Exception& e) { 
    JFR_TRACE(e," (reading file:" + filename_ + ")"); 
    throw; 
  }
  fileStream.close();
}

std::ostream& jafar::slam::operator <<(std::ostream& s, Map const& m_)
{
  s << m_.pointsMap.size();
  for (Map::map_const_it it = m_.pointsMap.begin() ; it !=  m_.pointsMap.end() ; ++it)
    {
      s << **it << std::endl;
    }
  return s;
}

std::istream& jafar::slam::operator >>(std::istream& s, Map& m_) 
{
  try {
    unsigned int mapSize;
    JFR_IO_STREAM(s >> mapSize, "reading size of map");
    Point3D* p;
    for (unsigned int i = 0 ; i < mapSize ; ++i) {
      p = new Point3D();
      s >> *p;
      m_.pointsMap.push_back(p);
    }
  }
  catch(jafar::kernel::Exception& e) {
    JFR_TRACE(e," reading map"); 
    throw; 
  }
  return s;
}

#endif // HAVE_TTL
