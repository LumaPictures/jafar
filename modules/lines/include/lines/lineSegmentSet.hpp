#ifndef LINE_SEGMENT_SET
#define LINE_SEGMENT_SET

#include <vector>
#include <iostream> 
#include "lines/lineSegment.hpp"


namespace jafar{
  namespace lines{
    
    class LineSegment;
  
    //! Container for a set of LineSegment
    /**
    This class provides a simples managment of a set of LineSegment instances. Thus the very most functions just call the LineSegment function for each instance in lineSegments vector.
    
    @ingroup lines
    */
    class LineSegmentSet{
    public:
      //! Erase all lines of lineSegments whose valid flag is not set
      int eraseInvalid();
      
      //!Clear the predFlags to indictae that the current prediction is (no longer) valid
      void clearPredFlags();
      
      //! Clear the parameters vector and the param1 value of all lines 
      void clearParameters();

      //! Add a line to the set, line is given by its address
      void addLine(LineSegment* lineSegment);
      
      //! Add a line to the set, line is given as reference
      void addLine(LineSegment& lineSegment);

      //! Clear lineSegments
      void clear();
      
      //bool load(String filename, int numberOfParallels=0);
      //bool save(String filename);
      
      //! Print endpoints, midpoint and directed orientation on std::out
      void print();
      
      //! Call extractGreyscale for all lines of lineSegments
      void extractGreyscales(jafar::image::Image* image, int parallel_flag=0);
      
      //! Call extractGradientscale for all lines of lineSegments
      void extractGradientscales(jafar::image::Image* image, int gradientimage, int parallel_flag=0);
      
      //! Call extractLaplacescalefor all lines of lineSegments
      void extractLaplacescales(jafar::image::Image* image, int laplaceimage, int parallel_flag=0);
      
      //! Call changeNumberOfNeighbours for all lines of lineSegments
      void changeNumberOfNeighbours(int number);
      
      //! Call calcHistogramDescriptor(image) for all lines of lineSegments
      void calcHistogramDescriptor(jafar::image::Image* image);
      
      //! Call calcHistogramDescriptor() for all lines of lineSegments
      void calcHistogramDescriptor();
      
      //! Call calcGreyspaceDescriptor for all lines of lineSegments
      void calcGreyspaceDescriptor(jafar::image::Image* image);
      
      //! Call orientLines for all lines of lineSegments
      void orientLines(jafar::image::Image* image);
    
      //! Call growLinesParallel for all lines of lineSegments
      void growLinesParallel( jafar::image::Image* image, int dist=3, int thresh=5, int distThresh=20);
      
      //! Call growLine for all lines of lineSegments
      void growLines( jafar::image::Image* image, int gapThresh=1, int thresh=40, int maxOffset=2);
      
      //! Call fitLine for all lines of lineSegments
      void fitLines( jafar::image::Image* image, int gradientimage=0, int nPoints=20, int pDist=25, jafar::image::Image* colorImage=0);
      
      //! Call fitLineOrientation for all lines of lineSegments
      void fitLinesOrientation(LineSegmentSet lsSet, jafar::image::Image* gradX, jafar::image::Image* gradY, int nPoints=10, int pDist=25, double aDist=M_PI*0.5, double minLength=0, double extend=0, jafar::image::Image* colorImage=0);
    
      //! Copys the old LineSegmentSet to the clone LineSegmentSet
      static void clone(LineSegmentSet& old, LineSegmentSet& clone);
      
      //! Storage for the lines
      std::vector<LineSegment> lineSegments;
    
      //! Returns size of lineSegments
      uint size(){return lineSegments.size();}
      
      //! Returns reference to lineSegments[index] 
      LineSegment& ls(uint index){return lineSegments[index];}
    };
    
  } // namespace lines
} // namespace jafar
#endif
