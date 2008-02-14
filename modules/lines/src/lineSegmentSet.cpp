#include "lines/lineSegmentSet.hpp"

using namespace std;
using namespace jafar;
using namespace image;
using namespace lines;

int LineSegmentSet::eraseInvalid(){
  
  vector<LineSegment> newLines;
  for(uint i=0; i<lineSegments.size(); i++){
    newLines.push_back(lineSegments[i]);
  }
  
  lineSegments.clear();
  for(uint i=0; i<newLines.size(); i++){
    if(newLines[i].isValid()){
      lineSegments.push_back(newLines[i]);
    }
  }
  
  return 1;
}

void LineSegmentSet::clearPredFlags(){
  for(uint i=0; i<lineSegments.size();i++){
    lineSegments[i].predFlag = 0;
  }
} 

void LineSegmentSet::clearParameters(){
  for(uint i=0; i<lineSegments.size();i++){
    lineSegments[i].param1 = -1;
    lineSegments[i].parameters.clear();
  }
} 

void LineSegmentSet::addLine(LineSegment* lineSegment){
  lineSegments.push_back( *lineSegment );
}

void LineSegmentSet::addLine(LineSegment& lineSegment){
  lineSegments.push_back( lineSegment );
}

void LineSegmentSet::clear(){
  lineSegments.clear();
}

void LineSegmentSet::orientLines(Image* image){
  for(uint i=0; i<lineSegments.size(); i++){
    lineSegments[i].orientLine(image);
  }
}

void LineSegmentSet::changeNumberOfNeighbours(int number){
  for(uint i=0; i<lineSegments.size(); i++){
    lineSegments[i].setNumberOfNeighbours(number);
  }
}

void LineSegmentSet::calcHistogramDescriptor(){
  for(uint i=0; i<lineSegments.size(); i++){
    lineSegments[i].calcHistogramDescriptor();
  }  
}

void LineSegmentSet::calcHistogramDescriptor(Image* image){
  for(uint i=0; i<lineSegments.size(); i++){
    lineSegments[i].calcHistogramDescriptor(image);
  }  
}

void LineSegmentSet::calcGreyspaceDescriptor(Image* image){
  for(uint i=0; i<lineSegments.size(); i++){
    lineSegments[i].calcGreyspaceDescriptor(image);
  }  
  }
  
/*! 
 Grow lines with use of the parallel lines
 */
void LineSegmentSet::growLinesParallel( Image* image, int dist, int thresh, int distThresh){
  for(uint i=0; i<lineSegments.size(); i++){
    lineSegments[i].growLineParallel(image, dist, thresh, distThresh);
  }
}

/*!
 Improve LineSegmentSet based on image
 */
void LineSegmentSet::growLines( Image* image, int gapThresh, int thresh, int maxOffset){
  for(uint i=0; i<lineSegments.size(); i++){
    lineSegments[i].growLine(image, gapThresh, thresh, maxOffset);
  }
}

/*! 
 fit lines to gradients
 */
void LineSegmentSet::fitLines( Image* image, int gradientimage, int nPoints, int pDist, Image* colorImage){
  for(uint i=0; i<lineSegments.size(); i++){
    lineSegments[i].fitLine( image, gradientimage, nPoints, pDist, colorImage);
  }
}

/*! 
 fit lines to gradients
 */
void LineSegmentSet::fitLinesOrientation(LineSegmentSet lsSet, Image* gradX, Image* gradY, int nPoints, int pDist, double aDist,double minLength, double extend, Image* colorImage){
  for(uint i=0; i<lineSegments.size(); i++){
    lineSegments[i].fitLineOrientation(lsSet, gradX, gradY,  nPoints,  pDist, aDist, minLength, extend, colorImage);
  }
}

/*
bool LineSegmentSet::load(QString filename, int numberOfParallels){
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)){
    cout << "pERROR: line segment set file can not be loaded\n";
    return 0;
  }

  // clear old line segments
  clear();
  QTextStream in(&file); 

  // read first line
  QString line = in.readLine();
  while(!line.isNull()){
    QStringList list = line.split(" ", QString::SkipEmptyParts);

    // ignore comments
    if(list.at(0) != "#"){
      if(numberOfParallels){ 
        LineSegment newLine(numberOfParallels);
        newLine.setPoints(list.at(1).toDouble(), list.at(2).toDouble(), list.at(3).toDouble(), list.at(4).toDouble());
        addLine(&newLine);

      }
      else{
        LineSegment newLine;
        newLine.setPoints(list.at(1).toDouble(), list.at(2).toDouble(), list.at(3).toDouble(), list.at(4).toDouble());
        addLine(&newLine);

      }

    }
    // read next line
    line = in.readLine();
  }
  file.close();
  
  return 1;
}
*/
void LineSegmentSet::extractGreyscales(Image* image, int parallel_flag){
  for(uint i=0; i<lineSegments.size(); i++){
    lineSegments[i].extractGreyscale(image, parallel_flag);
  }
}

void LineSegmentSet::extractGradientscales(Image* image, int gradientimage, int parallel_flag){
  for(uint i=0; i<lineSegments.size(); i++){
    lineSegments[i].extractGradientscale(image, gradientimage, parallel_flag);
  }
}
void LineSegmentSet::extractLaplacescales(Image* image, int laplaceimage, int parallel_flag){
  for(uint i=0; i<lineSegments.size(); i++){
    lineSegments[i].extractLaplacescale(image, laplaceimage, parallel_flag);
  }
}
void LineSegmentSet::print(){
  for(uint i=0; i<lineSegments.size(); i++){
    cout << i << " " << lineSegments[i].getX1() << " " << lineSegments[i].getY1() << " " << lineSegments[i].getX2() << " " << lineSegments[i].getY2() << " " << lineSegments[i].getMx() << " " << lineSegments[i].getMy() << " " << lineSegments[i].getOAlpha() << endl; 
  }
}
/*
bool LineSegmentSet::save(QString filename){
  QFile file(filename);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)){
    cout << "pERROR: LineSegmentSet file can not be opened\n";
    return 0;
  }

  QTextStream out(&file);
  out << "# line segment set\n";
  out << "# format: index x1 y1 x2 y2 mx my alpha oAlpha eucLength avL avR\n";
  for(uint i=0; i<lineSegments.size(); i++){
    out << i << " " << lineSegments[i].getX1() << " " << lineSegments[i].getY1() << " " << lineSegments[i].getX2() << " " << lineSegments[i].getY2() << " " << lineSegments[i].getMx() << " " << lineSegments[i].getMy() <<" " <<lineSegments[i].getAlpha() << " " << lineSegments[i].getOAlpha() <<" "<< lineSegments[i].getEucLength()<< " " << lineSegments[i].avL <<" " << lineSegments[i].avR << endl; 
  }
  file.close();
  return 1;
}
*/


void LineSegmentSet::clone(LineSegmentSet& old, LineSegmentSet& clone){
  clone.lineSegments.clear();
  for(uint i=0; i<old.lineSegments.size(); i++){
    clone.lineSegments.push_back(old.lineSegments[i]);
  }  
}

  
  
  
  

