# include "lines/histogram.hpp"

using namespace std;
using namespace jafar;
using namespace image;
using namespace lines;


Histogram::Histogram(){
  histo = 0;
  valid = 0;
}


Histogram::Histogram(const Histogram& refHisto){
  // copy data
  histo=0;
  if(refHisto.histo){ 
    cvCopyHist(refHisto.histo, &histo);
  }
  
  valid=refHisto.valid;
}


Histogram::~Histogram(){
  // release old data
  if(histo)cvReleaseHist(&histo);
}
  
Histogram &Histogram::operator=(const Histogram &refHisto){
  if (this != &refHisto){
    if(histo) cvReleaseHist(&histo);
    // copy data
    histo=0;
    if(refHisto.histo){ 
      cvCopyHist(refHisto.histo, &histo);
    }
  
    valid=refHisto.valid;
  }
  return *this;

}
  

void Histogram::initHisto(){
  // release old data    

  if(histo){
    cvReleaseHist(&histo);
  }

  int s=256;
  int histo_size[] = {s};         // number of bins
  float range[] = { 0, 255 };       // range of histogram
  float* ranges[] = { range};       // array of ranges, just one...
  histo = cvCreateHist( 1, histo_size, CV_HIST_ARRAY, ranges, 1 );
  cvClearHist(histo);
  valid = 1;
}


void Histogram::addVectorInfo( vector<int>& vec ){
  if (vec.size() > 0){
    IplImage* plane = cvCreateImage( cvSize(vec.size(),1), 8, 1 );
    for(uint i = 0; i < vec.size(); i++){
        CvScalar s;
        s.val[0]=vec[i];
        cvSet2D(plane,0,i,s);
    }
    IplImage* planes[] = {plane};
    cvCalcHist( planes, histo, 1, 0 );
  }

}


void Histogram::addImageInfo( Image* image ){
  IplImage* planes = *image;
  cvCalcHist( &planes, histo, 1, 0 );
}


void Histogram::print() const{
  if(histo){
    for(int i=0; i<256; i++){
      cout << "histo[" << i << "]: " << cvQueryHistValue_1D( histo, i ) << endl;
    }
  }
  else{
    cout << "histogram is empty\n";
  }
}


