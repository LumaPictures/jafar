# ifndef HISTOGRAM
# define HISTOGRAM

#include <iostream>
#include <vector>
#include <cmath> 

#include <image/Image.hpp>


namespace jafar{
  namespace lines{
    //! Wrap cvHistogram
    /*!
      This class wraps cvHistogram and provides more convinient access to its functions. In particular for the work with std::vector.and jafar::image::Image
    
      @ingroup lines
    */
    class Histogram{
      public:
        // Constructor
        Histogram();
        
        // Copy constructor
        Histogram(const Histogram& refHisto);
        // Destructor
        ~Histogram();
        //! Clear old data and creates new cvHistogram 
        void initHisto();
        //! Add content of a std::vector<int> to the histogram. Think about if you first want to clear the histogram
        void addVectorInfo( std::vector<int>& vec );
        //! Add content of an image to the histogram. Think about if you first want to clear the histogram
        void addImageInfo( jafar::image::Image* image );
        //! Print content of bins on std out
        void print() const;
        //! Indicates if histogram is initialized
        int isValid(){return valid;}
        // assignment operator
        Histogram& operator=(const Histogram &refHisto);
    
        
        CvHistogram* histo;   //!< Pointer to histogram data
        
      private:
        int valid;
    
    };
    
  }   //namespace lines
} // namespace jafar

# endif
