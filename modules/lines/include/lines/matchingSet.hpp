#ifndef MATCHING
#define MATCHING


#include <iostream>
#include <vector>

namespace jafar{
  namespace lines{
    
    //! Enumerates the kinds of representation in the MatchingSet
    /**
    Enumerates the kinds of representation in the MatchingSet.
    
    FAST_OLD_LINE_MATCH_ACCESS 
    is the representation with map member
    
    FAST_MATCH_ACCESS
    is the representation with oldIdx and newIdx vectors
    
    BOTH_MATCH_ACCESS_TYPES
    matchings are stored in map and in oldIdx and newIdx vectors
    
    @ingroup lines
    */
    enum MatchRepres{
      FAST_OLD_LINE_MATCH_ACCESS,
      FAST_MATCH_ACCESS,
      BOTH_MATCH_ACCESS_TYPES
    };
    
    
    //! Structure to store the result of a comparison of two matchings. 
    /**Structure to store the result of a comparison of two matchings. The following statistics are supported:
    
        "right" is total number of matchings with: 
            something from newMatching is in refMatching or there is nothing in both.
    
        "wrong" is total number of matchings with: 
            there is something in newMatching and this is all not in refMatching. 
    
        "notFound" is total number of matchings with: 
            there is nothing in newMatching but something in refMatching.
    
    @ingroup lines
    */
    struct MatchStatistic{
      int right; //!< total number of matchings with: something from newMatching is in refMatching or there is nothing in both
      int wrong;  //!< total number of matchings with: there is something in newMatching and this is all not in refMatching 
      int notFound; //!< total number of matchings with: there is nothing in newMatching but something in refMatching
      //int rightAssignmet;
      //int falseAssignment;
      
      void print(){
        std::cout << "== statistic print ==" << std::endl;
        std::cout << "\tright: " << right << std::endl;
        std::cout << "\twrong: " << wrong << std::endl;
        std::cout << "\tnotFound: " << notFound << std::endl;
      }
    };
    
    //! Container for a matching.
    /** Container for a matching. In jafar::lines it is used as container for matching of line segments, but there is no limitation to that.
    
    @ingroup lines
    */
    class MatchingSet{
    public:
      //! Constructor with indicator for the type of storage for the matchings
      /**
      Constructor with indicator for the type of storage for the matchings. The default value provids fast access to all matches of an old line. The other values are described in lines::MatchRepres enumeration.
      */
      MatchingSet(MatchRepres _matchRep=FAST_OLD_LINE_MATCH_ACCESS);
      
      //! Clears the matchings
      /** This function clears the content of the macthing and can be used to prepare the next macthing step.
      */
      void clear();
      
      //bool load(QString filename);
      //bool save(QString filename);
      
      //! Compares two matchings
      /**
        Compares two matchings.They must have the same maxInd value (means the same number of lines) 
        The result of the comparison is stored in statistic. Actually there is the following content:
      
            "right" is total number of matchings with: 
          something from newMatching is in refMatching or there is nothing in both.
        
            "wrong" is total number of matchings with: 
          there is something in newMatching and this is all not in refMatching. 
        
            "notFound" is total number of matchings with: 
          there is nothing in newMatching but something in refMatching.
       
         @param newMatching The matching that should be evaluated
         @param refMatching The reference matching (maybe validated manually or with a good tracking)
         @param statistic Storage for the result of comparison
       */
      static void compareMatchingSets( MatchingSet* newMatching, MatchingSet* refMatching, MatchStatistic* statistic);
    
      //! Copys a matching
      /**
      This function copys the content of old matching to clone matching. The prior content of clone is lost. 
        @param old The old matching set which content is copyed
        @param clone The storage for the copy.
      */
      static void clone(MatchingSet& old, MatchingSet& clone);
      
      //! Add a new matching to the storage.
      /** 
      This is the function to extend the matching set with a new matching. The internal data structure for the matching set is extended automatically. It is possible to assign several new indexes to one old by calling this function several times. Vice versa as well.
      
        @param indA Is the index of the part of the old set.
        @param indB Is the index of the part of the new set.
      */
      void addMatching(int indA, int indB);
      
      //! Print the matching on std::out
      /**
      Print the matching on std::out. The first column are the old indexes and right beside there is a list of the matchings in the new set.
      -1 value indicates there is no matching.
      */
      void print();
      
      /**
       * Searches matching between line indA and line indB
       * therefore indB is searches under all possible matchings for line indA
       * return 1 if found, else 0
       */
      bool searchMatching(int indA, int indB);
    
      //! To get maximum index of the matching.
      int getMaxInd(){return maxInd;}
    
      //! Container for the matching
      /** 
      This is one of the containers for the matching, its used if matchRep is set to FAST_OLD_LINE_MATCH_ACCESS,
      or BOTH_MATCH_ACCESS_TYPES. It is organized as follows: in map[i] there is a vector with all indices of new lines that are matched to old line i
      The size of map is updated automatically (if there is a matching for old index i, then all indexes smaller i get a matching of -1 if they are not matched yet.  
      */
      std::vector< std::vector<int> > map;
      
      //!This is one of the containers for the matching FAST_MATCH_ACCESS or BOTH_MATCH_ACCESS_TYPES. It contains the index of the old line, the index of the new line is at the same position in newIdx.
      std::vector<int> oldIdx;
      //!This is one of the containers for the matching FAST_MATCH_ACCESS or BOTH_MATCH_ACCESS_TYPES. It contains the index of the new line, the index of the old line is at the same position in oldIdx.
      std::vector<int> newIdx;
      
    private:
      //! maximum index is the number of last matched line  neccessary to identify non matched lines (out of interval [0 maxInd] or value -1) 
      int maxInd; 
      
      
      MatchRepres matchRep;   //!< Indicates the type of storage for the matchings
    };
  } // namespace lines
} // namespace jafar
#endif
