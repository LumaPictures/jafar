/* $Id$ */

#include "slampt/managerTools.hpp"

using namespace ublas;
using namespace jblas;
using namespace jafar::slampt;


/*
 * class FrameDataBase
 */

void FrameDataBase::changeLandmarkId(unsigned int oldId, unsigned int newId, 
				     unsigned int frameIndexBegin, unsigned int frameIndexEnd) {

  DBFrameList::iterator itFrameDB = frameList.begin();

  while(itFrameDB->index < frameIndexBegin && itFrameDB != frameList.end())
    ++itFrameDB;

  while(itFrameDB->index <= frameIndexEnd && itFrameDB != frameList.end()) {
    for (DBFrame::LandmarksList::iterator itLd = itFrameDB->landmarks.begin() ;
	 itLd != itFrameDB->landmarks.end() ; ++itLd)
      {
	if (itLd->id == oldId)
	  itLd->id = newId;
      }
    ++itFrameDB;
  }
}

/*
 * QualityIndexPoint
 */

bool jafar::slampt::operator<(QualityIndexPoint const& p1, QualityIndexPoint const& p2) {
  return p1.quality < p2.quality;
};
