/* $Id$ */

#include <fstream>

#include "slamlines/imageSegmentManager.hpp"

#include "slam/segmentFeature.hpp"

using namespace jafar;
using namespace jafar::slam;
using namespace jafar::slamlines;

namespace jafar { namespace slamlines {

  std::ostream& operator <<(std::ostream& s, ImageSegment const& seg) {
    s << seg.id << ": " << seg.ext1 << " " << seg.ext2;
    return s;
  }

  void readImageSegments(std::string const& filename, VecImageSegments& vecImageSegments)
  {
    JFR_TRACE_BEGIN;
    vecImageSegments.clear();
    std::ifstream in(filename.c_str());
    JFR_IO_STREAM(in, "error while opening file " << filename);
    std::string pipo;
    int nb;

    int index;
    int type;
    jblas::vec2 ext1,ext2;
    double alpha, beta, v_alpha, v_beta, v_alphabeta;

    in >> pipo; // Number
    in >> pipo; // of
    in >> pipo; // segments
    in >> pipo; // :
    in >> nb;
    //JFR_DEBUG("nb segments: " << nb);
    for (int i = 0 ; i < nb ; ++i) {
      in >> pipo >> pipo >> index;
      in >> type;
      //      in >> ext1(0) >> ext1(1) >> ext2(0) >> ext2(1);
      in >> ext1(1) >> ext1(0) >> ext2(1) >> ext2(0);
      in >> alpha >> beta;
      in >> v_alpha >> v_beta >> v_alphabeta;
      ImageSegment seg(ext1, ext2);
      //      JFR_DEBUG(seg);
      vecImageSegments.push_back(seg);
    }
    //JFR_DEBUG("readImageSegments: " << vecImageSegments.size());
    JFR_TRACE_END("readImageSegments: ");
  }
  

  void readImageSegmentsId(std::string const& filename, VecImageSegments& vecImageSegments)
  {
	  JFR_TRACE_BEGIN;
	  vecImageSegments.clear();
	  std::ifstream in(filename.c_str());
	  JFR_IO_STREAM(in, "error while opening file " << filename);
	  std::string pipo;
	  int nb;

	  int index;
	  double type;

	  jblas::vec2 ext1,ext2;
	  jblas::mat extCov(2,2);
	  double v_alpha, v_beta, v_alphabeta;

	  extCov.clear();

	  in >> pipo; // Number
	  in >> pipo; // of
	  in >> pipo; // segments
    in >> pipo; // :
    in >> nb;
    //JFR_DEBUG("nb segments: " << nb);
    for (int i = 0 ; i < nb ; ++i) {
	    in >> pipo >> index;
	    in >> type;
      //      in >> ext1(0) >> ext1(1) >> ext2(0) >> ext2(1);
	    in >> ext1(1) >> ext1(0) >> ext2(1) >> ext2(0);
	    in >> v_alpha >> v_beta >> v_alphabeta;

	    extCov(0,0)=v_alpha;
	    extCov(0,1)=v_alphabeta;
	    extCov(1,0)=v_alphabeta;
	    extCov(1,1)=v_beta;

            ///ID
	    //if (type <= 40 || type == 100 ) {
	    ImageSegment seg(ext1, ext2, extCov, extCov, index);
	    //JFR_DEBUG(seg);
	    vecImageSegments.push_back(seg);
	    //}
    }
    //JFR_DEBUG("readImageSegments: " << vecImageSegments.size());
    JFR_TRACE_END("readImageSegments: ");
  }
  
  void readImageSegmentsMatches(std::string const& filename, VecMatches& vecMatches)
  {
    JFR_TRACE_BEGIN;
    vecMatches.clear();
    std::ifstream in(filename.c_str());
    JFR_IO_STREAM(in, "error while opening file " << filename);
    std::string pipo;
    int nb;
    Match m;

    in >> pipo; // Number
    in >> pipo; // of
    in >> pipo; // matches
    in >> pipo; // :
    in >> nb;
    //    JFR_DEBUG("nb matches: " << nb);
    for (int i = 0 ; i < nb ; ++i) {
      in >> pipo;
      in >> m.first >> m.second;
      vecMatches.push_back(m);
    }
    JFR_TRACE_END("readImageSegmentsMatches: " << filename);
  }
  
  void lineSegmentSetToImageSegmentVec(jafar::lines::LineSegmentSet& lineSegmentSet, VecImageSegments& vecImageSegments)
  {
	  vecImageSegments.clear();
	  jblas::vec2 ext1,ext2;
	  jblas::mat extCov(2,2);
	  unsigned int index;
	  //std::vector<lines::LineSegment>::const_iterator it;
			  			  
	  for (unsigned int i =0; i< lineSegmentSet.lineSegments.size(); i++) {
		  ext1(0)=lineSegmentSet.lineSegments[i].getX1();
		  ext1(1)=lineSegmentSet.lineSegments[i].getY1();
		  ext2(0)=lineSegmentSet.lineSegments[i].getX2();
		  ext2(1)=lineSegmentSet.lineSegments[i].getY2();
		  extCov(0,0)=lineSegmentSet.lineSegments[i].covXX;
		  extCov(0,1)=lineSegmentSet.lineSegments[i].covXY;
		  extCov(1,0)=lineSegmentSet.lineSegments[i].covXY;
		  extCov(1,1)=lineSegmentSet.lineSegments[i].covYY;
		  index=lineSegmentSet.lineSegments[i].id;
		  ImageSegment seg(ext1, ext2, extCov, extCov, index);
		  //JFR_DEBUG(lineSegmentSet.lineSegments.size());
		  //JFR_DEBUG(seg);
		  vecImageSegments.push_back(seg);
	  }
  }
  
  
  void matchingSetToImageSegmentsMatches(jafar::lines::MatchingSet& matchingSet, VecMatches& vecMatches)
  {
    vecMatches.clear();
    Match m;

    for (unsigned int i = 0 ; i < matchingSet.oldIdx.size(); i++) {
	  m.first=matchingSet.oldIdx[i];
	  m.second=matchingSet.newIdx[i];
	  vecMatches.push_back(m);
  }
  
}
  unsigned int propagateId(VecMatches const& matches, 
			   VecImageSegments const& vecImageSegmentsRef, VecImageSegments& vecImageSegmentsMatch)
  {
    JFR_TRACE_BEGIN;
//     JFR_DEBUG("ref: " << vecImageSegmentsRef.size());
//     JFR_DEBUG("match: " << vecImageSegmentsMatch.size());
    unsigned int cpt = 0;
    for (VecMatches::const_iterator it = matches.begin() ; it!=matches.end() ; it++) {
//       JFR_DEBUG(it->first << " " << it->second);im
      if (vecImageSegmentsRef.at(it->first).id != ImageSegment::NO_ID) {
	vecImageSegmentsMatch.at(it->second).id = vecImageSegmentsRef.at(it->first).id;
	cpt++;
      }
    }
    return cpt;
    JFR_TRACE_END("propagateId");
  }

}}

/*
 * class ImageSegmentManager
 */
//Change DEFAULT VALUE TO FLAG
void ImageSegmentManager::featureSelection(VecImageSegments& vecImageSegments) {
  for (VecImageSegments::iterator it = vecImageSegments.begin() ; it != vecImageSegments.end() ; ++it) {
    if (it->id == ImageSegment::NO_ID && isInZone(it->ext1) && isInZone(it->ext2) && ublas::norm_2(it->ext2-it->ext1) > p_minSegmentSize) {
      it->id = getNewFeatureId();
      //JFR_DEBUG("select feature: " << it->id);
    }
    if (isInZone(it->ext1) && isInZone(it->ext2) && ublas::norm_2(it->ext2-it->ext1) > p_minSegmentSize) {
    } 
    else {
	    it->id = ImageSegment::NO_ID;
	    //JFR_DEBUG("deselect feature: " << it->id);
    }
  }
}

void ImageSegmentManager::slamProcessObservations(unsigned int currentFrameIndex, VecImageSegments& vecImageSegments) const
{
  SegmentObservation obs(type);
  jblas::vec2 line;

  typedef std::list<Observation*> ObsList;
  ObsList featuresObs;

  for (VecImageSegments::const_iterator it = vecImageSegments.begin() ; it != vecImageSegments.end() ; ++it) {
    if (it->id != ImageSegment::NO_ID) {
      line = ImageEuclideanPluckerFeatureObserveModel::extToLine(it->ext1, it->ext2);
      obs.set(it->id, line);
      obs.setExtremities(it->ext1, it->ext2);
//       JFR_DEBUG(obs);
      featuresObs.push_back(new SegmentObservation(obs));
    }
  }

  p_slam.processObservations(currentFrameIndex, featuresObs);

}

void ImageSegmentManager::initFrame(unsigned int frameIndex, VecImageSegments& vecImageSegments)
{
  JFR_DEBUG("ImageSegmentManager::initFrame:" << frameIndex << " nb segments: " << vecImageSegments.size());
  JFR_DEBUG("select features...");
  featureSelection(vecImageSegments);
  JFR_DEBUG("slam process...");
  slamProcessObservations(frameIndex, vecImageSegments);
}

void ImageSegmentManager::processFrame(unsigned int frameIndex, VecImageSegments& vecImageSegments)
{
  JFR_DEBUG("processFrame...");	
  featureSelection(vecImageSegments);
  slamProcessObservations(frameIndex, vecImageSegments);
}

unsigned int ImageSegmentManager::compareId(VecMatches const& matches, 
		 VecImageSegments const& vecImageSegmentsRef, VecImageSegments& vecImageSegments, jafar::lines::LineSegmentSet& lineSegmentSet)
{
	JFR_TRACE_BEGIN;
	unsigned int cpt = 0;
//     JFR_DEBUG("ref: " << vecImageSegmentsRef.size());
//     JFR_DEBUG("match: " << vecImageSegmentsMatch.size());
	for (VecMatches::const_iterator it = matches.begin() ; it!=matches.end() ; ++it) {
//       JFR_DEBUG(it->first << " " << it->second);im
		if (vecImageSegmentsRef.at(it->first).id != vecImageSegments.at(it->second).id) {
			if (p_slam.hasFeature(vecImageSegmentsRef.at(it->first).id))
			{
				//JFR_DEBUG(it->second << " " << it->first);
				vecImageSegments.at(it->second).id =vecImageSegmentsRef.at(it->first).id;
				lineSegmentSet.lineSegments[it->second].id=vecImageSegmentsRef.at(it->first).id;
				cpt++;
			}
		}
	}
	return cpt;
	JFR_TRACE_END("compareId");
}

//FIXME
unsigned int ImageSegmentManager::compareIdInv(VecMatches const& matches, 
				    VecImageSegments const& vecImageSegmentsRef, VecImageSegments& vecImageSegments, jafar::lines::LineSegmentSet& lineSegmentSet)
{
	JFR_TRACE_BEGIN;
	unsigned int cpt = 0;
//     JFR_DEBUG("ref: " << vecImageSegmentsRef.size());
//     JFR_DEBUG("match: " << vecImageSegmentsMatch.size());
	for (VecMatches::const_iterator it = matches.begin() ; it!=matches.end() ; ++it) {
//       JFR_DEBUG(it->first << " " << it->second);im
		if (vecImageSegments.at(it->first).id != vecImageSegmentsRef.at(it->second).id) {
			if (p_slam.hasFeature(vecImageSegmentsRef.at(it->second).id))
			{	
				//JFR_DEBUG(it->first << " " << it->second);
				JFR_DEBUG(vecImageSegments.at(it->first).id<< " " << vecImageSegmentsRef.at(it->second).id);
				//vecImageSegments.at(it->first).id=vecImageSegmentsRef.at(it->second).id;				
				lineSegmentSet.lineSegments[it->first].id=vecImageSegmentsRef.at(it->second).id;
				cpt++;
			}
		}
	}
	return cpt;
	JFR_TRACE_END("compareIdinv");
}

//FIXME
unsigned int ImageSegmentManager::compareIdInvAll(VecMatches const& matches, 
					       VecImageSegments const& vecImageSegmentsRef, VecImageSegments& vecImageSegments, jafar::lines::LineSegmentSet& lineSegmentSet)
{
	JFR_TRACE_BEGIN;
	unsigned int cpt = 0;
//     JFR_DEBUG("ref: " << vecImageSegmentsRef.size());
//     JFR_DEBUG("match: " << vecImageSegmentsMatch.size());
	for (VecMatches::const_iterator it = matches.begin() ; it!=matches.end() ; ++it) {
//       JFR_DEBUG(it->first << " " << it->second);im
		if (vecImageSegments.at(it->first).id != vecImageSegmentsRef.at(it->second).id) {
			//if (p_slam.hasFeature(vecImageSegmentsRef.at(it->second).id))
			//{	
				//JFR_DEBUG(it->first << " " << it->second);
			JFR_DEBUG(vecImageSegments.at(it->first).id<< " " << vecImageSegmentsRef.at(it->second).id);
				//vecImageSegments.at(it->first).id=vecImageSegmentsRef.at(it->second).id;				
			lineSegmentSet.lineSegments[it->first].id=vecImageSegmentsRef.at(it->second).id;
			cpt++;
			//}
		}
	}
	return cpt;
	JFR_TRACE_END("compareIdinv");
}


// void ImageSegmentManager::getSegmentsPrediction(lines::LineSegmentSet& lineSegmentSet, VecImageSegments& vecImageSegments)
// {
// 	JFR_DEBUG("getFullyInitialisedSegmentsPrediction...");
// }

/*
 * class ImageHomogenCoordSegmentManager
 */
//Change DEFAULT VALUE TO FLAG
void ImageHomogenCoordSegmentManager::featureSelection(VecImageSegments& vecImageSegments) {
	for (VecImageSegments::iterator it = vecImageSegments.begin() ; it != vecImageSegments.end() ; ++it) {
		if (it->id == ImageSegment::NO_ID && isInZone(it->ext1) && isInZone(it->ext2) && ublas::norm_2(it->ext2-it->ext1) > p_minSegmentSize) {
			it->id = getNewFeatureId();
			//JFR_DEBUG("select feature: " << it->id);
		}
		if (isInZone(it->ext1) && isInZone(it->ext2) && ublas::norm_2(it->ext2-it->ext1) > p_minSegmentSize) {
		} 
		else {
			it->id = ImageSegment::NO_ID;
			//JFR_DEBUG("deselect feature: " << it->id);
		}
	}
}

void ImageHomogenCoordSegmentManager::slamProcessObservations(unsigned int currentFrameIndex, VecImageSegments& vecImageSegments) const
{
	SegmentObservation obs(type);
	jblas::vec2 line;

	typedef std::list<Observation*> ObsList;
	ObsList featuresObs;

	for (VecImageSegments::const_iterator it = vecImageSegments.begin() ; it != vecImageSegments.end() ; ++it) {
		if (it->id != ImageSegment::NO_ID) {
			//FIXME line = ImagePluckerFeatureObserveModel::extToImageModel(it->ext1, it->ext2);
            line = RhoThetaImagePluckerFeatureObserveModel::extToImageModel(it->ext1, it->ext2);
//            line = ScaledHomogeneousImagePluckerFeatureObserveModel::extToImageModel(it->ext1, it->ext2);
			obs.set(it->id, line);
			obs.setExtremities(it->ext1, it->ext2);
//       JFR_DEBUG(obs);
			featuresObs.push_back(new SegmentObservation(obs));
		}
	}

	slam.processObservations(currentFrameIndex, featuresObs);

}

void ImageHomogenCoordSegmentManager::initFrame(unsigned int frameIndex, VecImageSegments& vecImageSegments)
{
	JFR_DEBUG("ImageSegmentManager::initFrame:" << frameIndex << " nb segments: " << vecImageSegments.size());
	JFR_DEBUG("select features...");
	featureSelection(vecImageSegments);
	JFR_DEBUG("slam process...");
	slamProcessObservations(frameIndex, vecImageSegments);
}

void ImageHomogenCoordSegmentManager::processFrame(unsigned int frameIndex, VecImageSegments& vecImageSegments)
{
	JFR_DEBUG("processFrame...");	
	featureSelection(vecImageSegments);
	slamProcessObservations(frameIndex, vecImageSegments);
}

unsigned int ImageHomogenCoordSegmentManager::compareId(VecMatches const& matches, 
					    VecImageSegments const& vecImageSegmentsRef, VecImageSegments& vecImageSegments, jafar::lines::LineSegmentSet& lineSegmentSet)
{
	JFR_TRACE_BEGIN;
	unsigned int cpt = 0;
//     JFR_DEBUG("ref: " << vecImageSegmentsRef.size());
//     JFR_DEBUG("match: " << vecImageSegmentsMatch.size());
	for (VecMatches::const_iterator it = matches.begin() ; it!=matches.end() ; ++it) {
//       JFR_DEBUG(it->first << " " << it->second);im
		if (vecImageSegmentsRef.at(it->first).id != vecImageSegments.at(it->second).id) {
			if (slam.hasFeature(vecImageSegmentsRef.at(it->first).id))
			{
				//JFR_DEBUG(it->second << " " << it->first);
				vecImageSegments.at(it->second).id =vecImageSegmentsRef.at(it->first).id;
				lineSegmentSet.lineSegments[it->second].id=vecImageSegmentsRef.at(it->first).id;
				cpt++;
			}
		}
	}
	return cpt;
	JFR_TRACE_END("compareId");
}

//FIXME
unsigned int ImageHomogenCoordSegmentManager::compareIdInv(VecMatches const& matches, 
					       VecImageSegments const& vecImageSegmentsRef, VecImageSegments& vecImageSegments, jafar::lines::LineSegmentSet& lineSegmentSet)
{
	JFR_TRACE_BEGIN;
	unsigned int cpt = 0;
//     JFR_DEBUG("ref: " << vecImageSegmentsRef.size());
//     JFR_DEBUG("match: " << vecImageSegmentsMatch.size());
	for (VecMatches::const_iterator it = matches.begin() ; it!=matches.end() ; ++it) {
//       JFR_DEBUG(it->first << " " << it->second);im
		if (vecImageSegments.at(it->first).id != vecImageSegmentsRef.at(it->second).id) {
			if (slam.hasFeature(vecImageSegmentsRef.at(it->second).id))
			{	
				//JFR_DEBUG(it->first << " " << it->second);
				//JFR_DEBUG(vecImageSegments.at(it->first).id<< " " << vecImageSegmentsRef.at(it->second).id);
				//vecImageSegments.at(it->first).id=vecImageSegmentsRef.at(it->second).id;				
				lineSegmentSet.lineSegments[it->first].id=vecImageSegmentsRef.at(it->second).id;
				cpt++;
			}
		}
	}
	return cpt;
	JFR_TRACE_END("compareIdinv");
}

//FIXME
 unsigned int ImageHomogenCoordSegmentManager::compareIdInvAll(VecMatches const& matches, 
		 VecImageSegments const& vecImageSegmentsRef, VecImageSegments& vecImageSegments, jafar::lines::LineSegmentSet& lineSegmentSet)
{
	JFR_TRACE_BEGIN;
	unsigned int cpt = 0;
//     JFR_DEBUG("ref: " << vecImageSegmentsRef.size());
//     JFR_DEBUG("match: " << vecImageSegmentsMatch.size());
	for (VecMatches::const_iterator it = matches.begin() ; it!=matches.end() ; ++it) {
//       JFR_DEBUG(it->first << " " << it->second);im
		if (vecImageSegments.at(it->first).id != vecImageSegmentsRef.at(it->second).id) {
			//if (slam.hasFeature(vecImageSegmentsRef.at(it->second).id))
			//{	
				//JFR_DEBUG(it->first << " " << it->second);
			JFR_DEBUG(vecImageSegments.at(it->first).id<< " " << vecImageSegmentsRef.at(it->second).id);
				//vecImageSegments.at(it->first).id=vecImageSegmentsRef.at(it->second).id;				
			lineSegmentSet.lineSegments[it->first].id=vecImageSegmentsRef.at(it->second).id;
			cpt++;
			//}
		}
	}
	return cpt;
	JFR_TRACE_END("compareIdinv");
}

