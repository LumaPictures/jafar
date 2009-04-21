/* $Id$ */

#ifndef SLAM_BEARING_ONLY_FEATURE_HPP
#define SLAM_BEARING_ONLY_FEATURE_HPP

#include <vector>
#include <list>

#include "boost/tuple/tuple.hpp"

#include "kernel/jafarMacro.hpp"

#include "jmath/jblas.hpp"
#include "jmath/gaussianVector.hpp"

#include "slam/slamException.hpp"
#include "slam/observation.hpp"
#include "slam/feature.hpp"
#include "slam/featureModel.hpp"

namespace jafar {
  namespace slam {


    /** Initial state gaussian member (or hypothesis).
     *
     * \ingroup slam
     */
    class InitStateMember : public jafar::jmath::WeightedGaussianVector {

    public:

      /// likelihood of last observation
      double l;

      /// SPRT accumulated log-likelihood ratio
      double sprtL;

      std::vector<jafar::jmath::GaussianVector> initParams;

      jafar::jmath::GaussianVector zPred;

      InitStateMember(const jblas::vec& x_, const jblas::sym_mat& P_,
		      double w_,
		      std::size_t sizeObs) :
	WeightedGaussianVector(x_,P_,w_),
	l(0.0),
	sprtL(0.0),
	initParams(0),
	zPred(sizeObs)
      {}

      InitStateMember(const jblas::vec& x_, const jblas::sym_mat& P_,
		      double w_,
		      jafar::jmath::GaussianVector const& param1,
		      std::size_t sizeObs) :
	WeightedGaussianVector(x_,P_,w_),
	l(0.0),
	sprtL(0.0),
	initParams(1),
	zPred(sizeObs)
      {
	initParams[0] = param1;
      }
	

      InitStateMember(const jblas::vec& x_, const jblas::sym_mat& P_, 
		      double w_,
		      jafar::jmath::GaussianVector const& param1,
		      jafar::jmath::GaussianVector const& param2,
		      std::size_t sizeObs) :
	WeightedGaussianVector(x_,P_,w_),
	l(0.0),
	sprtL(0.0),
	initParams(2),
	zPred(sizeObs)
      {
	initParams[0] = param1;
	initParams[1] = param2;
      }

    };

    std::ostream& operator <<(std::ostream& s, const InitStateMember& ism);

    /** A generic bearing only feature.
     *
     * \ingroup slam
     */
    class InitFeature {

    private:

      unsigned int m_id;
      unsigned int m_robotId;
    public:

      BaseFeature::FrameIndexesType frameIndexes;

      typedef std::map<unsigned int, Observation*> InitObsType;
      InitObsType initObs;

      jblas::vec previousInitPose;
      jblas::vec deltaPose;
      jblas::sym_mat deltaPoseCov;

      /// the initial state is a sum of gaussians
      typedef std::list<InitStateMember*> InitStateType;
      InitStateType initState;

      double baselineMax;
        
      InitFeature(unsigned int id, unsigned int robotId, std::size_t sizeRobotPose);
      virtual ~InitFeature();

      unsigned int id() const {return m_id;}
      unsigned int robotId() const {return m_robotId;}

      unsigned int getRefFrameIndex() const;
      Observation const& getRefObservation() const;

      void addInitObservation(unsigned int frameIndex, 
			      jblas::vec_range const& refPose,
			      Observation* obs);
      void removeInitObservation(unsigned int frameIndex);
      bool hasInitObservation(unsigned int frameIndex) const;

      InitStateMember const& getBestInitStateMember() const;

      void clearInit();

      void clearInitStateZPred();

      void normalizeInitState();

    protected:

      virtual void writeLogHeader(jafar::kernel::DataLogger& log) const;
      virtual void writeLogData(jafar::kernel::DataLogger& log) const;

    };

    std::ostream& operator <<(std::ostream& s, const InitFeature& f_);


    /** Bearing only feature observation model.
     *
     * \ingroup slam
     */
    class BearingOnlyFeatureObserveModel : public BaseFeatureObserveModel {

    protected:

      /// threshold for updating or not the init state
      double m_doUpdateTh;

      /// threshold for updating or not the init state
      double m_eraseHypothesisTh;

      /// computes the beta value
      static double kSigmaToBeta(double alpha, double kSigma);

      static void basicUniformGaussianSum(double min, double max, 
					  double sigma, double kSigma,
					  std::list<jafar::jmath::GaussianVector>& gaussianSum);

      static void geometricUniformGaussianSum(double alpha, double beta,
					      double sMin, double sMax,
					      std::list<jafar::jmath::WeightedGaussianVector>& gaussianSum);

      /** This fonction computes the special sum of gaussians representing
       * points from the line of sight, and the parameters of the sum.
       */
      void boPointsGeometricGaussianSum(jblas::vec const& d_, jblas::sym_mat const& dCov_,
					double alpha_, double beta_,
					double sMin_, double sMax_,
					InitFeature::InitStateType& gaussianSum);

    public: 

      BearingOnlyFeatureObserveModel(FeatureModel& model, std::size_t sizeObs_);
      ~BearingOnlyFeatureObserveModel();

      void setup(double doUpdateTh = 8, double eraseHypothesisTh = 20) {
	m_doUpdateTh = doUpdateTh;
	m_eraseHypothesisTh = eraseHypothesisTh;
      }

      double eraseHypothesisTh() const {return m_eraseHypothesisTh;}

      /** Approximate the initial probability function with a sum of
       * gaussians. Delegates work to initStateInSensorFrame() and
       * takes care of the robot frame to sensor frame transformation.
       */
      void initState(InitFeature& feature_, 
		     Observation const& obs_,
                     double dMin_, double dMax_);

      virtual void initStateInSensorFrame(InitFeature& feature_,
					  Observation const& obs_,
                                          double dMin_, double dMax_) = 0;

      bool doUpdateInitState(jblas::vec const& closestMemberState, jblas::vec const& deltaPose);

    };

    /** A feature at infinity observe model. This kind of feature has
     * the same properties as a FeatureObserveModel.
     *
     * \ingroup slam
     */
    class InfFeatureObserveModel : public FeatureObserveModel {

    public:

      InfFeatureObserveModel(FeatureModel& model, std::size_t sizeObs_);

      virtual ~InfFeatureObserveModel() {}

      void setBaselineTh(double baselineTh) {m_baselineTh = baselineTh;}
      double getBaselineTh() const {return m_baselineTh;}

      /** Computes the baseline which is gained between a reference (pose,
       * observation) and a given pose.
       */
      double computeBaseline(jblas::vec const& poseRef, Observation const& obsRef, 
			     jblas::vec_range const& poseCur) const;

    private:

      double m_baselineTh;

    protected:

      /** Computes the baseline in the sensor frame.  
       * \sa computeBaseline()
       */
      virtual double computeBaselineInSensorFrame(jblas::vec_range const& deltaPose, 
						  Observation const& obsRef) const = 0;

    };

  } // namespace slam
} // namespace jafar    

#endif // SLAM_BEARING_ONLY_FEATURE_HPP
