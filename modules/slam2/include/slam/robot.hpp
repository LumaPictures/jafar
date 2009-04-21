/* $Id$ */

#ifndef SLAM_ROBOT_HPP
#define SLAM_ROBOT_HPP

#include "jmath/jblas.hpp"

#include "slam/abstractMapObject.hpp"

namespace jafar {
  namespace filter {
    class JacobianBlockCommandPredictModel;
  }
  namespace slam {

    /**
     * Base robot model.
     * It gives access to information about the robot state in the filter, such as the index
     * in the filter, or convenient vectors/matrixes that point to the state and covariance.
     * \ingroup slam
     */
    class BaseRobot : public AbstractMapObject {
      friend class SlamEkf;
      public:
        jafar::filter::JacobianBlockCommandPredictModel& model;
      public:
        BaseRobot(unsigned int id, jafar::filter::JacobianBlockCommandPredictModel& model, std::size_t sizePose = 6);
        ~BaseRobot();
        /**
         * @return the size of the pose vector
         */
        std::size_t sizePose() const { return m_sizePose; }
        /**
         * @return the size of the state of the robot
         */
        std::size_t sizeState() const;
        /**
         * @return a vector range corresponding to the pose of the robot
         */
        jblas::vec_range* refPose() { return m_refPose; }
        /**
         * @return the covariance matrix corresponding to the pose of the robot
         */
        jblas::sym_mat_range* refPoseCov() { return m_refPoseCov; }
        /**
         * @return a vector range corresponding to the pose of the robot
         */
        const jblas::vec_range* refPose() const { return m_refPose; }
        /**
         * @return the covariance matrix corresponding to the pose of the robot
         */
        const jblas::sym_mat_range* refPoseCov() const { return m_refPoseCov; }
      private:
        std::size_t m_sizePose;
        jblas::vec_range* m_refPose;
        jblas::sym_mat_range* m_refPoseCov;
    };
    
    
//     /** Base robot model.
//      *
//      * \ingroup slam
//      */
//     template<std::size_t sizePose, class PredictModel>
//     class BaseRobotModel {

//     protected:

//     public:

//       PredictModel predictModel;

//       BaseRobotModel() : 
// 	predictModel()
//       {
// 	JFR_PRECOND(sizePose < predictModel.sizeState(),
// 		    "BaseRobotModel::BaseRobotModel: invalid sizePose");
//       };

//       virtual ~BaseRobotModel() {};

//       std::size_t sizePose() const {return sizePose;};
//       std::size_t sizeState() const {return predictModel.sizeState();};


//     };




  } // namespace slam
} // namespace jafar


#endif // SLAM_ROBOT_HPP
