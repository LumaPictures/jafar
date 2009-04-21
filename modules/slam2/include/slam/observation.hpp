/* $Id$ */

#ifndef SLAM_OBSERVATION_HPP
#define SLAM_OBSERVATION_HPP

#include <ostream>

#include "jmath/jblas.hpp"
#include "kernel/jafarMacro.hpp"

namespace jafar {
	namespace slam {


		/** Observation.
		*
		* \ingroup slam
		*/
		class Observation {
		 public:

			enum ObservationType {
				POINT_CARTESIAN,     ///< sizeless point observation in the robot cartesian frame
				POINT_POLAR,         ///< sizeless point observation in the robot polar frame
				POINT_STEREOIMAGE,   ///< sizeless point observation from a stereoimage pair
				POINT_BEARING,       ///< sizeless point partial observation in the robot polar frame
				POINT_IMAGE,         ///< sizeless point partial observation in the image plane
				POINT_OMNIIMAGE,     ///< sizeless point partial observation in the omnidirectional image plane
				SEGMENT_IMAGE,       ///< parameters of the supporting of a segment extracted from an image
				SEGMENTID_IMAGE,     ///< parameters of the supporting of an inv depth segment extracted from an image
				SEGMENTID_EXT_IMAGE, ///< extremeties of a segment (in inverse depth)
				SEGMENT_STEREOIMAGE, ///< parameters of the supporting of a segment extracted from stereoimage pair
				BASIS,               ///< observation of a basis (including a position vector and three euler angles)
				ROBOT                ///< observation of an other robot
			};

			static std::size_t observationSize(ObservationType ot) {
				switch(ot) {
					case POINT_CARTESIAN:     return 3;
					case POINT_POLAR:         return 3;
					case POINT_STEREOIMAGE:   return 3;
					case POINT_BEARING:       return 2;
					case POINT_IMAGE:         return 2;
					case POINT_OMNIIMAGE:     return 2;
					case SEGMENT_IMAGE:       return 2;
					case SEGMENTID_IMAGE:     return 2;
					case SEGMENTID_EXT_IMAGE: return 4;
					case SEGMENT_STEREOIMAGE: return 4;
					case BASIS:               return 6;
					case ROBOT:               return 6;
					default: JFR_RUN_TIME("Undefined observation size for type: " << ot);
				}
			}

			ObservationType type; ///< type of observation
			int sensorId;         ///< The id of the sensor (it's a unique number associated to an unique sensor, it's up to the user to choose the numbers)
			unsigned int id;      ///< Feature Id
			jblas::vec z;         ///< Observation vector
			unsigned int robotId; ///< Id of the robot which made an observation

			Observation(ObservationType type_, unsigned int robotId_ = 0) :
				type(type_), sensorId(), id(), z(observationSize(type)), robotId(robotId_)
			{}

			Observation(Observation const& obs) :
				type(obs.type), sensorId(obs.sensorId), id(obs.id), z(obs.z), robotId(obs.robotId)
			{}

			virtual ~Observation() {}

			inline std::size_t size() const {return z.size();};

			void set(unsigned int id_, const jblas::vec& z_, unsigned int sensorId_ = 0) {
				JFR_PRECOND(z_.size() == z.size(), "Observation::set: invalid size ");
				sensorId = sensorId_;
				id = id_;
				z.assign(z_);
			}

			void set(unsigned int id_, const jblas::vec& z_, unsigned int sensorId_, unsigned int robotId_) {
				JFR_PRECOND(z_.size() == z.size(), "Observation::set: invalid size ");
				sensorId = sensorId_;
				robotId = robotId_;
				id = id_;
				z.assign(z_);
			}

		}; // class Observation

		std::ostream& operator <<(std::ostream& s, Observation::ObservationType const& t);
		std::ostream& operator <<(std::ostream& s, Observation const& o_);

	} // namespace slam
} // namespace jafar


#endif // SLAM_OBSERVATION_HPP
