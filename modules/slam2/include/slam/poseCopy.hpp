/* $Id$ */

#ifndef SLAM_POSE_COPY_HPP
#define SLAM_POSE_COPY_HPP

#include "jmath/jblas.hpp"

#include "slam/abstractMapManager.hpp"

namespace jafar {
  namespace slam {

    class SlamEkf;

    /** A special object of the map to estimate a local frame.
     * \ingroup slam
     */
    class PoseCopy : public AbstractMapObject {

    public:

      PoseCopy(std::size_t sizeState, unsigned int id, unsigned int frameIndex) :
	AbstractMapObject(id),
	m_sizeState(sizeState),
	m_frameIndex(frameIndex),
        m_id(id),
	m_x(0),
	m_xCov(0)
      {}

      ~PoseCopy()
      {
	if (m_x) delete m_x;
	if (m_xCov) delete m_xCov;
      }

      std::size_t sizeState() const {return m_sizeState;}
      unsigned int frameIndex() const {return m_frameIndex;}

      void setState(jblas::vec& x, jblas::sym_mat& P)
      {
	if (m_x) delete m_x;
	if (m_xCov) delete m_xCov;
	ublas::range r(filterIndex(), filterIndex()+sizeState());
	m_x = new jblas::vec_range(x, r);
	m_xCov = new jblas::sym_mat_range(P, r, r);
      }

      unsigned int id() const {return m_id;}
      jblas::vec_range const& x() const {return *m_x;}
      jblas::sym_mat_range const& xCov() const {return *m_xCov;}

    private:

      std::size_t m_sizeState;
      unsigned int m_frameIndex;

      unsigned int m_id;
      jblas::vec_range* m_x;
      jblas::sym_mat_range* m_xCov;

      friend class SlamEkf;      

    }; // class PoseCopy

  }
}

#endif // SLAM_POSE_COPY_HPP
