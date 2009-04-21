/* $Id$ */

#ifndef SLAM_ABSTRACT_MAP_OBJECT_HPP
#define SLAM_ABSTRACT_MAP_OBJECT_HPP

#include "jmath/jblas.hpp"

namespace jafar {
  namespace slam {

    class SlamEkf;
    class DefaultMapManager;
    class LocalMapManager;
    class GlobalMapManager;
    class StrongMapManager;

	/// class Abstract Map Object
	///\ingroup slam
    class AbstractMapObject {

    public:
	  /// Constructor
		AbstractMapObject(unsigned int id) : m_id(id), m_x(0), m_P(0) {}
		
		/// Destructor
      virtual ~AbstractMapObject();
		
		/// Get object identifier
		///@return the object identifier
      unsigned int id() const {return m_id;}
	  
	  /// Get filter index
	  ///@return the filter index
      std::size_t filterIndex() const {return m_filterIndex;}

	  ///Get object state vector
	  ///@return the object's state vector
      jblas::vec_range& getX() {return *m_x;}
	  
	  ///Get object state vector
	  ///@return the object's state vector
      jblas::vec_range const& getX() const {return *m_x;}

	  ///Get object covariances matix
	  ///@return the object's covariances matrix
	  jblas::sym_mat_range& getP() {return *m_P;}
      
	  ///Get object covariances matix
	  ///@return the object's covariances matrix
	  jblas::sym_mat_range const& getP() const {return *m_P;}
            
	  /// Object's state size
      virtual std::size_t sizeState() const = 0;
	  
	  /// Set object's state vector and covariance
	  ///@param x_ state vector
	  ///@param P_ covariances matrix
      virtual void setState(jblas::vec& x_, jblas::sym_mat& P_);

    protected:

		  /// Object's ID
      unsigned int m_id;
	  
	  /// Filter index
      std::size_t m_filterIndex;
    private:
      
      jblas::vec_range* m_x;
      jblas::sym_mat_range* m_P;

      friend class DefaultMapManager;
      friend class LocalMapManager;
      friend class GlobalMapManager;
      friend class StrongMapManager;
    }; // AbstractMapObject
  }
}

#endif // SLAM_ABSTRACT_MAP_MANAGER_HPP
