/* $Id: lineTools.hpp 2059 2006-10-24 06:52:21Z tlemaire $ */

/** \file lineTools.hpp
*  Toolbox for operations related to lines in SLAM (Plucker, homogeneous, rho-theta, and others)
* \ingroup slam
*/




#ifndef SLAM_PLUCKER_TOOLS_HPP
#define SLAM_PLUCKER_TOOLS_HPP

#include "kernel/jafarException.hpp"
#include "jmath/jblas.hpp"
#include "boost/tuple/tuple.hpp"
#include "slam/eulerTools.hpp"

namespace jafar {
	namespace slam {
		/**
		* Functions for operations related to lines in SLAM (Plucker, homogeneous, rho-theta, and others)
		 * \ingroup slam
		 */
		namespace lineTools {	
			
			/// Build Plucker matrix from calibration parameters
			///@param u0 horizontal coordinate of principal point, in pixels
			///@param v0 vertical coordinate of principal point
			///@param au focal distance, in horizontal pixel units
			///@param av focal distance, in vertical pixel units
			///@param K  Plucker intrinsic matrix
			template<class Mat>
			void pluckerIntrinsicMatrix(double u0, double v0, double au, double av, 
										Mat& K);
			
			/// Build Plucker matrix from calibration vector
			///@param k calibration vector k = (u0, v0, au, av)
			///@param K  Plucker intrinsic matrix
			template<class Vec, class Mat>
			void pluckerIntrinsicMatrix(Vec const& k, 
										Mat& K);
			
			
			
			/// Build inverse Plucker camera matrix from calibration parameters
			///@param u0 horizontal coordinate of principal point, in pixels
			///@param v0 vertical coordinate of principal point
			///@param au focal distance, in horizontal pixel units
			///@param av focal distance, in vertical pixel units
			///@param iK inverse Plucker intrinsic matrix
			void pluckerInverseIntrinsicMatrix(double u0, double v0, double au, double av, 
											   jblas::mat& iK);
			
			
			/// Build inverse Plucker camera matrix from calibration vector
			///@param k calibration vector k = (u0, v0, au, av)
			///@param iK inverse Plucker intrinsic matrix
			void pluckerInverseIntrinsicMatrix(jblas::vec4 const& k, 
											   jblas::mat& iK);
			
			
			
			/// Homogeneous 2D line to rho-theta representation
			///@param l homogeneous 2D line in the image plane
			///@return the rho-theta representation for the same line
			jblas::vec2 homogeneousToRhoTheta(jblas::vec3 const& l);
			
			
			/// homogeneous 2D line to rho-theta, with Jacobian
			///@param l the homogeneous 2D line
			///@param rt the rho-theta representation
			///@param RT_l the Jacobian of rt wrt l
			void homogeneousToRhoTheta(jblas::vec3 const& l, 
									   jblas::vec2& rt,
									   jblas::mat& RT_l) ;
			
			
			
			/// Rho-theta to homogeneous 2D line transform
			///@param rt rho-theta representation of the 2D line
			///@return the homogeneous 2D line
			jblas::vec3 rhoThetaToHomogeneous(jblas::vec2 const& rt);
			
			
			/// Rho-theta to homogeneous 2D line transform -- Jacobian
			///@param rt the rho-theta representation
			///@param l the homogeneous 2D line
			///@param L_rt the Jacobian of l wrt rt
			void rhoThetaToHomogeneous(jblas::vec2 const& rt, 
									   jblas::vec3& l, 
									   jblas::mat&  L_rt);
			
			
			
			/// Project Plucker 3D line into homogeneous 2D line
			///@param K the Plucker intrinsic matrix
			///@param L the Plucker line
			///@return the homogeneous 2D line in the image plane
			jblas::vec3 projectPlucker(jblas::mat33 const& K, 
									   jblas::vec6 const& L);
			
			
			/// Project Plucker 3D line into homogeneous 2D line, with Jacobians
			///@param K the Plucker intrinsic matrix
			///@param L the Plucker line
			///@param l the homogeneous 2D line in the image plane
			///@param L_l the Jacobian of l wrt L
			void projectPlucker(jblas::mat33 const& K, 
								jblas::vec6 const& L, 
								jblas::vec3& l, 
								jblas::mat& L_l);
			
			
			
			/// Retro project homogeneous 2D line to representation plane normal vector
			///@param iK the inverse Plucker intrinsic matrix
			///@param l the homogeneous 2D line
			///@return the 3D Plucker line
			jblas::vec3 retroProjectPluckerPlaneNormal(jblas::mat33 const& iK, 
													   jblas::vec3  const& l);
			
			
			/// Retro project homogeneous 2D line to representation plane normal vector, with Jacobians
			/// @param iK the inverse plucker camera matrix
			/// @param l  the homogeneous 2D line
			/// @param n  the representation plane's normal vector
			/// @param N_l the Jacobian of n wrt l.
			void retroProjectPluckerPlaneNormal(jblas::mat33  const& iK, 
												jblas::vec3   const& l,
												jblas::vec3&  n,
												jblas::mat33& N_l);
			
			
			/// Retro project homogeneous 2D line to Plucker 3D line
			///@param iK the inverse Plucker intrinsic matrix
			///@param l the 2D homogeneous line
			///@param v the line's director vector, unmeasurable from l
			///@return the 3D Plucker line
			jblas::vec6 retroProjectPlucker(jblas::mat33  const& iK, 
											jblas::vec3 const& l, 
											jblas::vec3 const& v);
			
			
			/// Retro project homogeneous 2D line to Plucker 3D line, with Jacobians
			///@param iK the inverse Plucker intrinsic matrix
			///@param l the 2D homogeneous line
			///@param v the line's director vector, unmeasurable from l
			///@param L the 3D Plucker line
			///@param L_l the Jacobian of L wrt l
			///@param L_v the Jacobian of L wrt v
			void retroProjectPlucker(jblas::mat33  const& iK, 
									 jblas::vec3 const& l, 
									 jblas::vec3 const& v, 
									 jblas::vec6& L, 
									 jblas::mat& L_l, 
									 jblas::mat& L_v);
			
			
			
			
			/// Retro project homogeneous 2D line to Plucker 3D line providing base vector
			///@param iK the inverse Plucker intrinsic matrix
			///@param l the 2D homogeneous line
			///@param beta the line's director 2-vector defined in the local representation plane
			///@return the 3D Plucker line
			jblas::vec6 retroProjectPluckerFromBase(jblas::mat33 const& iK,
													jblas::vec3 const& l,
													jblas::vec2 const& beta);
			
			
			/// Retro project homogeneous 2D line to Plucker 3D line providing base vector, with Jacobians
			///@param iK the inverse Plucker intrinsic matrix
			///@param l the 2D homogeneous line
			///@param beta the line's director 2-vector defined in the local representation plane
			///@param L the 3D Plucker line
			///@param L_l the Jacobian of L wrt l
			///@param L_beta the Jacobian of L wrt beta
			void retroProjectPluckerFromBase(jblas::mat33 const& iK,
											 jblas::vec3 const& l,
											 jblas::vec2 const& beta,
											 jblas::vec6& L,
											 jblas::mat&  L_l,
											 jblas::mat&  L_beta);
			
			
			
			
			/// Plane sub-vector to plane base matrix
			///@param n the representation plane's normal vector
			///@return the base matrix transforming vectors in the representation plane to 3D vectors
			jblas::mat planeVecToBaseMatrix(jblas::vec3 const& n);
			
			
			
			
			/// Plane sub-vector 'n' and plane base to direction vector
			///@param n the representation plane's normal vector
			///@param beta the line's director 2-vector defined in the local representation plane
			///@return  the line's director 3-vector
			jblas::vec3 planeBaseToDirVector(jblas::vec3 const& n, 
											 jblas::vec2 const& beta);
			
			/// Plane sub-vector 'n' and plane base to direction vector, with Jacobians
			///@param n the representation plane's normal vector
			///@param beta the line's director 2-vector defined in the local representation plane
			///@param v the line's director 3-vector
			///@param V_n the Jacobian ov V wrt n
			///@param V_beta the Jacobian of V wrt beta
			void planeBaseToDirVector(jblas::vec3 const& n, 
									  jblas::vec2 const& beta, 
									  jblas::vec3& v, 
									  jblas::mat33& V_n, 
									  jblas::mat& V_beta);
			
			/// Plane sub-vector 'n' and plane base to direction vector, with Jacobians
			///@param n the representation plane's normal vector
			///@param beta the line's director 2-vector defined in the local representation plane
			///@param v the line's director 3-vector
			///@param V_n the Jacobian ov V wrt n
			///@param V_beta the Jacobian of V wrt beta
			void planeBaseToDirVector(jblas::vec_range const& n, 
									  jblas::vec2 const& beta, 
									  jblas::vec_range& v, 
									  jblas::mat33& V_n, 
									  jblas::mat& V_beta);
			
			
			
			/// Normalize 3-vector
			///@param v  input 3-vector
			///@return The normalized vector w = v/norm(v)
			jblas::vec3 normVec3(jblas::vec3 const& v);
			
			/// Normalize 3-vector, with Jacobians.
			///@param v  input 3-vector
			///@param nv  output normalized 3-vector 
			///@param NV_v  output 3x3 normalization Jacobian
			void normVec3(jblas::vec3 const& v, 
						  jblas::vec3& vn, 
						  jblas::mat33& VN_v);
			
			
			/// Normalize 3-vector, with Jacobians.
			///@param v  input 3-vector
			///@param nv  output normalized 3-vector 
			///@param NV_v  output 3x3 normalization Jacobian
			void normVec3(jblas::vec3 const& v, 
						  jblas::vec_range& vn, 
						  jblas::mat33& VN_v);
			
			
			/// Bring a vector to the norm of another one
			///@param v the nevtor to adjust
			///@param n the vector used as reference
			///@return w the adjusted vector
			jblas::vec3 adjustVec3(jblas::vec3 const& v,
								   jblas::vec3 const& n);
			
			/// Bring a vector to the norm of another one - with Jacobian
			///@param v the nevtor to adjust
			///@param n the vector used as reference
			///@param w the adjusted vector
			///@param W_v the Jacobian wrt v
			void adjustVec3(jblas::vec3 const& v,
							jblas::vec3 const& n,
							jblas::vec3& w,
							jblas::mat33& W_v);
			
			
			
			/// Plucker line's origin, the closest point to the coordinates origin.
			/// The point is computed with p0 = cross(n,v)/dot(v,v), where n and v are the sub-vectors of the Plucker line, L = [n;v].
			/// See the function with Jacobians for formulae.
			///@param L  Plucker line
			///@return  line origin, defined as the closest point to the global origin
			jblas::vec3 pluckerOrigin(jblas::vec6 const& L);
			
			
			/// Plucker line's origin, with Jacobian. \n
			/// The Plucker line is divided in sub-vectors: \n
			///   n   = subrange(L,0,3) is the plane normal, \n
			///   v   = subrange(L,3,6) is the director vector. \n
			/// Then the closest point of L to the global origin of coordinates is \n
			///   p0  = cross(n,v)/dot(v,v). \n
			///@param L  Plucker line
			///@param p0  line origin, defined as the closest point to the global origin
			///@param P0_l Jacobian of the origin with respect to the line
			void pluckerOrigin(jblas::vec6 const& L, 
							   jblas::vec3& p0, 
							   jblas::mat& P0_l);
			
			
			/// 3-by-3 skew-symmetric matrix.
			///@param v a 3-vector
			///@return the skew symmetric matrix (corresponding to the cross product)
			jblas::mat33 skewSymmMat33(jblas::vec3 const& v);
			
			
			/// Compute Plucker line endpoint
			///@param L the Plucker line
			///@param s the abcissa of the endpoint in the line's local frame
			///@return the 3D endpoint
			jblas::vec3 pluckerEndpoint(jblas::vec6 const& L, 
						    double const s);
			
			/// Compute Plucker line endpoint, with Jacobians
			///@param L Plucker line
			///@param s Endpoint's abscissa in the line's local frame
			///@param e 3D Endpoint
			///@param E_l Jacobian of e wrt L
			///@param E_s Jacobian of e wrt s
			void pluckerEndpoint(jblas::vec6 const& L, // Plucker line
					     double const s, // Abscissa
					     jblas::vec3& e, // Endpoint
					     jblas::mat& E_l,  // Jac wrt the line
					     jblas::mat& E_s); // Jac. wrt abscissa
			
			
			/// Ray from pixel, as a Plucker line
			///@param K intrinsic matrix
			///@param p a 2-pixel
			///@return the ray as a Plucker line
			jblas::vec6 pluckerRay(jblas::mat33 const& K, 
								   jblas::vec2 const& p);
			
			
			/// Ray as a Plucker line from pixel, with Jacobians
			///@param K intrinsic matrix
			///@param p a 2-pixel
			///@param r the ray as a Plucker line
			///@param R_p the Jacobian of r wrt p
			void pluckerRay(jblas::mat33 const& K, 
							jblas::vec2 const& p,
							jblas::vec6 r,
							jblas::mat R_p);
			
			
			
			/// Abscissas of the imperfect intersection of 2 Plucker lines. 
			/// This function computes the distance between two Plucker lines 
			/// and returns the abscisses of the points on both lines that are 
			/// closest to the other line. This represents an imperfect intersection.
			///@param line1 The first Plucker line
			///@param line2 The second Plucker line
			///@return a 3-tuple with s1, s2 the abscissas of the points in lines 1 and 2 closest to lines 2 and 1, and d the distance between both lines
			boost::tuple<double,double,double> pluckerSoftIntersectionAbscissa(jblas::vec6 const& line1, 
																			   jblas::vec6 const& line2);
			
			
			/// Normalize Plucker line to convert it to Euclidean Plucker line
			///@param L the Plucker line
			///@return the Euclidean Plucker line
			jblas::vec6 pluckerToEuclideanPlucker(jblas::vec6 const& L);
			
			/// Normalize Plucker line to convert it to Euclidean Plucker line, with Jacobian
			///@param L the Plucker line
			///@param eL the Euclidean Plucker line
			///@param EL_l the Jacobian 
			void pluckerToEuclideanPlucker(jblas::vec6 const& L, 
										   jblas::vec6& eL, 
										   jblas::mat66& EL_l);
			
			
			
			/// Two endpoints to homogeneous 2D line
			jblas::vec3 extToHomogeneous(jblas::vec2 const& p, jblas::vec2 const& q);
			
			
			/// Two endpoints to homogeneous 2D line - with Jacobians
			void extToHomogeneous(jblas::vec2 const& p, 
								  jblas::vec2 const& q,
								  jblas::vec3& hm,
								  jblas::mat& HM_p, 
								  jblas::mat& HM_q);
 

            /// Extract a plane from 3 euclidean points
             void points3plane(jblas::vec3 const& a, jblas::vec3 const& b, jblas::vec3 const& c, jblas::vec6& plane);

            
            /// Homogeneous Points to Plucker
			jblas::vec6 hmPointsToPlucker(jblas::vec4& A, jblas::vec4& B);
            
            /// Homogeneous Point to Plucker Jacobians
            void hmPointsToPlucker(jblas::vec4& A, jblas::vec4& B, jblas::vec6& L,jblas::mat& L_a,jblas::mat& L_b);

            /// stereo pixel to homogeneous Point
            jblas::vec4 uvdToHmPoint(jblas::vec3 const& uvd, jblas::vec4& k, double const alpha);

            /// stereo pixel to homogeneous Point Jacobian
            void uvdToHmPoint(jblas::vec3 const& uvd, jblas::vec4& k, double const alpha, jblas::vec4& hmP, jblas::mat& HmPu);
			
				
			
			/// Plucker line to two stereo homogeneous lines
			///@param k the intrinsic parameters vector (u0,v0,alphaU,alphaV)
			///@param alpha the baseline factor, alpha = b*alphaU
			///@param L the Plucker line
			///@return a 6-vector with 2 stacked homogeneous lines, for left and right images
			/*template <class IntrVec, class PlkVec>
				jblas::vec6 projectPluckerToStereo(IntrVec const& k,
												   double alpha, 
												   PlkVec const& L)
				{
					
					// right camera frame
					jblas::vec6 rightFrame;
					rightFrame.clear();
					rightFrame(0) = -alpha/k(2);
					
					
					jblas::vec6 LR;
					jafar::slam::EulerTools::lineToFrame(rightFrame, L, LR);
					
					jblas::mat33 K;
					pluckerIntrinsicMatrix(k, K);
					
					jblas::vec6 stLine;
					
					jblas::vec3 hm;
					
					hm = projectPlucker(K, L);
					subrange(stLine, 0, 3) = hm;
					
					hm = projectPlucker(K, LR);
					subrange(stLine, 3, 6) = hm;
					
					return stLine;
					
				}*/
			jblas::vec6 projectPluckerToStereo(jblas::vec4 const& k,
											   double alpha, 
											   jblas::vec6 const& L);
			
			
			
			
			/// Plucker line to two stereo homogeneous lines - with Jacobians
			///@param k the intrinsic parameters vector (u0,v0,alphaU,alphaV)
			///@param alpha the baseline factor, alpha = b*alphaU
			///@param L the Plucker line
			///@param stLine a 6-vector with 2 stacked homogeneous lines, for left and right images
			///@param StLine_l the Jacobian of stLine wrt L
			//template <class StereoLineObs, class IntrVec, class PlkVec, class JacMat>
			//	void projectPluckerToStereo(IntrVec const&  k, 
			//								double          alpha,
			//								PlkVec const&   L,
			//								StereoLineObs&  stLine,
			//								JacMat&         StLine_l);
			void projectPluckerToStereo(jblas::vec4 const&   k, 
										double               alpha,
										jblas::vec6 const&   L,
										jblas::vec6&         stLine,
										jblas::mat66&        StLine_l);
			
			
			/// Segment extremities to rho theta line conversion
			///@param ext1 one endpoint
			///@param ext2 the other endpoint
			///@return the rho-theta line
			jblas::vec2 extToRhoTheta(const jblas::vec2& ext1, const jblas::vec2& ext2);
			
			/// Segment extremities to rho theta line conversion - Jacobians
			///@param ext1 one endpoint
			///@param ext2 the other endpoint
			///@param rt the rho-theta line
			///@param RT_ee the Jacobian of \a rt wrt \a ext1 and \a ext2
			void extToRhoTheta(const jblas::vec2& ext1, const jblas::vec2& ext2, jblas::vec2& rt, jblas::mat& RT_ee);			
			
			
			
			/// Convert stereo segment to rho-theta stereo line
			///@param uvduvd a 6-vector with the two stereo endpoints in uvd format.
			///@return a 4-vector with two rho-theta lines
			jblas::vec4 uvdsToStereoRhoTheta(jblas::vec6 const& uvduvd);
			
			/// Convert stereo segment to rho-theta stereo line - Jacobians
			///@param uvduvd a 6-vector with the two stereo endpoints in uvd format.
			///@param rtrt a 4-vector with two rho-theta lines
			///@param RR_uu the Jacobian of \a rtrt wrt \a uvduvd 
			void uvdsToStereoRhoTheta(jblas::vec6 const& uvduvd, jblas::vec4& rtrt, jblas::mat& RR_uu);
			
			/// Convert stereo segment to rho-theta stereo line - only Jacobians
			///@param uvduvd a 6-vector with the two stereo endpoints in uvd format.
			///@param RR_uu the Jacobian of uvdsToStereoRhoTheta() wrt \a uvduvd
			void uvdsToStereoRhoTheta(jblas::vec6 const& uvduvd, jblas::mat& RR_uu);
			
      /**
       * @param _hm homogenous vector for the line parameters
       * @param 
       */
			void homogenousToExtDistances( const jblas::vec3& _hm, const jblas::vec2& ext1, const jblas::vec2& ext2, jblas::vec2& dd, jblas::mat& DD_hm, jblas::mat& DD_ext1, jblas::mat& DD_ext2 );
      
      void homogenousToPointDistance( const jblas::vec3& _hm, const jblas::vec2& _ext, double& d, jblas::mat_range& D_hm, jblas::mat_range& D_ext );
      
      void homogenousToExtDistances( const jblas::vec3& _hm, const jblas::vec2& ext1, const jblas::vec2& ext2, jblas::vec2& dd, jblas::mat& DD_hm);
      
      void homogenousToPointDistance( const jblas::vec3& _hm, const jblas::vec2& _ext, double& d, jblas::mat_range& D_hm );
		} 
	}
	
}

#endif

