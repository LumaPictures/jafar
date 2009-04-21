/* $Id$ */

/** \file eulerTools.hpp 
* Toolbox for operations related to Euler angles in SLAM
* \ingroup slam
*/

#ifndef SLAM_EULER_TOOLS_HPP
#define SLAM_EULER_TOOLS_HPP

#include "kernel/jafarException.hpp"

#include "jmath/jblas.hpp"

namespace jafar {
  namespace slam {

	  /**
	  * Functions for operations related to Euler angles lines in SLAM
	   * \ingroup slam
	   */
	  namespace EulerTools {
		
      enum H {h_0, h_1};

		namespace details {

	void toFrame(double x, double y, double z,
		     double yaw, double pitch, double roll, 
		     double v_x, double v_y, double v_z, H h,
		     double& vres_x, double& vres_y, double& vres_z);

	template<class M_Jframe, class M_Jv>
	void toFrameJac(double x, double y, double z,
			double yaw, double pitch, double roll,
			double v_x, double v_y, double v_z, H h,
			M_Jframe& Jframe, M_Jv& Jv);

	void fromFrame(double x, double y, double z,
		       double yaw, double pitch, double roll, 
		       double v_x, double v_y, double v_z, H h,
		       double& vres_x, double& vres_y, double& vres_z);

	template<class M_Jframe, class M_Jv>
	void fromFrameJac(double x, double y, double z,
			  double yaw, double pitch, double roll, 
			  double v_x, double v_y, double v_z, H h,
			  M_Jframe& Jframe, M_Jv& Jv);

	void composeFrame(double x2, double y2, double z2,
			  double yaw2, double pitch2, double roll2,
			  double x1, double y1, double z1,
			  double yaw1, double pitch1, double roll1,
			  double& x_res, double& y_res, double& z_res,
			  double& yaw_res, double& pitch_res, double& roll_res);

	void composeFrameJac(double x2, double y2, double z2,
			     double yaw2, double pitch2, double roll2,
			     double x1, double y1, double z1,
			     double yaw1, double pitch1, double roll1,
			     jblas::mat& Jframe1, jblas::mat& Jframe2);

	void invFrame(double x, double y, double z,
		      double yaw, double pitch, double roll,
		      double& x_inv, double& y_inv, double& z_inv,
		      double& yaw_inv, double& pitch_inv, double& roll_inv);

	void lineToFrame(double x, double y, double z, 
			 double yaw, double pitch, double roll, 
			 double n1, double n2, double n3, double u1, double u2, double u3,
			 double& n1Res, double& n2Res, double& n3Res, double& u1Res, double& u2Res, double& u3Res);
			

	void lineToFrameJac(double x, double y, double z,
			    double yaw, double pitch, double roll,
			    double n1, double n2, double n3, double u1, double u2, double u3,
			    jblas::mat& Jframe, jblas::mat& Jl);

	void lineFromFrame(double x, double y, double z, 
			   double yaw, double pitch, double roll,
			   double n1, double n2, double n3, double u1, double u2, double u3,
			   double& n1Res, double& n2Res, double& n3Res, double& u1Res, double& u2Res, double& u3Res);
	
	void lineFromFrameJac(double x, double y, double z,
			      double yaw, double pitch, double roll,
			      double n1, double n2, double n3, double u1, double u2, double u3,
			      jblas::mat& Jframe, jblas::mat& Jl);

    void planeToFrame(double x, double y, double z, 
             double yaw, double pitch, double roll, 
             double px, double py, double pz, double pw,
             double& px_res, double& py_res, double& pz_res, double& pw_res);
            

    void planeToFrameJac(double x, double y, double z,
                double yaw, double pitch, double roll,
                double px, double py, double pz, double pw,
                jblas::mat& Jframe, jblas::mat& Jp);

    void planeFromFrame(double x, double y, double z, 
             double yaw, double pitch, double roll, 
             double px, double py, double pz, double pw,
             double& px_res, double& py_res, double& pz_res, double& pw_res);
            

    void planeFromFrameJac(double x, double y, double z,
                double yaw, double pitch, double roll,
                double px, double py, double pz, double pw,
                jblas::mat& Jframe, jblas::mat& Jp);


	void odo3d(double x, double y, double z,
		   double yaw, double pitch, double roll,
		   double v, double w,
		   double dt,
		   double& x_res, double& y_res, double& z_res,
		   double& yaw_res, double& pitch_res, double& roll_res);

	void odo3dJac(double x, double y, double z,
		      double yaw, double pitch, double roll,
		      double v, double w,
		      double dt,
		      jblas::mat& Jx, jblas::mat& Jodo);

	void odo3dInRefFrame(//double x_robotToRef, double y_robotToRef, double z_robotToRef,
			     double yaw_robotToRef, 
			     //double pitch_robotToRef, double roll_robotToRef,
			     double x, double y, double z,
			     double yaw, double pitch, double roll,
			     double v, double w,
			     double dt,
			     double& x_res, double& y_res, double& z_res,
			     double& yaw_res, double& pitch_res, double& roll_res);
	
	void odo3dInRefFrameJac(// double x_robotToRef, double y_robotToRef, double z_robotToRef,
				double yaw_robotToRef, 
				// double pitch_robotToRef, double roll_robotToRef,
				double x, double y, double z,
				double yaw, double pitch, double roll,
				double v, double w,
				double dt,
				jblas::mat& Jx, jblas::mat& Jodo);

      } // namespace detail

      template<class VecBearings, class VecUnitVector>
      void bearingsToUnitVector(VecBearings const& b, VecUnitVector& u) 
      {
	JFR_PRECOND(b.size() == 2,
		    "EulerTools::directionToUnitVector: invalid size of b" << b);
	JFR_PRECOND(u.size() == 3,
		    "EulerTools::directionToUnitVector: invalid size of u");

	double t1 = cos(b(1));
	u(0) = t1*cos(b(0));
	u(1) = t1*sin(b(0));
	u(2) = -sin(b(1));
      };

      template<class VecBearings>
      void bearingsToUnitVectorJac(VecBearings const& b, jblas::mat& J)
      {
	JFR_PRECOND(b.size() == 2,
		    "EulerTools::bearingsToUnitVectorJac: invalid size of b" << b);
	JFR_PRECOND(J.size1()==3 && J.size2()==2,
		    "EulerTools::bearingsToUnitVectorJac: size of J does not match");

	/* begin copy/paste from maple */
	double t1 = cos(b(1));
	double t2 = sin(b(0));
	double t4 = sin(b(1));
	double t5 = cos(b(0));
	J(0,0) = -t1 * t2;
	J(0,1) = -t4 * t5;
	J(1,0) = t1 * t5;
	J(1,1) = -t4 * t2;
	J(2,0) = 0.0e0;
	J(2,1) = -t1;
	/* end copy/paste from maple */
      };

      template<class VecVector, class VecDirection>
      void vectorToBearings(VecVector const& v, VecDirection& b)
      {
 	JFR_PRECOND(v.size() == 3,
 		    "EulerTools::vectorToBearings: invalid size of v");
 	JFR_PRECOND(b.size() == 2,
 		    "EulerTools::vectorToBearings: invalid size of d");

 	b(0) = atan2(v(1),v(0));
 	b(1) = -atan2(v(2),sqrt(v(0)*v(0) + v(1)*v(1)));
      };
      
      template<class VecVector>
      void vectorToBearingsJac(VecVector const& v, jblas::mat& J)
      {
	JFR_PRECOND(v.size() == 3,
		    "EulerTools::vectorToDirectionJac: invalid size of v");
	JFR_PRECOND(J.size1()==2 && J.size2()==3,
		    "EulerTools::vectorToDirectionJac: size of J does not match");
	double x = v(0);
	double y = v(1);
	double z = v(2);

	/* begin copy/paste from maple */  
	double t1 = x * x;
	double t2 = 0.1e1 / t1;
	double t4 = y * y;
	double t7 = 0.1e1 / (0.1e1 + t4 * t2);
	double t11 = t1 + t4;
	double t12 = sqrt(t11);
	double t15 = z / t12 / t11;
	double t16 = z * z;
	double t20 = 0.1e1 / (0.1e1 + t16 / t11);
	J(0,0) = -y * t2 * t7;
	J(0,1) = 0.1e1 / x * t7;
	J(0,2) = 0.0e0;
	J(1,0) = t15 * x * t20;
	J(1,1) = t15 * y * t20;
	J(1,2) = -0.1e1 / t12 * t20;
	/* end copy/paste from maple */
      };
      
      /// Euler frames composition
      template<class VecFrame1, class VecFrame2, class VecFrameRes>
      inline void composeFrame(const VecFrame1& frame1_,
			       const VecFrame2& frame2_, 
			       VecFrameRes& frameRes) 
      {
        JFR_PRECOND(frame1_.size() == 6, "EulerTools::composeFrame: 3D robot pose is [x,y,z,roll,pitch,yaw]");
        JFR_PRECOND(frame2_.size() == 6, "EulerTools::composeFrame: 3D robot pose is [x,y,z,roll,pitch,yaw]");
        JFR_PRECOND(frameRes.size() == 6, "EulerTools::composeFrame: 3D robot pose is [x,y,z,roll,pitch,yaw]");
	details::composeFrame(frame1_(0), frame1_(1), frame1_(2), frame1_(3), frame1_(4), frame1_(5), 
			      frame2_(0), frame2_(1), frame2_(2), frame2_(3), frame2_(4), frame2_(5), 
			      frameRes(0), frameRes(1), frameRes(2), frameRes(3), frameRes(4), frameRes(5));
      };

      /// Euler frames composition jacobian
      template<class VecFrame1, class VecFrame2>
      inline void composeFrameJac(const VecFrame1& frame1_, const VecFrame2& frame2_, 
				  jblas::mat& Jframe2, jblas::mat& Jframe1) 
      {
        JFR_PRECOND(frame1_.size() == 6, "EulerTools::composeFrame: 3D robot pose is [x,y,z,roll,pitch,yaw]");
        JFR_PRECOND(frame2_.size() == 6, "EulerTools::composeFrame: 3D robot pose is [x,y,z,roll,pitch,yaw]");
        JFR_TRACE_BEGIN;
	details::composeFrameJac(frame1_(0), frame1_(1), frame1_(2), frame1_(3), frame1_(4), frame1_(5), 
				 frame2_(0), frame2_(1), frame2_(2), frame2_(3), frame2_(4), frame2_(5), 
				 Jframe2, Jframe1);
        JFR_TRACE_END("EulerTools::composeFrameJac()");
      };

      /// Euler frame inversion
      template<class VecFrame, class VecFrameInv>
      inline void invFrame(VecFrame const& frame, VecFrameInv& frameInv)
      {
        JFR_PRECOND(frame.size() == 6, "EulerTools::invFrameJac: 3D robot pose is [x,y,z,roll,pitch,yaw]");
        JFR_PRECOND(frameInv.size() == 6, "EulerTools::invFrameJac: 3D robot pose is [x,y,z,roll,pitch,yaw]");
	details::invFrame(frame(0), frame(1), frame(2), frame(3), frame(4), frame(5),
			  frameInv(0), frameInv(1), frameInv(2), frameInv(3), frameInv(4), frameInv(5));
      }

      /// 3D point to an other frame 
      template<class VecFrame, class Vec, class VecRes>
      inline void toFrame(const VecFrame& frame_, 
			  const Vec& v_, H h, 
			  VecRes& vRes) 
      {
        JFR_PRECOND(frame_.size() == 6, "EulerTools::toFrame: 3D robot pose is [x,y,z,roll,pitch,yaw]");
        JFR_PRECOND(v_.size() == 3, "EulerTools::toFrame: v_ must be a 3D vector");
        JFR_PRECOND(vRes.size() == 3, "EulerTools::toFrame: vres must be a 3D vector");
	details::toFrame(frame_(0), frame_(1), frame_(2), frame_(3), frame_(4), frame_(5),
			 v_(0), v_(1), v_(2), h,
			 vRes(0), vRes(1), vRes(2));
      };

      /// 3D point to an other frame jacobian
      template<class VecFrame, class Vec, class M_Jframe, class M_Jv>
      inline void toFrameJac(const VecFrame& frame_, 
			     const Vec& v_, H h,
			     M_Jframe& Jframe, M_Jv& Jv) 
      {
        JFR_PRECOND(frame_.size() == 6, "EulerTools::toFrameJac: 3D robot pose is [x,y,z,roll,pitch,yaw]");
        JFR_PRECOND(v_.size() == 3, "EulerTools::toFrameJac: v_ must be a 3D vector");
        JFR_TRACE_BEGIN;
	details::toFrameJac(frame_(0), frame_(1), frame_(2), frame_(3), frame_(4), frame_(5),
			    v_(0), v_(1), v_(2), h,
			    Jframe, Jv);
        JFR_TRACE_END("EulerTools::toFrameJac()");
      }

      /// 3D point from an other frame 
      template<class VecFrame, class Vec, class VecRes>
      inline void fromFrame(const VecFrame& frame_, 
			    const Vec& v_, H h,
			    VecRes& vRes) {
        JFR_PRECOND(frame_.size() == 6, "EulerTools::fromFrame: 3D robot pose is [x,y,z,roll,pitch,yaw]");
        JFR_PRECOND(v_.size() == 3, "EulerTools::fromFrame: v_ must be a 3D vector");
        JFR_PRECOND(vRes.size() == 3, "EulerTools::fromFrame: vres must be a 3D vector");
	details::fromFrame(frame_(0), frame_(1), frame_(2), frame_(3), frame_(4), frame_(5),
			   v_(0), v_(1), v_(2), h,
			   vRes(0), vRes(1), vRes(2));
      };

      /// 3D point from an other frame jacobian
      template<class VecFrame, class Vec, class M_Jframe, class M_Jv>
      inline void fromFrameJac(const VecFrame& frame_, 
			       const Vec& v_, H h,
			       M_Jframe& Jframe, M_Jv& Jv) {
        JFR_PRECOND(frame_.size() == 6, "EulerTools::fromFrameJac: 3D robot pose is [x,y,z,roll,pitch,yaw]");
        JFR_PRECOND(v_.size() == 3, "EulerTools::fromFrameJac: v_ must be a 3D vector");
        JFR_TRACE_BEGIN;
	details::fromFrameJac(frame_(0), frame_(1), frame_(2), frame_(3), frame_(4), frame_(5),
			      v_(0), v_(1), v_(2), h,
			      Jframe, Jv);
        JFR_TRACE_END("EulerTools::fromFrameJac()");
      };

      /// 3D line to an other frame 
      template<class VecFrame, class Vec, class VecRes>
      inline void lineToFrame(const VecFrame& frame_, const Vec& l_, VecRes& lRes) {
        JFR_PRECOND(frame_.size() == 6, "EulerTools::linetToFrame: 3D robot pose is [x,y,z,roll,pitch,yaw]");
        JFR_PRECOND(l_.size() == 6, "EulerTools::linetToFrame: l_ must be a 6-vector");
        JFR_PRECOND(lRes.size() == 6, "EulerTools::linetToFrame: lres must be a 6-vector");
	details::lineToFrame(frame_(0), frame_(1), frame_(2), frame_(3), frame_(4), frame_(5),
			     l_(0), l_(1), l_(2), l_(3), l_(4), l_(5),
			     lRes(0), lRes(1), lRes(2), lRes(3), lRes(4), lRes(5));
      };

      /// 3D line to an other frame jacobian
      template<class VecFrame, class Vec>
      inline void lineToFrameJac(const VecFrame& frame_, const Vec& l_,
				 jblas::mat& Jframe, jblas::mat& Jl) {
	JFR_PRECOND(frame_.size() == 6, "EulerTools::linetToFrameJac: 3D robot pose is [x,y,z,roll,pitch,yaw]");
        JFR_PRECOND(l_.size() == 6, "EulerTools::linetToFrameJac: l_ must be a 6-vector");
        JFR_TRACE_BEGIN;
	details::lineToFrameJac(frame_(0), frame_(1), frame_(2), frame_(3), frame_(4), frame_(5),
				l_(0), l_(1), l_(2), l_(3), l_(4), l_(5),
				Jframe, Jl);
        JFR_TRACE_END("EulerTools::lineToFrameJac()");
      }

      /// 3D line from an other frame 
      template<class VecFrame, class Vec, class VecRes>
      inline void lineFromFrame(const VecFrame& frame_, const Vec& l_, VecRes& lRes) {
	JFR_PRECOND(frame_.size() == 6, "EulerTools::lineFromFrame: 3D robot pose is [x,y,z,roll,pitch,yaw]");
        JFR_PRECOND(l_.size() == 6, "EulerTools::lineFromFrame: l_ must be a 6-vector");
        JFR_PRECOND(lRes.size() == 6, "EulerTools::lineFromFrame: lres must be a 6-vector");
	details::lineFromFrame(frame_(0), frame_(1), frame_(2), frame_(3), frame_(4), frame_(5),
			       l_(0), l_(1), l_(2), l_(3), l_(4), l_(5),
			       lRes(0), lRes(1), lRes(2), lRes(3), lRes(4), lRes(5));
      };

      /// 3D line from an other frame jacobian
      template<class VecFrame, class Vec>
      inline void lineFromFrameJac(const VecFrame& frame_, const Vec& l_,
				   jblas::mat& Jframe, jblas::mat& Jl) {
	JFR_PRECOND(frame_.size() == 6, "EulerTools::lineFromFrameJac: 3D robot pose is [x,y,z,roll,pitch,yaw]");
        JFR_PRECOND(l_.size() == 6, "EulerTools::lineFromFrameJac: l_ must be a 6-vector");
        JFR_TRACE_BEGIN;
	details::lineFromFrameJac(frame_(0), frame_(1), frame_(2), frame_(3), frame_(4), frame_(5),
				  l_(0), l_(1), l_(2), l_(3), l_(4), l_(5),
				  Jframe, Jl);
        JFR_TRACE_END("EulerTools::lineFromFrameJac()");
      };


    /// Plane to an other frame 
      template<class VecFrame, class Vec, class VecRes>
      inline void planeToFrame(const VecFrame& frame_, const Vec& p_, VecRes& pRes) {
        JFR_PRECOND(frame_.size() == 6, "EulerTools::planeToFrame: 3D robot pose is [x,y,z,roll,pitch,yaw]");
        JFR_PRECOND(p_.size() == 6, "EulerTools::planeToFrame: p_ must be a 6-vector");
        JFR_PRECOND(pRes.size() == 6, "EulerTools::planeToFrame: lres must be a 6-vector");
        details::planeToFrame(frame_(0), frame_(1), frame_(2), frame_(3), frame_(4), frame_(5),
                 p_(0), p_(1), p_(2), p_(3), p_(4), p_(5),
                 pRes(0), pRes(1), pRes(2), pRes(3), pRes(4), pRes(5));
      };

      /// Plane to an other frame jacobian
      template<class VecFrame, class Vec>
      inline void planeToFrameJac(const VecFrame& frame_, const Vec& p_,
                 jblas::mat& Jframe, jblas::mat& Jp) {
        JFR_PRECOND(frame_.size() == 6, "EulerTools::planeToFrameJac: 3D robot pose is [x,y,z,roll,pitch,yaw]");
        JFR_PRECOND(p_.size() == 6, "EulerTools::planeToFrameJac: p_ must be a 6-vector");
        JFR_TRACE_BEGIN;
        details::planeToFrameJac(frame_(0), frame_(1), frame_(2), frame_(3), frame_(4), frame_(5),
                p_(0), p_(1), p_(2), p_(3), p_(4), p_(5),
                Jframe, Jp);
        JFR_TRACE_END("EulerTools::planeToFrameJac()");
      }

      /// Plane from an other frame 
      template<class VecFrame, class Vec, class VecRes>
      inline void planeFromFrame(const VecFrame& frame_, const Vec& p_, VecRes& pRes) {
        JFR_PRECOND(frame_.size() == 6, "EulerTools::planeFromFrame: 3D robot pose is [x,y,z,roll,pitch,yaw]");
        JFR_PRECOND(p_.size() == 6, "EulerTools::planeFromFrame: p_ must be a 6-vector");
        JFR_PRECOND(pRes.size() == 6, "EulerTools::planeFromFrame: pres must be a 6-vector");
        details::planeFromFrame(frame_(0), frame_(1), frame_(2), frame_(3), frame_(4), frame_(5),
                   p_(0), p_(1), p_(2), p_(3), p_(4), p_(5),
                   pRes(0), pRes(1), pRes(2), pRes(3), pRes(4), pRes(5));
      };

      /// Plane from an other frame jacobian
      template<class VecFrame, class Vec>
      inline void planeFromFrameJac(const VecFrame& frame_, const Vec& p_,
                   jblas::mat& Jframe, jblas::mat& Jp) {
        JFR_PRECOND(frame_.size() == 6, "EulerTools::planeFromFrameJac: 3D robot pose is [x,y,z,roll,pitch,yaw]");
        JFR_PRECOND(p_.size() == 6, "EulerTools::planeFromFrameJac: p_ must be a 6-vector");
        JFR_TRACE_BEGIN;
        details::planeFromFrameJac(frame_(0), frame_(1), frame_(2), frame_(3), frame_(4), frame_(5),
                  p_(0), p_(1), p_(2), p_(3), p_(4), p_(5),
                  Jframe, Jp);
        JFR_TRACE_END("EulerTools::planeFromFrameJac()");
      };

      /*
       * for unit vector
       */

//       template<class VecFrame, class Vec, class VecRes>
//       inline void vectorToFrame(const VecFrame& frame_, const Vec& v_, VecRes& vRes) {
//         JFR_PRECOND(frame_.size() == 6, "EulerTools::vectorToFrame: 3D robot pose is [x,y,z,roll,pitch,yaw]");
//         JFR_PRECOND(v_.size() == 3, "EulerTools::vectorToFrame: v_ must be a 3D vector");
//         JFR_PRECOND(vRes.size() == 3, "EulerTools::vectorToFrame: vres must be a 3D vector");
// 	details::toFrame(0.0 ,0.0 ,0.0 , frame_(3), frame_(4), frame_(5),
// 			 v_(0), v_(1), v_(2),
// 			 vRes(0), vRes(1), vRes(2));
//       };

//       template<class VecFrame, class Vec>
//       inline void vectorToFrameJac(const VecFrame& frame_, const Vec& v_,
// 				   jblas::mat& Jframe, jblas::mat& Jv) {
//         JFR_PRECOND(frame_.size() == 6, "EulerTools::vectorToFrameJac: 3D robot pose is [x,y,z,roll,pitch,yaw]");
//         JFR_PRECOND(v_.size() == 3, "EulerTools::vectorToFrameJac: v_ must be a 3D vector");
//         JFR_TRACE_BEGIN;
// 	details::toFrameJac(0.0, 0.0, 0.0, frame_(3), frame_(4), frame_(5),
// 			    v_(0), v_(1), v_(2),
// 			    Jframe, Jv);
//         JFR_TRACE_END("EulerTools::vectorToFrameJac()");
//       }

//       template<class VecFrame, class Vec, class VecRes>
//       inline void vectorFromFrame(const VecFrame& frame_, const Vec& v_, VecRes& vRes) {
//         JFR_PRECOND(frame_.size() == 6, "EulerTools::vectorFromFrame: 3D robot pose is [x,y,z,roll,pitch,yaw]");
//         JFR_PRECOND(v_.size() == 3, "EulerTools::vectorFromFrame: v_ must be a 3D vector");
//         JFR_PRECOND(vRes.size() == 3, "EulerTools::vectorFromFrame: vres must be a 3D vector");
// 	details::fromFrame(0.0, 0.0, 0.0, frame_(3), frame_(4), frame_(5),
// 			   v_(0), v_(1), v_(2),
// 			   vRes(0), vRes(1), vRes(2));
//       };
      
//       template<class VecFrame, class Vec>
//       inline void vectorFromFrameJac(const VecFrame& frame_, const Vec& v_,
// 				     jblas::mat& Jframe, jblas::mat& Jv) {
//         JFR_PRECOND(frame_.size() == 6, "EulerTools::vectorFromFrameJac: 3D robot pose is [x,y,z,roll,pitch,yaw]");
//         JFR_PRECOND(v_.size() == 3, "EulerTools::vectorFromFrameJac: v_ must be a 3D vector");
//         JFR_TRACE_BEGIN;
// 	details::fromFrameJac(0.0, 0.0, 0.0, frame_(3), frame_(4), frame_(5),
// 			      v_(0), v_(1), v_(2),
// 			      Jframe, Jv);
//         JFR_TRACE_END("EulerTools::vectorFromFrameJac()");
//       }

      /*
       * 3d odometry computation
       */

      template<class VecOdo, class VecXRes>
      void odo3d(VecOdo const& u, double dt, VecXRes& xRes) 
      {
	JFR_PRECOND(u.size() == 2,
		    "EulerTools::odo3d: u command is [v,w]");
	JFR_PRECOND(xRes.size() == 6,
		    "EulerTools::odo3d: 3D robot pose is [x,y,z,roll,pitch,yaw]");		

	details::odo3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		       u(0), u(1),
		       dt,
		       xRes(0), xRes(1), xRes(2), xRes(3), xRes(4), xRes(5));
      }

      template<class VecOdo>
      void odo3dJac(VecOdo const& u, double dt, 
		    jblas::mat& Ju) 
      {
	JFR_PRECOND(u.size() == 2,
		    "EulerTools::odo3dJac: u command is [v,w]");

	jblas::mat Jx(6,2);

	details::odo3dJac(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			  u(0), u(1), dt,
			  Jx, Ju);
      }
		
      /*
       * 3d odometry composition
       */

      template<class VecX, class VecOdo, class VecXRes>
      void odo3d(VecX const& x, VecOdo const& u, double dt, VecXRes& xRes) 
      {
	JFR_PRECOND(x.size() == 6,
		    "EulerTools::odo3d: 3D robot pose is [x,y,z,roll,pitch,yaw]");
	JFR_PRECOND(u.size() == 2,
		    "EulerTools::odo3d: u command is [v,w]");
	JFR_PRECOND(xRes.size() == 6,
		    "EulerTools::odo3d: 3D robot pose is [x,y,z,roll,pitch,yaw]");		

	details::odo3d(x(0), x(1), x(2), x(3), x(4), x(5),
		       u(0), u(1),
		       dt,
		       xRes(0), xRes(1), xRes(2), xRes(3), xRes(4), xRes(5));
      }

      template<class VecX, class VecOdo>
      void odo3dJac(VecX const& x, VecOdo const& u, double dt, 
		    jblas::mat& Jx, jblas::mat& Ju) 
      {
	JFR_PRECOND(x.size() == 6,
		    "EulerTools::odo3dJac: 3D robot pose is [x,y,z,roll,pitch,yaw]");
	JFR_PRECOND(u.size() == 2,
		    "EulerTools::odo3dJac: u command is [v,w]");
		
	details::odo3dJac(x(0), x(1), x(2), x(3), x(4), x(5),
			  u(0), u(1), dt,
			  Jx, Ju);
      }

      /*
       * 3d odometry in reference frame
       */

      template<class VecX, class VecOdo, class VecXRes>
      void odo3dInRefFrame(double yaw_robotToRef, 
			   VecX const& x, VecOdo const& u, double dt, VecXRes& xRes) 
      {
// 	JFR_PRECOND(robotToRef.size() == 6,
// 		    "EulerTools::odo3dInRefFrame: robotToRef transformation [x,y,z,roll,pitch,yaw]");
	JFR_PRECOND(x.size() == 6,
		    "EulerTools::odo3dInRefFrame: 3D robot pose is [x,y,z,roll,pitch,yaw]");
	JFR_PRECOND(u.size() == 2,
		    "EulerTools::odo3dInRefFrame: u command is [v,w]");
	JFR_PRECOND(xRes.size() == 6,
		    "EulerTools::odo3dInRefFrame: 3D robot pose is [x,y,z,roll,pitch,yaw]");		

	details::odo3dInRefFrame(yaw_robotToRef,
				 x(0), x(1), x(2), x(3), x(4), x(5),
				 u(0), u(1),
				 dt,
				 xRes(0), xRes(1), xRes(2), xRes(3), xRes(4), xRes(5));
      }

      template<class VecX, class VecOdo>
      void odo3dInRefFrameJac(double yaw_robotToRef, 
			      VecX const& x, VecOdo const& u, double dt, 
			      jblas::mat& Jx, jblas::mat& Ju) 
      {
// 	JFR_PRECOND(robotToRef.size() == 6,
// 		    "EulerTools::odo3dInRefFrameJac: robotToRef transformation [x,y,z,roll,pitch,yaw]");
	JFR_PRECOND(x.size() == 6,
		    "EulerTools::odo3dInRefFrameJac: 3D robot pose is [x,y,z,roll,pitch,yaw]");
	JFR_PRECOND(u.size() == 2,
		    "EulerTools::odo3dInRefFrameJac: u command is [v,w]");
		
	details::odo3dInRefFrameJac(yaw_robotToRef,
				    x(0), x(1), x(2), x(3), x(4), x(5),
				    u(0), u(1), dt,
				    Jx, Ju);
      }

      namespace details {

	template<class M_Jframe, class M_Jv>
	void fromFrameJac(double x, double y, double z,
			  double yaw, double pitch, double roll, 
			  double v_x, double v_y, double v_z, H h,
			  M_Jframe& Jframe, M_Jv& Jv)
	{
	  JFR_PRECOND(Jframe.size1() == 3 && Jframe.size2() == 6,
		      "fromFrameJac: size of Jframe does not match");
	  JFR_PRECOND(Jv.size1() == 3 && Jv.size2() == 3,
		      "fromFrameJac: size of Jv does not match");

	  switch (h) {
	  case h_1:
	    {
	      double t1 = sin(yaw);
	      double t2 = cos(pitch);
	      double t3 = t1 * t2;
	      double t6 = sin(pitch);
	      double t7 = t1 * t6;
	      double t8 = sin(roll);
	      double t10 = cos(yaw);
	      double t11 = cos(roll);
	      double t14 = -t7 * t8 - 0.10e1 * t10 * t11;
	      double t19 = -t7 * t11 + 0.10e1 * t10 * t8;
	      double t22 = t10 * t6;
	      double t25 = t10 * t2;
	      double t26 = t8 * v_y;
	      double t28 = t11 * v_z;
	      double t34 = t22 * t11 + 0.10e1 * t1 * t8;
	      double t39 = -t22 * t8 + 0.10e1 * t1 * t11;
	      double t66 = t2 * t11;
	      double t69 = t2 * t8;

	      Jframe(0,0) = 0.10e1;
	      Jframe(0,1) = 0.0e0;
	      Jframe(0,2) = 0.0e0;
	      Jframe(0,3) = -0.10e1 * t3 * v_x + t14 * v_y + t19 * v_z;
	      Jframe(0,4) = -0.10e1 * t22 * v_x + t25 * t26 + t25 * t28;
	      Jframe(0,5) = t34 * v_y + t39 * v_z;
	      Jframe(1,0) = 0.0e0;
	      Jframe(1,1) = 0.10e1;
	      Jframe(1,2) = 0.0e0;
	      Jframe(1,3) = 0.10e1 * t25 * v_x - t39 * v_y + t34 * v_z;
	      Jframe(1,4) = -0.10e1 * t7 * v_x + t3 * t26 + t3 * t28;
	      Jframe(1,5) = -t19 * v_y + t14 * v_z;
	      Jframe(2,0) = 0.0e0;
	      Jframe(2,1) = 0.0e0;
	      Jframe(2,2) = 0.10e1;
	      Jframe(2,3) = 0.0e0;
	      Jframe(2,4) = -0.100e1 * t2 * v_x - 0.10e1 * t6 * t8 * v_y - 0.10e1 * t6 * t11 * v_z;
	      Jframe(2,5) = 0.10e1 * t66 * v_y - 0.10e1 * t69 * v_z;

	      Jv(0,0) = 0.10e1 * t25;
	      Jv(0,1) = -t39;
	      Jv(0,2) = t34;
	      Jv(1,0) = 0.10e1 * t3;
	      Jv(1,1) = -t14;
	      Jv(1,2) = -t19;
	      Jv(2,0) = -0.100e1 * t6;
	      Jv(2,1) = 0.10e1 * t69;
	      Jv(2,2) = 0.10e1 * t66;

	      break;
	    }
	  case h_0:
	    {
	      double t1 = sin(yaw);
	      double t2 = cos(pitch);
	      double t3 = t1 * t2;
	      double t6 = sin(pitch);
	      double t7 = t1 * t6;
	      double t8 = sin(roll);
	      double t10 = cos(yaw);
	      double t11 = cos(roll);
	      double t14 = -t7 * t8 - 0.10e1 * t10 * t11;
	      double t19 = -t7 * t11 + 0.10e1 * t10 * t8;
	      double t22 = t10 * t6;
	      double t25 = t10 * t2;
	      double t26 = t8 * v_y;
	      double t28 = t11 * v_z;
	      double t34 = t22 * t11 + 0.10e1 * t1 * t8;
	      double t39 = -t22 * t8 + 0.10e1 * t1 * t11;
	      double t66 = t2 * t11;
	      double t69 = t2 * t8;

	      Jframe(0,0) = 0.0e0;
	      Jframe(0,1) = 0.0e0;
	      Jframe(0,2) = 0.0e0;
	      Jframe(0,3) = -0.10e1 * t3 * v_x + t14 * v_y + t19 * v_z;
	      Jframe(0,4) = -0.10e1 * t22 * v_x + t25 * t26 + t25 * t28;
	      Jframe(0,5) = t34 * v_y + t39 * v_z;
	      Jframe(1,0) = 0.0e0;
	      Jframe(1,1) = 0.0e0;
	      Jframe(1,2) = 0.0e0;
	      Jframe(1,3) = 0.10e1 * t25 * v_x - t39 * v_y + t34 * v_z;
	      Jframe(1,4) = -0.10e1 * t7 * v_x + t3 * t26 + t3 * t28;
	      Jframe(1,5) = -t19 * v_y + t14 * v_z;
	      Jframe(2,0) = 0.0e0;
	      Jframe(2,1) = 0.0e0;
	      Jframe(2,2) = 0.0e0;
	      Jframe(2,3) = 0.0e0;
	      Jframe(2,4) = -0.100e1 * t2 * v_x - 0.10e1 * t6 * t8 * v_y - 0.10e1 * t6 * t11 * v_z;
	      Jframe(2,5) = 0.10e1 * t66 * v_y - 0.10e1 * t69 * v_z;

	      Jv(0,0) = 0.10e1 * t25;
	      Jv(0,1) = -t39;
	      Jv(0,2) = t34;
	      Jv(1,0) = 0.10e1 * t3;
	      Jv(1,1) = -t14;
	      Jv(1,2) = -t19;
	      Jv(2,0) = -0.100e1 * t6;
	      Jv(2,1) = 0.10e1 * t69;
	      Jv(2,2) = 0.10e1 * t66;

	      break;
	    }
	  }
	}

	template<class M_Jframe, class M_Jv>
	void toFrameJac(double x, double y, double z,
			double yaw, double pitch, double roll,
			double v_x, double v_y, double v_z, H h,
			M_Jframe& Jframe, M_Jv& Jv)
	{
	  JFR_PRECOND(Jframe.size1() == 3 && Jframe.size2() == 6,
		      "toFrameJac: size of Jframe does not match");
	  JFR_PRECOND(Jv.size1() == 3 && Jv.size2() == 3,
		      "toFrameJac: size of Jv does not match");

	  switch (h) {
	  case h_1:
	    {
	      double t1 = cos(yaw);
	      double t2 = cos(pitch);
	      double t3 = t1 * t2;
	      double t5 = sin(yaw);
	      double t6 = t5 * t2;
	      double t8 = sin(pitch);
	      double t19 = t1 * t8;
	      double t22 = t5 * t8;
	      double t37 = sin(roll);
	      double t38 = t19 * t37;
	      double t40 = cos(roll);
	      double t41 = t5 * t40;
	      double t44 = t22 * t37;
	      double t46 = t1 * t40;
	      double t49 = t2 * t37;
	      double t52 = -t44 - 0.10e1 * t46;
	      double t55 = t38 - 0.10e1 * t41;
	      double t66 = t8 * t37;
	      double t78 = t19 * t40;
	      double t79 = t5 * t37;
	      double t81 = t78 + 0.10e1 * t79;
	      double t83 = t22 * t40;
	      double t84 = t1 * t37;
	      double t86 = t83 - 0.10e1 * t84;
	      double t88 = t2 * t40;
	      double t117 = t8 * t40;
	      Jframe(0,0) = -0.100e1 * t3;
	      Jframe(0,1) = -0.100e1 * t6;
	      Jframe(0,2) = 0.1000e1 * t8;
	      Jframe(0,3) = -0.10e1 * t6 * v_x + 0.10e1 * t3 * v_y + 0.100e1 * t6 * x - 0.100e1 * t3 * y;
	      Jframe(0,4) = -0.10e1 * t19 * v_x - 0.10e1 * t22 * v_y - 0.100e1 * t2 * v_z + 0.100e1 * t19 * x + 0.100e1 * t22 * y + 0.1000e1 * t2 * z;
	      Jframe(0,5) = 0.0e0;
	      Jframe(1,0) = -0.10e1 * t38 + 0.100e1 * t41;
	      Jframe(1,1) = -0.10e1 * t44 - 0.100e1 * t46;
	      Jframe(1,2) = -0.100e1 * t49;
	      Jframe(1,3) = t52 * v_x + t55 * v_y - 0.10e1 * t52 * x - 0.10e1 * t55 * y;
	      Jframe(1,4) = t3 * t37 * v_x + t6 * t37 * v_y - 0.10e1 * t66 * v_z - 0.10e1 * t3 * t37 * x - 0.10e1 * t6 * t37 * y + 0.100e1 * t66 * z;
	      Jframe(1,5) = t81 * v_x + t86 * v_y + 0.10e1 * t88 * v_z - 0.10e1 * t81 * x - 0.10e1 * t86 * y - 0.100e1 * t88 * z;
	      Jframe(2,0) = -0.10e1 * t78 - 0.100e1 * t79;
	      Jframe(2,1) = -0.10e1 * t83 + 0.100e1 * t84;
	      Jframe(2,2) = -0.100e1 * t88;
	      Jframe(2,3) = -t86 * v_x + t81 * v_y + 0.10e1 * t86 * x - 0.10e1 * t81 * y;
	      Jframe(2,4) = t3 * t40 * v_x + t6 * t40 * v_y - 0.10e1 * t117 * v_z - 0.10e1 * t3 * t40 * x - 0.10e1 * t6 * t40 * y + 0.100e1 * t117 * z;
	      Jframe(2,5) = -t55 * v_x + t52 * v_y - 0.10e1 * t49 * v_z + 0.10e1 * t55 * x - 0.10e1 * t52 * y + 0.100e1 * t49 * z;

	      Jv(0,0) = 0.10e1 * t3;
	      Jv(0,1) = 0.10e1 * t6;
	      Jv(0,2) = -0.100e1 * t8;
	      Jv(1,0) = t55;
	      Jv(1,1) = -t52;
	      Jv(1,2) = 0.10e1 * t49;
	      Jv(2,0) = t81;
	      Jv(2,1) = t86;
	      Jv(2,2) = 0.10e1 * t88;

	      break;
	    }
	  case h_0:
	    {
	      double t1 = sin(yaw);
	      double t2 = cos(pitch);
	      double t3 = t1 * t2;
	      double t6 = cos(yaw);
	      double t7 = t6 * t2;
	      double t11 = sin(pitch);
	      double t12 = t6 * t11;
	      double t15 = t1 * t11;
	      double t24 = sin(roll);
	      double t26 = cos(roll);
	      double t29 = -t15 * t24 - 0.10e1 * t6 * t26;
	      double t34 = t12 * t24 - 0.10e1 * t1 * t26;
	      double t48 = t12 * t26 + 0.10e1 * t1 * t24;
	      double t53 = t15 * t26 - 0.10e1 * t6 * t24;
	      double t55 = t2 * t26;
	      double t59 = t2 * t24;

	      Jframe(0,0) = 0.0e0;
	      Jframe(0,1) = 0.0e0;
	      Jframe(0,2) = 0.0e0;
	      Jframe(0,3) = -0.10e1 * t3 * v_x + 0.10e1 * t7 * v_y;
	      Jframe(0,4) = -0.10e1 * t12 * v_x - 0.10e1 * t15 * v_y - 0.100e1 * t2 * v_z;
	      Jframe(0,5) = 0.0e0;
	      Jframe(1,0) = 0.0e0;
	      Jframe(1,1) = 0.0e0;
	      Jframe(1,2) = 0.0e0;
	      Jframe(1,3) = t29 * v_x + t34 * v_y;
	      Jframe(1,4) = t7 * t24 * v_x + t3 * t24 * v_y - 0.10e1 * t11 * t24 * v_z;
	      Jframe(1,5) = t48 * v_x + t53 * v_y + 0.10e1 * t55 * v_z;
	      Jframe(2,0) = 0.0e0;
	      Jframe(2,1) = 0.0e0;
	      Jframe(2,2) = 0.0e0;
	      Jframe(2,3) = -t53 * v_x + t48 * v_y;
	      Jframe(2,4) = t7 * t26 * v_x + t3 * t26 * v_y - 0.10e1 * t11 * t26 * v_z;
	      Jframe(2,5) = -t34 * v_x + t29 * v_y - 0.10e1 * t59 * v_z;

	      Jv(0,0) = 0.10e1 * t7;
	      Jv(0,1) = 0.10e1 * t3;
	      Jv(0,2) = -0.100e1 * t11;
	      Jv(1,0) = t34;
	      Jv(1,1) = -t29;
	      Jv(1,2) = 0.10e1 * t59;
	      Jv(2,0) = t48;
	      Jv(2,1) = t53;
	      Jv(2,2) = 0.10e1 * t55;

	      break;
	    }
	  }
	}

      }

    } // namespace EulerTools

  } // namespace slam
} // namespace jafar

#endif // SLAM_EULER_TOOLS_HPP
