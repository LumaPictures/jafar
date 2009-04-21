/* $Id: lineTools.cpp 3079 2008-10-13 09:34:54Z jsola $ */

#include <cmath>

#include "kernel/jafarDebug.hpp"
#include "slam/lineTools.hpp"
#include "jmath/ublasExtra.hpp"
#include "jmath/jblas.hpp"
#include "slam/eulerTools.hpp"

using namespace jblas;
using namespace ublas;

// using namespace jafar::slam;
//using namespace jafar::slam::EulerTools;

namespace jafar { namespace slam { namespace lineTools {
	
	// Build Plucker matrix from calibration parameters
	template<class Mat>
	void pluckerIntrinsicMatrix(double u0, double v0, double au, double av, Mat& K) 
	{
		K.clear();
		K(0,0) = av;
		K(1,1) = au;
		K(2,0) = -av*u0;
		K(2,1) = -au*v0;
		K(2,2) = au*av;					
	}
	
	// Build Plucker matrix from calibration vector
	//void pluckerIntrinsicMatrix(vec4 const& k, mat& K) 
	template<class Vec, class Mat>
	void pluckerIntrinsicMatrix(Vec const& k, 
								Mat& K)
	{
		pluckerIntrinsicMatrix(k(0), k(1), k(2), k(3), K);		
	}
	
	
	
	
	// Build inverse Plucker camera matrix from calibration parameters
	void pluckerInverseIntrinsicMatrix(double u0, double v0, double au, double av, mat& iK)
	{
	  iK.clear();
	  iK(0,0)=au;
	  iK(1,1)=av;
	  iK(2,0)=u0;
	  iK(2,1)=v0;
	  iK(2,2)=1.0;
	}
	
	
	// Build inverse Plucker camera matrix from calibration vector
	void pluckerInverseIntrinsicMatrix(vec4 const& k, mat& iK)
	{
		pluckerInverseIntrinsicMatrix(k(0), k(1), k(2), k(3), iK);		
	}
	
	
	
	
	// Homogeneous 2D line to rho-theta representation
	vec2 homogeneousToRhoTheta(vec3 const& l_) 
	{
		vec3 l = l_;
		
		// orientation convention : l(2) must be negative so that rho is positive
		if (l(2)>0.0) 
			l *= -1.0;
		
		vec2 rt;
		rt(0) = -1.0*l(2)/sqrt(l(0)*l(0) + l(1)*l(1));
		rt(1) = atan2(l(1), l(0));
		return rt;
	}
	
	
	// homogeneous 2D line to rho-theta, with Jacobians
	void homogeneousToRhoTheta(vec3 const& l_,
				   vec2& rt,
				   mat& RT_l) 
	{
		vec3 l = l_;
		
		// orientation convention : l(2) must be negative so that rho is positive
		if (l_(2)>0.0) 
		  l *= -1.0;
		
		rt(0) = -1.0*l(2)/sqrt(l(0)*l(0) + l(1)*l(1));
		rt(1) = atan2(l(1), l(0));
		
		double a = l(0);
		double b = l(1);
		double c = l(2);
		
		double r2 = a*a+b*b; //dot([a,b],[a,b]);
		double r  = sqrt(r2);
		
		double r3 = r*r2;
		double cir3 = c/r3;
		
		RT_l(0,0) = a*cir3;
		RT_l(0,1) = b*cir3;
		RT_l(0,2) = -1.0/r;
		RT_l(1,0) = -b/r2;
		RT_l(1,1) = a/r2;
		RT_l(1,2) = 0.0;
		
		// Jacobian changes sign depending on sign of l_(2)
		if (l_(2)>0.0)
		  RT_l *= -1.0; 
	}	
	
	
	// Rho-theta to homogeneous 2D line transform
	vec3 rhoThetaToHomogeneous(vec2 const& rt_)
	{
		vec3 l;
		
		l(0) = cos(rt_(1));
		l(1) = sin(rt_(1));
		l(2) = -rt_(0);
		return l;
	}
	
	
	/// Rho-theta to homogeneous 2D line transform -- Jacobian
	void rhoThetaToHomogeneous(vec2 const& rt_, 
							   vec3& l_, mat& L_rt)
	{
		// line
		l_(0) = cos(rt_(1));
		l_(1) = sin(rt_(1));
		l_(2) = -rt_(0);
		
		// Jacobian
		L_rt.clear();
		
		L_rt(0,1) = -sin(rt_(1));
		L_rt(1,1) =  cos(rt_(1));
		L_rt(2,0) = -1;
	}
	
	
	
	
	// Project Plucker 3D line into homogeneous 2D line
	vec3 projectPlucker(mat33 const& K, vec6 const& L)
	{
		vec3 l = prod(K,subrange(L,0,3));
		return l;
	}
	
	
	// Project Plucker 3D line into homogeneous 2D line, with Jacobians
	void projectPlucker(mat33 const& K, vec6 const& L, 
						vec3& l, mat& L_l)
	{
		// homogeneous line in image plane
		l = prod(K,subrange(L,0,3));
		
		// Jacobians
		L_l.clear();
		subrange(L_l, 0,3, 0,3 ).assign(K);
	}
	
	
	
	
	
	// Retro project homogeneous 2D line to Plucker 3D line
	vec6 retroProjectPlucker(mat33 const& iK, vec3 const& l, vec3 const& v)
	{
		vec6 L;
		vec3 n = prod(iK,l);
		subrange(L,0,3).assign(n);
		subrange(L,3,6).assign(v);
		
		return L;
	}
	
	// Retro project homogeneous 2D line to Plucker 3D line, with Jacobians
	void retroProjectPlucker(mat33 const& iK, vec3 const& l, vec3 const& v, 
							 vec6& L, mat& L_l, mat& L_v)
	{
		// Plucker line
		L = retroProjectPlucker(iK, l, v);
		
		// Jacobian wrt observed 2D line
		L_l.clear();
		subrange(L_l,0,3,0,3).assign(iK);
		
		// Jacobian wrt non observable dir. vector
		L_v.clear();
		subrange(L_v,3,6,0,3).assign(identity_mat(3));
	}
	
	
	
	// Retro project homogeneous 2D line to representation plane normal vector
	vec3 retroProjectPluckerPlaneNormal(mat33 const& iK, vec3 const& l)
	{
		vec3 n = prod(iK,l);
		return n;
	}
	
	/// Retro project homogeneous 2D line to representation plane normal vector, with Jacobian
	void retroProjectPluckerPlaneNormal(mat33 const& iK, 
					    vec3  const& l, 
					    vec3&  n, 
					    mat33& Nl)
	{
		n  = prod(iK,l);
		Nl = iK;
	}
	
	
	
	// Retro project homogeneous 2D line to Plucker 3D line providing base vector
	vec6 retroProjectPluckerFromBase(mat33 const& iK, vec3 const& l, vec2 const& beta)
	{
		vec3 nn = retroProjectPluckerPlaneNormal(iK, l);
		vec3 n = normVec3(nn);
		vec3 v = planeBaseToDirVector(n, beta);
		vec6 L;
		subrange(L,0,3).assign(n);
		subrange(L,3,6).assign(v);
		return L;
	}
	
	// Retro project homogeneous 2D line to Plucker 3D line providing base vector, with Jacobians
	void retroProjectPluckerFromBase(mat33 const& iK, vec3 const& l, vec2 const& beta,
									 vec6& L, mat&  L_l, mat&  L_beta)
	{
		// non-normalized plane normal nn
		vec3 nn;
		mat33 NN_l;
		retroProjectPluckerPlaneNormal(iK, l, nn, NN_l);
		
		// normalised plane normal n
		vec3 n;
		mat33 N_nn;
		normVec3(nn, n, N_nn);
		mat N_l = prod(N_nn, NN_l); // Jacobian wrt homogeneous line l
		
		// line dir vector v
		vec3  v;
		mat33 V_n;
		mat   V_beta(3,2);
		planeBaseToDirVector(n, beta, v, V_n, V_beta);

		// Plucker line L
		subrange(L,0,3).assign(n);
		subrange(L,3,6).assign(v);
		
		// Jacobians wrt l
		subrange(L_l,0,3,0,3).assign(N_l);
		subrange(L_l,3,6,0,3).assign(prod(V_n,N_l));
		
		// Jacobian wrt beta
		L_beta.clear();
		subrange(L_beta,3,6,0,2).assign(V_beta);

	}
	
	
	
	// Plane sub-vector to plane base matrix
	mat planeVecToBaseMatrix(vec3 const& n)
	{
		double n01n = sqrt(n(0)*n(0)+n(1)*n(1));
		
		vec3 e0;
		vec3 e1; // Vectors of the plane base
		
		// vector parallel to image plane
		e0(0) =  n(1)/n01n;
		e0(1) = -n(0)/n01n;
		e0(2) = 0;
		
		// vector perpendicular to the image segment
		jmath::ublasExtra::crossProd(n, e0, e1);
		
		// plane base matrix
		mat E(3,2);
		column(E,0) = e0;
		column(E,1) = e1;
		
		return E;
	}
	
	
	
	// Plane sub-vector 'n' and plane base to direction vector
	vec3 planeBaseToDirVector(vec3 const& n_, vec2 const&  beta_)
	{
		mat E = planeVecToBaseMatrix(n_);			
		return prod(E, beta_);
	}
	
	// Plane sub-vector 'n' and plane base to direction vector, with Jacobians
	void planeBaseToDirVector(vec3 const& n, vec2 const&  beta, vec3& v, mat33& V_n, mat& V_beta)
	{
		V_beta = planeVecToBaseMatrix(n); // base matrix 
		
		v = prod(V_beta, beta);
		
		double n01n2 = n(0)*n(0)+n(1)*n(1);
		double n01n  = sqrt(n01n2);
		double n01n3 = n01n*n01n2;
		
		V_n(0,0) = -n(1)*(beta(0)*n(0)-n(2)*beta(1)*n(1))/n01n3;
		V_n(0,1) =  n(0)*(beta(0)*n(0)-n(2)*beta(1)*n(1))/n01n3;
		V_n(0,2) =  n(0)*beta(1)/n01n;
		V_n(1,0) = -n(1)*(beta(0)*n(1)+n(0)*n(2)*beta(1))/n01n3;
		V_n(1,1) =  n(0)*(beta(0)*n(1)+n(0)*n(2)*beta(1))/n01n3;
		V_n(1,2) =  n(1)*beta(1)/n01n;
		V_n(2,0) = -n(0)*beta(1)/n01n;
		V_n(2,1) = -n(1)*beta(1)/n01n;
		V_n(2,2) =  0.0;
		
	}
	
	// Plane sub-vector 'n' and plane base to direction vector, with Jacobians
	void planeBaseToDirVector(vec_range const& n, vec2 const&  beta, vec_range& v, mat33& V_n, mat& V_beta)
	{
		V_beta = planeVecToBaseMatrix(n); // base matrix 
		
		v = prod(V_beta, beta);
		
		double n01n2 = n(0)*n(0)+n(1)*n(1);
		double n01n  = sqrt(n01n2);
		double n01n3 = n01n*n01n2;
		
		V_n(0,0) = -n(1)*(beta(0)*n(0)-n(2)*beta(1)*n(1))/n01n3;
		V_n(0,1) =  n(0)*(beta(0)*n(0)-n(2)*beta(1)*n(1))/n01n3;
		V_n(0,2) =  n(0)*beta(1)/n01n;
		V_n(1,0) = -n(1)*(beta(0)*n(1)+n(0)*n(2)*beta(1))/n01n3;
		V_n(1,1) =  n(0)*(beta(0)*n(1)+n(0)*n(2)*beta(1))/n01n3;
		V_n(1,2) =  n(1)*beta(1)/n01n;
		V_n(2,0) = -n(0)*beta(1)/n01n;
		V_n(2,1) = -n(1)*beta(1)/n01n;
		V_n(2,2) =  0.0;
	}
	
	
	
	// Normalize 3-vector
	vec3 normVec3(vec3 const& v)
	{
		return v/norm_2(v);	
	}
	
	// Normalize 3-vector, with Jacobians
	void normVec3(vec3 const& v, vec3& vn, mat33& VN_v)
	{
		double in = 1.0/norm_2(v);
		vn = v*in;
		double in3 = pow(in,3);
		
		VN_v(0,0) = in - v(0)*v(0)*in3;
		VN_v(0,1) =    - v(0)*v(1)*in3;
		VN_v(0,2) =    - v(0)*v(2)*in3;
		VN_v(1,0) =    - v(1)*v(0)*in3;
		VN_v(1,1) = in - v(1)*v(1)*in3;
		VN_v(1,2) =    - v(1)*v(2)*in3;
		VN_v(2,0) =    - v(2)*v(0)*in3;
		VN_v(2,1) =    - v(2)*v(1)*in3;
		VN_v(2,2) = in - v(2)*v(2)*in3;
	}
	
	// Normalize 3-vector, with Jacobians
	void normVec3(vec3 const& v, vec_range& vn, mat33& VN_v)
	{
		double in = 1.0/norm_2(v);
		vn = v*in;
		double in3 = pow(in,3);
		
		VN_v(0,0) = in - v(0)*v(0)*in3;
		VN_v(0,1) =    - v(0)*v(1)*in3;
		VN_v(0,2) =    - v(0)*v(2)*in3;
		VN_v(1,0) =    - v(1)*v(0)*in3;
		VN_v(1,1) = in - v(1)*v(1)*in3;
		VN_v(1,2) =    - v(1)*v(2)*in3;
		VN_v(2,0) =    - v(2)*v(0)*in3;
		VN_v(2,1) =    - v(2)*v(1)*in3;
		VN_v(2,2) = in - v(2)*v(2)*in3;
	}
	
	
	/// Bring a vector to the norm of another one
	vec3 adjustVec3(vec3 const& v,
					vec3 const& n)
	{
		double c = norm_2(n)/norm_2(v);
		return c*v;
	}
	
	/// Bring a vector to the norm of another one - with Jacobian
	///@param v the nevtor to adjust
	///@param n the vector used as reference
	///@param w the adjusted vector
	///@param W_v the Jacobian wrt v
	void adjustVec3(vec3 const& v,
					vec3 const& n,
					vec3& w,
					mat33& W_v)
	{
		double c = norm_2(n)/norm_2(v);
		w = c*v;
		W_v = c*identity_mat(3);
	}
	
	
	
	
	
	// Plucker line's origin, the closest point to the coordinates origin.
	vec3 pluckerOrigin(vec6 const& L)
	{
		// The Plucker line is divided in sub-vectors
		//   n  = subrange(L,0,3) is the plane normal
		//   v  = subrange(L,3,6) is the director vector
		// then the closest point of L to the global origin of coordinates is
		//   p0 = cross(n,v)/dot(v,v).
		
		// the sub-vectors
		vec3 n  = subrange(L, 0, 3);
		vec3 v  = subrange(L, 3, 6);
		
		// the point
		vec3 p0  = jmath::ublasExtra::crossProd(n,v);
		p0      /= inner_prod(v,v);
		return p0;
	}
	
	// Plucker line's origin, with Jacobian
	void pluckerOrigin(vec6 const& L, vec3& p0, mat& P0_l)
	{
		// The Plucker line is divided in sub-vectors
		//   n   = subrange(L,0,3) is the plane normal
		//   v   = subrange(L,3,6) is the director vector
		// then the closest point of L to the global origin of coordinates is
		//   p0  = cross(n,v)/dot(v,v).
		// This is computed in the following code by doing
		//   f   = cross(n,v)
		//   g   = dot(v,v)
		//   p0  = f/g = f*ig  (with  ig  = 1/g)
		// The Jacobians are built following the chain rule. We have
		//   F_n  = df/dn  = -[v]_x
		//   F_v  = df/dv  =  [n]_x
		//   G_v  = dg/dv  = 2*v'
		//   P0_n = dp0/dn = F_n/g
		//   P0_v = dp0/dv = (F_v*g - f*G_v)/g^2
		//   P0_l = dp0/dL = [P0_n;P0_v].
		
		// the sub-vectors
		vec3 n  = subrange(L, 0, 3);
		vec3 v  = subrange(L, 3, 6);
		
		// the numerator f, with Jacobians
		vec3  f  = jmath::ublasExtra::crossProd(n,v);
		mat33 F_n = -skewSymmMat33(v);
		mat33 F_v =  skewSymmMat33(n);
		
		// the denominator g, with Jacobians
		double g  = inner_prod(v,v);
		vec3 G_v   = 2.0*v; // the Jacobian is in fact the vector Gv transposed
		double ig = 1.0/g;
		
		// The 3D point
		p0       = f*ig;
		
		// The Jacobians wrt sub-vectors
		mat33 P0_n = F_n*ig;
		mat33 P0_v = (F_v*g - outer_prod(f,G_v))*ig*ig;
		
		// The full Jacobian
		subrange(P0_l,0,3,0,3) = P0_n;
		subrange(P0_l,0,3,3,6) = P0_v;
	}
	
	
	
	// Skew-symmetric matrix, 3-by-3, from 3-vector
	mat33 skewSymmMat33(vec3 const& v)
	{
		mat33 S;
		S(0,0) =  0.0;
		S(0,1) = -v(2);
		S(0,2) =  v(1);
		S(1,0) =  v(2);
		S(1,1) =  0.0;
		S(1,2) = -v(0);
		S(2,0) = -v(1);
		S(2,1) =  v(0);
		S(2,2) =  0.0;
		
		return S;
	}
	
	
	// Ray from pixel, as a Plucker line
	vec6 pluckerRay(mat33 const& iK, 
					vec2 const& p)
	{
		vec3 ph; // homogeneous pixel
		subrange(ph,0,2) = p;
		ph(2) = 1.0;
		
		vec6 r; // ray
		subrange(r,0,3).assign(zero_vec(3));
		subrange(r,3,6).assign(prod(iK,ph));
		
		return r;
	}
	
	
	// Ray as a Plucker line from pixel, with Jacobians
	void pluckerRay(mat33 const& iK, 
					vec2 const& p,
					vec6 r,
					mat R_p)
	{
		r  = pluckerRay(iK, p);
		subrange(R_p, 3,6, 0,2).assign(zero_mat(3,2));
		subrange(R_p, 3,6, 0,2).assign(subrange(iK, 0,3, 0,2));
	}
	
	
	
	// Compute Plucker line endpoint
	vec3 pluckerEndpoint(vec6 const& L, double const s)
	{
		vec3 p0 = pluckerOrigin(L);
		vec3 v  = subrange(L, 3, 6);
		return p0 + s*normVec3(v);
	}
	
	// Compute Plucker line endpoint, with Jacobians
	void pluckerEndpoint(vec6 const& L, double const s, 
			     vec3& e, mat& E_l, mat& E_s)
	{
		// Line origin
		vec3 p0;
		mat  P0_l;
		pluckerOrigin(L, p0, P0_l);
		
		// Normalized direction vector
		vec3  vn;
		mat33 VN_v; // Jac. wrt v
		normVec3(subrange(L,3,6), vn, VN_v);
		
		// Endpoint
		e         = p0 + s*vn;
		//identity_mat I(3,3); // these lines are not necessary, they are here only for documentation
		//mat33 E_p0 = I;
		//mat33 E_vn = s*I; // Jac. wrt vn
		column(E_s,0) = vn;  // Jac wrt. s
		
		// Full Jacobian
		// El = E_p0*P0_l + E_vn*VN_v*V_l =...= P0_l + [0 s*VN_v]
		E_l = P0_l;
		subrange(E_l, 0,3, 3,6) += s*VN_v; 
	}
	
	
	// Imperfect intersection of 2 Plucker lines \n
	boost::tuple<double,double,double> pluckerSoftIntersectionAbscissa(vec6 const& line1_, 
										 vec6 const& line2_)
	{
		JFR_PRECOND(line1_.size() == 6 && line2_.size() == 6, 
					"ImageSegmentFeatureObserveModel::pluckerLinesDistance:");
		JFR_TRACE_BEGIN;
		
		vec line1 = line1_;
		vec line2 = line2_;
		
		line1 /= norm_2(subrange(line1, 3, 6));
		line2 /= norm_2(subrange(line2, 3, 6));
		
		vec3 v1 = subrange(line1, 3, 6);
		vec3 v2 = subrange(line2, 3, 6);
		vec3 o1;
		vec3 o2;
		
		jmath::ublasExtra::crossProd(v1, project(line1, range(0, 3)), o1);
		jmath::ublasExtra::crossProd(v2, project(line2, range(0, 3)), o2);
		
		vec3 o1o2 = o2 - o1;
		
		double s1, s2, d;
		
		s1 = (inner_prod(o1o2, v1) - inner_prod(o1o2, v2)*inner_prod(v1,v2)) / (1 - pow(inner_prod(v1,v2),2));
		s2 = (inner_prod(o1o2, v1)*inner_prod(v1,v2) - inner_prod(o1o2, v2)) / (1 - pow(inner_prod(v1,v2),2));
		d = norm_2( (o1+s1*v1) - (o2+s2*v2) );
		
		return boost::make_tuple(s1, s2, d);
		
		JFR_TRACE_END("ImageSegmentHomogenCoordFeatureObserveModel::pluckerLinesDistance()");
	}
	
	
	// Transformation from Homogenous to Euclidean Plucker
	vec6 pluckerToEuclideanPlucker(vec6 const& L)
	{		
		return L/norm_2(subrange(L,3,6));
	}
	
	
	// Transformation from Homogenous to Euclidean Plucker, with Jacobians
	void pluckerToEuclideanPlucker(vec6 const& L, vec6& eL, mat66& EL_l)
	{
		vec3 n = subrange(L,0,3);
		vec3 v = subrange(L,3,6);
		double vNorm = norm_2(v); // norm of v
		vec3 VN_v = v/vNorm; // Jacobian of the norm (the transpose of this vector)
		
	 	subrange(eL,0,3) = n/vNorm; // the n sub-vector
		// mat33 EN_hn = identity_mat(3,3)/vNorm; // the Jac wrt the homogeneous n sub-vector
		mat33 EN_v = -outer_prod(subrange(L,0,3),VN_v)/pow(vNorm,2); // the jac wrt the homogeneous v sub-vector
		
		mat33 EV_v;
		vec3 ev;
		normVec3(v,ev,EV_v);
		subrange(eL,3,6) = ev;
		
		subrange(EL_l, 0,3, 0,3) = (identity_mat(3))/vNorm;
		subrange(EL_l, 0,3, 3,6) = EN_v;
		subrange(EL_l, 3,6, 0,3) = zero_mat(3,3);
		subrange(EL_l, 3,6, 3,6) = EV_v;
	}
	
	
	
	/// Two endpoints to homogeneous 2D line
	vec3 extToHomogeneous(vec2 const& p, vec2 const& q)
	{
		vec3 l;
		l(0) = p(1) - q(1);
		l(1) = q(0) - p(0);
		l(2) = p(0)*q(1) - p(1)*q(0);
		
		return l;
	}
	
	
	/// Two endpoints to homogeneous 2D line - with Jacobians
	void extToHomogeneous(vec2 const& p,
						  vec2 const& q,
						  vec3& hm,
						  mat& HM_p,
						  mat& HM_q)
	{
		hm(0) = p(1) - q(1);
		hm(1) = q(0) - p(0);
		hm(2) = p(0)*q(1) - p(1)*q(0);
		
		HM_p(0,0) =   0;
		HM_p(0,1) =   1;
		HM_p(1,0) =  -1;
		HM_p(1,1) =   0;
		HM_p(2,0) =  q(1);
		HM_p(2,1) = -q(0);
		
		HM_q(0,0) =   0;
		HM_q(0,1) =  -1;
		HM_q(1,0) =   1;
		HM_q(1,1) =   0;
		HM_q(2,0) = -p(1);
		HM_q(2,1) =  p(0);
	}
	
    /// Extract a plane from 3 euclidean points
    void points3plane(vec3 const& a, vec3 const& b, vec3 const& c, vec6& plane)
    {
        vec3 ab = b-a;
        vec3 ac = c-a;
        vec3 n = jmath::ublasExtra::crossProd(ab,ac);
        subrange(plane,0,3).assign(n);
        plane(3) = inner_prod(-n,a);
    }

    vec6 hmPointsToPlucker(vec4& A, vec4& B)
    {
        vec6 L;
        subrange(L,0,3).assign(jmath::ublasExtra::crossProd(subrange(A,0,3),subrange(B,0,3)));
        subrange(L,3,6).assign(A(3)*subrange(B,0,3)-B(3)*subrange(A,0,3));

        return L;
    }

    void hmPointsToPlucker(vec4& A, vec4& B, vec6& L, mat& L_a, mat& L_b)
    {
        subrange(L,0,3).assign(jmath::ublasExtra::crossProd(subrange(A,0,3),subrange(B,0,3)));
        subrange(L,3,6).assign(A(3)*subrange(B,0,3)-B(3)*subrange(A,0,3));

        L_a(0,0)= 0;
        L_a(0,1)= B(2);
        L_a(0,2)= -B(1);
        L_a(0,3)= 0;
        L_a(1,0)= -B(2);
        L_a(1,1)= 0;
        L_a(1,2)= B(0);
        L_a(1,3)= 0;
        L_a(2,0)= B(1);
        L_a(2,1)= -B(0);
        L_a(2,2)= 0;
        L_a(2,3)= 0;
        L_a(3,0)= -B(3);
        L_a(3,1)= 0;
        L_a(3,2)= 0;
        L_a(3,3)= B(0);
        L_a(4,0)= 0;
        L_a(4,1)= -B(3);
        L_a(4,2)= 0;  
        L_a(4,3)= B(1);
        L_a(5,0)= 0;
        L_a(5,1)= 0;
        L_a(5,2)= -B(3);
        L_a(5,3)= B(2);

        L_b(0,0)= 0;
        L_b(0,1)= -A(2);
        L_b(0,2)= A(1);
        L_b(0,3)=  0;
        L_b(1,0)= A(2);
        L_b(1,1)= 0;
        L_b(1,2)= -A(0);
        L_b(1,3)=  0;
        L_b(2,0)= -A(1);
        L_b(2,1)= A(0);
        L_b(2,2)= 0;
        L_b(2,3)= 0;
        L_b(3,0)= A(3);
        L_b(3,1)= 0;
        L_b(3,2)= 0;
        L_b(3,3)= -A(0);
        L_b(4,0)= 0;
        L_b(4,1)= A(3);
        L_b(4,2)= 0;
        L_b(4,3)= -A(1);
        L_b(5,0)= 0;
        L_b(5,1)= 0;
        L_b(5,2)= A(3);
        L_b(5,3)= -A(2);
    }


    vec4 uvdToHmPoint(vec3 const& uvd, vec4& k, double const alpha)
    {
        double u0 = k(0);
        double v0 = k(1);
        double au = k(2);
        double av = k(3);
        double u = uvd(0);
        double v = uvd(1);
        double d = uvd(2);

        vec4 hmP;
        hmP(0) = (u-u0)*alpha/au;
        hmP(1) = (v-v0)*alpha/av;
        hmP(2) = alpha;
        hmP(3) = d;

        return hmP;
    }

    void uvdToHmPoint(vec3 const& uvd, vec4& k, double const alpha, vec4& hmP, mat& HmP_u)
    {
        double u0 = k(0);
        double v0 = k(1);
        double au = k(2);
        double av = k(3);
        double u = uvd(0);
        double v = uvd(1);
        double d = uvd(2);

        hmP(0) = (u-u0)*alpha/au;
        hmP(1) = (v-v0)*alpha/av;
        hmP(2) = alpha;
        hmP(3) = d; 

        HmP_u.clear();
        HmP_u(0,0) = (alpha/au);
        HmP_u(1,1) = (alpha/av);
        HmP_u(3,2) = 1;
    }
	
	
	
	
	//Plucker line to two stereo homogeneous lines
	//template <class IntrVec, class PlkVec>
	// vec6 projectPluckerToStereo(IntrVec const& k, 
	//								   double alpha,
	//								   PlkVec const& L)
	vec6 projectPluckerToStereo(vec4 const&  k, 
								double       alpha,
								vec6 const&  L)
	{
		
		// right camera frame
		vec6 rightFrame;
		rightFrame.clear();
		rightFrame(0) = alpha/k(2);
		
		
		vec6 LR;
		jafar::slam::EulerTools::lineToFrame(rightFrame, L, LR);
		
		mat33 K;
		pluckerIntrinsicMatrix(k, K);
		
		vec6 stLine;
		
		vec3 hm;
		
		hm = projectPlucker(K, L);
		subrange(stLine, 0, 3) = hm;
//         JFR_DEBUG( "hm = " << hm );
		
		hm = projectPlucker(K, LR);
		subrange(stLine, 3, 6) = hm;
// 		JFR_DEBUG( "hm = " << hm );
//         JFR_DEBUG( "L = " << L << " LR = " << LR );
//         JFR_DEBUG( "alpha = " << alpha << " rightFrame = " << rightFrame << " k = " << k << " K = " << K );
		return stLine;
		 
	}
	
	
	//Plucker line to two stereo homogeneous lines - jac
	//template <class StereoLineObs, class IntrVec, class PlkVec, class JacMat>
	//inline void projectPluckerToStereo(IntrVec const&  k, 
	//								   double          alpha,
	//								   PlkVec const&   L,
	//								   StereoLineObs&  stLine,
	//								   JacMat&         StLine_l)
	void projectPluckerToStereo(vec4 const&   k, 
								double               alpha,
								vec6 const&   L,
								vec6&         stLine,
								mat66&        StLine_l)
	{
		
		vec6 rightFrame;
		rightFrame.clear();
		rightFrame(0) = -alpha/k(2);
		
		mat LR_l(6,6);
		
		// bring line to right camera frame
		vec6 LR;
		jafar::slam::EulerTools::lineToFrame(rightFrame, L, LR);
		mat LR_frame(6,6);
		jafar::slam::EulerTools::lineToFrameJac(rightFrame, L, LR_frame, LR_l);
		
		// build intrinsic matrix
		mat33 K;
		pluckerIntrinsicMatrix(k, K);
		
		// project lines
		mat HML_l(3,6);
		mat HMR_lr(3,6);
		vec3 hm;
		projectPlucker(K, L,  hm, HML_l);
		subrange(stLine, 0, 3) = hm;
		projectPlucker(K, LR, hm, HMR_lr);
		subrange(stLine, 3, 6) = hm;
		
		// jacobians
		subrange(StLine_l, 0,3, 0,6) = HML_l;
		subrange(StLine_l, 3,6, 0,6) = prod(HMR_lr,LR_l);
		 
		
	}
	
	// Segment extremities to rho theta line conversion
	vec2 extToRhoTheta(const vec2& ext1, const vec2& ext2)
	{
		vec3 hm = extToHomogeneous(ext1, ext2);
		return homogeneousToRhoTheta(hm);
	}	
	
	
	/*// Segment extremities to rho theta line conversion - Jacobians
	void extToRhoTheta(const vec2& ext1, const vec2& ext2, vec2& rt, mat& RT_ee)
	{
		vec3 hm;
		mat HM_e1(3,2);
		mat HM_e2(3,2);
		mat HM_ee(3,4);
		extToHomogeneous(ext1, ext2, hm, HM_e1, HM_e2);
		subrange(HM_ee, 0,3, 0,2) = HM_e1;
		subrange(HM_ee, 0,3, 2,4) = HM_e2;
		
		mat RT_hm(2,3);
		homogeneousToRhoTheta(hm, rt, RT_hm);
		RT_ee = prod(RT_hm,HM_ee);
	}*/

    void extToRhoTheta(const vec2& ext1, const vec2& ext2, vec2& rt, mat& J)
    {
        //JFR_PRECOND(ext1.size() == 2 && ext2.size() == 2, "LineTools::extToImageModelJac");
        //JFR_PRECOND(J.size1() == 2 && J.size2() == 4, "RhoThetaImagePluckerObserveModel::extToImageModelJac");

        double u1 = ext1(0);
        double v1 = ext1(1);
        double u2 = ext2(0);
        double v2 = ext2(1);

        double rt1 = u2 * u2;
        double rt4 = u1 * u1;
        double rt5 = v2 * v2;
        double rt8 = v1 * v1;
        double rt10 = sqrt(rt1 - 0.2e1 * u2 * u1 + rt4 + rt5 - 0.2e1 * v2 * v1 + rt8);
        double rt11 = 0.1e1 / rt10;
        double rt13 = u2 - u1;
        double rt16 = v2 - v1;
        double rt19 = (-u1 * rt11 * rt13 - v1 * rt11 * rt16) * rt11;
        double rt21 = u1 + rt19 * rt13;
        double rt22 = rt21 * rt21;
        double rt24 = v1 + rt19 * rt16;
        double rt25 = rt24 * rt24;
        double rt27 = sqrt(rt22 + rt25);
        double rt28 = atan2(rt24, rt21);
        rt(0) = rt27;
        rt(1) = rt28;
            
        double t1 = v2 * v1;
        double t2 = u2 * u2;
        double t3 = t2 * u2;
        double t5 = v1 * v1;
        double t7 = u1 * t2;
        double t8 = v2 * v2;
        double t10 = t7 * t5;
        double t11 = t8 * v2;
        double t14 = t8 * t5;
        double t15 = t14 * u2;
        double t17 = t5 * v1;
        double t18 = v2 * t17;
        double t19 = t18 * u2;
        double t21 = u1 * u1;
        double t22 = t8 * u2 * t21;
        double t23 = t21 * u2;
        double t25 = t14 * u1;
        double t26 = t8 * t8;
        double t29 = t11 * u1 * v1;
        double t31 = -t1 * t3 + t3 * t5 + t7 * t8 - t10 - t11 * v1 * u2 + 0.2e1 * t15 - t19 - t22 + t23 * t1 + t25 + u1 * t26 - 0.2e1 * t29;
        double t32 = u2 * u1;
        double t35 = t2 - 0.2e1 * t32 + t21 + t8 - 0.2e1 * t1 + t5;
        double t36 = t35 * t35;
        double t37 = 0.1e1 / t36;
        double t39 = u1 * v2;
        double t42 = pow(-t39 + u2 * v1, 0.2e1);
        double t43 = 0.1e1 / t35;
        double t45 = sqrt(t42 * t43);
        double t46 = 0.1e1 / t45;
        double t48 = t2 * t2;
        double t50 = t3 * u1;
        double t52 = t50 * v1;
        double t54 = t2 * t21;
        double t55 = t54 * v1;
        double t56 = v2 * t5;
        double t57 = t56 * t2;
        double t58 = t8 * v1;
        double t60 = t54 * v2;
        double t65 = t21 * u1;
        double t66 = u2 * t65;
        double t67 = t66 * v2;
        double t70 = t21 * v1 * t8;
        double t71 = v1 * t48 - t50 * v2 - 0.2e1 * t52 + t55 - t57 + t58 * t2 + 0.2e1 * t60 - t11 * u2 * u1 + t32 * t56 - t67 + t11 * t21 - t70;
        double t78 = t5 * t5;
        double t85 = t10 - t39 * v1 * t2 + t22 - t23 * t5 - t15 + 0.2e1 * t19 - u2 * t78 - t8 * t65 + t18 * u1 + t65 * v2 * v1 + t29 - 0.2e1 * t25;
        double t96 = t21 * t21;
        double t98 = t52 + t57 - t60 - 0.2e1 * t55 - t2 * t17 + t32 * t17 + t66 * v1 + 0.2e1 * t67 - t32 * t58 - t21 * v2 * t5 - t96 * v2 + t70;
        double t102 = (-v2 + v1) * t43;
        double t104 = (u2 - u1) * t43;
        J(0,0) = t31 * t37 * t46;
        J(0,1) = t71 * t37 * t46;
        J(0,2) = -t85 * t37 * t46;
        J(0,3) = -t98 * t37 * t46;
        J(1,0) = -t102;
        J(1,1) = -t104;
        J(1,2) = t102;
        J(1,3) = t104;    
    /* End Maple */
    
    }

	
	
	/// Convert stereo segment to rho-theta stereo line
	vec4 uvdsToStereoRhoTheta(vec6 const& uvduvd)
	{
		vec4 rtrt;
		vec2 ext1;
		vec2 ext2;
		
		// left line
		ext1 = subrange(uvduvd, 0,2);
		ext2 = subrange(uvduvd, 3,5);
		//JFR_DEBUG("ext1: " << ext1 << " ext2: " << ext2);
		subrange(rtrt,0,2) = extToRhoTheta(ext1, ext2);

		// right line
		ext1(0) = uvduvd(0) - uvduvd(2);
		ext1(1) = uvduvd(1);
		ext2(0) = uvduvd(3) - uvduvd(5);
		ext2(1) = uvduvd(4);
		//JFR_DEBUG("ext1: " << ext1 << " ext2: " << ext2);
		subrange(rtrt,2,4) = extToRhoTheta(ext1, ext2);
		
		return rtrt;
	}
	
		
	/// Convert stereo segment to rho-theta stereo line - Jacobians
	void uvdsToStereoRhoTheta(vec6 const& uvduvd, vec4& rtrt, mat& RR_uu)
	{
		vec2 ext1;
		vec2 ext2;
		vec2 rt;
		mat  RT_ee(2,4);
		
		// left line.
		// endpoints
		ext1 = subrange(uvduvd, 0,2);
		ext2 = subrange(uvduvd, 3,5);
		// rho theta
		extToRhoTheta(ext1, ext2, rt, RT_ee);
		subrange(rtrt,0,2) = rt;
		
		// full Jacobian - left part
		RR_uu.clear();
		subrange(RR_uu, 0,2, 0,2) = subrange(RT_ee, 0,2, 0,2);
		subrange(RR_uu, 0,2, 3,5) = subrange(RT_ee, 0,2, 2,4);
		
		// right line.
		// endpoints
		ext1(0) = uvduvd(0) - uvduvd(2);
		ext1(1) = uvduvd(1);
		ext2(0) = uvduvd(3) - uvduvd(5);
		ext2(1) = uvduvd(4);
		mat E_u(2,3);  // the Jac of a right-hand pixel wrt the uvd pixel
		E_u.clear();
		E_u(0,0) =  1.0;
		E_u(1,1) =  1.0;
		E_u(0,2) = -1.0;
		// rho theta
		extToRhoTheta(ext1, ext2, rt, RT_ee);
		subrange(rtrt,2,4) = rt;
		
		// full Jacobian - right part
		subrange(RR_uu, 2,4, 0,3) = prod(subrange(RT_ee, 0,2, 0,2), E_u);
		subrange(RR_uu, 2,4, 3,6) = prod(subrange(RT_ee, 0,2, 2,4), E_u);
		
	}
	
	/// Convert stereo segment to rho-theta stereo line - only Jacobians
	void uvdsToStereoRhoTheta(vec6 const& uvduvd, mat& RR_uu)
	{
		vec2 ext1;
		vec2 ext2;
		vec2 rt;
		mat  RT_ee(2,4);
		
		// left line.
		// endpoints
		ext1 = subrange(uvduvd, 0,2);
		ext2 = subrange(uvduvd, 3,5);
		// rho theta
		extToRhoTheta(ext1, ext2, rt, RT_ee);
		
		// full Jacobian - left part
		RR_uu.clear();
		subrange(RR_uu, 0,2, 0,2) = subrange(RT_ee, 0,2, 0,2); // first endpoint
		subrange(RR_uu, 0,2, 3,5) = subrange(RT_ee, 0,2, 2,4); // second endpoint
		//JFR_DEBUG( JFR_PP_VAR( RR_uu ) << JFR_PP_VAR( RT_ee ) );
		
		// right line.
		// endpoints
		ext1(0) = uvduvd(0) - uvduvd(2);
		ext1(1) = uvduvd(1);
		ext2(0) = uvduvd(3) - uvduvd(5);
		ext2(1) = uvduvd(4);
		mat ER_uvd(2,3);  // the Jac of a right-hand pixel wrt the uvd pixel
		ER_uvd.clear();
		ER_uvd(0,0) =  1.0;
		ER_uvd(1,1) =  1.0;
		ER_uvd(0,2) = -1.0;
		// rho theta
		extToRhoTheta(ext1, ext2, rt, RT_ee);
		
		// full Jacobian - right part
		subrange(RR_uu, 2,4, 0,3) = prod(subrange(RT_ee, 0,2, 0,2), ER_uvd); // first endpoint
		subrange(RR_uu, 2,4, 3,6) = prod(subrange(RT_ee, 0,2, 2,4), ER_uvd); // second endpoint
        subrange(RR_uu,0,2,2,4)=zero_mat(2,2);
        subrange(RR_uu,2,4,0,2)=zero_mat(2,2);
		//JFR_DEBUG( JFR_PP_VAR( RR_uu ) << JFR_PP_VAR( RT_ee ) << JFR_PP_VAR( ER_uvd ) );
	}
	
	

void homogenousToExtDistances( const jblas::vec3& _hm, const jblas::vec2& ext1, const jblas::vec2& ext2, jblas::vec2& dd, jblas::mat& DD_hm, jblas::mat& DD_ext1, jblas::mat& DD_ext2 )
{
  JFR_ASSERT( DD_ext1.size1() == 2 and DD_ext1.size2() == 2, "Invalid size for D_ext");
  JFR_ASSERT( DD_ext2.size1() == 2 and DD_ext2.size2() == 2, "Invalid size for D_ext");
  JFR_ASSERT( DD_hm.size1() == 2 and DD_hm.size2() == 3, "Invalid size for D_hm");
  
  DD_ext1.clear();
  DD_ext2.clear();
  jblas::mat_range rangeDD_hm1 = subrange( DD_hm, 0,1, 0,3);
  jblas::mat_range rangeDD_ext1 = subrange(DD_ext1, 0, 1, 0, 2);
  homogenousToPointDistance( _hm, ext1, dd(0), rangeDD_hm1, rangeDD_ext1 );
  jblas::mat_range rangeDD_hm2 = subrange( DD_hm, 1,2, 0,3);
  jblas::mat_range rangeDD_ext2 = subrange(DD_ext2, 1, 2, 0, 2);
  homogenousToPointDistance( _hm, ext2, dd(1), rangeDD_hm2, rangeDD_ext2 );
}
     
void homogenousToPointDistance( const jblas::vec3& _hm, const jblas::vec2& _ext, double& d, jblas::mat_range& D_hm, jblas::mat_range& D_ext )
{
  JFR_ASSERT( D_ext.size1() == 1 and D_ext.size2() == 2, "Invalid size for D_ext");
  JFR_ASSERT( D_hm.size1() == 1 and D_hm.size2() == 3, "Invalid size for D_hm");
  double ab2 = 1.0 / (_hm(0) * _hm(0) + _hm(1) * _hm(1) );
  double ab = sqrt( ab2 );
  double ab3 = ab * ab2;
  d = ab * (_hm(0) * _ext(0) + _hm(1) * _ext(1) + _hm(2) );
  D_hm(0,0) = _ext(0) * ab - d * _hm(0) * ab3;
  D_hm(0,1) = _ext(1) * ab - d * _hm(1) * ab3;
  D_hm(0,2) = ab;
  D_ext(0,0) = _hm(0) * ab;
  D_ext(0,1) = _hm(1) * ab;
}

void homogenousToExtDistances( const jblas::vec3& _hm, const jblas::vec2& ext1, const jblas::vec2& ext2, jblas::vec2& dd, jblas::mat& DD_hm )
{
  JFR_ASSERT( DD_hm.size1() == 2 and DD_hm.size2() == 3, "Invalid size for D_hm");
  
  jblas::mat_range rangeDD_hm1 = subrange( DD_hm, 0,1, 0,3);
  homogenousToPointDistance( _hm, ext1, dd(0), rangeDD_hm1 );
  jblas::mat_range rangeDD_hm2 = subrange( DD_hm, 1,2, 0,3);
  homogenousToPointDistance( _hm, ext2, dd(1), rangeDD_hm2 );
}
     
void homogenousToPointDistance( const jblas::vec3& _hm, const jblas::vec2& _ext, double& d, jblas::mat_range& D_hm)
{
  JFR_ASSERT( D_hm.size1() == 1 and D_hm.size2() == 3, "Invalid size for D_hm");
  double ab2 = 1.0 / (_hm(0) * _hm(0) + _hm(1) * _hm(1) );
  double ab = sqrt( ab2 );
  double ab3 = ab * ab2;
  d = ab * (_hm(0) * _ext(0) + _hm(1) * _ext(1) + _hm(2) );
  D_hm(0,0) = _ext(0) * ab - d * _hm(0) * ab3;
  D_hm(0,1) = _ext(1) * ab - d * _hm(1) * ab3;
  D_hm(0,2) = ab;
}

}}} // namespaces lineTools ; slam ; jafar


