/* $Id$ */

#include <cmath>

#include "kernel/jafarDebug.hpp"

#include "slam/eulerTools.hpp"

using namespace jblas;
using namespace ublas;

// using namespace jafar::slam;

namespace jafar { namespace slam { namespace EulerTools {

  namespace details {

    void fromFrame(double x, double y, double z,
		   double yaw, double pitch, double roll, 
		   double v_x, double v_y, double v_z, H h,
		   double& vres_x, double& vres_y, double& vres_z) 
    {
      
      switch (h) {
      case h_1:
	{
	  double t1 = cos(yaw);
	  double t2 = cos(pitch);
	  double t6 = sin(pitch);
	  double t7 = t1 * t6;
	  double t8 = sin(roll);
	  double t10 = sin(yaw);
	  double t11 = cos(roll);
	  double t26 = t10 * t6;

	  vres_x = 0.10e1 * t1 * t2 * v_x + (t7 * t8 - 0.10e1 * t10 * t11) * v_y + (t7 * t11 + 0.10e1 * t10 * t8) * v_z + 0.10e1 * x;
	  vres_y = 0.10e1 * t10 * t2 * v_x + (t26 * t8 + 0.10e1 * t1 * t11) * v_y + (t26 * t11 - 0.10e1 * t1 * t8) * v_z + 0.10e1 * y;
	  vres_z = -0.100e1 * t6 * v_x + 0.10e1 * t2 * t8 * v_y + 0.10e1 * t2 * t11 * v_z + 0.10e1 * z;

	  break;
	}
      case h_0:
	{
	  double t1 = cos(yaw);
	  double t2 = cos(pitch);
	  double t6 = sin(pitch);
	  double t7 = t1 * t6;
	  double t8 = sin(roll);
	  double t10 = sin(yaw);
	  double t11 = cos(roll);
	  double t25 = t10 * t6;

	  vres_x = 0.10e1 * t1 * t2 * v_x + (t7 * t8 - 0.10e1 * t10 * t11) * v_y + (t7 * t11 + 0.10e1 * t10 * t8) * v_z;
	  vres_y = 0.10e1 * t10 * t2 * v_x + (t25 * t8 + 0.10e1 * t1 * t11) * v_y + (t25 * t11 - 0.10e1 * t1 * t8) * v_z;
	  vres_z = -0.100e1 * t6 * v_x + 0.10e1 * t2 * t8 * v_y + 0.10e1 * t2 * t11 * v_z;

	  break;
	}
      }
    }

    void toFrame(double x, double y, double z,
		 double yaw, double pitch, double roll, 
		 double v_x, double v_y, double v_z, H h,
		 double& vres_x, double& vres_y, double& vres_z) 
    {
      switch (h) {
      case h_1:
	{
	  double t1 = cos(yaw);
	  double t2 = cos(pitch);
	  double t3 = t1 * t2;
	  double t6 = sin(yaw);
	  double t7 = t6 * t2;
	  double t10 = sin(pitch);
	  double t20 = t1 * t10;
	  double t21 = sin(roll);
	  double t23 = cos(roll);
	  double t26 = t20 * t21 - 0.10e1 * t6 * t23;
	  double t28 = t6 * t10;
	  double t32 = t28 * t21 + 0.10e1 * t1 * t23;
	  double t34 = t2 * t21;
	  double t47 = t20 * t23 + 0.10e1 * t6 * t21;
	  double t52 = t28 * t23 - 0.10e1 * t1 * t21;
	  double t54 = t2 * t23;

	  vres_x = 0.10e1 * t3 * v_x + 0.10e1 * t7 * v_y - 0.100e1 * t10 * v_z - 0.100e1 * t3 * x - 0.100e1 * t7 * y + 0.1000e1 * t10 * z;
	  vres_y = t26 * v_x + t32 * v_y + 0.10e1 * t34 * v_z - 0.10e1 * t26 * x - 0.10e1 * t32 * y - 0.100e1 * t34 * z;
	  vres_z = t47 * v_x + t52 * v_y + 0.10e1 * t54 * v_z - 0.10e1 * t47 * x - 0.10e1 * t52 * y - 0.100e1 * t54 * z;

	  break;
	}
      case h_0:
	{
	  double t1 = cos(yaw);
	  double t2 = cos(pitch);
	  double t6 = sin(yaw);
	  double t10 = sin(pitch);
	  double t14 = t1 * t10;
	  double t15 = sin(roll);
	  double t17 = cos(roll);
	  double t22 = t10 * t6;

	  vres_x = 0.10e1 * t1 * t2 * v_x + 0.10e1 * t6 * t2 * v_y - 0.100e1 * t10 * v_z;
	  vres_y = (t14 * t15 - 0.10e1 * t6 * t17) * v_x + (t22 * t15 + 0.10e1 * t1 * t17) * v_y + 0.10e1 * t2 * t15 * v_z;
	  vres_z = (t14 * t17 + 0.10e1 * t6 * t15) * v_x + (t22 * t17 - 0.10e1 * t1 * t15) * v_y + 0.10e1 * t2 * t17 * v_z;

	  break;
	}
      }
    }

    void composeFrame(double x1, double y1, double z1,
		      double yaw1, double pitch1, double roll1,
		      double x2, double y2, double z2,
		      double yaw2, double pitch2, double roll2,
		      double& x_res, double& y_res, double& z_res,
		      double& yaw_res, double& pitch_res, double& roll_res)
    {

      /* begin maple */

      double t1 = cos(yaw1);
      double t2 = cos(pitch1);
      double t3 = t1 * t2;
      double t6 = sin(pitch1);
      double t7 = t1 * t6;
      double t8 = sin(roll1);
      double t10 = sin(yaw1);
      double t11 = cos(roll1);
      double t14 = t7 * t8 - 0.10e1 * t10 * t11;
      double t19 = t7 * t11 + 0.10e1 * t10 * t8;
      double t23 = t10 * t2;
      double t26 = t10 * t6;
      double t30 = t26 * t8 + 0.10e1 * t1 * t11;
      double t35 = t26 * t11 - 0.10e1 * t1 * t8;
      double t41 = t2 * t8;
      double t44 = t2 * t11;
      double t49 = cos(yaw2);
      double t50 = cos(pitch2);
      double t51 = t49 * t50;
      double t54 = sin(yaw2);
      double t58 = sin(pitch2);
      double t70 = atan2(0.100e1 * t51 * t23 + 0.10e1 * t30 * t54 * t50 - 0.100e1 * t35 * t58, 0.100e1 * t51 * t3 + 0.10e1 * t14 * t54 * t50 - 0.100e1 * t58 * t19);
      double t80 = asin(0.1000e1 * t6 * t49 * t50 - 0.100e1 * t41 * t54 * t50 + 0.1000e1 * t44 * t58);
      double t81 = t49 * t58;
      double t82 = sin(roll2);
      double t84 = cos(roll2);
      double t90 = t54 * t58;
      double t117 = atan2(-0.100e1 * t6 * (t81 * t82 - 0.10e1 * t54 * t84) + 0.10e1 * t41 * (t90 * t82 + 0.10e1 * t49 * t84) + 0.100e1 * t50 * t82 * t44, -0.100e1 * (t81 * t84 + 0.10e1 * t54 * t82) * t6 + 0.10e1 * t41 * (t90 * t84 - 0.10e1 * t49 * t82) + 0.100e1 * t50 * t84 * t44);

      x_res = 0.10e1 * t3 * x2 + t14 * y2 + t19 * z2 + 0.10e1 * x1;
      y_res = 0.10e1 * t23 * x2 + t30 * y2 + t35 * z2 + 0.10e1 * y1;
      z_res = -0.100e1 * t6 * x2 + 0.10e1 * t41 * y2 + 0.10e1 * t44 * z2 + 0.10e1 * z1;
      yaw_res = t70;
      pitch_res = t80;
      roll_res = t117;
      /* End maple */
  
    }

    void composeFrameJac(double x1, double y1, double z1,
			 double yaw1, double pitch1, double roll1,
			 double x2, double y2, double z2,
			 double yaw2, double pitch2, double roll2,
			 mat& Jframe1, mat& Jframe2) {

      JFR_PRECOND(Jframe1.size1() == 6 && Jframe1.size2() == 6,
		  "fromFrameJac: size of Jframe1 does not match");
      JFR_PRECOND(Jframe2.size1() == 6 && Jframe2.size2() == 6,
		  "fromFrameJac: size of Jframe2 does not match");
      /* Begin maple */
      double t1 = sin(yaw1);
      double t2 = cos(pitch1);
      double t3 = t1 * t2;
      double t6 = sin(pitch1);
      double t7 = t1 * t6;
      double t8 = sin(roll1);
      double t10 = cos(yaw1);
      double t11 = cos(roll1);
      double t14 = -t7 * t8 - 0.10e1 * t10 * t11;
      double t19 = -t7 * t11 + 0.10e1 * t10 * t8;
      double t22 = t10 * t6;
      double t25 = t10 * t2;
      double t26 = t8 * y2;
      double t28 = t11 * z2;
      double t34 = t22 * t11 + 0.10e1 * t1 * t8;
      double t39 = -t22 * t8 + 0.10e1 * t1 * t11;
      double t59 = t6 * t8;
      double t62 = t6 * t11;
      double t66 = t2 * t11;
      double t69 = t2 * t8;
      double t76 = cos(yaw2);
      double t77 = cos(pitch2);
      double t78 = t76 * t77;
      double t80 = 0.100e1 * t78 * t3;
      double t81 = sin(yaw2);
      double t82 = -t14 * t81;
      double t85 = sin(pitch2);
      double t88 = t80 + 0.10e1 * t82 * t77 + 0.100e1 * t19 * t85;
      double t91 = -t39 * t81;
      double t96 = 0.100e1 * t78 * t25 + 0.10e1 * t91 * t77 - 0.100e1 * t85 * t34;
      double t97 = t96 * t96;
      double t98 = 0.1e1 / t97;
      double t99 = t88 * t98;
      double t108 = t88 * t88;
      double t111 = 0.1e1 / (0.1e1 + t108 * t98);
      double t116 = t8 * t81 * t77;
      double t123 = 0.1e1 / t96;
      double t152 = t81 * t77;
      double t169 = t76 * t85;
      double t196 = t6 * t76;
      double t204 = pow(0.1000e1 * t196 * t77 - 0.100e1 * t69 * t152 + 0.1000e1 * t66 * t85, 0.2e1);
      double t206 = sqrt(0.1e1 - t204);
      double t207 = 0.1e1 / t206;
      double t224 = t81 * t85;
      double t231 = sin(roll2);
      double t233 = cos(roll2);
      double t236 = t169 * t231 - 0.10e1 * t81 * t233;
      double t242 = t224 * t231 + 0.10e1 * t76 * t233;
      double t245 = t77 * t231;
      double t252 = t169 * t233 + 0.10e1 * t81 * t231;
      double t258 = t224 * t233 - 0.10e1 * t76 * t231;
      double t261 = t77 * t233;
      double t264 = -0.100e1 * t252 * t6 + 0.10e1 * t69 * t258 + 0.100e1 * t261 * t66;
      double t265 = 0.1e1 / t264;
      double t272 = 0.100e1 * t245 * t66;
      double t273 = -0.100e1 * t6 * t236 + 0.10e1 * t69 * t242 + t272;
      double t274 = t264 * t264;
      double t275 = 0.1e1 / t274;
      double t276 = t273 * t275;
      double t286 = t273 * t273;
      double t289 = 0.1e1 / (0.1e1 + t286 * t275);
      Jframe1(0,0) = 0.10e1;
      Jframe1(0,1) = 0.0e0;
      Jframe1(0,2) = 0.0e0;
      Jframe1(0,3) = -0.10e1 * t3 * x2 + t14 * y2 + t19 * z2;
      Jframe1(0,4) = -0.10e1 * t22 * x2 + t25 * t26 + t25 * t28;
      Jframe1(0,5) = t34 * y2 + t39 * z2;
      Jframe1(1,0) = 0.0e0;
      Jframe1(1,1) = 0.10e1;
      Jframe1(1,2) = 0.0e0;
      Jframe1(1,3) = 0.10e1 * t25 * x2 - t39 * y2 + t34 * z2;
      Jframe1(1,4) = -0.10e1 * t7 * x2 + t3 * t26 + t3 * t28;
      Jframe1(1,5) = -t19 * y2 + t14 * z2;
      Jframe1(2,0) = 0.0e0;
      Jframe1(2,1) = 0.0e0;
      Jframe1(2,2) = 0.10e1;
      Jframe1(2,3) = 0.0e0;
      Jframe1(2,4) = -0.100e1 * t2 * x2 - 0.10e1 * t59 * y2 - 0.10e1 * t62 * z2;
      Jframe1(2,5) = 0.10e1 * t66 * y2 - 0.10e1 * t69 * z2;
      Jframe1(3,0) = 0.0e0;
      Jframe1(3,1) = 0.0e0;
      Jframe1(3,2) = 0.0e0;
      Jframe1(3,3) = (0.1e1 - t99 * (-t80 + 0.10e1 * t14 * t81 * t77 - 0.100e1 * t19 * t85)) * t111;
      Jframe1(3,4) = ((-0.100e1 * t78 * t7 + 0.10e1 * t3 * t116 - 0.100e1 * t3 * t11 * t85) * t123 - t99 * (-0.100e1 * t78 * t22 + 0.10e1 * t25 * t116 - 0.100e1 * t85 * t10 * t66)) * t111;
      Jframe1(3,5) = ((-0.10e1 * t19 * t81 * t77 - 0.100e1 * t85 * t14) * t123 - t99 * (0.10e1 * t34 * t81 * t77 - 0.100e1 * t85 * t39)) * t111;
      Jframe1(4,0) = 0.0e0;
      Jframe1(4,1) = 0.0e0;
      Jframe1(4,2) = 0.0e0;
      Jframe1(4,3) = 0.0e0;
      Jframe1(4,4) = (0.1000e1 * t2 * t76 * t77 + 0.100e1 * t59 * t152 - 0.1000e1 * t62 * t85) * t207;
      Jframe1(4,5) = (-0.100e1 * t66 * t152 - 0.1000e1 * t69 * t85) * t207;
      Jframe1(5,0) = 0.0e0;
      Jframe1(5,1) = 0.0e0;
      Jframe1(5,2) = 0.0e0;
      Jframe1(5,3) = 0.0e0;
      Jframe1(5,4) = ((-0.100e1 * t2 * t236 - 0.10e1 * t59 * t242 - 0.100e1 * t245 * t62) * t265 - t276 * (-0.100e1 * t252 * t2 - 0.10e1 * t59 * t258 - 0.100e1 * t261 * t62)) * t289;
      Jframe1(5,5) = ((0.10e1 * t66 * t242 - 0.100e1 * t245 * t69) * t265 - t276 * (0.10e1 * t66 * t258 - 0.100e1 * t261 * t69)) * t289;

      Jframe2(0,0) = 0.10e1 * t25;
      Jframe2(0,1) = -t39;
      Jframe2(0,2) = t34;
      Jframe2(0,3) = 0.0e0;
      Jframe2(0,4) = 0.0e0;
      Jframe2(0,5) = 0.0e0;
      Jframe2(1,0) = 0.10e1 * t3;
      Jframe2(1,1) = -t14;
      Jframe2(1,2) = -t19;
      Jframe2(1,3) = 0.0e0;
      Jframe2(1,4) = 0.0e0;
      Jframe2(1,5) = 0.0e0;
      Jframe2(2,0) = -0.100e1 * t6;
      Jframe2(2,1) = 0.10e1 * t69;
      Jframe2(2,2) = 0.10e1 * t66;
      Jframe2(2,3) = 0.0e0;
      Jframe2(2,4) = 0.0e0;
      Jframe2(2,5) = 0.0e0;
      Jframe2(3,0) = 0.0e0;
      Jframe2(3,1) = 0.0e0;
      Jframe2(3,2) = 0.0e0;
      Jframe2(3,3) = ((-0.100e1 * t152 * t3 - 0.10e1 * t14 * t76 * t77) * t123 - t99 * (-0.100e1 * t152 * t25 - 0.10e1 * t39 * t76 * t77)) * t111;
      Jframe2(3,4) = ((-0.100e1 * t169 * t3 - 0.10e1 * t82 * t85 + 0.100e1 * t19 * t77) * t123 - t99 * (-0.100e1 * t169 * t25 - 0.10e1 * t91 * t85 - 0.100e1 * t77 * t34)) * t111;
      Jframe2(3,5) = 0.0e0;
      Jframe2(4,0) = 0.0e0;
      Jframe2(4,1) = 0.0e0;
      Jframe2(4,2) = 0.0e0;
      Jframe2(4,3) = (-0.1000e1 * t6 * t81 * t77 - 0.100e1 * t69 * t78) * t207;
      Jframe2(4,4) = (-0.1000e1 * t196 * t85 + 0.100e1 * t69 * t224 + 0.1000e1 * t66 * t77) * t207;
      Jframe2(4,5) = 0.0e0;
      Jframe2(5,0) = 0.0e0;
      Jframe2(5,1) = 0.0e0;
      Jframe2(5,2) = 0.0e0;
      Jframe2(5,3) = ((0.100e1 * t242 * t6 + 0.10e1 * t69 * t236) * t265 - t276 * (0.100e1 * t258 * t6 + 0.10e1 * t69 * t252)) * t289;
      Jframe2(5,4) = ((-0.100e1 * t196 * t245 + 0.10e1 * t69 * t152 * t231 - 0.100e1 * t85 * t231 * t66) * t265 - t276 * (-0.100e1 * t78 * t233 * t6 + 0.10e1 * t69 * t152 * t233 - 0.100e1 * t85 * t233 * t66)) * t289;
      Jframe2(5,5) = (0.1e1 - t276 * (0.100e1 * t6 * t236 - 0.10e1 * t69 * t242 - t272)) * t289;

      /* End maple */
    }

    void invFrame(double x, double y, double z,
		  double yaw, double pitch, double roll,
		  double& x_inv, double& y_inv, double& z_inv,
		  double& yaw_inv, double& pitch_inv, double& roll_inv)
    {
      double t1 = cos(yaw);
      double t2 = cos(pitch);
      double t3 = t1 * t2;
      double t6 = sin(yaw);
      double t10 = sin(pitch);
      double t14 = t1 * t10;
      double t15 = sin(roll);
      double t17 = cos(roll);
      double t20 = t14 * t15 - 0.10e1 * t6 * t17;
      double t22 = t6 * t10;
      double t35 = t14 * t17 + 0.10e1 * t6 * t15;
      double t40 = t22 * t17 - 0.10e1 * t1 * t15;
      double t42 = t2 * t17;
      double t47 = atan2(t20, 0.10e1 * t3);
      double t48 = asin(t35);
      double t50 = atan2(t40, 0.10e1 * t42);
      x_inv = -0.10e1 * t3 * x - 0.10e1 * t6 * t2 * y + 0.100e1 * t10 * z;
      y_inv = -t20 * x - (t22 * t15 + 0.10e1 * t1 * t17) * y - 0.10e1 * t2 * t15 * z;
      z_inv = -t35 * x - t40 * y - 0.10e1 * t42 * z;
      yaw_inv = t47;
      pitch_inv = -t48;
      roll_inv = t50;
    }

    void lineToFrame(double x, double y, double z, 
		     double yaw, double pitch, double roll, 
		     double n1, double n2, double n3, double u1, double u2, double u3,
		     double& n1Res, double& n2Res, double& n3Res, double& u1Res, double& u2Res, double& u3Res)
    {
      /* Begin maple */
      double t1 = cos(yaw);
      double t2 = cos(pitch);
      double t3 = t1 * t2;
      double t6 = sin(yaw);
      double t7 = t6 * t2;
      double t10 = sin(pitch);
      double t13 = t1 * t10;
      double t14 = cos(roll);
      double t16 = sin(roll);
      double t19 = t13 * t14 + 0.10e1 * t6 * t16;
      double t21 = t6 * t10;
      double t25 = t21 * t14 - 0.10e1 * t1 * t16;
      double t27 = t2 * t14;
      double t30 = t19 * x + t25 * y + 0.10e1 * t27 * z;
      double t34 = t13 * t16 - 0.10e1 * t6 * t14;
      double t40 = t21 * t16 + 0.10e1 * t1 * t14;
      double t42 = t2 * t16;
      double t45 = -t34 * x - t40 * y - 0.10e1 * t42 * z;
      double t75 = 0.10e1 * t3 * x + 0.10e1 * t7 * y - 0.100e1 * t10 * z;
      n1Res = 0.10e1 * t3 * n1 + 0.10e1 * t7 * n2 - 0.100e1 * t10 * n3 + (t30 * t34 + t45 * t19) * u1 + (t30 * t40 + t45 * t25) * u2 + (0.10e1 * t30 * t2 * t16 + 0.10e1 * t45 * t2 * t14) * u3;
      n2Res = t34 * n1 + t40 * n2 + 0.10e1 * t42 * n3 + (-0.10e1 * t30 * t1 * t2 + t75 * t19) * u1 + (-0.10e1 * t30 * t6 * t2 + t75 * t25) * u2 + (0.100e1 * t30 * t10 + 0.10e1 * t75 * t2 * t14) * u3;
      n3Res = t19 * n1 + t25 * n2 + 0.10e1 * t27 * n3 + (-0.10e1 * t45 * t1 * t2 - t75 * t34) * u1 + (-0.10e1 * t45 * t6 * t2 - t75 * t40) * u2 + (0.100e1 * t45 * t10 - 0.10e1 * t75 * t2 * t16) * u3;
      u1Res = 0.10e1 * t3 * u1 + 0.10e1 * t7 * u2 - 0.100e1 * t10 * u3;
      u2Res = t34 * u1 + t40 * u2 + 0.10e1 * t42 * u3;
      u3Res = t19 * u1 + t25 * u2 + 0.10e1 * t27 * u3;
      /* End maple */
    }
			

    void lineToFrameJac(double x, double y, double z,
			double yaw, double pitch, double roll,
			double n1, double n2, double n3, double u1, double u2, double u3,
			jblas::mat& Jframe, jblas::mat& Jl)
    {
      JFR_PRECOND(Jframe.size1() == 6 && Jframe.size2() == 6,
		  "EulerTools::lineToFrameJac: size of Jframe does not match");
      JFR_PRECOND(Jl.size1() == 6 && Jl.size2() == 6,
		  "EulerTools::lineToFrameJac: size of Jl does not match");  
      /* Begin maple */
      double t1 = cos(yaw);
      double t2 = sin(pitch);
      double t3 = t1 * t2;
      double t4 = sin(roll);
      double t6 = sin(yaw);
      double t7 = cos(roll);
      double t10 = t3 * t4 - 0.10e1 * t6 * t7;
      double t14 = t3 * t7 + 0.10e1 * t6 * t4;
      double t19 = t6 * t2;
      double t23 = t19 * t4 + 0.10e1 * t1 * t7;
      double t28 = t19 * t7 - 0.10e1 * t1 * t4;
      double t32 = cos(pitch);
      double t35 = 0.10e1 * t14 * t32 * t4;
      double t52 = 0.10e1 * t28 * t32 * t4;
      double t59 = t32 * t7;
      double t69 = t6 * t32;
      double t72 = t1 * t32;
      double t77 = -t28 * x + y * t14;
      double t83 = t14 * x + t28 * y + 0.10e1 * t59 * z;
      double t87 = x * t23 - y * t10;
      double t91 = t32 * t4;
      double t93 = 0.10e1 * t91 * z;
      double t94 = -t10 * x - t23 * y - t93;
      double t99 = t83 * t10;
      double t101 = t94 * t14;
      double t123 = t2 * t7;
      double t126 = t72 * t7 * x + t69 * t7 * y - 0.10e1 * t123 * z;
      double t128 = t83 * t1;
      double t134 = t2 * t4;
      double t137 = -t72 * t4 * x - t69 * t4 * y + 0.10e1 * t134 * z;
      double t144 = t83 * t6;
      double t154 = t83 * t2;
      double t168 = -t10 * x - t23 * y - t93;
      double t184 = t83 * t32;
      double t187 = -t83 * t32;
      double t190 = t94 * t32;
      double t196 = 0.10e1 * t72;
      double t197 = 0.10e1 * t69;
      double t198 = 0.100e1 * t2;
      double t224 = t32 * t32;
      double t227 = 0.100e1 * t1 * t224 * t7;
      double t249 = 0.100e1 * t6 * t224 * t7;
      double t267 = -t83 * t6;
      double t269 = 0.10e1 * t267 * t32;
      double t276 = -0.10e1 * x * t6 * t32 + 0.10e1 * y * t1 * t32;
      double t284 = 0.10e1 * t72 * x + 0.10e1 * t69 * y - 0.100e1 * t2 * z;
      double t285 = -t284 * t28;
      double t291 = -t83 * t1;
      double t293 = 0.10e1 * t291 * t32;
      double t295 = t284 * t14;
      double t323 = -0.10e1 * t3 * x - 0.10e1 * t19 * y - 0.100e1 * t32 * z;
      double t358 = -t284 * t10;
      double t364 = -t284 * t23;
      double t369 = t284 * t32;
      double t375 = 0.10e1 * t91;
      double t389 = 0.10e1 * t6 * t10 * t32 - 0.10e1 * t72 * t23;
      double t393 = t224 * t4;
      double t396 = -0.100e1 * t2 * t10 - 0.100e1 * t393 * t1;
      double t404 = -0.100e1 * t2 * t23 - 0.100e1 * t393 * t6;
      double t415 = -t94 * t6;
      double t417 = 0.10e1 * t415 * t32;
      double t425 = -t94 * t1;
      double t427 = 0.10e1 * t425 * t32;
      double t492 = -t284 * t32;
      double t498 = 0.10e1 * t59;
      Jframe(0,0) = (t14 * t23 - t10 * t28) * u2 + (t35 - 0.10e1 * t10 * t32 * t7) * u3;
      Jframe(0,1) = (t10 * t28 - t14 * t23) * u1 + (t52 - 0.10e1 * t23 * t32 * t7) * u3;
      Jframe(0,2) = (0.10e1 * t59 * t10 - t35) * u1 + (0.10e1 * t59 * t23 - t52) * u2;
      Jframe(0,3) = -0.10e1 * t69 * n1 + 0.10e1 * t72 * n2 + (t77 * t10 - t83 * t23 + t87 * t14 - t94 * t28) * u1 + (t77 * t23 + t99 + t87 * t28 + t101) * u2 + (0.10e1 * t77 * t32 * t4 + 0.10e1 * t87 * t32 * t7) * u3;
      Jframe(0,4) = -0.10e1 * t3 * n1 - 0.10e1 * t19 * n2 - 0.100e1 * t32 * n3 + (t126 * t10 + t128 * t91 + t137 * t14 + t94 * t1 * t59) * u1 + (t126 * t23 + t144 * t91 + t137 * t28 + t94 * t6 * t59) * u2 + (0.10e1 * t126 * t32 * t4 - 0.10e1 * t154 * t4 + 0.10e1 * t137 * t32 * t7 - 0.10e1 * t94 * t2 * t7) * u3;
      Jframe(0,5) = (t168 * t10 - t94 * t10) * u1 + (t168 * t23 - t94 * t23) * u2 + (0.10e1 * t168 * t32 * t4 + 0.10e1 * t184 * t7 + 0.10e1 * t187 * t7 - 0.10e1 * t190 * t4) * u3;
      Jl(0,0) = t196;
      Jl(0,1) = t197;
      Jl(0,2) = -t198;
      Jl(0,3) = t99 + t101;
      Jl(0,4) = t83 * t23 + t94 * t28;
      Jl(0,5) = 0.10e1 * t184 * t4 + 0.10e1 * t190 * t7;
      Jframe(1,0) = (-0.10e1 * t1 * t14 * t32 + 0.10e1 * t72 * t14) * u1 + (-0.10e1 * t6 * t14 * t32 + 0.10e1 * t72 * t28) * u2 + (0.100e1 * t2 * t14 + t227) * u3;
      Jframe(1,1) = (-0.10e1 * t28 * t1 * t32 + 0.10e1 * t69 * t14) * u1 + (-0.10e1 * t28 * t6 * t32 + 0.10e1 * t69 * t28) * u2 + (0.100e1 * t28 * t2 + t249) * u3;
      Jframe(1,2) = (-t227 - 0.100e1 * t2 * t14) * u1 + (-t249 - 0.100e1 * t28 * t2) * u2;
      Jframe(1,3) = -t23 * n1 + t10 * n2 + (-0.10e1 * t77 * t1 * t32 - t269 + t276 * t14 + t285) * u1 + (-0.10e1 * t77 * t6 * t32 + t293 + t276 * t28 + t295) * u2 + (0.100e1 * t77 * t2 + 0.10e1 * t276 * t32 * t7) * u3;
      Jframe(1,4) = t72 * t4 * n1 + t69 * t4 * n2 - 0.10e1 * t134 * n3 + (-0.10e1 * t126 * t1 * t32 - 0.10e1 * t291 * t2 + t323 * t14 + t284 * t1 * t59) * u1 + (-0.10e1 * t126 * t6 * t32 - 0.10e1 * t267 * t2 + t323 * t28 + t284 * t6 * t59) * u2 + (0.100e1 * t126 * t2 - 0.100e1 * t187 + 0.10e1 * t323 * t32 * t7 - 0.10e1 * t284 * t2 * t7) * u3;
      Jframe(1,5) = t14 * n1 + t28 * n2 + 0.10e1 * t59 * n3 + (-0.10e1 * t168 * t1 * t32 + t358) * u1 + (-0.10e1 * t168 * t6 * t32 + t364) * u2 + (0.100e1 * t168 * t2 - 0.10e1 * t369 * t4) * u3;
      Jl(1,0) = t10;
      Jl(1,1) = t23;
      Jl(1,2) = t375;
      Jl(1,3) = t293 + t295;
      Jl(1,4) = t269 + t284 * t28;
      Jl(1,5) = 0.100e1 * t83 * t2 + 0.10e1 * t369 * t7;
      Jframe(2,0) = t389 * u2 + t396 * u3;
      Jframe(2,1) = -t389 * u1 + t404 * u3;
      Jframe(2,2) = -t396 * u1 - t404 * u2;
      Jframe(2,3) = -t28 * n1 + t14 * n2 + (-0.10e1 * t87 * t1 * t32 - t417 - t276 * t10 + t284 * t23) * u1 + (-0.10e1 * t87 * t6 * t32 + t427 - t276 * t23 + t358) * u2 + (0.100e1 * t87 * t2 - 0.10e1 * t276 * t32 * t4) * u3;
      Jframe(2,4) = t72 * t7 * n1 + t69 * t7 * n2 - 0.10e1 * t123 * n3 + (-0.10e1 * t137 * t1 * t32 - 0.10e1 * t425 * t2 - t323 * t10 - t284 * t1 * t91) * u1 + (-0.10e1 * t137 * t6 * t32 - 0.10e1 * t415 * t2 - t323 * t23 - t284 * t6 * t91) * u2 + (0.100e1 * t137 * t2 + 0.100e1 * t94 * t32 - 0.10e1 * t323 * t32 * t4 + 0.10e1 * t284 * t2 * t4) * u3;
      Jframe(2,5) = -t10 * n1 - t23 * n2 - 0.10e1 * t91 * n3 + (0.10e1 * t128 * t32 - t284 * t14) * u1 + (0.10e1 * t144 * t32 + t285) * u2 + (-0.100e1 * t154 + 0.10e1 * t492 * t7) * u3;
      Jl(2,0) = t14;
      Jl(2,1) = t28;
      Jl(2,2) = t498;
      Jl(2,3) = t427 + t358;
      Jl(2,4) = t417 + t364;
      Jl(2,5) = 0.100e1 * t94 * t2 + 0.10e1 * t492 * t4;
      Jframe(3,0) = 0.0e0;
      Jframe(3,1) = 0.0e0;
      Jframe(3,2) = 0.0e0;
      Jframe(3,3) = -0.10e1 * t69 * u1 + 0.10e1 * t72 * u2;
      Jframe(3,4) = -0.10e1 * t3 * u1 - 0.10e1 * t19 * u2 - 0.100e1 * t32 * u3;
      Jframe(3,5) = 0.0e0;
      Jl(3,0) = 0.0e0;
      Jl(3,1) = 0.0e0;
      Jl(3,2) = 0.0e0;
      Jl(3,3) = t196;
      Jl(3,4) = t197;
      Jl(3,5) = -t198;
      Jframe(4,0) = 0.0e0;
      Jframe(4,1) = 0.0e0;
      Jframe(4,2) = 0.0e0;
      Jframe(4,3) = -t23 * u1 + t10 * u2;
      Jframe(4,4) = t72 * t4 * u1 + t69 * t4 * u2 - 0.10e1 * t134 * u3;
      Jframe(4,5) = t14 * u1 + t28 * u2 + 0.10e1 * t59 * u3;
      Jl(4,0) = 0.0e0;
      Jl(4,1) = 0.0e0;
      Jl(4,2) = 0.0e0;
      Jl(4,3) = t10;
      Jl(4,4) = t23;
      Jl(4,5) = t375;
      Jframe(5,0) = 0.0e0;
      Jframe(5,1) = 0.0e0;
      Jframe(5,2) = 0.0e0;
      Jframe(5,3) = -t28 * u1 + t14 * u2;
      Jframe(5,4) = t72 * t7 * u1 + t69 * t7 * u2 - 0.10e1 * t123 * u3;
      Jframe(5,5) = -t10 * u1 - t23 * u2 - 0.10e1 * t91 * u3;
      Jl(5,0) = 0.0e0;
      Jl(5,1) = 0.0e0;
      Jl(5,2) = 0.0e0;
      Jl(5,3) = t14;
      Jl(5,4) = t28;
      Jl(5,5) = t498;
      /* End maple */
    }

    void lineFromFrame(double x, double y, double z, 
		       double yaw, double pitch, double roll,
		       double n1, double n2, double n3, double u1, double u2, double u3,
		       double& n1Res, double& n2Res, double& n3Res, double& u1Res, double& u2Res, double& u3Res)
    {
      /* Begin maple */
      double t1 = cos(yaw);
      double t2 = cos(pitch);
      double t3 = t1 * t2;
      double t6 = sin(pitch);
      double t7 = t1 * t6;
      double t8 = sin(roll);
      double t10 = sin(yaw);
      double t11 = cos(roll);
      double t14 = t7 * t8 - 0.10e1 * t10 * t11;
      double t19 = t7 * t11 + 0.10e1 * t10 * t8;
      double t28 = t10 * t6;
      double t32 = t28 * t8 + 0.10e1 * t1 * t11;
      double t34 = y * t2;
      double t42 = t28 * t11 - 0.10e1 * t1 * t8;
      double t49 = t10 * t2;
      double t62 = x * t2;
      double t75 = t2 * t8;
      double t78 = t2 * t11;
      n1Res = 0.10e1 * t3 * n1 + t14 * n2 + t19 * n3 + (-0.10e1 * z * t10 * t2 - 0.100e1 * y * t6) * u1 + (-z * t32 + 0.10e1 * t34 * t8) * u2 + (-z * t42 + 0.10e1 * t34 * t11) * u3;
      n2Res = 0.10e1 * t49 * n1 + t32 * n2 + t42 * n3 + (0.10e1 * z * t1 * t2 + 0.100e1 * x * t6) * u1 + (z * t14 - 0.10e1 * t62 * t8) * u2 + (z * t19 - 0.10e1 * t62 * t11) * u3;
      n3Res = -0.100e1 * t6 * n1 + 0.10e1 * t75 * n2 + 0.10e1 * t78 * n3 + (-0.10e1 * y * t1 * t2 + 0.10e1 * x * t10 * t2) * u1 + (-y * t14 + x * t32) * u2 + (-y * t19 + x * t42) * u3;
      u1Res = 0.10e1 * t3 * u1 + t14 * u2 + t19 * u3;
      u2Res = 0.10e1 * t49 * u1 + t32 * u2 + t42 * u3;
      u3Res = -0.100e1 * t6 * u1 + 0.10e1 * t75 * u2 + 0.10e1 * t78 * u3;
      /* End maple */
    }
	
    void lineFromFrameJac(double x, double y, double z,
			  double yaw, double pitch, double roll,
			  double n1, double n2, double n3, double u1, double u2, double u3,
			  jblas::mat& Jframe, jblas::mat& Jl)
    {
      JFR_PRECOND(Jframe.size1() == 6 && Jframe.size2() == 6,
		  "EulerTools::lineFromFrameJac: size of Jframe does not match");
      JFR_PRECOND(Jl.size1() == 6 && Jl.size2() == 6,
		  "EulerTools::lineFromFrameJac: size of Jl does not match");  

      /* Begin maple */    
      double t1 = sin(pitch);
      double t4 = cos(pitch);
      double t5 = sin(roll);
      double t6 = t4 * t5;
      double t9 = cos(roll);
      double t10 = t4 * t9;
      double t13 = -0.100e1 * t1 * u1 + 0.10e1 * t6 * u2 + 0.10e1 * t10 * u3;
      double t14 = sin(yaw);
      double t15 = t14 * t4;
      double t17 = 0.10e1 * t15 * u1;
      double t18 = t14 * t1;
      double t20 = cos(yaw);
      double t23 = -t18 * t5 - 0.10e1 * t20 * t9;
      double t28 = -t18 * t9 + 0.10e1 * t20 * t5;
      double t30 = -t17 + t23 * u2 + t28 * u3;
      double t35 = z * t20;
      double t36 = t4 * u1;
      double t39 = t20 * t1;
      double t43 = t39 * t5 - 0.10e1 * t14 * t9;
      double t44 = z * t43;
      double t49 = t39 * t9 + 0.10e1 * t14 * t5;
      double t50 = z * t49;
      double t55 = t20 * t4;
      double t56 = t5 * n2;
      double t58 = t9 * n3;
      double t60 = z * t14;
      double t63 = y * t4;
      double t68 = y * t1;
      double t84 = z * t28 + 0.10e1 * t63 * t9;
      double t86 = z * t23;
      double t88 = 0.10e1 * t63 * t5;
      double t92 = 0.10e1 * t55;
      double t100 = 0.10e1 * t55 * u1;
      double t103 = t100 + t43 * u2 + t49 * u3;
      double t120 = x * t4;
      double t125 = x * t1;
      double t140 = t50 - 0.10e1 * t120 * t9;
      double t144 = 0.10e1 * t120 * t5;
      double t148 = 0.10e1 * t15;
      double t177 = t1 * t5;
      double t180 = t1 * t9;
      double t183 = y * t20;
      double t186 = x * t14;
      double t206 = -y * t49 - x * t28;
      double t213 = 0.100e1 * t1;
      double t214 = 0.10e1 * t6;
      double t215 = 0.10e1 * t10;
      double t226 = t5 * u2;
      double t228 = t9 * u3;
       Jframe(0,0) = 0.0e0;
       Jframe(0,1) = t13;
       Jframe(0,2) = t30;
       Jframe(0,3) = -0.10e1 * t15 * n1 + t23 * n2 + t28 * n3 - 0.10e1 * t35 * t36 - t44 * u2 - t50 * u3;
       Jframe(0,4) = -0.10e1 * t39 * n1 + t55 * t56 + t55 * t58 + (0.10e1 * t60 * t1 - 0.100e1 * t63) * u1 + (-t60 * t6 - 0.10e1 * t68 * t5) * u2 + (-t60 * t10 - 0.10e1 * t68 * t9) * u3;
       Jframe(0,5) = t49 * n2 - t43 * n3 + t84 * u2 + (-t86 - t88) * u3;
      Jl(0,0) = t92;
      Jl(0,1) = t43;
      Jl(0,2) = t49;
      Jl(0,3) = -0.10e1 * t60 * t4 - 0.100e1 * t68;
      Jl(0,4) = z * t23 + t88;
      Jl(0,5) = t84;
       Jframe(1,0) = -t13;
       Jframe(1,1) = 0.0e0;
       Jframe(1,2) = t103;
       Jframe(1,3) = 0.10e1 * t55 * n1 + t43 * n2 + t49 * n3 - 0.10e1 * t60 * t36 + t86 * u2 + z * t28 * u3;
       Jframe(1,4) = -0.10e1 * t18 * n1 + t15 * t56 + t15 * t58 + (-0.10e1 * t35 * t1 + 0.100e1 * t120) * u1 + (t35 * t6 + 0.10e1 * t125 * t5) * u2 + (t35 * t10 + 0.10e1 * t125 * t9) * u3;
       Jframe(1,5) = -t28 * n2 + t23 * n3 + t140 * u2 + (-z * t43 + t144) * u3;
      Jl(1,0) = t148;
      Jl(1,1) = -t23;
      Jl(1,2) = -t28;
      Jl(1,3) = 0.10e1 * t35 * t4 + 0.100e1 * t125;
      Jl(1,4) = t44 - t144;
      Jl(1,5) = t140;
       Jframe(2,0) = t17 - t23 * u2 - t28 * u3;
       Jframe(2,1) = -t100 - t43 * u2 - t49 * u3;
       Jframe(2,2) = 0.0e0;
       Jframe(2,3) = (0.10e1 * t15 * y + 0.10e1 * t55 * x) * u1 + (-y * t23 + t43 * x) * u2 + (-y * t28 + t49 * x) * u3;
       Jframe(2,4) = -0.100e1 * t4 * n1 - 0.10e1 * t177 * n2 - 0.10e1 * t180 * n3 + (0.10e1 * t183 * t1 - 0.10e1 * t186 * t1) * u1 + (-t183 * t6 + t186 * t6) * u2 + (-t183 * t10 + t186 * t10) * u3;
       Jframe(2,5) = 0.10e1 * t10 * n2 - 0.10e1 * t6 * n3 + t206 * u2 + (y * t43 + x * t23) * u3;
      Jl(2,0) = -t213;
      Jl(2,1) = t214;
      Jl(2,2) = t215;
      Jl(2,3) = -0.10e1 * t183 * t4 + 0.10e1 * t186 * t4;
      Jl(2,4) = -y * t43 - x * t23;
      Jl(2,5) = t206;
       Jframe(3,0) = 0.0e0;
       Jframe(3,1) = 0.0e0;
       Jframe(3,2) = 0.0e0;
       Jframe(3,3) = t30;
       Jframe(3,4) = -0.10e1 * t39 * u1 + t55 * t226 + t55 * t228;
       Jframe(3,5) = t49 * u2 - t43 * u3;
      Jl(3,0) = 0.0e0;
      Jl(3,1) = 0.0e0;
      Jl(3,2) = 0.0e0;
      Jl(3,3) = t92;
      Jl(3,4) = t43;
      Jl(3,5) = t49;
       Jframe(4,0) = 0.0e0;
       Jframe(4,1) = 0.0e0;
       Jframe(4,2) = 0.0e0;
       Jframe(4,3) = t103;
       Jframe(4,4) = -0.10e1 * t18 * u1 + t15 * t226 + t15 * t228;
       Jframe(4,5) = -t28 * u2 + t23 * u3;
      Jl(4,0) = 0.0e0;
      Jl(4,1) = 0.0e0;
      Jl(4,2) = 0.0e0;
      Jl(4,3) = t148;
      Jl(4,4) = -t23;
      Jl(4,5) = -t28;
       Jframe(5,0) = 0.0e0;
       Jframe(5,1) = 0.0e0;
       Jframe(5,2) = 0.0e0;
       Jframe(5,3) = 0.0e0;
       Jframe(5,4) = -0.100e1 * t36 - 0.10e1 * t177 * u2 - 0.10e1 * t180 * u3;
       Jframe(5,5) = 0.10e1 * t10 * u2 - 0.10e1 * t6 * u3;
      Jl(5,0) = 0.0e0;
      Jl(5,1) = 0.0e0;
      Jl(5,2) = 0.0e0;
      Jl(5,3) = -t213;
      Jl(5,4) = t214;
      Jl(5,5) = t215;
      /* End maple */
    }

    void planeToFrame(double x, double y, double z,
           double yaw, double pitch, double roll, 
           double px, double py, double pz, double pw, 
           double& px_res, double& py_res, double& pz_res, double& pw_res)
    {
        px_res = cos(pitch)*cos(yaw)*px+cos(pitch)*sin(yaw)*py-sin(pitch)*pz;
        py_res = (-cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw))*px+(cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw))*py+sin(roll)*cos(pitch)*pz;
        pz_res = (sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw))*px+(-sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw))*py+cos(roll)*cos(pitch)*pz;
        pw_res =  x*px+y*py+z*pz+pw;
    }

     void planeToFrameJac(double x, double y, double z,
           double yaw, double pitch, double roll, 
           double px, double py, double pz, double pw, 
           jblas::mat& Jframe, jblas::mat& Jp)
    {
 //double& px_res, double& py_res, double& pz_res, double& pw_res, 
        Jframe(0,0)= cos(pitch)*cos(yaw);
        Jframe(0,1)= cos(pitch)*sin(yaw);
        Jframe(0,2)= -sin(pitch);
        Jframe(0,3)= 0;
        Jframe(1,0)= -cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw);
        Jframe(1,1)= cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw);
        Jframe(1,2)= sin(roll)*cos(pitch);
        Jframe(1,3)= 0;
        Jframe(2,0)= sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
        Jframe(2,1)= -sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw);
        Jframe(2,2)= cos(roll)*cos(pitch);
        Jframe(2,3)= 0;
        Jframe(3,0)= x;
        Jframe(3,1)= y;
        Jframe(3,2)= z;
        Jframe(3,3)= 1;


        Jp(0,0)= 0;
        Jp(0,1)= 0;
        Jp(0,2)= 0;
        Jp(1,0)= 0;
        Jp(1,1)= 0;
        Jp(1,2)= 0;
        Jp(2,0)= 0;
        Jp(2,1)= 0;
        Jp(2,2)= 0;
        Jp(3,0)= px;
        Jp(3,1)= py;
        Jp(3,2)= pz;


        Jp(0,3) = 0;
        Jp(0,4) = -sin(pitch)*cos(yaw)*px-sin(pitch)*sin(yaw)*py-cos(pitch)*pz;
        Jp(0,5) = -cos(pitch)*sin(yaw)*px+cos(pitch)*cos(yaw)*py;
        Jp(1,3) = (sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw))*px+(-sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw))*py+cos(roll)*cos(pitch)*pz;
        Jp(1,4) = sin(roll)*cos(pitch)*cos(yaw)*px+sin(roll)*cos(pitch)*sin(yaw)*py-sin(roll)*sin(pitch)*pz;
        Jp(1,5) = (-cos(roll)*cos(yaw)-sin(roll)*sin(pitch)*sin(yaw))*px+(-cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw))*py;
        Jp(2,3) = (cos(roll)*sin(yaw)-sin(roll)*sin(pitch)*cos(yaw))*px+(-cos(roll)*cos(yaw)-sin(roll)*sin(pitch)*sin(yaw))*py-sin(roll)*cos(pitch)*pz;
        Jp(2,4) = cos(roll)*cos(pitch)*cos(yaw)*px+cos(roll)*cos(pitch)*sin(yaw)*py-cos(roll)*sin(pitch)*pz;
        Jp(2,5) = (sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw))*px+(sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw))*py;
        Jp(3,3) = 0;
        Jp(3,4) = 0;
        Jp(3,5) = 0;
    }

  void planeFromFrame(double x, double y, double z,
           double yaw, double pitch, double roll, 
           double px, double py, double pz, double pw, 
           double& px_res, double& py_res, double& pz_res, double& pw_res)
    {
        px_res = cos(pitch)*cos(yaw)*px+(-cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw))*py+(sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw))*pz;
        py_res = cos(pitch)*sin(yaw)*px+(cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw))*py+(-sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw))*pz;
        pz_res = -sin(pitch)*px+sin(roll)*cos(pitch)*py+cos(roll)*cos(pitch)*pz;
        pw_res = (-x*cos(pitch)*cos(yaw)-y*cos(pitch)*sin(yaw)+z*sin(pitch))*px+(-x*(-cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw))-y*(cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw))-z*sin(roll)*cos(pitch))*py+(-x*(sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw))-y*(-sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw))-z*cos(roll)*cos(pitch))*pz+pw;
    }

     void planeFromFrameJac(double x, double y, double z,
           double yaw, double pitch, double roll, 
           double px, double py, double pz, double pw, 
           jblas::mat& Jframe, jblas::mat& Jp)
    {            //double& px_res, double& py_res, double& pz_res, double& pw_res, 
        Jframe(0,0)= cos(pitch)*cos(yaw);
        Jframe(0,1)= -cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw);
        Jframe(0,2)= sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
        Jframe(0,3)= 0;
        Jframe(1,0)= cos(pitch)*sin(yaw);
        Jframe(1,1)= cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw);
        Jframe(1,2)= -sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw);
        Jframe(1,3)= 0;
        Jframe(2,0)= -sin(pitch);
        Jframe(2,1)= sin(roll)*cos(pitch);
        Jframe(2,2)= cos(roll)*cos(pitch);
        Jframe(2,3)= 0;
        Jframe(3,0)= -x*cos(pitch)*cos(yaw)-y*cos(pitch)*sin(yaw)+z*sin(pitch);
        Jframe(3,1)= -x*(-cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw))-y*(cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw))-z*sin(roll)*cos(pitch);
        Jframe(3,2)= -x*(sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw))-y*(-sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw))-z*cos(roll)*cos(pitch);
        Jframe(3,3)= 1;

//         vec p(4);
//         vec p_res(4);
//         p.assign(px,py,pz,pw);
//         p_res.assign(product(Jframe,p));
        double s1 = sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
        double s2 = cos(roll)*sin(pitch)*sin(yaw);
        double s3 = sin(roll)*cos(yaw);
        double s4 = sin(roll)*sin(pitch)*cos(yaw);
        double s5 = cos(roll)*sin(yaw);
        double s6 = -cos(roll)*cos(yaw)-sin(roll)*sin(pitch)*sin(yaw);

        Jp(0,0)= 0;
        Jp(0,1)= 0;
        Jp(0,2)= 0;
        Jp(1,0)= 0;
        Jp(1,1)= 0;
        Jp(1,2)= 0;
        Jp(2,0)= 0;
        Jp(2,1)= 0;
        Jp(2,2)= 0;
        Jp(3,0)= -cos(pitch)*cos(yaw)*px+py*cos(roll)*sin(yaw)-py*sin(roll)*sin(pitch)*cos(yaw)-pz*sin(roll)*sin(yaw)-pz*cos(roll)*sin(pitch)*cos(yaw);
        Jp(3,1)= -cos(pitch)*sin(yaw)*px-py*cos(roll)*cos(yaw)-py*sin(roll)*sin(pitch)*sin(yaw)+pz*sin(roll)*cos(yaw)-pz*cos(roll)*sin(pitch)*sin(yaw);
        Jp(3,2)= sin(pitch)*px-sin(roll)*cos(pitch)*py-cos(roll)*cos(pitch)*pz;

        Jp(0,3)= s1*py+pz*s5-pz*s4;
        Jp(0,4)= cos(yaw)*(-sin(pitch)*px+sin(roll)*cos(pitch)*py+cos(roll)*cos(pitch)*pz);
        Jp(0,5)= -cos(pitch)*sin(yaw)*px+s6*py+pz*s3-pz*s2;
        Jp(1,3)= -py*s3+py*s2+s6*pz;
        Jp(1,4)= sin(yaw)*(-sin(pitch)*px+sin(roll)*cos(pitch)*py+cos(roll)*cos(pitch)*pz);
        Jp(1,5)= cos(pitch)*cos(yaw)*px-py*s5+py*s4+s1*pz;
        Jp(2,3)= -cos(pitch)*(-cos(roll)*py+sin(roll)*pz);
        Jp(2,4)= -cos(pitch)*px-sin(roll)*sin(pitch)*py-cos(roll)*sin(pitch)*pz;
        Jp(2,5)= 0;
        Jp(3,3)= -py*x*s1+py*y*s3-py*y*s2-py*z*cos(roll)*cos(pitch)-pz*x*s5+pz*x*s4-pz*y*s6+pz*z*sin(roll)*cos(pitch);
        Jp(3,4)= px*x*sin(pitch)*cos(yaw)+px*y*sin(pitch)*sin(yaw)+px*z*cos(pitch)-py*x*sin(roll)*cos(pitch)*cos(yaw)-py*y*sin(roll)*cos(pitch)*sin(yaw)+py*z*sin(roll)*sin(pitch)-pz*x*cos(roll)*cos(pitch)*cos(yaw)-pz*y*cos(roll)*cos(pitch)*sin(yaw)+pz*z*cos(roll)*sin(pitch);
        Jp(3,5)= px*x*cos(pitch)*sin(yaw)-px*y*cos(pitch)*cos(yaw)-py*x*s6+py*y*s5-py*y*s4-pz*x*s3+pz*x*s2-pz*y*s1;
    }


    void odo3d(double x, double y, double z,
	       double yaw, double pitch, double roll,
	       double v, double w, double dt,
	       double& x_res, double& y_res, double& z_res,
	       double& yaw_res, double& pitch_res, double& roll_res)
    {
      /* Begin maple */    
      double t1 = cos(yaw);
      double t2 = cos(pitch);
      double t3 = t1 * t2;
      double t4 = dt * v;
      double t5 = dt * w;
      double t6 = t5 / 0.2e1;
      double t7 = cos(t6);
      double t8 = t4 * t7;
      double t11 = sin(pitch);
      double t13 = sin(roll);
      double t15 = sin(yaw);
      double t16 = cos(roll);
      double t19 = t1 * t11 * t13 - 0.10e1 * t15 * t16;
      double t21 = sin(t6);
      double t22 = v * t21;
      double t26 = t15 * t2;
      double t33 = t15 * t11 * t13 + 0.10e1 * t1 * t16;
      double t42 = t2 * t13;
      double t48 = cos(t5);
      double t51 = sin(t5);
      double t60 = atan2(0.10e1 * t26 * t48 + 0.1e1 * t33 * t51, 0.10e1 * t3 * t48 + 0.1e1 * t19 * t51);
      double t66 = asin(-0.100e1 * t11 * t48 + 0.10e1 * t42 * t51);
      double t74 = atan2(0.100e1 * t11 * t51 + 0.10e1 * t42 * t48, 0.100e1 * t2 * t16);
      x_res = 0.10e1 * t3 * t8 + t19 * dt * t22 + 0.10e1 * x;
      y_res = 0.10e1 * t26 * t8 + t33 * dt * t22 + 0.10e1 * y;
      z_res = -0.100e1 * t11 * dt * v * t7 + 0.10e1 * t42 * t4 * t21 + 0.10e1 * z;
      yaw_res = t60;
      pitch_res = -t66;
      roll_res = t74; 
      /* End maple */
    }

    void odo3dJac(double x, double y, double z,
		  double yaw, double pitch, double roll,
		  double v, double w, double dt,
		  jblas::mat& Jx, jblas::mat& Ju)
    {
      JFR_PRECOND(Jx.size1()==6 && Jx.size2()==6,
		  "EulerTools::odo3dJac: size of Jx does not match");
      JFR_PRECOND(Ju.size1()==6 && Ju.size2()==2,
		  "EulerTools::odo3dJac: size of Ju does not match"); 

      /* Begin maple */  
      double t1 = sin(yaw);
      double t2 = cos(pitch);
      double t3 = t1 * t2;
      double t4 = dt * v;
      double t5 = dt * w;
      double t6 = t5 / 0.2e1;
      double t7 = cos(t6);
      double t8 = t4 * t7;
      double t11 = sin(pitch);
      double t12 = t1 * t11;
      double t13 = sin(roll);
      double t15 = cos(yaw);
      double t16 = cos(roll);
      double t19 = -t12 * t13 - 0.10e1 * t15 * t16;
      double t21 = sin(t6);
      double t22 = v * t21;
      double t25 = t15 * t11;
      double t28 = t15 * t2;
      double t30 = t4 * t21;
      double t36 = t25 * t16 + 0.10e1 * t1 * t13;
      double t39 = dt * t7;
      double t45 = t25 * t13 - 0.10e1 * t1 * t16;
      double t46 = t45 * dt;
      double t49 = dt * dt;
      double t50 = t49 * v;
      double t51 = t50 * t21;
      double t55 = v * t7;
      double t71 = t12 * t16 - 0.10e1 * t15 * t13;
      double t88 = t11 * t13;
      double t92 = t2 * t16;
      double t98 = t2 * t13;
      double t110 = cos(t5);
      double t112 = 0.10e1 * t3 * t110;
      double t113 = sin(t5);
      double t116 = t112 - 0.1e1 * t19 * t113;
      double t121 = 0.10e1 * t28 * t110 + 0.1e1 * t45 * t113;
      double t122 = t121 * t121;
      double t123 = 0.1e1 / t122;
      double t124 = t116 * t123;
      double t130 = t116 * t116;
      double t133 = 0.1e1 / (0.1e1 + t130 * t123);
      double t137 = t13 * t113;
      double t141 = 0.1e1 / t121;
      double t159 = t113 * dt;
      double t181 = t11 * t110;
      double t186 = pow(-0.100e1 * t181 + 0.10e1 * t98 * t113, 0.2e1);
      double t188 = sqrt(0.1e1 - t186);
      double t189 = 0.1e1 / t188;
      double t194 = t11 * t113;
      double t207 = 0.1e1 / t2;
      double t209 = 0.1e1 / t16;
      double t215 = 0.100e1 * t194 + 0.10e1 * t98 * t110;
      double t216 = t2 * t2;
      double t217 = 0.1e1 / t216;
      double t223 = t215 * t215;
      double t225 = t16 * t16;
      double t226 = 0.1e1 / t225;
      double t230 = 0.1e1 / (0.1e1 + 0.1000000000e1 * t223 * t217 * t226);
      Jx(0,0) = 0.10e1;
      Jx(0,1) = 0.0e0;
      Jx(0,2) = 0.0e0;
      Jx(0,3) = -0.10e1 * t3 * t8 + t19 * dt * t22;
      Jx(0,4) = -0.10e1 * t25 * t8 + t28 * t13 * t30;
      Jx(0,5) = t36 * dt * t22;

      Ju(0,0) = 0.10e1 * t28 * t39 + t46 * t21;
      Ju(0,1) = -0.5000000000e0 * t28 * t51 + t45 * t49 * t55 / 0.2e1;

      Jx(1,0) = 0.0e0;
      Jx(1,1) = 0.10e1;
      Jx(1,2) = 0.0e0;
      Jx(1,3) = 0.10e1 * t28 * t8 + t46 * t22;
      Jx(1,4) = -0.10e1 * t12 * t8 + t3 * t13 * t30;
      Jx(1,5) = t71 * dt * t22;

      Ju(1,0) = 0.10e1 * t3 * t39 - t19 * dt * t21;
      Ju(1,1) = -0.5000000000e0 * t3 * t51 - t19 * t49 * t55 / 0.2e1;

      Jx(2,0) = 0.0e0;
      Jx(2,1) = 0.0e0;
      Jx(2,2) = 0.10e1;
      Jx(2,3) = 0.0e0;
      Jx(2,4) = -0.100e1 * t2 * dt * t55 - 0.10e1 * t88 * t30;
      Jx(2,5) = 0.10e1 * t92 * t30;

      Ju(2,0) = -0.100e1 * t11 * dt * t7 + 0.10e1 * t98 * dt * t21;
      Ju(2,1) = 0.5000000000e0 * t11 * t49 * t22 + 0.5000000000e0 * t98 * t50 * t7;

      Jx(3,0) = 0.0e0;
      Jx(3,1) = 0.0e0;
      Jx(3,2) = 0.0e0;
      Jx(3,3) = (0.1e1 - t124 * (-t112 + 0.1e1 * t19 * t113)) * t133;
      Jx(3,4) = ((-0.10e1 * t12 * t110 + 0.1e1 * t3 * t137) * t141 - t124 * (-0.10e1 * t25 * t110 + 0.1e1 * t28 * t137)) * t133;
      Jx(3,5) = (0.1e1 * t71 * t113 * t141 - 0.1e1 * t124 * t36 * t113) * t133;

      Ju(3,0) = 0.0e0;
      Ju(3,1) = ((-0.10e1 * t3 * t159 - 0.1e1 * t19 * t110 * dt) * t141 - t124 * (-0.10e1 * t28 * t159 + 0.1e1 * t45 * t110 * dt)) * t133;

      Jx(4,0) = 0.0e0;
      Jx(4,1) = 0.0e0;
      Jx(4,2) = 0.0e0;
      Jx(4,3) = 0.0e0;
      Jx(4,4) = -(-0.100e1 * t2 * t110 - 0.10e1 * t88 * t113) * t189;
      Jx(4,5) = -0.10e1 * t92 * t113 * t189;

      Ju(4,0) = 0.0e0;
      Ju(4,1) = -(0.100e1 * t194 * dt + 0.10e1 * t98 * t110 * dt) * t189;

      Jx(5,0) = 0.0e0;
      Jx(5,1) = 0.0e0;
      Jx(5,2) = 0.0e0;
      Jx(5,3) = 0.0e0;
      Jx(5,4) = (0.1000000000e1 * (0.100e1 * t2 * t113 - 0.10e1 * t88 * t110) * t207 * t209 + 0.1000000000e1 * t215 * t217 * t209 * t11) * t230;
      Jx(5,5) = (0.1000000000e1 * t110 + 0.1000000000e1 * t215 * t207 * t226 * t13) * t230;

      Ju(5,0) = 0.0e0;
      Ju(5,1) = 0.1000000000e1 * (0.100e1 * t181 * dt - 0.10e1 * t98 * t159) * t207 * t209 * t230;
      /* End maple */
    }

    void odo3dInRefFrame(//double x_robotToRef, double y_robotToRef, double z_robotToRef,
			 double yaw_robotToRef, 
			 //double pitch_robotToRef, double roll_robotToRef,
			 double x, double y, double z,
			 double yaw, double pitch, double roll,
			 double v, double w,
			 double dt,
			 double& x_res, double& y_res, double& z_res,
			 double& yaw_res, double& pitch_res, double& roll_res)
    {
      /* Begin maple */  
      double t1 = cos(yaw);
      double t2 = cos(pitch);
      double t4 = cos(yaw_robotToRef);
      double t7 = sin(pitch);
      double t9 = sin(roll);
      double t11 = sin(yaw);
      double t12 = cos(roll);
      double t16 = sin(yaw_robotToRef);
      double t19 = 0.10e1 * t1 * t2 * t4 - 0.1e1 * (t1 * t7 * t9 - 0.10e1 * t11 * t12) * t16;
      double t30 = 0.10e1 * t11 * t2 * t4 - 0.1e1 * (t11 * t7 * t9 + 0.10e1 * t1 * t12) * t16;
      double t31 = t30 * t30;
      double t32 = t19 * t19;
      double t34 = sqrt(t31 + t32);
      double t35 = 0.1e1 / t34;
      double t36 = t19 * t35;
      double t37 = t7 * t4;
      double t39 = t2 * t9;
      double t40 = t39 * t16;
      double t42 = 0.100e1 * t37 + 0.10e1 * t40;
      double t43 = t42 * t42;
      double t44 = 0.1e1 - t43;
      double t45 = sqrt(t44);
      double t47 = dt * v;
      double t48 = dt * w;
      double t49 = t48 / 0.2e1;
      double t50 = cos(t49);
      double t51 = t47 * t50;
      double t58 = -0.100e1 * t7 * t16 + 0.10e1 * t39 * t4;
      double t60 = t58 * t58;
      double t61 = t2 * t2;
      double t62 = t12 * t12;
      double t65 = t60 + 0.10000e1 * t61 * t62;
      double t66 = sqrt(t65);
      double t67 = 0.1e1 / t66;
      double t68 = t42 * t58 * t67;
      double t70 = t30 * t35;
      double t71 = t2 * t12;
      double t72 = t71 * t67;
      double t75 = t36 * t68 - 0.1000e1 * t70 * t72;
      double t77 = sin(t49);
      double t78 = v * t77;
      double t89 = t70 * t68 + 0.1000e1 * t36 * t72;
      double t97 = -0.10000e1 * t37 - 0.1000e1 * t40;
      double t102 = t45 * t58;
      double t109 = cos(t48);
      double t110 = t45 * t109;
      double t113 = sin(t48);
      double t116 = 0.10e1 * t70 * t110 + 0.1e1 * t89 * t113;
      double t117 = t116 * t116;
      double t122 = 0.10e1 * t36 * t110 + 0.1e1 * t75 * t113;
      double t123 = t122 * t122;
      double t125 = sqrt(t117 + t123);
      double t126 = 0.1e1 / t125;
      double t127 = t116 * t126;
      double t128 = t97 * t109;
      double t131 = t102 * t67 * t113;
      double t133 = 0.1e1 * t128 + 0.10e1 * t131;
      double t134 = t133 * t133;
      double t136 = sqrt(0.1e1 - t134);
      double t137 = t136 * t4;
      double t145 = -0.1e1 * t97 * t113 + 0.10e1 * t102 * t67 * t109;
      double t147 = t145 * t145;
      double t154 = sqrt(t147 + 0.100000000e1 * t44 * t61 * t62 / t65);
      double t155 = 0.1e1 / t154;
      double t156 = -t133 * t145 * t155;
      double t158 = t122 * t126;
      double t161 = t71 * t67 * t155;
      double t178 = atan2(0.10e1 * t127 * t137 + 0.1e1 * (t127 * t156 + 0.100000e1 * t158 * t45 * t161) * t16, 0.10e1 * t158 * t137 + 0.1e1 * (t158 * t156 - 0.100000e1 * t127 * t45 * t161) * t16);
      double t181 = 0.100e1 * t128 + 0.1000e1 * t131;
      double t184 = t136 * t145;
      double t189 = asin(0.1e1 * t181 * t4 + 0.10e1 * t184 * t155 * t16);
      double t202 = atan2(-0.1e1 * t181 * t16 + 0.10e1 * t184 * t155 * t4, 0.1000000e1 * t136 * t45 * t2 * t12 * t67 * t155);
      x_res = 0.100e1 * t36 * t45 * t51 + 0.10e1 * t75 * dt * t78 + 0.1000e1 * x;
      y_res = 0.100e1 * t70 * t45 * t51 + 0.10e1 * t89 * dt * t78 + 0.1000e1 * y;
      z_res = 0.10e1 * t97 * dt * v * t50 + 0.100e1 * t102 * t67 * t47 * t77 + 0.1000e1 * z;
      yaw_res = t178;
      pitch_res = -t189;
      roll_res = t202;
      /* End maple */
    }
	
    void odo3dInRefFrameJac(// double x_robotToRef, double y_robotToRef, double z_robotToRef,
			    double yaw_robotToRef, 
			    // double pitch_robotToRef, double roll_robotToRef,
			    double x, double y, double z,
			    double yaw, double pitch, double roll,
			    double v, double w,
			    double dt,
			    jblas::mat& Jx, jblas::mat& Ju)
    {
      JFR_PRECOND(Jx.size1()==6 && Jx.size2()==6,
		  "EulerTools::odo3dInRefFrameJac: size of Jx does not match");
      JFR_PRECOND(Ju.size1()==6 && Ju.size2()==2,
		  "EulerTools::odo3dInRefFrameJac: size of Ju does not match"); 

      /* Begin maple */
      double t1 = sin(yaw);
      double t2 = cos(pitch);
      double t3 = t1 * t2;
      double t4 = cos(yaw_robotToRef);
      double t6 = 0.10e1 * t3 * t4;
      double t7 = sin(pitch);
      double t8 = t1 * t7;
      double t9 = sin(roll);
      double t11 = cos(yaw);
      double t12 = cos(roll);
      double t15 = -t8 * t9 - 0.10e1 * t11 * t12;
      double t16 = sin(yaw_robotToRef);
      double t19 = -t6 - 0.1e1 * t15 * t16;
      double t22 = t6 + 0.1e1 * t15 * t16;
      double t23 = t22 * t22;
      double t24 = t11 * t2;
      double t27 = t11 * t7;
      double t34 = 0.10e1 * t24 * t4 - 0.1e1 * (t27 * t9 - 0.10e1 * t1 * t12) * t16;
      double t35 = t34 * t34;
      double t36 = t23 + t35;
      double t37 = sqrt(t36);
      double t38 = 0.1e1 / t37;
      double t39 = t19 * t38;
      double t40 = t7 * t4;
      double t42 = t2 * t9;
      double t43 = t42 * t16;
      double t45 = 0.100e1 * t40 + 0.10e1 * t43;
      double t46 = t45 * t45;
      double t47 = 0.1e1 - t46;
      double t48 = sqrt(t47);
      double t50 = dt * v;
      double t51 = dt * w;
      double t52 = t51 / 0.2e1;
      double t53 = cos(t52);
      double t54 = t50 * t53;
      double t58 = 0.1e1 / t37 / t36;
      double t59 = t34 * t58;
      double t60 = t59 * t48;
      double t63 = t22 * t34 + t34 * t19;
      double t65 = 0.2e1 * t50 * t53 * t63;
      double t72 = -0.100e1 * t7 * t16 + 0.10e1 * t42 * t4;
      double t74 = t72 * t72;
      double t75 = t2 * t2;
      double t76 = t12 * t12;
      double t77 = t75 * t76;
      double t79 = t74 + 0.10000e1 * t77;
      double t80 = sqrt(t79);
      double t81 = 0.1e1 / t80;
      double t82 = t45 * t72 * t81;
      double t84 = t59 * t45;
      double t85 = t72 * t81;
      double t86 = 0.2e1 * t85 * t63;
      double t89 = t34 * t38;
      double t90 = t2 * t12;
      double t91 = t90 * t81;
      double t93 = 0.1000e1 * t89 * t91;
      double t94 = t22 * t58;
      double t95 = t94 * t2;
      double t96 = t12 * t81;
      double t97 = 0.2e1 * t96 * t63;
      double t100 = t39 * t82 - t84 * t86 / 0.2e1 - t93 + 0.5000000000e0 * t95 * t97;
      double t102 = sin(t52);
      double t103 = v * t102;
      double t109 = t9 * t16;
      double t112 = -0.10e1 * t27 * t4 - 0.1e1 * t24 * t109;
      double t113 = t112 * t38;
      double t121 = -0.10e1 * t8 * t4 - 0.1e1 * t3 * t109;
      double t124 = t22 * t121 + t34 * t112;
      double t126 = 0.2e1 * t50 * t53 * t124;
      double t129 = 0.1e1 / t48;
      double t130 = t129 * dt;
      double t132 = v * t53;
      double t133 = t2 * t4;
      double t135 = t7 * t9;
      double t136 = t135 * t16;
      double t138 = 0.100e1 * t133 - 0.10e1 * t136;
      double t139 = t45 * t138;
      double t140 = t132 * t139;
      double t144 = 0.2e1 * t85 * t124;
      double t148 = t138 * t72 * t81;
      double t154 = -0.100e1 * t2 * t16 - 0.10e1 * t135 * t4;
      double t156 = t45 * t154 * t81;
      double t158 = t89 * t45;
      double t160 = 0.1e1 / t80 / t79;
      double t161 = t72 * t160;
      double t167 = 0.2e1 * t72 * t154 - 0.20000e1 * t2 * t76 * t7;
      double t168 = t161 * t167;
      double t171 = t121 * t38;
      double t174 = 0.2e1 * t96 * t124;
      double t177 = t22 * t38;
      double t178 = t7 * t12;
      double t179 = t178 * t81;
      double t182 = t177 * t2;
      double t183 = t12 * t160;
      double t184 = t183 * t167;
      double t187 = t113 * t82 - t84 * t144 / 0.2e1 + t89 * t148 + t89 * t156 - t158 * t168 / 0.2e1 - 0.1000e1 * t171 * t91 + 0.5000000000e0 * t95 * t174 + 0.1000e1 * t177 * t179 + 0.5000000000e0 * t182 * t184;
      double t195 = t27 * t12 + 0.10e1 * t1 * t9;
      double t196 = t195 * t16;
      double t197 = t196 * t38;
      double t198 = t48 * dt;
      double t199 = t198 * t132;
      double t205 = t8 * t12 - 0.10e1 * t11 * t9;
      double t212 = -0.2e1 * t22 * t205 * t16 - 0.2e1 * t34 * t195 * t16;
      double t214 = t50 * t53 * t212;
      double t217 = t130 * v;
      double t220 = t90 * t16;
      double t221 = t53 * t45 * t220;
      double t226 = t85 * t212;
      double t229 = t89 * t2;
      double t230 = t12 * t16;
      double t231 = t230 * t85;
      double t234 = t4 * t81;
      double t235 = t90 * t234;
      double t239 = t12 * t4;
      double t245 = 0.20e1 * t72 * t2 * t239 - 0.20000e1 * t75 * t12 * t9;
      double t246 = t161 * t245;
      double t249 = t205 * t16;
      double t250 = t249 * t38;
      double t253 = t96 * t212;
      double t256 = t42 * t81;
      double t259 = t183 * t245;
      double t262 = -0.1e1 * t197 * t82 - t84 * t226 / 0.2e1 + 0.10e1 * t229 * t231 + 0.10e1 * t158 * t235 - t158 * t246 / 0.2e1 + 0.1000e1 * t250 * t91 + 0.5000000000e0 * t95 * t253 + 0.1000e1 * t177 * t256 + 0.5000000000e0 * t182 * t259;
      double t267 = t198 * t53;
      double t270 = t89 * t82;
      double t273 = t270 - 0.1000e1 * t177 * t91;
      double t278 = t89 * t48;
      double t279 = dt * dt;
      double t280 = t279 * v;
      double t281 = t280 * t102;
      double t290 = t94 * t48;
      double t293 = t94 * t45;
      double t298 = t59 * t2;
      double t301 = t270 - t293 * t86 / 0.2e1 + 0.1000e1 * t39 * t91 - 0.5000000000e0 * t298 * t97;
      double t319 = t177 * t45;
      double t330 = t171 * t82 - t293 * t144 / 0.2e1 + t177 * t148 + t177 * t156 - t319 * t168 / 0.2e1 + 0.1000e1 * t113 * t91 - 0.5000000000e0 * t298 * t174 - 0.1000e1 * t89 * t179 - 0.5000000000e0 * t229 * t184;
      double t360 = -0.1e1 * t250 * t82 - t293 * t226 / 0.2e1 + 0.10e1 * t182 * t231 + 0.10e1 * t319 * t235 - t319 * t246 / 0.2e1 - 0.1000e1 * t197 * t91 - 0.5000000000e0 * t298 * t253 - 0.1000e1 * t89 * t256 - 0.5000000000e0 * t229 * t259;
      double t368 = t177 * t82 + t93;
      double t382 = -0.10000e1 * t133 + 0.1000e1 * t136;
      double t386 = t129 * t72;
      double t387 = t81 * dt;
      double t392 = t48 * t154;
      double t397 = t48 * t72;
      double t398 = t397 * t160;
      double t412 = t48 * t2;
      double t424 = -0.10000e1 * t40 - 0.1000e1 * t43;
      double t440 = cos(t51);
      double t441 = t48 * t440;
      double t443 = 0.10e1 * t89 * t441;
      double t444 = 0.2e1 * t441 * t63;
      double t447 = sin(t51);
      double t450 = t443 - 0.5000000000e0 * t94 * t444 + 0.1e1 * t301 * t447;
      double t455 = 0.10e1 * t177 * t441 + 0.1e1 * t368 * t447;
      double t456 = t455 * t455;
      double t459 = t443 + 0.1e1 * t273 * t447;
      double t460 = t459 * t459;
      double t461 = t456 + t460;
      double t462 = sqrt(t461);
      double t463 = 0.1e1 / t462;
      double t464 = t450 * t463;
      double t465 = t424 * t440;
      double t467 = t81 * t447;
      double t468 = t397 * t467;
      double t470 = 0.1e1 * t465 + 0.10e1 * t468;
      double t471 = t470 * t470;
      double t472 = 0.1e1 - t471;
      double t473 = sqrt(t472);
      double t474 = t473 * t4;
      double t478 = 0.1e1 / t462 / t461;
      double t479 = t455 * t478;
      double t487 = 0.10e1 * t39 * t441 - 0.5000000000e0 * t59 * t444 + 0.1e1 * t100 * t447;
      double t489 = t455 * t450 + t459 * t487;
      double t490 = 0.2e1 * t474 * t489;
      double t493 = t424 * t447;
      double t495 = t81 * t440;
      double t498 = -0.1e1 * t493 + 0.10e1 * t397 * t495;
      double t500 = t498 * t498;
      double t501 = t47 * t75;
      double t502 = 0.1e1 / t79;
      double t503 = t76 * t502;
      double t506 = t500 + 0.100000000e1 * t501 * t503;
      double t507 = sqrt(t506);
      double t508 = 0.1e1 / t507;
      double t509 = -t470 * t498 * t508;
      double t511 = -t479 * t470;
      double t512 = t498 * t508;
      double t513 = 0.2e1 * t512 * t489;
      double t516 = t487 * t463;
      double t518 = t81 * t508;
      double t519 = t90 * t518;
      double t522 = t459 * t478;
      double t523 = t522 * t412;
      double t525 = 0.2e1 * t96 * t508 * t489;
      double t532 = t459 * t463;
      double t536 = t455 * t463;
      double t537 = t536 * t48;
      double t543 = 0.10e1 * t532 * t474 + 0.1e1 * (t532 * t509 - 0.100000e1 * t537 * t519) * t16;
      double t544 = 0.1e1 / t543;
      double t549 = t532 * t48;
      double t555 = 0.10e1 * t536 * t474 + 0.1e1 * (t536 * t509 + 0.100000e1 * t549 * t519) * t16;
      double t556 = t543 * t543;
      double t557 = 0.1e1 / t556;
      double t558 = t555 * t557;
      double t564 = -t522 * t470;
      double t570 = t479 * t412;
      double t579 = t555 * t555;
      double t582 = 0.1e1 / (0.1e1 + t579 * t557);
      double t586 = 0.2e1 * t441 * t124;
      double t591 = t440 * t45 * t138;
      double t596 = 0.10e1 * t171 * t441 - 0.5000000000e0 * t94 * t586 - 0.10e1 * t177 * t129 * t591 + 0.1e1 * t330 * t447;
      double t597 = t596 * t463;
      double t610 = 0.10e1 * t113 * t441 - 0.5000000000e0 * t59 * t586 - 0.10e1 * t89 * t129 * t591 + 0.1e1 * t187 * t447;
      double t612 = t455 * t596 + t459 * t610;
      double t613 = 0.2e1 * t474 * t612;
      double t616 = 0.1e1 / t473;
      double t617 = t536 * t616;
      double t618 = t4 * t470;
      double t619 = t382 * t440;
      double t621 = t386 * t81;
      double t624 = t621 * t447 * t45 * t138;
      double t626 = t392 * t467;
      double t628 = t160 * t447;
      double t631 = 0.5000000000e0 * t397 * t628 * t167;
      double t632 = 0.1e1 * t619 - 0.10e1 * t624 + 0.10e1 * t626 - t631;
      double t633 = t618 * t632;
      double t637 = 0.2e1 * t512 * t612;
      double t641 = -t632 * t498 * t508;
      double t649 = t160 * t440;
      double t653 = -0.1e1 * t382 * t447 - 0.10e1 * t621 * t591 + 0.10e1 * t392 * t495 - 0.5000000000e0 * t397 * t649 * t167;
      double t655 = -t470 * t653 * t508;
      double t657 = -t536 * t470;
      double t659 = 0.1e1 / t507 / t506;
      double t660 = t498 * t659;
      double t670 = t79 * t79;
      double t672 = t76 / t670;
      double t676 = 0.2e1 * t498 * t653 - 0.200000000e1 * t139 * t77 * t502 - 0.200000000e1 * t47 * t2 * t503 * t7 - 0.100000000e1 * t501 * t672 * t167;
      double t677 = t660 * t676;
      double t680 = t610 * t463;
      double t685 = 0.2e1 * t96 * t508 * t612;
      double t688 = t129 * t2;
      double t690 = t508 * t45;
      double t692 = t96 * t690 * t138;
      double t695 = t178 * t518;
      double t698 = t532 * t412;
      double t700 = t183 * t508 * t167;
      double t704 = t96 * t659 * t676;
      double t707 = t597 * t509 - t511 * t637 / 0.2e1 + t536 * t641 + t536 * t655 - t657 * t677 / 0.2e1 + 0.100000e1 * t680 * t48 * t519 - 0.5000000000e0 * t523 * t685 - 0.100000e1 * t532 * t688 * t692 - 0.100000e1 * t549 * t695 - 0.5000000000e0 * t698 * t700 - 0.5000000000e0 * t698 * t704;
      double t716 = t532 * t616;
      double t724 = -t532 * t470;
      double t737 = t536 * t412;
      double t742 = t680 * t509 - t564 * t637 / 0.2e1 + t532 * t641 + t532 * t655 - t724 * t677 / 0.2e1 - 0.100000e1 * t597 * t48 * t519 + 0.5000000000e0 * t570 * t685 + 0.100000e1 * t536 * t688 * t692 + 0.100000e1 * t537 * t695 + 0.5000000000e0 * t737 * t700 + 0.5000000000e0 * t737 * t704;
      double t750 = t38 * t48 * t440;
      double t753 = t441 * t212;
      double t756 = t129 * t440;
      double t759 = t45 * t2 * t230;
      double t764 = -0.10e1 * t249 * t750 - 0.5000000000e0 * t94 * t753 - 0.1000000000e1 * t177 * t756 * t759 + 0.1e1 * t360 * t447;
      double t765 = t764 * t463;
      double t778 = -0.10e1 * t196 * t750 - 0.5000000000e0 * t59 * t753 - 0.1000000000e1 * t89 * t756 * t759 + 0.1e1 * t262 * t447;
      double t780 = t455 * t764 + t459 * t778;
      double t781 = 0.2e1 * t474 * t780;
      double t785 = t90 * t16 * t440;
      double t789 = 0.1000000000e1 * t386 * t467 * t759;
      double t790 = t412 * t12;
      double t792 = t790 * t234 * t447;
      double t796 = 0.5000000000e0 * t397 * t628 * t245;
      double t797 = -0.1000e1 * t785 - t789 + 0.100e1 * t792 - t796;
      double t798 = t618 * t797;
      double t802 = 0.2e1 * t512 * t780;
      double t806 = -t797 * t498 * t508;
      double t820 = 0.1000e1 * t90 * t16 * t447 - 0.1000000000e1 * t386 * t495 * t759 + 0.100e1 * t790 * t234 * t440 - 0.5000000000e0 * t397 * t649 * t245;
      double t822 = -t470 * t820 * t508;
      double t840 = 0.2e1 * t498 * t820 - 0.2000000000e1 * t45 * t75 * t2 * t76 * t12 * t16 * t502 - 0.200000000e1 * t501 * t12 * t502 * t9 - 0.100000000e1 * t501 * t672 * t245;
      double t841 = t660 * t840;
      double t844 = t778 * t463;
      double t849 = 0.2e1 * t96 * t508 * t780;
      double t852 = t129 * t75;
      double t856 = t76 * t81 * t690 * t16;
      double t859 = t42 * t518;
      double t863 = t183 * t508 * t245;
      double t867 = t96 * t659 * t840;
      double t870 = t765 * t509 - t511 * t802 / 0.2e1 + t536 * t806 + t536 * t822 - t657 * t841 / 0.2e1 + 0.100000e1 * t844 * t48 * t519 - 0.5000000000e0 * t523 * t849 - 0.1000000000e1 * t532 * t852 * t856 - 0.100000e1 * t549 * t859 - 0.5000000000e0 * t698 * t863 - 0.5000000000e0 * t698 * t867;
      double t902 = t844 * t509 - t564 * t802 / 0.2e1 + t532 * t806 + t532 * t822 - t724 * t841 / 0.2e1 - 0.100000e1 * t765 * t48 * t519 + 0.5000000000e0 * t570 * t849 + 0.1000000000e1 * t536 * t852 * t856 + 0.100000e1 * t537 * t859 + 0.5000000000e0 * t737 * t863 + 0.5000000000e0 * t737 * t867;
      double t910 = t48 * t447 * dt;
      double t916 = -0.10e1 * t177 * t910 + 0.1e1 * t368 * t440 * dt;
      double t917 = t916 * t463;
      double t926 = -0.10e1 * t89 * t910 + 0.1e1 * t273 * t440 * dt;
      double t928 = t455 * t916 + t459 * t926;
      double t929 = 0.2e1 * t474 * t928;
      double t932 = t493 * dt;
      double t935 = t397 * t495 * dt;
      double t937 = -0.1e1 * t932 + 0.10e1 * t935;
      double t938 = t618 * t937;
      double t942 = 0.2e1 * t512 * t928;
      double t946 = -t937 * t498 * t508;
      double t953 = -0.1e1 * t465 * dt - 0.10e1 * t397 * t467 * dt;
      double t955 = -t470 * t953 * t508;
      double t958 = t500 * t659 * t953;
      double t960 = t926 * t463;
      double t965 = 0.2e1 * t96 * t508 * t928;
      double t969 = t96 * t660 * t953;
      double t1006 = 0.100e1 * t619 - 0.1000e1 * t624 + 0.1000e1 * t626 - t631;
      double t1010 = t616 * t498 * t508;
      double t1011 = t16 * t470;
      double t1015 = t473 * t653;
      double t1016 = t508 * t16;
      double t1019 = t473 * t498;
      double t1020 = t659 * t16;
      double t1027 = 0.100e1 * t465 + 0.1000e1 * t468;
      double t1033 = pow(0.1e1 * t1027 * t4 + 0.10e1 * t1019 * t1016, 0.2e1);
      double t1035 = sqrt(0.1e1 - t1033);
      double t1036 = 0.1e1 / t1035;
      double t1040 = -0.100000e1 * t785 - t789 + 0.10000e1 * t792 - t796;
      double t1046 = t473 * t820;
      double t1056 = -0.100e1 * t932 + 0.1000e1 * t935;
      double t1062 = t473 * t953;
      double t1065 = t473 * t500;
      double t1075 = t508 * t4;
      double t1078 = t659 * t4;
      double t1085 = 0.1e1 / t2;
      double t1086 = 0.1e1 / t12;
      double t1088 = t80 * t507;
      double t1089 = t1085 * t1086 * t1088;
      double t1096 = -0.1e1 * t1027 * t16 + 0.10e1 * t1019 * t1075;
      double t1100 = t129 * t1085;
      double t1101 = t1096 / t473 / t472 * t1100;
      double t1102 = t1086 * t80;
      double t1103 = t507 * t470;
      double t1108 = t1096 * t616;
      double t1110 = 0.1e1 / t48 / t47;
      double t1118 = 0.1e1 / t75;
      double t1125 = t1108 * t1100;
      double t1126 = t1086 * t81;
      double t1136 = t1096 * t1096;
      double t1141 = 0.1e1 / t76;
      double t1148 = 0.1e1 / (0.1e1 + 0.1000000000e1 * t1136 / t472 / t47 * t1118 * t1141 * t79 * t506);

      Jx(0,0) = 0.1000e1;
      Jx(0,1) = 0.0e0;
      Jx(0,2) = 0.0e0;
      Jx(0,3) = 0.100e1 * t39 * t48 * t54 - 0.5000000000e0 * t60 * t65 + 0.10e1 * t100 * dt * t103;
      Jx(0,4) = 0.100e1 * t113 * t48 * t54 - 0.5000000000e0 * t60 * t126 - 0.100e1 * t89 * t130 * t140 + 0.10e1 * t187 * dt * t103;
      Jx(0,5) = -0.100e1 * t197 * t199 - 0.5000000000e0 * t60 * t214 - 0.1000000000e1 * t89 * t217 * t221 + 0.10e1 * t262 * dt * t103;
      Jx(1,0) = 0.0e0;
      Jx(1,1) = 0.1000e1;
      Jx(1,2) = 0.0e0;
      Jx(1,3) = 0.100e1 * t278 * t54 - 0.5000000000e0 * t290 * t65 + 0.10e1 * t301 * dt * t103;
      Jx(1,4) = 0.100e1 * t171 * t48 * t54 - 0.5000000000e0 * t290 * t126 - 0.100e1 * t177 * t130 * t140 + 0.10e1 * t330 * dt * t103;
      Jx(1,5) = -0.100e1 * t250 * t199 - 0.5000000000e0 * t290 * t214 - 0.1000000000e1 * t177 * t217 * t221 + 0.10e1 * t360 * dt * t103;
      Jx(2,0) = 0.0e0;
      Jx(2,1) = 0.0e0;
      Jx(2,2) = 0.1000e1;
      Jx(2,3) = 0.0e0;
      Jx(2,4) = 0.10e1 * t382 * dt * t132 - 0.100e1 * t386 * t387 * t103 * t139 + 0.100e1 * t392 * t81 * t50 * t102 - 0.5000000000e0 * t398 * t50 * t102 * t167;
      Jx(2,5) = -0.10000e1 * t220 * t54 - 0.1000000000e1 * t386 * t387 * v * t102 * t45 * t220 + 0.1000e1 * t412 * t239 * t387 * t103 - 0.5000000000e0 * t398 * t50 * t102 * t245;
      Jx(3,0) = 0.0e0;
      Jx(3,1) = 0.0e0;
      Jx(3,2) = 0.0e0;
      Jx(3,3) = ((0.10e1 * t464 * t474 - 0.5000000000e0 * t479 * t490 + 0.1e1 * (t464 * t509 - t511 * t513 / 0.2e1 + 0.100000e1 * t516 * t48 * t519 - 0.5000000000e0 * t523 * t525) * t16) * t544 - t558 * (0.10e1 * t516 * t474 - 0.5000000000e0 * t522 * t490 + 0.1e1 * (t516 * t509 - t564 * t513 / 0.2e1 - 0.100000e1 * t464 * t48 * t519 + 0.5000000000e0 * t570 * t525) * t16)) * t582;
      Jx(3,4) = ((0.10e1 * t597 * t474 - 0.5000000000e0 * t479 * t613 - 0.10e1 * t617 * t633 + 0.1e1 * t707 * t16) * t544 - t558 * (0.10e1 * t680 * t474 - 0.5000000000e0 * t522 * t613 - 0.10e1 * t716 * t633 + 0.1e1 * t742 * t16)) * t582;
      Jx(3,5) = ((0.10e1 * t765 * t474 - 0.5000000000e0 * t479 * t781 - 0.10e1 * t617 * t798 + 0.1e1 * t870 * t16) * t544 - t558 * (0.10e1 * t844 * t474 - 0.5000000000e0 * t522 * t781 - 0.10e1 * t716 * t798 + 0.1e1 * t902 * t16)) * t582;
      Jx(4,0) = 0.0e0;
      Jx(4,1) = 0.0e0;
      Jx(4,2) = 0.0e0;
      Jx(4,3) = 0.0e0;
      Jx(4,4) = -(0.1e1 * t1006 * t4 - 0.10e1 * t1010 * t1011 * t632 + 0.10e1 * t1015 * t1016 - 0.5000000000e0 * t1019 * t1020 * t676) * t1036;
      Jx(4,5) = -(0.1e1 * t1040 * t4 - 0.10e1 * t1010 * t1011 * t797 + 0.10e1 * t1046 * t1016 - 0.5000000000e0 * t1019 * t1020 * t840) * t1036;
      Jx(5,0) = 0.0e0;
      Jx(5,1) = 0.0e0;
      Jx(5,2) = 0.0e0;
      Jx(5,3) = 0.0e0;
      Jx(5,4) = (0.1000000000e1 * (-0.1e1 * t1006 * t16 - 0.10e1 * t1010 * t633 + 0.10e1 * t1015 * t1075 - 0.5000000000e0 * t1019 * t1078 * t676) * t616 * t129 * t1089 + 0.1000000000e1 * t1101 * t1102 * t1103 * t632 + 0.1000000000e1 * t1108 * t1110 * t1085 * t1102 * t507 * t45 * t138 + 0.1000000000e1 * t1108 * t129 * t1118 * t1102 * t507 * t7 + 0.5000000000e0 * t1125 * t1126 * t507 * t167 + 0.5000000000e0 * t1125 * t1102 * t508 * t676) * t1148;
      Jx(5,5) = (0.1000000000e1 * (-0.1e1 * t1040 * t16 - 0.10e1 * t1010 * t798 + 0.10e1 * t1046 * t1075 - 0.5000000000e0 * t1019 * t1078 * t840) * t616 * t129 * t1089 + 0.1000000000e1 * t1101 * t1102 * t1103 * t797 + 0.1000000000e1 * t1108 * t1110 * t1088 * t45 * t16 + 0.1000000000e1 * t1125 * t1141 * t80 * t507 * t9 + 0.5000000000e0 * t1125 * t1126 * t507 * t245 + 0.5000000000e0 * t1125 * t1102 * t508 * t840) * t1148;

      Ju(0,0) = 0.100e1 * t89 * t267 + 0.10e1 * t273 * dt * t102;
      Ju(0,1) = -0.5000000000e0 * t278 * t281 + 0.5000000000e0 * t273 * t279 * t132;
      Ju(1,0) = 0.100e1 * t177 * t267 + 0.10e1 * t368 * dt * t102;
      Ju(1,1) = -0.5000000000e0 * t177 * t48 * t281 + 0.5000000000e0 * t368 * t279 * t132;
      Ju(2,0) = 0.10e1 * t424 * dt * t53 + 0.100e1 * t397 * t387 * t102;
      Ju(2,1) = -0.5000000000e0 * t424 * t279 * t103 + 0.5000000000e0 * t397 * t81 * t280 * t53;
      Ju(3,0) = 0.0e0;
      Ju(3,1) = ((0.10e1 * t917 * t474 - 0.5000000000e0 * t479 * t929 - 0.10e1 * t617 * t938 + 0.1e1 * (t917 * t509 - t511 * t942 / 0.2e1 + t536 * t946 + t536 * t955 - t657 * t958 + 0.100000e1 * t960 * t48 * t519 - 0.5000000000e0 * t523 * t965 - 0.100000e1 * t698 * t969) * t16) * t544 - t558 * (0.10e1 * t960 * t474 - 0.5000000000e0 * t522 * t929 - 0.10e1 * t716 * t938 + 0.1e1 * (t960 * t509 - t564 * t942 / 0.2e1 + t532 * t946 + t532 * t955 - t724 * t958 - 0.100000e1 * t917 * t48 * t519 + 0.5000000000e0 * t570 * t965 + 0.100000e1 * t737 * t969) * t16)) * t582;
      Ju(4,0) = 0.0e0;
      Ju(4,1) = -(0.1e1 * t1056 * t4 - 0.10e1 * t1010 * t1011 * t937 + 0.10e1 * t1062 * t1016 - 0.10e1 * t1065 * t1020 * t953) * t1036;
      Ju(5,0) = 0.0e0;
      Ju(5,1) = (0.1000000000e1 * (-0.1e1 * t1056 * t16 - 0.10e1 * t1010 * t938 + 0.10e1 * t1062 * t1075 - 0.10e1 * t1065 * t1078 * t953) * t616 * t129 * t1089 + 0.1000000000e1 * t1101 * t1102 * t1103 * t937 + 0.1000000000e1 * t1125 * t1102 * t512 * t953) * t1148;
      /* End maple */
    }

  } // namespace detail

}}}
