#include "slam/basisFeature.hpp"

#include "jmath/jblas.hpp"

namespace jafar {
namespace slam {

namespace details {
  inline void toFrame(double x, double y, double z,
                    double yaw, double pitch, double roll,
                    double v_x, double v_y, double v_z,
                    double v_yaw, double v_pitch, double v_roll,
                    double& x_res, double& y_res, double& z_res,
                    double& yaw_res, double& pitch_res, double& roll_res)
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
    double t64 = cos(v_yaw);
    double t66 = cos(v_pitch);
    double t69 = sin(v_yaw);
    double t73 = sin(v_pitch);
    double t86 = atan2(0.10e1 * t26 * t64 * t66 + 0.10e1 * t32 * t69 * t66 - 0.1000e1 * t34 * t73, 0.100e1 * t3 * t64 * t66 + 0.100e1 * t7 * t69 * t66 + 0.10000e1 * t10 * t73);
    double t96 = asin(0.10e1 * t47 * t64 * t66 + 0.10e1 * t52 * t69 * t66 - 0.1000e1 * t54 * t73);
    double t97 = t64 * t73;
    double t98 = sin(v_roll);
    double t100 = cos(v_roll);
    double t105 = t69 * t73;
    double t129 = atan2(t47 * (t97 * t98 - 0.10e1 * t69 * t100) + t52 * (t105 * t98 + 0.10e1 * t64 * t100) + 0.100e1 * t54 * t66 * t98, t47 * (t97 * t100 + 0.10e1 * t69 * t98) + t52 * (t105 * t100 - 0.10e1 * t64 * t98) + 0.100e1 * t54 * t66 * t100);
    x_res = 0.10e1 * t3 * v_x + 0.10e1 * t7 * v_y - 0.100e1 * t10 * v_z - 0.100e1 * t3 * x - 0.100e1 * t7 * y + 0.1000e1 * t10 * z;
    y_res = t26 * v_x + t32 * v_y + 0.10e1 * t34 * v_z - 0.10e1 * t26 * x - 0.10e1 * t32 * y - 0.100e1 * t34 * z;
    z_res = t47 * v_x + t52 * v_y + 0.10e1 * t54 * v_z - 0.10e1 * t47 * x - 0.10e1 * t52 * y - 0.100e1 * t54 * z;
    yaw_res = t86;
    pitch_res = -t96;
    roll_res = t129;
  }
  template<class M_Jframe, class M_Jv>
  inline void toFrameJac(double x, double y, double z,
                    double yaw, double pitch, double roll,
                    double v_x, double v_y, double v_z,
                    double v_yaw, double v_pitch, double v_roll,
                    M_Jframe& Jframe, M_Jv& Jv)
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
      double t141 = cos(v_yaw);
      double t143 = cos(v_pitch);
      double t146 = sin(v_yaw);
      double t149 = 0.10e1 * t55 * t146 * t143;
      double t151 = t141 * t143;
      double t154 = t146 * t143;
      double t157 = sin(v_pitch);
      double t160 = 0.100e1 * t3 * t151 + 0.100e1 * t6 * t154 + 0.10000e1 * t8 * t157;
      double t161 = 0.1e1 / t160;
      double t163 = t55 * t141;
      double t166 = -t52 * t146;
      double t170 = 0.1000e1 * t49 * t157;
      double t171 = 0.10e1 * t163 * t143 + 0.10e1 * t166 * t143 - t170;
      double t172 = t160 * t160;
      double t173 = 0.1e1 / t172;
      double t174 = t171 * t173;
      double t179 = -0.100e1 * t6 * t151 + 0.100e1 * t3 * t154;
      double t182 = t171 * t171;
      double t185 = 0.1e1 / (0.1e1 + t182 * t173);
      double t209 = t81 * t141;
      double t212 = t86 * t146;
      double t217 = 0.10e1 * t209 * t143 + 0.10e1 * t212 * t143 - 0.1000e1 * t88 * t157;
      double t236 = t141 * t157;
      double t239 = t146 * t157;
      double t253 = 0.10e1 * t81 * t146 * t143;
      double t255 = t217 * t217;
      double t257 = sqrt(0.1e1 - t255);
      double t258 = 0.1e1 / t257;
      double t293 = sin(v_roll);
      double t295 = cos(v_roll);
      double t298 = t236 * t293 - 0.10e1 * t146 * t295;
      double t303 = t239 * t293 + 0.10e1 * t141 * t295;
      double t309 = t236 * t295 + 0.10e1 * t146 * t293;
      double t314 = t239 * t295 - 0.10e1 * t141 * t293;
      double t316 = t143 * t295;
      double t319 = t81 * t309 + t86 * t314 + 0.100e1 * t88 * t316;
      double t320 = 0.1e1 / t319;
      double t324 = t143 * t293;
      double t326 = 0.100e1 * t88 * t324;
      double t327 = t81 * t298 + t86 * t303 + t326;
      double t328 = t319 * t319;
      double t329 = 0.1e1 / t328;
      double t330 = t327 * t329;
      double t336 = t327 * t327;
      double t339 = 0.1e1 / (0.1e1 + t336 * t329);
      Jframe(0,0) = -0.100e1 * t3;
      Jframe(0,1) = -0.100e1 * t6;
      Jframe(0,2) = 0.1000e1 * t8;
      Jframe(0,3) = -0.10e1 * t6 * v_x + 0.10e1 * t3 * v_y + 0.100e1 * t6 * x - 0.100e1 * t3 * y;
      Jframe(0,4) = -0.10e1 * t19 * v_x - 0.10e1 * t22 * v_y - 0.100e1 * t2 * v_z + 0.100e1 * t19 * x + 0.100e1 * t22 * y + 0.1000e1 * t2 * z;
      Jframe(0,5) = 0.0e0;
      Jv(0,0) = 0.10e1 * t3;
      Jv(0,1) = 0.10e1 * t6;
      Jv(0,2) = -0.100e1 * t8;
      Jv(0,3) = 0.0e0;
      Jv(0,4) = 0.0e0;
      Jv(0,5) = 0.0e0;
      Jframe(1,0) = -0.10e1 * t38 + 0.100e1 * t41;
      Jframe(1,1) = -0.10e1 * t44 - 0.100e1 * t46;
      Jframe(1,2) = -0.100e1 * t49;
      Jframe(1,3) = t52 * v_x + t55 * v_y - 0.10e1 * t52 * x - 0.10e1 * t55 * y;
      Jframe(1,4) = t3 * t37 * v_x + t6 * t37 * v_y - 0.10e1 * t66 * v_z - 0.10e1 * t3 * t37 * x - 0.10e1 * t6 * t37 * y + 0.100e1 * t66 * z;
      Jframe(1,5) = t81 * v_x + t86 * v_y + 0.10e1 * t88 * v_z - 0.10e1 * t81 * x - 0.10e1 * t86 * y - 0.100e1 * t88 * z;
      Jv(1,0) = t55;
      Jv(1,1) = -t52;
      Jv(1,2) = 0.10e1 * t49;
      Jv(1,3) = 0.0e0;
      Jv(1,4) = 0.0e0;
      Jv(1,5) = 0.0e0;
      Jframe(2,0) = -0.10e1 * t78 - 0.100e1 * t79;
      Jframe(2,1) = -0.10e1 * t83 + 0.100e1 * t84;
      Jframe(2,2) = -0.100e1 * t88;
      Jframe(2,3) = -t86 * v_x + t81 * v_y + 0.10e1 * t86 * x - 0.10e1 * t81 * y;
      Jframe(2,4) = t3 * t40 * v_x + t6 * t40 * v_y - 0.10e1 * t117 * v_z - 0.10e1 * t3 * t40 * x - 0.10e1 * t6 * t40 * y + 0.100e1 * t117 * z;
      Jframe(2,5) = -t55 * v_x + t52 * v_y - 0.10e1 * t49 * v_z + 0.10e1 * t55 * x - 0.10e1 * t52 * y + 0.100e1 * t49 * z;
      Jv(2,0) = t81;
      Jv(2,1) = t86;
      Jv(2,2) = 0.10e1 * t88;
      Jv(2,3) = 0.0e0;
      Jv(2,4) = 0.0e0;
      Jv(2,5) = 0.0e0;
      Jframe(3,0) = 0.0e0;
      Jframe(3,1) = 0.0e0;
      Jframe(3,2) = 0.0e0;
      Jframe(3,3) = ((0.10e1 * t52 * t141 * t143 + t149) * t161 - t174 * t179) * t185;
      Jframe(3,4) = ((0.10e1 * t3 * t37 * t141 * t143 + 0.10e1 * t6 * t37 * t146 * t143 + 0.1000e1 * t66 * t157) * t161 - t174 * (-0.100e1 * t19 * t151 - 0.100e1 * t22 * t154 + 0.10000e1 * t2 * t157)) * t185;
      Jframe(3,5) = t217 * t161 * t185;
      Jv(3,0) = 0.0e0;
      Jv(3,1) = 0.0e0;
      Jv(3,2) = 0.0e0;
      Jv(3,3) = ((-t149 - 0.10e1 * t52 * t141 * t143) * t161 + t174 * t179) * t185;
      Jv(3,4) = ((-0.10e1 * t163 * t157 - 0.10e1 * t166 * t157 - 0.1000e1 * t49 * t143) * t161 - t174 * (-0.100e1 * t3 * t236 - 0.100e1 * t6 * t239 + 0.10000e1 * t8 * t143)) * t185;
      Jv(3,5) = 0.0e0;
      Jframe(4,0) = 0.0e0;
      Jframe(4,1) = 0.0e0;
      Jframe(4,2) = 0.0e0;
      Jframe(4,3) = (0.10e1 * t86 * t141 * t143 - t253) * t258;
      Jframe(4,4) = (-0.10e1 * t3 * t40 * t141 * t143 - 0.10e1 * t6 * t40 * t146 * t143 - 0.1000e1 * t117 * t157) * t258;
      Jframe(4,5) = (0.10e1 * t55 * t141 * t143 - 0.10e1 * t52 * t146 * t143 - t170) * t258;
      Jv(4,0) = 0.0e0;
      Jv(4,1) = 0.0e0;
      Jv(4,2) = 0.0e0;
      Jv(4,3) = (t253 - 0.10e1 * t86 * t141 * t143) * t258;
      Jv(4,4) = (0.10e1 * t209 * t157 + 0.10e1 * t212 * t157 + 0.1000e1 * t88 * t143) * t258;
      Jv(4,5) = 0.0e0;
      Jframe(5,0) = 0.0e0;
      Jframe(5,1) = 0.0e0;
      Jframe(5,2) = 0.0e0;
      Jframe(5,3) = ((-t86 * t298 + t81 * t303) * t320 - t330 * (-t86 * t309 + t81 * t314)) * t339;
      Jframe(5,4) = ((t3 * t40 * t298 + t6 * t40 * t303 - 0.100e1 * t117 * t324) * t320 - t330 * (t3 * t40 * t309 + t6 * t40 * t314 - 0.100e1 * t117 * t316)) * t339;
      Jframe(5,5) = ((-t55 * t298 + t52 * t303 - 0.100e1 * t49 * t324) * t320 - t330 * (-t55 * t309 + t52 * t314 - 0.100e1 * t49 * t316)) * t339;
      Jv(5,0) = 0.0e0;
      Jv(5,1) = 0.0e0;
      Jv(5,2) = 0.0e0;
      Jv(5,3) = ((-t81 * t303 + t86 * t298) * t320 - t330 * (-t81 * t314 + t86 * t309)) * t339;
      Jv(5,4) = ((t209 * t324 + t212 * t324 - 0.100e1 * t88 * t157 * t293) * t320 - t330 * (t209 * t316 + t212 * t316 - 0.100e1 * t88 * t157 * t295)) * t339;
      Jv(5,5) = (0.1e1 - t330 * (-t81 * t298 - t86 * t303 - t326)) * t339;
  }
  inline void fromFrame(double x, double y, double z,
                    double yaw, double pitch, double roll,
                    double v_x, double v_y, double v_z,
                    double v_yaw, double v_pitch, double v_roll,
                    double& x_res, double& y_res, double& z_res,
                    double& yaw_res, double& pitch_res, double& roll_res)
  {
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
    double t23 = t10 * t2;
    double t26 = t10 * t6;
    double t30 = t26 * t8 + 0.10e1 * t1 * t11;
    double t35 = t26 * t11 - 0.10e1 * t1 * t8;
    double t41 = t2 * t8;
    double t44 = t2 * t11;
    double t49 = cos(v_yaw);
    double t50 = cos(v_pitch);
    double t51 = t49 * t50;
    double t54 = sin(v_yaw);
    double t58 = sin(v_pitch);
    double t70 = atan2(0.100e1 * t23 * t51 + 0.10e1 * t30 * t54 * t50 - 0.100e1 * t35 * t58, 0.100e1 * t3 * t51 + 0.10e1 * t14 * t54 * t50 - 0.100e1 * t19 * t58);
    double t80 = asin(-0.1000e1 * t6 * t49 * t50 + 0.100e1 * t41 * t54 * t50 - 0.1000e1 * t44 * t58);
    double t81 = t49 * t58;
    double t82 = sin(v_roll);
    double t84 = cos(v_roll);
    double t90 = t54 * t58;
    double t117 = atan2(-0.100e1 * t6 * (t81 * t82 - 0.10e1 * t54 * t84) + 0.10e1 * t41 * (t90 * t82 + 0.10e1 * t49 * t84) + 0.100e1 * t44 * t50 * t82, -0.100e1 * t6 * (t81 * t84 + 0.10e1 * t54 * t82) + 0.10e1 * t41 * (t90 * t84 - 0.10e1 * t49 * t82) + 0.100e1 * t44 * t50 * t84);
    x_res = 0.10e1 * t3 * v_x + t14 * v_y + t19 * v_z + 0.10e1 * x;
    y_res = 0.10e1 * t23 * v_x + t30 * v_y + t35 * v_z + 0.10e1 * y;
    z_res = -0.100e1 * t6 * v_x + 0.10e1 * t41 * v_y + 0.10e1 * t44 * v_z + 0.10e1 * z;
    yaw_res = t70;
    pitch_res = -t80;
    roll_res = t117;
  }
  template<class M_Jframe, class M_Jv>
  inline void fromFrameJac(double x, double y, double z,
                    double yaw, double pitch, double roll,
                    double v_x, double v_y, double v_z,
                    double v_yaw, double v_pitch, double v_roll,
                    M_Jframe& Jframe, M_Jv& Jv)
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
    double t59 = t6 * t8;
    double t62 = t6 * t11;
    double t66 = t2 * t11;
    double t69 = t2 * t8;
    double t76 = cos(v_yaw);
    double t77 = cos(v_pitch);
    double t78 = t76 * t77;
    double t80 = 0.100e1 * t3 * t78;
    double t81 = sin(v_yaw);
    double t82 = -t14 * t81;
    double t85 = sin(v_pitch);
    double t88 = t80 + 0.10e1 * t82 * t77 + 0.100e1 * t19 * t85;
    double t91 = -t39 * t81;
    double t96 = 0.100e1 * t25 * t78 + 0.10e1 * t91 * t77 - 0.100e1 * t34 * t85;
    double t97 = t96 * t96;
    double t98 = 0.1e1 / t97;
    double t99 = t88 * t98;
    double t108 = t88 * t88;
    double t111 = 0.1e1 / (0.1e1 + t108 * t98);
    double t116 = t8 * t81 * t77;
    double t119 = t11 * t85;
    double t123 = 0.1e1 / t96;
    double t151 = t81 * t77;
    double t168 = t76 * t85;
    double t195 = t6 * t76;
    double t203 = pow(-0.1000e1 * t195 * t77 + 0.100e1 * t69 * t151 - 0.1000e1 * t66 * t85, 0.2e1);
    double t205 = sqrt(0.1e1 - t203);
    double t206 = 0.1e1 / t205;
    double t223 = t81 * t85;
    double t230 = sin(v_roll);
    double t232 = cos(v_roll);
    double t235 = t168 * t230 - 0.10e1 * t81 * t232;
    double t241 = t223 * t230 + 0.10e1 * t76 * t232;
    double t244 = t77 * t230;
    double t251 = t168 * t232 + 0.10e1 * t81 * t230;
    double t257 = t223 * t232 - 0.10e1 * t76 * t230;
    double t260 = t77 * t232;
    double t263 = -0.100e1 * t6 * t251 + 0.10e1 * t69 * t257 + 0.100e1 * t66 * t260;
    double t264 = 0.1e1 / t263;
    double t271 = 0.100e1 * t66 * t244;
    double t272 = -0.100e1 * t6 * t235 + 0.10e1 * t69 * t241 + t271;
    double t273 = t263 * t263;
    double t274 = 0.1e1 / t273;
    double t275 = t272 * t274;
    double t285 = t272 * t272;
    double t288 = 0.1e1 / (0.1e1 + t285 * t274);
    Jframe(0,0) = 0.10e1;
    Jframe(0,1) = 0.0e0;
    Jframe(0,2) = 0.0e0;
    Jframe(0,3) = -0.10e1 * t3 * v_x + t14 * v_y + t19 * v_z;
    Jframe(0,4) = -0.10e1 * t22 * v_x + t25 * t26 + t25 * t28;
    Jframe(0,5) = t34 * v_y + t39 * v_z;
    Jv(0,0) = 0.10e1 * t25;
    Jv(0,1) = -t39;
    Jv(0,2) = t34;
    Jv(0,3) = 0.0e0;
    Jv(0,4) = 0.0e0;
    Jv(0,5) = 0.0e0;
    Jframe(1,0) = 0.0e0;
    Jframe(1,1) = 0.10e1;
    Jframe(1,2) = 0.0e0;
    Jframe(1,3) = 0.10e1 * t25 * v_x - t39 * v_y + t34 * v_z;
    Jframe(1,4) = -0.10e1 * t7 * v_x + t3 * t26 + t3 * t28;
    Jframe(1,5) = -t19 * v_y + t14 * v_z;
    Jv(1,0) = 0.10e1 * t3;
    Jv(1,1) = -t14;
    Jv(1,2) = -t19;
    Jv(1,3) = 0.0e0;
    Jv(1,4) = 0.0e0;
    Jv(1,5) = 0.0e0;
    Jframe(2,0) = 0.0e0;
    Jframe(2,1) = 0.0e0;
    Jframe(2,2) = 0.10e1;
    Jframe(2,3) = 0.0e0;
    Jframe(2,4) = -0.100e1 * t2 * v_x - 0.10e1 * t59 * v_y - 0.10e1 * t62 * v_z;
    Jframe(2,5) = 0.10e1 * t66 * v_y - 0.10e1 * t69 * v_z;
    Jv(2,0) = -0.100e1 * t6;
    Jv(2,1) = 0.10e1 * t69;
    Jv(2,2) = 0.10e1 * t66;
    Jv(2,3) = 0.0e0;
    Jv(2,4) = 0.0e0;
    Jv(2,5) = 0.0e0;
    Jframe(3,0) = 0.0e0;
    Jframe(3,1) = 0.0e0;
    Jframe(3,2) = 0.0e0;
    Jframe(3,3) = (0.1e1 - t99 * (-t80 + 0.10e1 * t14 * t81 * t77 - 0.100e1 * t19 * t85)) * t111;
    Jframe(3,4) = ((-0.100e1 * t7 * t78 + 0.10e1 * t3 * t116 - 0.100e1 * t3 * t119) * t123 - t99 * (-0.100e1 * t22 * t78 + 0.10e1 * t25 * t116 - 0.100e1 * t25 * t119)) * t111;
    Jframe(3,5) = ((-0.10e1 * t19 * t81 * t77 - 0.100e1 * t14 * t85) * t123 - t99 * (0.10e1 * t34 * t81 * t77 - 0.100e1 * t39 * t85)) * t111;
    Jv(3,0) = 0.0e0;
    Jv(3,1) = 0.0e0;
    Jv(3,2) = 0.0e0;
    Jv(3,3) = ((-0.100e1 * t3 * t151 - 0.10e1 * t14 * t76 * t77) * t123 - t99 * (-0.100e1 * t25 * t151 - 0.10e1 * t39 * t76 * t77)) * t111;
    Jv(3,4) = ((-0.100e1 * t3 * t168 - 0.10e1 * t82 * t85 + 0.100e1 * t19 * t77) * t123 - t99 * (-0.100e1 * t25 * t168 - 0.10e1 * t91 * t85 - 0.100e1 * t34 * t77)) * t111;
    Jv(3,5) = 0.0e0;
    Jframe(4,0) = 0.0e0;
    Jframe(4,1) = 0.0e0;
    Jframe(4,2) = 0.0e0;
    Jframe(4,3) = 0.0e0;
    Jframe(4,4) = -(-0.1000e1 * t2 * t76 * t77 - 0.100e1 * t59 * t151 + 0.1000e1 * t62 * t85) * t206;
    Jframe(4,5) = -(0.100e1 * t66 * t151 + 0.1000e1 * t69 * t85) * t206;
    Jv(4,0) = 0.0e0;
    Jv(4,1) = 0.0e0;
    Jv(4,2) = 0.0e0;
    Jv(4,3) = -(0.1000e1 * t6 * t81 * t77 + 0.100e1 * t69 * t78) * t206;
    Jv(4,4) = -(0.1000e1 * t195 * t85 - 0.100e1 * t69 * t223 - 0.1000e1 * t66 * t77) * t206;
    Jv(4,5) = 0.0e0;
    Jframe(5,0) = 0.0e0;
    Jframe(5,1) = 0.0e0;
    Jframe(5,2) = 0.0e0;
    Jframe(5,3) = 0.0e0;
    Jframe(5,4) = ((-0.100e1 * t2 * t235 - 0.10e1 * t59 * t241 - 0.100e1 * t62 * t244) * t264 - t275 * (-0.100e1 * t2 * t251 - 0.10e1 * t59 * t257 - 0.100e1 * t62 * t260)) * t288;
    Jframe(5,5) = ((0.10e1 * t66 * t241 - 0.100e1 * t69 * t244) * t264 - t275 * (0.10e1 * t66 * t257 - 0.100e1 * t69 * t260)) * t288;
    Jv(5,0) = 0.0e0;
    Jv(5,1) = 0.0e0;
    Jv(5,2) = 0.0e0;
    Jv(5,3) = ((0.100e1 * t6 * t241 + 0.10e1 * t69 * t235) * t264 - t275 * (0.100e1 * t6 * t257 + 0.10e1 * t69 * t251)) * t288;
    Jv(5,4) = ((-0.100e1 * t195 * t244 + 0.10e1 * t69 * t151 * t230 - 0.100e1 * t66 * t85 * t230) * t264 - t275 * (-0.100e1 * t195 * t260 + 0.10e1 * t69 * t151 * t232 - 0.100e1 * t66 * t85 * t232)) * t288;
    Jv(5,5) = (0.1e1 - t275 * (0.100e1 * t6 * t235 - 0.10e1 * t69 * t241 - t271)) * t288;
  }
}

BasisFeatureModel::BasisFeatureModel() : FeatureModel(6, 6, 6 )
{
  
}
void BasisFeatureModel::toFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec& xRes)
{
  details::toFrame(frame_(0), frame_(1), frame_(2), frame_(3), frame_(4), frame_(5), x_(0), x_(1), x_(2), x_(3), x_(4), x_(5), xRes(0), xRes(1), xRes(2), xRes(3), xRes(4), xRes(5));
}

void BasisFeatureModel::toFrameJac(const jblas::vec& frame_, const jblas::vec& x_)
{
//   JFR_DEBUG("toFrameJac " << frame_ << " " << x_ );
  details::toFrameJac(frame_(0), frame_(1), frame_(2), frame_(3), frame_(4), frame_(5), x_(0), x_(1), x_(2), x_(3), x_(4), x_(5), Jframe, Jx );
}

void BasisFeatureModel::fromFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec& xRes)
{
  details::fromFrame(frame_(0), frame_(1), frame_(2), frame_(3), frame_(4), frame_(5), x_(0), x_(1), x_(2), x_(3), x_(4), x_(5), xRes(0), xRes(1), xRes(2), xRes(3), xRes(4), xRes(5));
}

void BasisFeatureModel::fromFrame(jblas::vec const& frame_, jblas::vec const& x_, jblas::vec_range& xRes)
{
  details::fromFrame(frame_(0), frame_(1), frame_(2), frame_(3), frame_(4), frame_(5), x_(0), x_(1), x_(2), x_(3), x_(4), x_(5), xRes(0), xRes(1), xRes(2), xRes(3), xRes(4), xRes(5));
}

void BasisFeatureModel::fromFrameJac(const jblas::vec& frame_, const jblas::vec& x_)
{
//   JFR_DEBUG("fromFrameJac " << frame_ << " " << x_);
  details::fromFrameJac(frame_(0), frame_(1), frame_(2), frame_(3), frame_(4), frame_(5), x_(0), x_(1), x_(2), x_(3), x_(4), x_(5), Jframe, Jx );
}

void BasisFeatureModel::computeMergeState( const jblas::vec& x_, jblas::vec& mergeX_, jblas::mat& jac )
{
  mergeX_ = x_;
  jac = jblas::identity_mat( sizeMergeState() );
}

BasisFeatureObserveModel::BasisFeatureObserveModel(BasisFeatureModel& featureModel_) :
  FeatureObserveModel(featureModel_, 6)
{
#if 0
  std::stringstream s;
  s << "[6,6]((8.21274e-05,4.77278e-05,4.71992e-05,4.91789e-05,-9.7271e-05,2.21879e-05),(4.77278e-05,3.06132e-05,2.71875e-05,2.35587e-05,1.52859e-05,-1.24068e-06),(4.71992e-05,2.71875e-05,2.7978e-05,6.36425e-05,-6.3809e-05,1.9811e-05),(4.91789e-05,2.35587e-05,6.36425e-05,0.00357626,-8.49898e-05,0.000646724),(-9.7271e-05,1.52859e-05,-6.3809e-05,-8.49898e-05,0.00617412,-0.00040452),(2.21879e-05,-1.24068e-06,1.9811e-05,0.000646724,-0.00040452,0.00216609))";

  jblas::mat m(6,6);
  JFR_IO_STREAM(s >> m, "reading matrix from string");
  JobsSensor.assign(m);
  JinvObs.assign(m);
#endif
#if 0
  R.clear();
#endif
  std::stringstream s;
//   s << "[6,6]((8.21274e-05,4.77278e-05,4.71992e-05,4.91789e-05,-9.7271e-05,2.21879e-05),(4.77278e-05,3.06132e-05,2.71875e-05,2.35587e-05,1.52859e-05,-1.24068e-06),(4.71992e-05,2.71875e-05,2.7978e-05,6.36425e-05,-6.3809e-05,1.9811e-05),(4.91789e-05,2.35587e-05,6.36425e-05,0.00357626,-8.49898e-05,0.000646724),(-9.7271e-05,1.52859e-05,-6.3809e-05,-8.49898e-05,0.00617412,-0.00040452),(2.21879e-05,-1.24068e-06,1.9811e-05,0.000646724,-0.00040452,0.00216609))";
//   s << "[6,6]((0.1,0.1,0.1,0.1,0.1,0.1),(0.1,0.1,0.1,0.1,0.1,0.1),(0.1,0.1,0.1,0.1,0.1,0.1),(0.1,0.1,0.1,0.1,0.1,0.1),(0.1,0.1,0.1,0.1,0.1,0.1),(0.1,0.1,0.1,0.1,0.1,0.1))";
  s << "[6,6]((0.01,0.001,0.001,0.001,0.001,0.001),(0.001,0.01,0.001,0.001,0.001,0.001),(0.001,0.001,0.01,0.001,0.001,0.001),(0.001,0.001,0.001,0.01,0.001,0.001),(0.001,0.001,0.001,0.001,0.01,0.001),(0.001,0.001,0.001,0.001,0.001,0.01))";
  jblas::mat m(6,6);
  JFR_IO_STREAM(s >> m, "reading matrix from string");
  R.assign(m);
#if 1
  JobsSensor.assign(jblas::identity_mat(6));
  JinvObs.assign(jblas::identity_mat(6));
#endif
}

BasisFeatureObserveModel::~BasisFeatureObserveModel()
{
}


jblas::vec BasisFeatureObserveModel::inverseObservationInSensorFrame(Observation const& obs_)
{
  return obs_.z;
}

void BasisFeatureObserveModel::inverseObservationInSensorFrameJac(Observation const& /*obs_*/)
{
  // jacobian is the constant (3x3) identity matrix
  // initialized in the constructor
}

jblas::vec const& BasisFeatureObserveModel::predictObservationInSensorFrame(const jblas::vec& feature_)
{
  z.assign(feature_);
  return z;
}

void BasisFeatureObserveModel::predictObservationInSensorFrameJac(const jblas::vec& feature_) 
{
  // jacobian is the constant (3x3) identity matrix
  // initialized in the constructor
}


}
}
