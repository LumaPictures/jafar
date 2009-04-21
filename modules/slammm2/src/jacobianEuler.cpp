/* $Id: jacobianEuler.cpp 1708 2006-05-18 14:02:42Z tlemaire $ */

#include <cmath>

#include "kernel/jafarException.hpp"

#include "jacobianEuler.hpp"

using namespace jblas;

void jafar::slammm::gJacTEuler(vec const& p, vec const& pp, vec const& T, sym_mat& J)
{
  JFR_PRECOND(p.size() == 3,
	      "vme::gJacTEuler: invalid size");
  JFR_PRECOND(pp.size() == 3,
	      "vme::gJacTEuler: invalid size");
  JFR_PRECOND(T.size() == 6,
	      "vme::gJacTEuler: invalid size");
  JFR_PRECOND(J.size1() == 6 && J.size2() == 6,
	      "vme::gJacTEuler: invalid size");

  double px = p(0);
  double py = p(1);
  double pz = p(2);

  double ppx = pp(0);
  double ppy = pp(1);
  double ppz = pp(2);

  double tx = T(0);
  double ty = T(1);
  double tz = T(2);
  double yaw = T(3);
  double pitch = T(4);
  double roll = T(5);

  /* begin Maple copy/paste */
  double t1 = sin(yaw);
  double t2 = cos(pitch);
  double t3 = t1 * t2;
  double t4 = t3 * ppx;
  double t6 = sin(pitch);
  double t7 = t1 * t6;
  double t8 = sin(roll);
  double t10 = cos(yaw);
  double t11 = cos(roll);
  double t14 = -t7 * t8 - 0.10e1 * t10 * t11;
  double t15 = t14 * ppy;
  double t20 = -t7 * t11 + 0.10e1 * t10 * t8;
  double t21 = t20 * ppz;
  double t23 = -0.20e1 * t4 + 0.2e1 * t15 + 0.2e1 * t21;
  double t24 = t10 * t6;
  double t25 = t24 * ppx;
  double t27 = t10 * t2;
  double t28 = t8 * ppy;
  double t29 = t27 * t28;
  double t31 = t11 * ppz;
  double t32 = t27 * t31;
  double t34 = -0.20e1 * t25 + 0.2e1 * t29 + 0.2e1 * t32;
  double t38 = t24 * t11 + 0.10e1 * t1 * t8;
  double t43 = -t24 * t8 + 0.10e1 * t1 * t11;
  double t45 = t38 * ppy + t43 * ppz;
  double t46 = t27 * ppx;
  double t48 = -t43 * ppy;
  double t50 = t38 * ppz;
  double t52 = 0.20e1 * t46 + 0.2e1 * t48 + 0.2e1 * t50;
  double t53 = t7 * ppx;
  double t55 = t3 * t28;
  double t57 = t3 * t31;
  double t59 = -0.20e1 * t53 + 0.2e1 * t55 + 0.2e1 * t57;
  double t62 = -t20 * ppy + t14 * ppz;
  double t63 = t2 * ppx;
  double t65 = t6 * t8;
  double t66 = t65 * ppy;
  double t68 = t6 * t11;
  double t69 = t68 * ppz;
  double t71 = -0.200e1 * t63 - 0.20e1 * t66 - 0.20e1 * t69;
  double t72 = t2 * t11;
  double t73 = t72 * ppy;
  double t75 = t2 * t8;
  double t76 = t75 * ppz;
  double t78 = 0.20e1 * t73 - 0.20e1 * t76;
  double t79 = 0.10e1 * t4;
  double t80 = t79 - t15 - t21;
  double t81 = t80 * t80;
  double t82 = 0.10e1 * t46;
  double t83 = px - t82 - t48 - t50 - tx;
  double t84 = t43 * ppy;
  double t85 = -t38 * ppz;
  double t88 = -t82 - t48 - t50;
  double t89 = t88 * t88;
  double t92 = py - t79 + t14 * ppy + t20 * ppz - ty;
  double t96 = 0.10e1 * t25 - t29 - t32;
  double t99 = -0.10e1 * t53 + t55 + t57;
  double t103 = t96 * t80 + t83 * t99 - t99 * t88 + t92 * t96;
  double t111 = -t45 * t80 + t83 * (-t20 * ppy + t14 * ppz) - t62 * t88 - t92 * t45;
  double t112 = t96 * t96;
  double t117 = t99 * t99;
  double t125 = 0.100e1 * t63 + 0.10e1 * t66 + 0.10e1 * t69;
  double t126 = t125 * t125;
  double t128 = 0.100e1 * t6 * ppx;
  double t130 = 0.10e1 * t75 * ppy;
  double t132 = 0.10e1 * t72 * ppz;
  double t133 = pz + t128 - t130 - t132 - tz;
  double t138 = t11 * ppy;
  double t140 = t8 * ppz;
  double t151 = -0.10e1 * t73 + 0.10e1 * t76;
  double t159 = -t45 * t96 + t83 * (-t27 * t138 + t27 * t140) + t62 * t99 + t92 * (-t3 * t138 + t3 * t140) + t151 * t125 + t133 * (0.10e1 * t68 * ppy - 0.10e1 * t65 * ppz);
  double t160 = t45 * t45;
  double t163 = t62 * t62;
  double t166 = t151 * t151;
  J(0,0) = 0.2e1;
  J(0,1) = 0.0e0;
  J(0,2) = 0.0e0;
  J(0,3) = t23;
  J(0,4) = t34;
  J(0,5) = 0.2e1 * t45;
//   J(1,0) = 0.0e0;
  J(1,1) = 0.2e1;
  J(1,2) = 0.0e0;
  J(1,3) = t52;
  J(1,4) = t59;
  J(1,5) = 0.2e1 * t62;
//   J(2,0) = 0.0e0;
//   J(2,1) = 0.0e0;
  J(2,2) = 0.2e1;
  J(2,3) = 0.0e0;
  J(2,4) = t71;
  J(2,5) = t78;
//   J(3,0) = t23;
//   J(3,1) = t52;
//   J(3,2) = 0.0e0;
  J(3,3) = 0.2e1 * t81 + 0.2e1 * t83 * (t82 - t84 - t85) + 0.2e1 * t89 + 0.2e1 * t92 * t80;
  J(3,4) = 0.2e1 * t103;
  J(3,5) = 0.2e1 * t111;
//   J(4,0) = t34;
//   J(4,1) = t59;
//   J(4,2) = t71;
//   J(4,3) = 0.2e1 * t103;
  J(4,4) = 0.2e1 * t112 + 0.2e1 * t83 * (t82 + t24 * t28 + t24 * t31) + 0.2e1 * t117 + 0.2e1 * t92 * (t79 + t7 * t28 + t7 * t31) + 0.2e1 * t126 + 0.2e1 * t133 * (-t128 + t130 + t132);
  J(4,5) = 0.2e1 * t159;
//   J(5,0) = 0.2e1 * t45;
//   J(5,1) = 0.2e1 * t62;
//   J(5,2) = t78;
//   J(5,3) = 0.2e1 * t111;
//   J(5,4) = 0.2e1 * t159;
  J(5,5) = 0.2e1 * t160 + 0.2e1 * t83 * (-t84 - t85) + 0.2e1 * t163 + 0.2e1 * t92 * (-t15 - t21) + 0.2e1 * t166 + 0.2e1 * t133 * (t130 + t132);
  /* end Maple copy/paste */
}

void jafar::slammm::gJacpEuler(vec const& p, vec const& pp, vec const& T, mat& J)
{
  JFR_PRECOND(p.size() == 3,
	      "vme::gJacpEuler: invalid size");
  JFR_PRECOND(pp.size() == 3,
	      "vme::gJacpEuler: invalid size");
  JFR_PRECOND(T.size() == 6,
	      "vme::gJacpEuler: invalid size");
  JFR_PRECOND(J.size1() == 6 && J.size2() == 3,
	      "vme::gJacpEuler: invalid size");

//   double px = p(0);
//   double py = p(1);
//   double pz = p(2);

  double ppx = pp(0);
  double ppy = pp(1);
  double ppz = pp(2);

//   double tx = T(0);
//   double ty = T(1);
//   double tz = T(2);
  double yaw = T(3);
  double pitch = T(4);
  double roll = T(5);

  /* begin Maple copy/paste */
  double t1 = sin(yaw);
  double t2 = cos(pitch);
  double t3 = t1 * t2;
  double t6 = sin(pitch);
  double t7 = t1 * t6;
  double t8 = sin(roll);
  double t10 = cos(yaw);
  double t11 = cos(roll);
  double t14 = -t7 * t8 - 0.10e1 * t10 * t11;
  double t20 = -t7 * t11 + 0.10e1 * t10 * t8;
  double t24 = t10 * t2;
  double t27 = t10 * t6;
  double t31 = t27 * t8 - 0.10e1 * t1 * t11;
  double t37 = t27 * t11 + 0.10e1 * t1 * t8;
  double t43 = t8 * ppy;
  double t46 = t11 * ppz;
  J(0,0) = -0.2e1;
  J(0,1) = 0.0e0;
  J(0,2) = 0.0e0;
  J(1,0) = 0.0e0;
  J(1,1) = -0.2e1;
  J(1,2) = 0.0e0;
  J(2,0) = 0.0e0;
  J(2,1) = 0.0e0;
  J(2,2) = -0.2e1;
  J(3,0) = 0.20e1 * t3 * ppx - 0.2e1 * t14 * ppy - 0.2e1 * t20 * ppz;
  J(3,1) = -0.20e1 * t24 * ppx - 0.2e1 * t31 * ppy - 0.2e1 * t37 * ppz;
  J(3,2) = 0.0e0;
  J(4,0) = 0.20e1 * t27 * ppx - 0.2e1 * t24 * t43 - 0.2e1 * t24 * t46;
  J(4,1) = 0.20e1 * t7 * ppx - 0.2e1 * t3 * t43 - 0.2e1 * t3 * t46;
  J(4,2) = 0.200e1 * t2 * ppx + 0.20e1 * t6 * t8 * ppy + 0.20e1 * t6 * t11 * ppz;
  J(5,0) = -0.2e1 * t37 * ppy + 0.2e1 * t31 * ppz;
  J(5,1) = 0.2e1 * t20 * ppy - 0.2e1 * t14 * ppz;
  J(5,2) = -0.20e1 * t2 * t11 * ppy + 0.20e1 * t2 * t8 * ppz;
  /* end Maple copy/paste */
}

void jafar::slammm::gJacppEuler(vec const& p, vec const& pp, vec const& T, mat& J)
{
  JFR_PRECOND(p.size() == 3,
	      "vme::gJacppEuler: invalid size");
  JFR_PRECOND(pp.size() == 3,
	      "vme::gJacppEuler: invalid size");
  JFR_PRECOND(T.size() == 6,
	      "vme::gJacppEuler: invalid size");
  JFR_PRECOND(J.size1() == 6 && J.size2() == 3,
	      "vme::gJacppEuler: invalid size");

  double px = p(0);
  double py = p(1);
  double pz = p(2);

  double ppx = pp(0);
  double ppy = pp(1);
  double ppz = pp(2);

  double tx = T(0);
  double ty = T(1);
  double tz = T(2);
  double yaw = T(3);
  double pitch = T(4);
  double roll = T(5);

  /* begin Maple copy/paste */
  double t1 = cos(yaw);
  double t2 = cos(pitch);
  double t3 = t1 * t2;
  double t5 = sin(pitch);
  double t6 = t1 * t5;
  double t7 = sin(roll);
  double t8 = t6 * t7;
  double t10 = sin(yaw);
  double t11 = cos(roll);
  double t12 = t10 * t11;
  double t15 = t6 * t11;
  double t17 = t10 * t7;
  double t20 = t10 * t2;
  double t22 = t10 * t5;
  double t23 = t22 * t7;
  double t25 = t1 * t11;
  double t28 = t22 * t11;
  double t30 = t1 * t7;
  double t34 = t2 * t7;
  double t36 = t2 * t11;
  double t39 = 0.10e1 * t20 * ppx;
  double t41 = -t23 - 0.10e1 * t25;
  double t44 = -t28 + 0.10e1 * t30;
  double t46 = t39 - t41 * ppy - t44 * ppz;
  double t50 = 0.10e1 * t3 * ppx;
  double t52 = t8 - 0.10e1 * t12;
  double t53 = t52 * ppy;
  double t55 = t15 + 0.10e1 * t17;
  double t56 = t55 * ppz;
  double t57 = px - t50 - t53 - t56 - tx;
  double t61 = -t50 - t53 - t56;
  double t66 = py - t39 + t41 * ppy + t44 * ppz - ty;
  double t83 = t7 * ppy;
  double t85 = t11 * ppz;
  double t87 = 0.10e1 * t6 * ppx - t3 * t83 - t3 * t85;
  double t90 = t57 * t1;
  double t97 = 0.10e1 * t22 * ppx - t20 * t83 - t20 * t85;
  double t100 = t66 * t10;
  double t111 = 0.100e1 * t2 * ppx + 0.10e1 * t5 * t7 * ppy + 0.10e1 * t5 * t11 * ppz;
  double t120 = pz + 0.100e1 * t5 * ppx - 0.10e1 * t34 * ppy - 0.10e1 * t36 * ppz - tz;
  double t121 = t120 * t2;
  double t134 = t120 * t5;
  double t153 = -t55 * ppy + t52 * ppz;
  double t158 = t44 * ppy - t41 * ppz;
  double t165 = -0.10e1 * t36 * ppy + 0.10e1 * t34 * ppz;
  J(0,0) = 0.20e1 * t3;
  J(0,1) = 0.2e1 * t8 - 0.20e1 * t12;
  J(0,2) = 0.2e1 * t15 + 0.20e1 * t17;
  J(1,0) = 0.20e1 * t20;
  J(1,1) = 0.2e1 * t23 + 0.20e1 * t25;
  J(1,2) = 0.2e1 * t28 - 0.20e1 * t30;
  J(2,0) = -0.200e1 * t5;
  J(2,1) = 0.20e1 * t34;
  J(2,2) = 0.20e1 * t36;
  J(3,0) = -0.20e1 * t3 * t46 + 0.20e1 * t57 * t10 * t2 - 0.20e1 * t20 * t61 - 0.20e1 * t66 * t1 * t2;
  J(3,1) = -0.2e1 * t52 * t46 - 0.2e1 * t57 * t41 + 0.2e1 * t41 * t61 - 0.2e1 * t66 * t52;
  J(3,2) = -0.2e1 * t55 * t46 - 0.2e1 * t57 * t44 + 0.2e1 * t44 * t61 - 0.2e1 * t66 * t55;
  J(4,0) = -0.20e1 * t3 * t87 + 0.20e1 * t90 * t5 - 0.20e1 * t20 * t97 + 0.20e1 * t100 * t5 + 0.200e1 * t5 * t111 + 0.200e1 * t121;
  J(4,1) = -0.2e1 * t52 * t87 - 0.2e1 * t90 * t34 + 0.2e1 * t41 * t97 - 0.2e1 * t100 * t34 - 0.20e1 * t34 * t111 + 0.20e1 * t134 * t7;
  J(4,2) = -0.2e1 * t55 * t87 - 0.2e1 * t90 * t36 + 0.2e1 * t44 * t97 - 0.2e1 * t100 * t36 - 0.20e1 * t36 * t111 + 0.20e1 * t134 * t11;
  J(5,0) = -0.20e1 * t3 * t153 - 0.20e1 * t20 * t158 + 0.200e1 * t5 * t165;
  J(5,1) = -0.2e1 * t52 * t153 - 0.2e1 * t57 * t55 + 0.2e1 * t41 * t158 + 0.2e1 * t66 * t44 - 0.20e1 * t34 * t165 - 0.20e1 * t121 * t11;
  J(5,2) = -0.2e1 * t55 * t153 + 0.2e1 * t57 * t52 + 0.2e1 * t44 * t158 - 0.2e1 * t66 * t41 - 0.20e1 * t36 * t165 + 0.20e1 * t121 * t7;
  /* end Maple copy/paste */
}

