/* $Id$ */

#include "slam/eulerTools.hpp"

#include "slam/odo3dPredictModel.hpp"

using namespace jafar::slam;

Odo3dPredictModel::Odo3dPredictModel() : 
	JacobianBlockCommandPredictModel(6,2),
	kzs(0.0),
	kprs(0.0),
	robotToRef_yaw(0.0)
{
	Q.clear();
}

void Odo3dPredictModel::setModel(double kvv_, double kvw_, double kwv_, double kww_, double kzs_, double kprs_)
{
	odoNoiseModel.set(kvv_, kvw_, kwv_, kww_);
	kzs = kzs_;
	kprs = kprs_;
}

void Odo3dPredictModel::predict(jblas::vec_range& x_r, jblas::vec const& u)
{
	JFR_PRECOND(x_r.size() == sizeState(),
		"Odo3dPredictModel::predict: invalid state size");
	JFR_PRECOND(u.size() == sizeCommand(),
		"Odo3dPredictModel::predict: invalid command size");

	double dt = getTimeStep().total_microseconds() * 1e-6;

	// dynamic system noise model
	double ds = fabs(dt*u(0));
	Q(2,2) = kzs*kzs * ds;
	Q(4,4) = kprs*kprs * ds;
	Q(5,5) = kprs*kprs * ds;

	jblas::vec xPred(6);

	if (robotToRef_yaw == 0.0) {
		EulerTools::odo3d(x_r, u, dt, xPred);
		EulerTools::odo3dJac(x_r, u, dt, F, G);
	}
	else {
		EulerTools::odo3dInRefFrame(robotToRef_yaw, x_r, u, dt, xPred);
		EulerTools::odo3dInRefFrameJac(robotToRef_yaw, x_r, u, dt, F, G);
	}
	x_r.assign(xPred);
}
