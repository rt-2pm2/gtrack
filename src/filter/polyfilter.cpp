/**
 * @author Luigi Pannocchi
 * @file tracker.cpp
 *
 */

#include <Eigen/Eigenvalues>
#include <iostream>
#include <stdio.h>

#include "filter/polyfilter.hpp"

using namespace Eigen;

PolyFilter::PolyFilter(const Vector3d& p0,
		const SigmaXMat& sigma_x, const SigmaYMat& sigma_y, double dt) : 
	_x {XMat::Zero()}, _y{YMat::Zero()},
	_u{UMat::Zero()}, _P{PMat::Zero()} {

	setPos(p0);

	updateSigmas(sigma_x, sigma_y);
	updateSampleTime(dt);

	_P = PMat::Identity();
}

PolyFilter::~PolyFilter() {}


void PolyFilter::reset(const XMat& x0, double s0) {

    _mx.lock();
	_x = x0;
    _mx.unlock();

    _P = PMat::Identity() * s0;
}

void PolyFilter::setU(const UMat& u) {}

const Vector3d PolyFilter::getPos() const {
	_mx.lock();
	Vector3d out(_x.block<3,1>(0,0));
	_mx.unlock();
	return out;
}


const Vector3d PolyFilter::getVel() const {
	_mx.lock();
    Vector3d out(_x.block<3,1>(3,0));
	_mx.unlock();
	return out;
}

const Vector3d PolyFilter::getAcc() const {
	_mx.lock();
    Vector3d out(_x.block<3,1>(6,0));
	_mx.unlock();
	return out;
}

const QMat PolyFilter::getQ() const {
	_mx.lock();
	QMat out(_params.Q);
	_mx.unlock();

	return out;	
}

const DynMat PolyFilter::getA() const {
	_mx.lock();
	QMat out(_params.A);
	_mx.unlock();

	return out;	
}

const PMat PolyFilter::getP() const {
	_mx.lock();
	QMat out(_P);
	_mx.unlock();

	return out;	
}

const HMat PolyFilter::getH() const {
	_mx.lock();
	HMat out(_params.H);
	_mx.unlock();

	return out;	
}

const RMat PolyFilter::getR() const {
	_mx.lock();
	RMat out(_params.R);
	_mx.unlock();

	return out;	
}

void PolyFilter::setPos(const Vector3d& p){
	_mx.lock();
	_x.block<3,1>(0,0) = p;
	_mx.unlock();
}

void PolyFilter::setVel(const Vector3d& v) {
	_mx.lock();
    _x.block<3,1>(3,0) = v;
	_mx.unlock();
}

void PolyFilter::setAcc(const Vector3d& a) {
	_mx.lock();
    _x.block<3,1>(6,0) = a;
	_mx.unlock();
}



/**
 * Predict the covariance given the linearized model of the system.
 */
PMat PolyFilter::predictXcovariance(
		PMat& Px,
		QMat& Q,
		DynMat& A) {
	PMat P_ = A * Px * A.transpose() + Q;

	return P_; 
}

PMat PolyFilter::updateXcovariance(
		PMat& Px,
		HMat& H,
		KMat& K,
		RMat& R) {
	RMat S = H * Px * H.transpose() + R;
	PMat P = Px - K * S * K.transpose();
	// Force symmetry
	P = 0.5 * (P + P.transpose());
	return P;
}

KMat PolyFilter::computeK(QMat& P, HMat& H, RMat& R) {
	RMat Temp = H * P * H.transpose() + R;
	KMat K = P * H.transpose() * Temp.inverse();

	return K;
}


void PolyFilter::prediction(double dt) {

    //_mx.lock();

	// Integrate position 
	Vector3d new_pos = getPos() + getVel() * dt +
		0.5 * Acc() * (dt * dt);

	// Integrate the velocity
	Vector3d new_vel = getVel() + getAcc() * dt;

	// =================
	if (_params.T != dt) {
		updateSampleTime(dt);
	}

	// =================
	// Covariance Prediction
	// P_  = A' * P * A + Q
	_P = predictXcovariance(
			_P,
			_params.Q,
			_params.A);	

	setPos(new_pos);
	setVel(new_vel);

    //_mx.unlock();
}
 

void PolyFilter::update(const YMat& y) {

    //_mx.lock();

    HMat H_pos = HMat::Zero();
    H_pos.block<3,3>(0,0) = Matrix<double, 3, 3>::Identity();

	// Compute the kalman gain
	KMat K = computeK(_P, H_pos, _params.R); 

	Vector3d innov = y - getPos();
	XMat dx = K * innov;

	setPos(getPos() + dx.head<3>());
    setVel(getVel() + dx.segment<3>(3));
    setAcc(getAcc() + dx.tail<3>());

	_P = updateXcovariance(_P, H_pos, K, _params.R);

    //_mx.unlock();
}


/**
 * Generate the dynamics matrix of the system.
 */
DynMat PolyFilter::computeA(double T) {
	DynMat out = DynMat::Identity();

    out.block<3,3>(0, 3) =
		Matrix<double, 3, 3>::Identity() * T;

    out.block<3,3>(0, 6) =
		Matrix<double, 3, 3>::Identity() * T * T * 0.5;

    out.block<3,3>(3, 6) =
		Matrix<double, 3, 3>::Identity() * T;

	return out;
}


QMat PolyFilter::computeQ(double dt, SigmaXMat& sigmas) {
	QMat Q = QMat::Identity();

	double p = pow(dt, 2.0) * 0.5;
	double v = dt;
	double a = 1.0;

	double w[] {p, v, a};

	for (int i = 0; i < 3; i++) {
		Q.block<3, 3>(i * 3, 0) =
			Matrix3d::Identity() * w[i] * w[0] * sigmas(SIGMA_ACC_IND);

		Q.block<3, 3>(i * 3, 3) =
			Matrix3d::Identity() * w[i] * w[1] * sigmas(SIGMA_ACC_IND);

		Q.block<3, 3>(i * 3, 6) =
			Matrix3d::Identity() * w[i] * w[2] * sigmas(SIGMA_ACC_IND);
	}

	return Q;
}


void PolyFilter::updateSigmas(const SigmaXMat& x, 
		const SigmaYMat& y) {
	_params.sigma_x = x;
	_params.sigma_y = y;

	_params.R = RMat::Identity();
	_params.R(0,0) = y(0);
	_params.R(1,1) = y(1);
	_params.R(2,2) = y(2);

	std::cout << "UPDATED SIGMAS" << std::endl;
	std::cout << "X: " << _params.sigma_x.transpose() << std::endl;
	std::cout << "Y: " << _params.sigma_y.transpose() << std::endl;
}

void PolyFilter::updateSampleTime(double dt) {
	_params.T = dt;
	_params.A = computeA(dt); 
	_params.Q = computeQ(dt, _params.sigma_x);
}


void PolyFilter::updateSigmaY(const Vector3d& s) {
	_params.sigma_y = s;

    _params.R = RMat::Identity();
    _params.R(0,0) = s(0);
    _params.R(1,1) = s(1);
    _params.R(2,2) = s(2);
}

// ===========================================================
const Vector3d PolyFilter::Pos() const {
	Vector3d out(_x.block<3,1>(0,0));
	return out;
}

const Vector3d PolyFilter::Vel() const {
    Vector3d out(_x.block<3,1>(3,0));
	return out;
}

const Vector3d PolyFilter::Acc() const {
    Vector3d out(_x.block<3,1>(6,0));
	return out;
}
