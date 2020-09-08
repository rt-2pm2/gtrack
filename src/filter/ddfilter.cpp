/**
 * @author Luigi Pannocchi
 * @file tracker.cpp
 *
 */

#include <Eigen/Eigenvalues>
#include <iostream>
#include <stdio.h>

#include "filter/ddfilter.hpp"

using namespace Eigen;

DDFilter::DDFilter():
    _x{DDXMat::Zero()}, _y{DDYMat::Zero()}, _u{DDUMat::Zero()} {

	setPos(Eigen::Vector3d::Zero());
	setSteps(5);
	computeC();
	updateSampleTime(0.03);
}


DDFilter::DDFilter(const Vector3d& p0, int Nsteps, double dt) : 
	_x {DDXMat::Zero()}, _y{DDYMat::Zero()}, _u{DDUMat::Zero()} {

	setPos(p0);
	setSteps(Nsteps);

	computeC();

	updateSampleTime(dt);
}

DDFilter::~DDFilter() {}


void DDFilter::reset(const DDXMat& x0, double s0) {
    _mx.lock();
	_x = x0;
    _mx.unlock();
}

void DDFilter::setU(const DDUMat& u) {}

void DDFilter::setSteps(int n) {
	_Nsteps = n;
}


DDXMat DDFilter::getState() const {
	_mx.lock();
	DDXMat out {_x};
	_mx.unlock();
	return out;
}

const Vector3d DDFilter::getPos() const {
	_mx.lock();
	Vector3d out(_x.block<3,1>(0,0));
	_mx.unlock();
	return out;
}

const Vector3d DDFilter::getVel() const {
	_mx.lock();
    Vector3d out(_x.block<3,1>(3,0));
	_mx.unlock();
	return out;
}

const Vector3d DDFilter::getAcc() const {
	_mx.lock();
    Vector3d out(_x.block<3,1>(6,0));
	_mx.unlock();
	return out;
}

const Vector3d DDFilter::getJerk() const {
	_mx.lock();
	Vector3d out(_x.block<3,1>(9,0));
	_mx.unlock();
	return out;
}


void DDFilter::setPos(const Vector3d& p){
	_mx.lock();
	_x.block<3,1>(0,0) = p;
	_mx.unlock();
}

void DDFilter::setVel(const Vector3d& v) {
	_mx.lock();
    _x.block<3,1>(3,0) = v;
	_mx.unlock();
}

void DDFilter::setAcc(const Vector3d& a) {
	_mx.lock();
    _x.block<3,1>(6,0) = a;
	_mx.unlock();
}

void DDFilter::setJerk(const Vector3d& a) {
	_mx.lock();
    _x.block<3,1>(9,0) = a;
	_mx.unlock();
}


void DDFilter::prediction(double dt) {
	DDXMat s = getState();

	// =================
	if (_T != dt) {
		updateSampleTime(dt);
	}

	double dt2 = dt * dt;

	// Integrate position 
	Vector3d new_pos = ExtractPos(s) + ExtractVel(s) * dt +
		0.5 * ExtractAcc(s) * (dt2) + ExtractJerk(s) * dt2 * dt / 3.0;

	// Integrate the velocity
	Vector3d new_vel = ExtractVel(s) + ExtractAcc(s) * dt + 
		ExtractJerk(s) * dt2 / 2.0;

	// Integrate acceleration
	Vector3d new_acc = ExtractAcc(s) + ExtractJerk(s) * dt;

	setPos(new_pos);
	setVel(new_vel);
	setAcc(new_acc);
	// The jerk is updated with the measurement...
}
 

void DDFilter::update(const DDYMat& y) {

	// Update Measurement queue
	_YQueue.push(y);

	if (_YQueue.size() <= _Nsteps) {
		return;
	}

	_YQueue.pop();


	// Create Measurement Vector
	Matrix<double, Dynamic, 1> Meas; 
	Meas.resize(DDFILTER_MEAS_DIM * _Nsteps, 1);

	for (int i = 0; i < _Nsteps; i++) {
		int index = 3 * ((_Nsteps -1) - i);
		Meas.block<3,1>(index, 0) = _YQueue.front();
		_YQueue.pop();
	}
	
	for (int i = 0; i < _Nsteps; i++) {
		int index = 3 * ((_Nsteps -1) - i);
		_YQueue.push(Meas.block<3,1>(index, 0));
	}

	// Create ObsMatrix
	Matrix<double, Dynamic, DDFILTER_STATE_DIM> Theta;
	Theta.resize(DDFILTER_MEAS_DIM * _Nsteps, DDFILTER_STATE_DIM);
	for (int i = 0; i < _Nsteps; i++) {
		int index = i * DDFILTER_MEAS_DIM;

		Theta.block<DDFILTER_MEAS_DIM, DDFILTER_STATE_DIM>(index, 0) = _C;

		for (int j = 0; j < i; j++) {
			Theta.block<DDFILTER_MEAS_DIM, DDFILTER_STATE_DIM>(index, 0) *= _A.inverse();
		}
	}	

	// Calculate PseudoInv
	//Eigen::MatrixXd pinv = Theta.completeOrthogonalDecomposition().pseudoInverse();

	//std::cout << "Size = " << pinv.rows() << " " << pinv.cols() << std::endl;

	// Compute State as Obs^-1 * Meas
	DDXMat dx = Theta.completeOrthogonalDecomposition().solve(Meas);
	// Update internal State
	setPos(dx.head<3>());
    setVel(dx.segment<3>(3));
	setAcc(dx.segment<3>(6));
    setJerk(dx.segment<3>(9));
}


/**
 * Generate the dynamics matrix of the system.
 */
DDDynMat DDFilter::computeA(double T) {
	DDDynMat out = DDDynMat::Identity();

    out.block<3,3>(0, 3) =
		Matrix<double, 3, 3>::Identity() * T;

    out.block<3,3>(0, 6) =
		Matrix<double, 3, 3>::Identity() * T * T * 0.5;

	out.block<3,3>(0, 9) =
		Matrix<double, 3, 3>::Identity() * T * T * T / 3.0;

    out.block<3,3>(3, 6) =
		Matrix<double, 3, 3>::Identity() * T;

	out.block<3,3>(3, 9) =
		Matrix<double, 3, 3>::Identity() * T * T * 0.5;

	out.block<3,3>(6, 9) =
		Matrix<double, 3, 3>::Identity() * T;

	return out;
}

/**
 * Compute C Matrix
 */
DDCMat DDFilter::computeC() {
	_C = DDCMat::Zero();
	_C.block<3,3>(0, 0) = Matrix<double, 3, 3>::Identity();
	return _C;
}


void DDFilter::updateSampleTime(double dt) {
	_T = dt;
	_A = computeA(dt); 
}


// ===========================================================
const Vector3d DDFilter::ExtractPos(const DDXMat& _x) const {
	Vector3d out(_x.block<3,1>(0,0));
	return out;
}

const Vector3d DDFilter::ExtractVel(const DDXMat& _x) const {
    Vector3d out(_x.block<3,1>(3,0));
	return out;
}

const Vector3d DDFilter::ExtractAcc(const DDXMat& _x) const {
    Vector3d out(_x.block<3,1>(6,0));
	return out;
}

const Vector3d DDFilter::ExtractJerk(const DDXMat& _x) const {
    Vector3d out(_x.block<3,1>(9,0));
	return out;
}
