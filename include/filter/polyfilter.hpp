/**
 * @author Luigi Pannocchi
 * @file polyfilter.hpp
 */

#ifndef _POLYFILTER_HPP_
#define _POLYFILTER_HPP_

#include <Eigen/Dense>
#include <mutex>

#define STATE_DIM (9)
#define INPUT_DIM (3)
#define MEAS_DIM (3)

#define SIGMAX_DIM (3)
#define SIGMAY_DIM (3)

#define SIGMA_ACC_IND (2)

// Redefinition of types to reduce the cluttering...
typedef Eigen::Matrix<double, STATE_DIM, 1> XMat;
typedef Eigen::Matrix<double, INPUT_DIM, 1> UMat;
typedef Eigen::Matrix<double, MEAS_DIM, 1> YMat;

typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> DynMat;
typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> PMat;
typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> QMat;

typedef Eigen::Matrix<double, MEAS_DIM, STATE_DIM> HMat;
typedef Eigen::Matrix<double, MEAS_DIM, MEAS_DIM> RMat;

typedef Eigen::Matrix<double, STATE_DIM, MEAS_DIM> KMat;

typedef Eigen::Matrix<double, SIGMAX_DIM, 1> SigmaXMat;
typedef Eigen::Matrix<double, SIGMAY_DIM, 1> SigmaYMat;



/**
 * Data structure containing the Kalman filter 
 * variables.
 */
struct TrackerParam {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double T;

	// Standard deviations of the model uncertainty. 
	SigmaXMat sigma_x;

    // Variance vector of the measurement noise.
    SigmaYMat sigma_y;

    // Linearization of the dynamics model
    DynMat A;
    QMat Q;

    // Linearization of the measurement model 
    HMat H;
    RMat R;

    // Constructor
    TrackerParam() : 
        T(1.0),
        sigma_x(SigmaXMat::Ones()),
        sigma_y(SigmaYMat::Ones()),
        A(DynMat::Identity()),
        Q(QMat::Identity()), 
        H(HMat::Zero()),
        R(RMat::Identity()) {}
};


class PolyFilter {

	public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		PolyFilter(const Eigen::Vector3d& p0,
				const SigmaXMat& sigma_x, const SigmaYMat& sigma_y,
				double dt);
		~PolyFilter();

		void reset(const XMat& x0, double s0);

		void prediction(double dt);

		void setU(const UMat& u);

		void update(const YMat&);

		// Fetchers
        const Eigen::Vector3d getPos() const;
        const Eigen::Vector3d getVel() const;
        const Eigen::Vector3d getAcc() const;

		const QMat getQ() const;
		const DynMat getA() const;
		const PMat getP() const;
		const HMat getH() const;
		const RMat getR() const;

		void updateSigmas(const SigmaXMat& x, const SigmaYMat& y);

    private:
        mutable std::mutex _mx;

		XMat _x;
		UMat _u;
		YMat _y;

		PMat _P;

        TrackerParam _params;

		void updateSampleTime(double dt);

		void  setPos(const Eigen::Vector3d& p);
        void  setVel(const Eigen::Vector3d& v);
        void  setAcc(const Eigen::Vector3d& a);

		/**
		 * Fetcher Private
		 */
		const Eigen::Vector3d Pos() const;
        const Eigen::Vector3d Vel() const;
        const Eigen::Vector3d Acc() const;

        /**
         * Compute the covariance matrix of the 
         * model uncertainties.
         */
        QMat computeQ(double dt, SigmaXMat& sigmas);

        /**
         * Compute the matrix of the dynamcs.
         */
        DynMat computeA(double dt);

        /**
         * Compute the kalman gain.
         */
        KMat computeK(QMat& P, HMat& H, RMat& R);

        /**
         * Predict the covariance of the process considering
         * a linear model
         */
        PMat predictXcovariance(PMat& Px, QMat& Q, DynMat& A);

        /**
         * Update the covariance matrix
         */
        PMat updateXcovariance(PMat& Px, HMat& H, KMat& K, RMat& R);


        /**
         * Update the covariance of the model uncertainty.
         */
        void updateSigmaX(const SigmaXMat& s);

        /**
         * Update the covariance of the measurement
         * uncertainty.
         **/
        void updateSigmaY(const SigmaYMat& s);
};

#endif
