#ifndef AUTOBIN__EKF_CHCV_HPP
#define AUTOBIN__EKF_CHCV_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>

class EKF_CHCV
{
    public:

        void initial_state(double x, double y, double yaw, Eigen::Matrix<double, 6,1> &x_out, Eigen::Matrix<double, 6,6> &p_out)
        {
            //initialize state x
            x_out(0) = x;
            x_out(1) = y;
            x_out(2) = 0;
            x_out(3) = 0;
            x_out(4) = yaw;
            x_out(5) = 0;

            Eigen::Matrix<double, 6,6> P;
            P.block<2,2>(0,0) = Eigen::Matrix<double, 2,2>::Identity()*10;
            P.block<2,2>(2,2) = Eigen::Matrix<double, 2,2>::Identity()*10;
            P.block<2,2>(4,4) = Eigen::Matrix<double, 2,2>::Identity()*1;

            p_out = P;

        }

        void prediction(long double dt, Eigen::Matrix<double, 6,1> x_in, Eigen::Matrix<double, 6,6> p_in, Eigen::Matrix<double,6,1> &x_out, Eigen::Matrix<double, 6,6> &p_out)
        {
            //prediction
            //=========================================

            Eigen::Matrix<double,6,6> A;

            A << 1, 0, dt, 0, 0, 0,
                0, 1, 0, dt, 0, 0,
                0, 0, 1, 0, 0, 0,
                0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 1, dt,
                0, 0, 0, 0, 0, 1;

            x_in = A * x_in;

            double sj = 0.1;

            Eigen::Matrix<double, 6, 6> q;
            Eigen::Matrix<double, 6, 6> Q;

            q << pow(dt,4)/4, 0, pow(dt,3)/2, 0, 0, 0,
                0, pow(dt,4)/4, 0, pow(dt,3)/2, 0, 0,
                pow(dt,3)/2, 0, pow(dt,2), 0, 0, 0,
                0, pow(dt,3)/2, 0, pow(dt,2), 0, 0,
                0, 0, 0, 0, pow(2*dt,2), 0,
                0, 0, 0, 0, 0, pow(4*dt,2);

            Q = q * sj;

            p_in = (A * p_in * A.transpose()) + Q;

            p_out = p_in;

            x_out = x_in;
        }

        void correction(const Eigen::Matrix<double, 3, 1> variance, Eigen::Matrix<double, 3, 1> cor_state, Eigen::Matrix<double, 6, 1> x_in, Eigen::Matrix<double, 6, 6> p_in, Eigen::Matrix<double, 6, 1> &x_out, Eigen::Matrix<double, 6, 6> &p_out)
        {
            //Correction
            //=============================

            Eigen::Matrix<double, 3, 3> R;

            R << pow(variance(1),2), 0, 0,
                    0 ,pow(variance(1),2), 0,
                    0 , 0, pow(variance(1),2);

            Eigen::Matrix<double, 3, 1> Z;

            Z << cor_state(0), cor_state(1), cor_state(2);

            Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();

            Eigen::Matrix<double, 3, 6> H;

            H << 1, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 0, 0,
                    0, 0, 0, 0, 1, 0;

            Eigen::MatrixXd K = (p_in * H.transpose()) * ((H* p_in * H.transpose()) + R).inverse();

            Eigen::Matrix<double, 3, 1> Y = Z - (H * x_in);

            Eigen::Matrix<double, 6, 1> ES = K * Y;

            x_in = x_in + ES;

            x_out = x_in;

            p_in = (I - (K * H)) * p_in;

            p_out = p_in;

        }

    private:

};

#endif