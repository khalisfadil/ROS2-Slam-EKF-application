#ifndef AUTOBIN__EKF_CHCA_HPP
#define AUTOBIN__EKF_CHCA_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>

class EKF_CHCA
{
    public:

        void initial_state(double x, double y, double yaw, Eigen::Matrix<double, 8,1> &x_out, Eigen::Matrix<double, 8,8> &p_out)
        {
            //initialize state x
            x_out(0) = x;
            x_out(1) = y;
            x_out(2) = 0;
            x_out(3) = 0;
            x_out(4) = 0;
            x_out(5) = 0;
            x_out(6) = yaw;
            x_out(7) = 0;

            Eigen::Matrix<double, 8,8> P;
            P.block<2,2>(0,0) = Eigen::Matrix<double, 2,2>::Identity()*10;
            P.block<2,2>(2,2) = Eigen::Matrix<double, 2,2>::Identity()*10;
            P.block<2,2>(4,4) = Eigen::Matrix<double, 2,2>::Identity()*1;
            P.block<2,2>(6,6) = Eigen::Matrix<double, 2,2>::Identity()*1;

            p_out = P;

        }

        void prediction(long double dt, Eigen::Matrix<double, 8,1> x_in, Eigen::Matrix<double, 8,8> p_in, Eigen::Matrix<double,8,1> &x_out, Eigen::Matrix<double, 8,8> &p_out)
        {
            //prediction
            //=========================================

            Eigen::Matrix<double,8,8> A;

            A << 1, 0, dt, 0, (1/2)*pow(dt,2), 0, 0, 0,
                0, 1, 0, dt, 0, (1/2)*pow(dt,2), 0, 0,
                0, 0, 1, 0, dt, 0, 0, 0,
                0, 0, 0, 1, 0, dt, 0, 0,
                0, 0, 0, 0, 1, 0, 0, 0,
                0, 0, 0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 0, 0, 1, dt,
                0, 0, 0, 0, 0, 0, 0, 1;

            x_in = A * x_in;

            double sj = 0.1;

            Eigen::Matrix<double, 8, 8> q;
            Eigen::Matrix<double, 8, 8> Q;

            q << pow(dt,6)/36, 0, pow(dt,5)/12, 0, pow(dt,4)/6, 0, 0, 0,
                0, pow(dt,6)/36, 0, pow(dt,5)/12, 0, pow(dt,4)/6, 0, 0,
                pow(dt,5)/12, 0, pow(dt,4)/4, 0, pow(dt,3)/2, 0, 0, 0,
                0, pow(dt,5)/12, 0, pow(dt,4)/4, 0, pow(dt,3)/2, 0, 0,
                pow(dt,4)/6, 0, pow(dt,3)/2, 0, pow(dt,2), 0, 0, 0,
                0, pow(dt,4)/6, 0, pow(dt,3)/2, 0, pow(dt,2), 0, 0,
                0, 0, 0, 0, 0, 0, 1, 1,
                0, 0, 0, 0, 0, 0, 1, 1;

            Q = q * sj;

            p_in = (A * p_in * A.transpose()) + Q;

            p_out = p_in;

            x_out = x_in;
        }

        void correction(const Eigen::Matrix<double, 3, 1> variance, Eigen::Matrix<double, 3, 1> cor_state, Eigen::Matrix<double, 8, 1> x_in, Eigen::Matrix<double, 8, 8> p_in, Eigen::Matrix<double, 8, 1> &x_out, Eigen::Matrix<double, 8, 8> &p_out)
        {
            //Correction
            //=============================

            Eigen::Matrix<double, 3, 3> R;

            R << pow(variance(1),2), 0, 0,
                    0 ,pow(variance(1),2), 0,
                    0 , 0, pow(variance(1),2);

            Eigen::Matrix<double, 3, 1> Z;

            Z << cor_state(0), cor_state(1), cor_state(2);

            Eigen::Matrix<double, 8, 8> I = Eigen::Matrix<double, 8, 8>::Identity();

            Eigen::Matrix<double, 3, 8> H;

            H << 1, 0, 0, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 1, 0;

            Eigen::MatrixXd K = (p_in * H.transpose()) * ((H* p_in * H.transpose()) + R).inverse();

            Eigen::Matrix<double, 3, 1> Y = Z - (H * x_in);

            Eigen::Matrix<double, 8, 1> ES = K * Y;

            x_in = x_in + ES;

            x_out = x_in;

            p_in = (I - (K * H)) * p_in;

            p_out = p_in;

        }

    private:

};

#endif