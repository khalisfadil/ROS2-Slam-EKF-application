
#ifndef MY_EKF_EKF_HPP_
#define MY_EKF_EKF_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>

class EKF_CHCV //extended kalman filter constant heading constant velocity
{
    public:
        EKF_CHCV()
        : P(Eigen::Matrix4d::Identity()*1000) //initial uncertainty
        {
            state_x << 0,0,0,0;
        }

        //-----------------------------------------------
        //-----------------------------------------------
        /*
        X = [ x y tau v]

        dynamix matrix function  = Matrix([[x+v*dts*cos(tau)],
                                            [y+v*dts*sin(tau)],
                                            [tau],
                                            [v]])
        
        A = [1 0 -dts*v*sin(tau) dts*cos(tau)
                0 1 dts*v*cos(tau) dts*sin(tau)
                0 0 1 0
                0 0 0 1]

        */

        void predict_correct(const double current_stamp_gnss, const Eigen::Vector2d & gnss_input, const Eigen::Vector2d & variance)
        {
            
            //from here initial state is used for the first time
            //the value of each state than calcualted

            //Time Update (Prediction)
            //=======================
            //derive for the dynamic matrix of A [JA] by using the state vector [state_x]
            
            double dt = current_stamp_gnss - previous_stamp_gnss;
            previous_stamp_gnss = current_stamp_gnss;

            state_x(STATE::X) = state_x(STATE::X) + dt * state_x(STATE::V) * cos(state_x(STATE::PSIS));

            state_x(STATE::Y) = state_x(STATE::Y) + dt * state_x(STATE::V) * sin(state_x(STATE::PSIS));

            state_x(STATE::PSIS) = fmod(state_x(STATE::PSIS) + M_PI, 2.0 * M_PI) - M_PI;

            state_x(STATE::V) = state_x(STATE::V);
            
            //calculation of the Jacobian of dynamic Matrix A, [JA] with respect to the state vector [state_x] 
            double a13 = -1* dt * state_x(STATE::V) * sin(state_x(STATE::PSIS));

            double a14 = dt * cos(state_x(STATE::PSIS));

            double a23 = dt * state_x(STATE::V) * cos(state_x(STATE:PSIS));

            double a24 = dt * sin(state_x(STATE::PSIS));

            Eigen::Matrix4d JA;

            JA << 1, 0, a13, a14,
                0, 1, a23, a24,
                0, 0, 1, 0,
                0, 0, 0, 1;

            //calculation of noise covariance matrix Q
            double sGPS = 0.5 * 8.8 * pow(dt,2.0); //assume 8.8m/s2 as maximum acceleration
            double sCourse = 2.0 * dt ;//assume 0.5rad/s as maximum turn rate
            double sVelocity = 35.0* dt; // assume 8.8m/s2 as maximum acceleration

            Eigen::Matrix4d Q;

            Q << sGPS, 0, 0, 0,
                0, sGPS, 0, 0,
                0, 0, sCourse, 0,
                0, 0, 0, sVelocity;

            //prediction of the error covariance
            P = JA * P * JA.transpose() + Q;

            //Measurement Update (Correction)
            //=============================
            
            //defining the error state R [var_R]

            Eigen::Matrix2d R;
            R << variance.x(), 0,
                0 ,variance.y();

            //measurement function of h [H] and its jacobian [JH]
            
            Eigen::Vector2d H;

            H << state_x(STATE::X), state_x(STATE::Y);

            //define JH
        
            Eigen::MatrixXd JH = Eigen::Matrix<double, 2, num_state_>::Zero();
            
            JH.block<2,2>(0,0) = Eigen::matrix2d::Identity();

            Eigen::MatrixXd K = P * JH.transpose() * (JH* P * JH.transpose() + R).inverse(); //matrix of 4x2

            Eigen::Vector2d Z = gnss_input; //Z is from input msg of the gps

            Eigen::VectorXd dx = K * (Z - H); //vector 4x1;

            //update the value of latest state x [state_x]

            state_x(STATE::X) = state_x(STATE::X) + dx(0);

            state_x(STATE::Y) = state_x(STATE::Y) + dx(1);

            state_x(SATE::PSIS) = state_x(STATE::PSIS);

            state_x(STATE::V) = state_x(STATE::V);

            //update the error covariance
            P = (Eigen::Matrix4d::Identity() - K * JH) * P;
            

        }


        void setInitialState(Eigen::VectorXd x)
        {
            state_x = x;
        }

        Eigen::VectorXd getX()
        {
            return state_x;
        }

        Eigen::MatrixXd getMatrixCovariance()
        {
            return P;
        }

        int getNumState()
        {
            return num_state_;
        }
    
    private:

    static const int num_state_{4};

    Eigen::Matrix<double, num_state_, 1> state_x; //state x (4*1)

    Eigen::Matrix<double,  num_state_,  num_state_> P; //initial uncertainty P (4*4)

    enum STATE
    {
        X =  0 , Y = 1 , PSIS = 2 , V = 3
    };

    
    

}

#endif //MY_EKF_EKF_HPP_