#ifndef AUTOBIN__EKF_HPP_
#define AUTOBIN__EKF_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>

class EKF_CHCV //extended kalman filter constant heading constant velocity
{
    public:
        EKF_CHCV()
        : P(Eigen::Matrix4d::Identity()*1000) //initial uncertainty
        {
            state_x << 0.0,0.0,0.0,0.0;
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
            //put the value of cummulative gnss input into a GPS state; ERROR STATE
            state_gps(GPSSTATE::DX) = gnss_input.x();

            state_gps(GPSSTATE::DY) = gnss_input.y();

            //assert the cummulative value into the state x; the state x will change accordingly:

            state_x(STATE::X) = gnss_input.x();

            state_x(STATE::Y) = gnss_input.y();
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

            double a23 = dt * state_x(STATE::V) * cos(state_x(STATE::PSIS));

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

            Q << pow(sGPS,2), 0, 0, 0,
                0, pow(sGPS,2), 0, 0,
                0, 0, pow(sCourse,2), 0,
                0, 0, 0, pow(sVelocity,2);

            //prediction of the error covariance
            P = JA * P * JA.transpose() + Q;

            //Measurement Update (Correction)
            //=============================
            
            //defining the error state R [var_R]

            Eigen::Matrix2d R;
            R << variance.x(), 0,
                0 ,variance.y();

            //measurement function of h [H] 
            
            Eigen::Vector2d H;

            H << state_x(STATE::X), state_x(STATE::Y); //prediction state x and y
            
            //measurement for GPS state / error state

            Eigen::Vector2d Z;

            Z << state_gps(GPSSTATE::DX), state_gps(GPSSTATE::DY); //GPS input state x and y
            
            
            //define JH jacobian of H
        
            Eigen::MatrixXd JH = Eigen::Matrix<double, 2, num_state_>::Zero();
            
            JH.block<2,2>(0,0) = Eigen::Matrix2d::Identity();

            //compute the Kalman gain

            Eigen::MatrixXd K = P * JH.transpose() * (JH* P * JH.transpose() + R).inverse(); //matrix of 4x2

            //measurement of error state Y and correction value ES

            Eigen::Vector2d Y = Z - H; //vector 2x1;

            Eigen::Vector4d ES = K * Y; //vector 4x1

            //update the value of latest state x [state_x]

            state_x(STATE::X) = state_x(STATE::X) + ES(0);

            state_x(STATE::Y) = state_x(STATE::Y) + ES(1);

            state_x(STATE::PSIS) = state_x(STATE::PSIS) + ES(2);

            state_x(STATE::V) = state_x(STATE::V) + ES(3);

            //update the error covariance
            P = (Eigen::Matrix4d::Identity() - K * JH) * P;

            state_covariance(COVARIANCESTATE::P1) = P(0,0);

            state_covariance(COVARIANCESTATE::P2) = P(1,1);
            
            state_covariance(COVARIANCESTATE::P1) = P(2,2);

            state_covariance(COVARIANCESTATE::P2) = P(3,3);

            
            //check the value of the matrixS
            
            std::cout <<"============================================================================"<< std::endl;
            std::cout <<"=================================matrix Check==============================="<< std::endl;
            std::cout << "dt: " << dt << std::endl;
            std::cout << "Y: " << Y.x() << "; " << Y.y() << std::endl;
            std::cout << "K: " << K(0,0) << ", " << K(0,1) << "; " << K(1,0) << ", " << K(1,1) << "; " << K(2,0) << ", " << K(2,1) << "; " << K(3,0) << ", " << K(3,1) << ";" << std::endl;
            std::cout << "ES: " << ES(0) << "; " << ES(1) << "; " << ES(2) << "; " << ES(3) << "; " << std::endl;
            std::cout << "X: " <<  state_x(STATE::X) << "    " << "Y:  " << state_x(STATE::Y)<< "    " << "Yaw:  " << state_x(STATE::PSIS) << "    " << "Vel:  " << state_x(STATE::V) << std::endl;
            std::cout <<"============================================================================"<< std::endl;
            
        }


        void setInitialState(Eigen::Vector4d x)
        {
            state_x = x;
        }

        Eigen::VectorXd getX()
        {
            return state_x;
        }

        Eigen::Vector2d getX_GPS()
        {
            return state_gps;
        }

        Eigen::MatrixXd getMatrixCovariance()
        {
            return state_covariance;
        }

        int getNumState()
        {
            return num_state_;
        }
    
    private:

    double previous_stamp_gnss;

    static const int num_state_{4};

    //Eigen::Matrix<double, 4, 1> state_x; //state x (4*1)
    Eigen::Vector4d state_x;

    Eigen::Vector2d state_gps;

    Eigen::Vector4d state_covariance;

    Eigen::Matrix<double,  num_state_,  num_state_> P; //initial uncertainty P (4*4)

    enum STATE
    {
        X =  0 , Y = 1 , PSIS = 2 , V = 3
    };

    enum GPSSTATE
    {
        DX = 0 , DY = 1
    };

    enum COVARIANCESTATE
    {
        P1 = 0, P2 = 1, P3 = 2, P4 = 3
    };

};

#endif  // AUTOBIN__EKF_HPP_