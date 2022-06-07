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

        void initializing(const double x, const double y)
        {
            state_x(STATE::X) = x; // = 0
            state_x(STATE::Y) = y; // = 0
            state_x(STATE::PSIS) =  0.5* M_PI;
            state_x(STATE::V) = 0.00;

            //check the value of the matrix for each iterations
            std::cout <<"============================================================================"<< std::endl;
            std::cout <<"===========================Initialize Matrix Check=========================="<< std::endl;
            std::cout << "X: " <<  state_x(STATE::X) << "    " << "Y:  " << state_x(STATE::Y)<< "    " << "Yaw:  " << state_x(STATE::PSIS) << "    " << "Vel:  " << state_x(STATE::V) << std::endl;
            std::cout <<"============================================================================"<< std::endl;
        }

        void predict(const double current_stamp_gnss, Eigen::Matrix4d &P)
        {
            //Prediction
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

            //check the value of the matrix for each iterations
            std::cout <<"============================================================================"<< std::endl;
            std::cout <<"=============================Predict Matrix Check==========================="<< std::endl;
            std::cout << "X: " <<  state_x(STATE::X) << "    " << "Y:  " << state_x(STATE::Y)<< "    " << "Yaw:  " << state_x(STATE::PSIS) << "    " << "Vel:  " << state_x(STATE::V) << std::endl;
            std::cout << "JA02: " << JA(0,2) << " "<< "JA03: " << JA(0,3) << " "<< "JA12: " << JA(1,2) << " "<< "JA13: " << JA(1,3) << std::endl;
            std::cout << "Q: " << Q(0,0) << "; " << Q(1,1) << "; " << Q(2,2) << "; " << Q(3,3) << "; " << std::endl;
            std::cout << "P1: " <<  P(0,0) << "    " << "P2:  " << P(1,1)<< "    " << "P3:  " << P(2,2) << "    " << "P4:  " << P(3,3) << std::endl;
            std::cout << "dt :" << dt << std::endl;
            std::cout <<"============================================================================"<< std::endl;
        }

        void correct(const Eigen::Vector2d correction_state, const Eigen::Vector2d variance, Eigen::Matrix4d P)
        {

            //Correction
            //=============================
            //parse the correction value
            state_correction(CORRECTIONSTATE::DX) = correction_state(0);
            state_correction(CORRECTIONSTATE::DY) = correction_state(1);
            
            //defining the error state R [var_R]
            Eigen::Matrix2d R;
            R << variance.x(), 0,
                0 ,variance.y();

            //measurement function of h [H] 
            
            Eigen::Vector2d H;

            H << state_x(STATE::X), state_x(STATE::Y); //prediction state x and y
            
            //measurement for GPS state / error state

            Eigen::Vector2d Z;

            Z << state_correction(CORRECTIONSTATE::DX), state_correction(CORRECTIONSTATE::DY); //GPS input state x and y

            //declare identity matrix
            Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
            
            
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
            P = (I - (K * JH)) * P;

            state_covariance(COVARIANCESTATE::P1) = P(0,0);

            state_covariance(COVARIANCESTATE::P2) = P(1,1);
            
            state_covariance(COVARIANCESTATE::P3) = P(2,2);

            state_covariance(COVARIANCESTATE::P4) = P(3,3);

            
            //check the value of the matrix for each iterations
            
            std::cout <<"============================================================================"<< std::endl;
            std::cout <<"=============================Correct Matrix Check==========================="<< std::endl;
            std::cout << "X: " <<  state_x(STATE::X) << "    " << "Y:  " << state_x(STATE::Y)<< "    " << "Yaw:  " << state_x(STATE::PSIS) << "    " << "Vel:  " << state_x(STATE::V) << std::endl;
            std::cout << "Y: " << Y.x() << "; " << Y.y() << std::endl;
            std::cout << "K: " << K(0,0) << ", " << K(0,1) << "; " << K(1,0) << ", " << K(1,1) << "; " << K(2,0) << ", " << K(2,1) << "; " << K(3,0) << ", " << K(3,1) << ";" << std::endl;
            std::cout << "ES: " << ES(0) << "; " << ES(1) << "; " << ES(2) << "; " << ES(3) << "; " << std::endl;
            std::cout << "P1: " <<  state_covariance(COVARIANCESTATE::P1) << "    " << "P2:  " << state_covariance(COVARIANCESTATE::P2)<< "    " << "P3:  " << state_covariance(COVARIANCESTATE::P3) << "    " << "P4:  " << state_covariance(COVARIANCESTATE::P4) << std::endl;
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

        Eigen::Vector2d getX_Correction()
        {
            return state_correction;
        }

        Eigen::MatrixXd getX_Covariance()
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

    Eigen::Vector2d state_correction;

    Eigen::Vector4d state_covariance;

    Eigen::Matrix<double,  num_state_,  num_state_> P; //initial uncertainty P (4*4)

    enum STATE
    {
        X =  0 , Y = 1 , PSIS = 2 , V = 3
    };

    enum CORRECTIONSTATE
    {
        DX = 0 , DY = 1
    };

    enum COVARIANCESTATE
    {
        P1 = 0, P2 = 1, P3 = 2, P4 = 3
    };

};

#endif  // AUTOBIN__EKF_HPP_