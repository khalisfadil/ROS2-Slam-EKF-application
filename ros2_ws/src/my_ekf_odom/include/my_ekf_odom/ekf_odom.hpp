#ifndef AUTOBIN__EKFO_HPP_
#define AUTOBIN__EKFO_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>

class EKFODOM_CHCV //extended kalman filter constant heading constant velocity
{
    public:

        void initializing(const double x, const double y, Eigen::Vector4d &init_x_out, Eigen::Matrix4d &init_p_out)
        {   
            //initializinh state_x
            state_x(STATE::X) = x; // = 0
            state_x(STATE::Y) = y; // = 0
            state_x(STATE::PSIS) =  0.5* M_PI; //90 degress
            state_x(STATE::V) = 0.00;

            init_x_out(0)=state_x(STATE::X); // = 0
            init_x_out(1)=state_x(STATE::Y); // = 0
            init_x_out(2)=state_x(STATE::PSIS); //90 degress
            init_x_out(3)=state_x(STATE::V);


            //initialiting state P
            Eigen::Matrix4d P;
            P = Eigen::Matrix4d::Identity()*1000;

            init_p_out= P;

            //check the value of the matrix for each iterations
            std::cout <<"============================================================================"<< std::endl;
            std::cout <<"===========================Initialize Matrix Check=========================="<< std::endl;
            std::cout <<"X: " <<  state_x(STATE::X) << "    " << "Y:  " << state_x(STATE::Y)<< "    " << "PSIS:  " << state_x(STATE::PSIS) << "    " << "V:  " << state_x(STATE::V) << std::endl;
            std::cout <<"P1: " <<  init_p_out(0,0) << "    " << "P2:  " << init_p_out(1,1)<< "    " << "P3:  " << init_p_out(2,2) << "    " << "P4:  " << init_p_out(3,3) << std::endl;
            std::cout <<"============================================================================"<< std::endl;
        }

        void predict(long double dt, Eigen::Vector4d X_in, Eigen::Matrix4d P_in, Eigen::Vector4d &X_out, Eigen::Matrix4d &P_out)//kena masukkan value jangan pakai STATE!!!
        {   
            //Prediction
            //=======================
            //derive for the dynamic matrix of A [JA] by using the state vector [state_x]

            X_in(0) = X_in(0) + (dt*X_in(3)*cos(X_in(2)));
            X_in(1) = X_in(1) + (dt*X_in(3)*sin(X_in(2)));
            X_in(2) = fmod((X_in(2)+M_PI), (2.0*M_PI)) - M_PI;
            X_in(3) = X_in(3);
            //double a;
            //a = fmod((X_in(2)+M_PI), (2.0*M_PI));
            //calculation of the Jacobian of dynamic Matrix A, [JA] with respect to the state vector [state_x] 
            double a13 = -dt * X_in(3) * sin(X_in(2));
            double a14 = dt * cos(X_in(2));
            double a23 = dt * X_in(3) * cos(X_in(2));
            double a24 = dt * sin(X_in(2));

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

            //prediction of the error covariance //P_in is the latest value of P
            P_in = (JA * P_in * JA.transpose()) + Q;

            P_out = P_in;

            //parse the value X_in to X_out
            X_out = X_in;

            //store value in state_prediction
            state_prediction(PREDICTIONSTATE::X_) = X_out(0);
            state_prediction(PREDICTIONSTATE::Y_) = X_out(1);
            state_prediction(PREDICTIONSTATE::PSIS_) = X_out(2);
            state_prediction(PREDICTIONSTATE::V_) = X_out(3);

            //check the value of the matrix for each iterations
            o++;
            std::cout <<"============================================================================"<< std::endl;
            std::cout <<"=============================Predict Matrix Check==========================="<< std::endl;
            std::cout <<"STEP:  "<< o << std::endl;
            std::cout <<"                                                                            " <<std::endl;
            //std::cout << a<<std::endl;
            std::cout << "X: " <<  X_out(0) << "    " << "Y:  " << X_out(1)<< "    " << "Yaw:  " << X_out(2) << "    " << "Vel:  " << X_out(3) << std::endl;
            std::cout << "JA13: " << JA(0,2) << ";  "<< "JA24: " << JA(0,3) << ";  "<< "JA23: " << JA(1,2) << ";  "<< "JA24: " << JA(1,3) << std::endl;
            std::cout << "Q: " << Q(0,0) << "; " << Q(1,1) << "; " << Q(2,2) << "; " << Q(3,3) << "; " << std::endl;
            std::cout << "sGPS: " <<  sGPS << "    " << "sCourse:  " << sCourse<< "    " << "sVelocity:  " << sVelocity << std::endl;
            std::cout << "P1: " <<  P_out(0,0) << "    " << "P2:  " << P_out(1,1)<< "    " << "P3:  " << P_out(2,2) << "    " << "P4:  " <<  P_out(3,3) << std::endl;
            std::cout << "dt :" << dt << std::endl;
            std::cout <<"============================================================================"<< std::endl;
        }

        void correct(const Eigen::Vector2d var, Eigen::Vector2d cor_state, Eigen::Vector4d X_in_, Eigen::Matrix4d P_in_, Eigen::Vector4d &X_out_, Eigen::Matrix4d &P_out_)
        {
            //Correction
            //=============================
            //parse the correction value
            
            //defining the error state R [var_R]
            Eigen::Matrix2d R;
            R << var(0), 0,
                0 ,var(1);

            //measurement function of h [H]  current predicted value
            
            Eigen::Vector2d H;

            H << X_in_(0), X_in_(1); //prediction state x and y //predicted state of x and y
            
            //measurement for GPS state / error state

            Eigen::Vector2d Z;

            Z << cor_state(0), cor_state(1); //GPS input state x and y //cumsum

            //declare identity matrix
            Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
            
            
            //define JH jacobian of H //maybe put it outside
        
            Eigen::MatrixXd JH = Eigen::Matrix<double, 2, num_state_>::Zero();
            
            JH.block<2,2>(0,0) = Eigen::Matrix2d::Identity();

            //compute the Kalman gain

            Eigen::MatrixXd K = (P_in_ * JH.transpose()) * ((JH* P_in_ * JH.transpose()) + R).inverse(); //matrix of 4x2

            //measurement of error state Y and correction value ES

            Eigen::Vector2d Y = Z - H; //vector 2x1;

            Eigen::Vector4d ES = K * Y; //vector 4x1

            //update the value of latest state x [state_x]

            X_in_ += ES;

            //parse the value x to x out

            X_out_ = X_in_;

            //parse the value correction state;
            state_correction(CORRECTIONSTATE::DX) = X_out_(0);
            state_correction(CORRECTIONSTATE::DY) = X_out_(1);
            state_correction(CORRECTIONSTATE::DPSIS) = X_out_(2);
            state_correction(CORRECTIONSTATE::DV) = X_out_(3);

            //update the error covariance
            P_in_ = (I - (K * JH)) * P_in_;

            //parse the P to output P
            P_out_ = P_in_;

            //update the state covariance
            state_covariance(COVARIANCESTATE::P1) = P_out_(0,0);
            state_covariance(COVARIANCESTATE::P2) = P_out_(1,1);
            state_covariance(COVARIANCESTATE::P3) = P_out_(2,2);
            state_covariance(COVARIANCESTATE::P4) = P_out_(3,3);

            //check the value of the matrix for each iterations
            u++;
            
            std::cout <<"============================================================================"<< std::endl;
            std::cout <<"=============================Correct Matrix Check==========================="<< std::endl;
            std::cout <<"STEP:  "<< u << std::endl;
            std::cout <<"                                                                            " <<std::endl;
            std::cout << "X: " <<  X_out_(0) << "    " << "Y:  " << X_out_(1)<< "    " << "Yaw:  " << X_out_(2) << "    " << "Vel:  " << X_out_(3) << std::endl;
            std::cout << "Y: " << Y(0) << "; " << Y(1)<< "    "<<"R:   "  << R(0,0) <<"; " << R(1,1) <<std::endl;
            std::cout << "K: " << K(0,0) << ", " << K(0,1) << "; " << K(1,0) << ", " << K(1,1) << "; " << K(2,0) << ", " << K(2,1) << "; " << K(3,0) << ", " << K(3,1) << ";" << std::endl;
            std::cout << "ES: " << ES(0) << "; " << ES(1) << "; " << ES(2) << "; " << ES(3) << "; " << std::endl;
            std::cout << "P1: " <<  state_covariance(COVARIANCESTATE::P1) << "    " << "P2:  " << state_covariance(COVARIANCESTATE::P2)<< "    " << "P3:  " << state_covariance(COVARIANCESTATE::P3) << "    " << "P4:  " << state_covariance(COVARIANCESTATE::P4) << std::endl;
            std::cout <<"============================================================================"<< std::endl;
            
        }

        Eigen::VectorXd getX()
        {
            return state_x;
        }

        Eigen::Vector4d getX_Correction()
        {
            return state_correction;
        }

        Eigen::MatrixXd getX_Prediction()
        {
            return state_prediction;
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

    int o = 0;
    int u = 0;

    //Eigen::Matrix<double, 4, 1> state_x; //state x (4*1)
    Eigen::Vector4d state_x;

    Eigen::Vector4d state_prediction;

    Eigen::Vector4d state_correction;

    Eigen::Vector4d state_covariance;

    enum STATE
    {
        X =  0 , Y = 1 , PSIS = 2 , V = 3
    };


    enum PREDICTIONSTATE
    {
        X_ =  0 , Y_ = 1 , PSIS_ = 2 , V_ = 3
    };

    enum CORRECTIONSTATE
    {
        DX = 0 , DY = 1, DPSIS = 2, DV = 3
    };

    enum COVARIANCESTATE
    {
        P1 = 0, P2 = 1, P3 = 2, P4 = 3
    };

};

#endif  // AUTOBIN__EKFK_HPP_