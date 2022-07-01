#ifndef AUTOBIN__MS_EKF_HPP_
#define AUTOBIN__MS_EKF_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>

class MY_SCANMATCHING_EKF
{
    public:


        void initialize_state(const double x, const double y, const double yaw, Eigen::Matrix<double, 6, 1> &init_x_out, Eigen::Matrix<double, 6, 6> &init_p_out)
        {
            //initializinh state_x
            init_x_out(0,0) = x; // = 0
            init_x_out(1,0) = y; // = 0
            init_x_out(2,0) = 0.00; //90 degress
            init_x_out(3,0) = 0.00;
            init_x_out(4,0) = yaw;
            init_x_out(5,0) = 0;

            //initialiting state P
            Eigen::Matrix<double, 6, 6> P;
            P = Eigen::Matrix<double, 6, 6>::Identity()*1000;

            init_p_out= P;

            //check the value of the matrix for each iterations
            /*
            std::cout <<"============================================================================"<< std::endl;
            std::cout <<"===========================Initialize Matrix Debug=========================="<< std::endl;
            std::cout <<"X: " <<  init_x_out(0) << "    " << "Y:  " << init_x_out(1) << "    " << "PSIS:  " << init_x_out(2) << "    " << "V:  " << init_x_out(3) << std::endl;
            std::cout <<"P1: " <<  init_p_out(0,0) << "    " << "P2:  " << init_p_out(1,1)<< "    " << "P3:  " << init_p_out(2,2) << "    " << "P4:  " << init_p_out(3,3) << std::endl;
            std::cout <<"============================================================================"<< std::endl;*/
        }
        
        void prediction(long double dt, Eigen::Matrix<double, 6, 1> X_in, Eigen::Matrix<double, 6, 6> P_in, Eigen::Matrix<double, 6, 1> &X_out, Eigen::Matrix<double, 6, 6> &P_out)//kena masukkan value jangan pakai STATE!!!
        {   
            //Prediction
            //=======================
            //derive for the dynamic matrix of A [JA] by using the state vector [state_x]

            X_in(0,0) = X_in(0,0) + (dt*X_in(2,0));
            X_in(1,0) = X_in(1,0) + (dt*X_in(3,0));
            X_in(2,0) = X_in(2,0);
            X_in(3,0) = X_in(3,0);
            X_in(4,0) = X_in(4,0) + (dt*X_in(5,0));
            X_in(5,0) = X_in(5,0);
            //double a;
            //a = fmod((X_in(2,0)+M_PI), (2.0*M_PI));
            //calculation of the Jacobian of dynamic Matrix A, [JA] with respect to the state vector [state_x] 
            double a02 = dt * X_in(2,0);
            double a13 = dt * X_in(3,0);
            double a45 = dt * X_in(5,0);

            Eigen::Matrix<double, 6, 6> JA;

            JA << 1, 0, a02, 0, 0, 0,
                0, 1, 0, a13, 0, 0,
                0, 0, 1, 0, 0, 0,
                0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 1, a45,
                0, 0, 0, 0, 0, 0;

            //calculation of noise covariance matrix Q
            double sGPS = 0.5 * 8.8 * pow(dt,2.0); //assume 8.8m/s2 as maximum acceleration
            double sCourse = 2.0 * dt ;//assume 0.5rad/s as maximum turn rate
            double sVelocity = 35.0* dt; // assume 8.8m/s2 as maximum acceleration

            Eigen::Matrix<double, 6, 6> Q;

            Q << pow(sGPS,2), 0, 0, 0, 0, 0,
                0, pow(sGPS,2), 0, 0, 0, 0,
                0, 0, pow(sVelocity,2), 0, 0, 0,
                0, 0, 0, pow(sVelocity,2), 0, 0,
                0, 0, 0, 0, pow(sCourse,2), 0,
                0, 0, 0, 0, 0, pow(sCourse,2);


            //prediction of the error covariance //P_in is the latest value of P
            P_in = (JA * P_in * JA.transpose()) + Q;

            P_out = P_in;

            //parse the value X_in to X_out
            X_out = X_in;
            
            /*
            //check the value of the matrix for each iterations
            o++;
            std::cout <<"============================================================================"<< std::endl;
            std::cout <<"=============================Predict Matrix Debug==========================="<< std::endl;
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
            */
        }

        void correction(const Eigen::Matrix<double, 3, 1> var, Eigen::Matrix<double, 3, 1> cor_state, Eigen::Matrix<double, 6, 1> X_in_, Eigen::Matrix<double, 6, 6> P_in_, Eigen::Matrix<double, 6, 1> &X_out_, Eigen::Matrix<double, 6, 6> &P_out_)
        {
            //Correction
            //=============================
            //parse the correction value
            
            //defining the error state R [var_R]
            Eigen::Matrix<double, 3, 3> R;
            
            R << var(0,0), 0, 0,
                0 ,var(1,0), 0,
                0 , 0, var(3,0);

            //measurement function of h [H]  current predicted value
            
            Eigen::Matrix<double, 3, 1> H;

            H << X_in_(0,0), X_in_(1,0), X_in_(4,0); //prediction state x and y //predicted state of x and y
            
            //measurement for GPS state / error state

            Eigen::Matrix<double, 3, 1> Z;

            Z << cor_state(0,0), cor_state(1,0), cor_state(2,0); //GPS input state x and y //cumsum

            //declare identity matrix
            Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
            
            
            //define JH jacobian of H //maybe put it outside
        
            Eigen::Matrix<double, 3, 6> JH;
            
            //JH.block<2,2>(0,0) = Eigen::Matrix2d::Identity();
            //size JH 3x6
            JH << 1, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 0, 0,
                    0, 0, 0, 0, 1, 0;

            //compute the Kalman gain

            Eigen::MatrixXd K = (P_in_ * JH.transpose()) * ((JH* P_in_ * JH.transpose()) + R).inverse(); // 6x3 * 3x3 = 6x3

            //measurement of error state Y and correction value ES

            Eigen::Matrix<double, 3, 1> Y = Z - H; //vector 3x1;

            Eigen::Matrix<double, 6, 1> ES = K * Y; // 6x3 * 3x1 = 6x1

            //update the value of latest state x [state_x]
            X_in_ = X_in_ + ES;

            //parse the value x to x out
            X_out_ = X_in_;

            //update the error covariance
            P_in_ = (I - (K * JH)) * P_in_;

            //parse the P to output P
            P_out_ = P_in_;

            
            //check the value of the matrix for each iterations
            u++;
            
            std::cout <<"============================================================================"<< std::endl;
            std::cout <<"=============================Correct Matrix Debug==========================="<< std::endl;
            std::cout <<"STEP:  "<< u << std::endl;
            std::cout <<"                                                                            " <<std::endl;
            std::cout << "X: " <<  X_out_(0) << "    " << "Y:  " << X_out_(1)<< "    " << "X.dot:  " << X_out_(2) << "    " << "Y.dot:  " << X_out_(3) << "    " << "Yaw:  " << X_out_(4) << "    " << "Yaw.dot:  " << X_out_(5) << std::endl;
            std::cout << "Y: " << Y(0) << "; " << Y(1)<< "; " << Y(2) <<"    "<<"R:   "  << R(0,0) <<"; " << R(1,1) << "; " << R(2,2) << std::endl;
            //std::cout << "K: " << K(0,0) << ", " << K(0,1) << "; " << K(1,0) << ", " << K(1,1) << "; " << K(2,0) << ", " << K(2,1) << "; " << K(3,0) << ", " << K(3,1) << ";" << std::endl;
            std::cout << "ES: " << ES(0) << "; " << ES(1) << "; " << ES(2) << "; " << ES(3) << "; " << ES(4) << "; " << ES(5) << std::endl;
            std::cout << "P1: " <<  P_out_(0,0) << "    " << "P2:  " << P_out_(1,1)<< "    " << "P3:  " << P_out_(2,2) << "    " << "P4:  " << P_out_(3,3)<< "P5:  " << P_out_(4,4)<< "P6:  " << P_out_(5,5) << std::endl;
            std::cout <<"============================================================================"<< std::endl;
            
        }

    private:
    int o = 0;
    int u = 0;

};

#endif  // AUTOBIN__MS_EKF_HPP_