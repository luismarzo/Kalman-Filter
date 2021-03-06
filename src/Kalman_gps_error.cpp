//---------------------------------------------------------------------------------------------------------------------
//  
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 Luis Marzo Román a.k.a Luichi,  luismarzor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:  
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#include <iostream>
#include <vector>
#include <fstream>
#include "Eigen/Dense"

/* TODO 

    Calcular la covarianza del sensor de forma correcta con ecuaciones
    Include matplotlip.h and use the graphical representation in the same file
    Control signals
    Simulate tunel case, where the gps loses data
    Kalman filter class
    Change the method to pase vector to the Python program
*/

int main(int argc, char *argv[])
{

    /* 

        State-Space representation

        x(k+1)=A*x(k)+B*u(k)+D*+p(k)
        y(k)=C*x(k)+r(k)

    */

    int n = 4; // Matrix n-dimension
    int m = 1; // Matrix m-dimension

    double dt = 1.0 / 1.0; // Time step
    int time = 400;

    Eigen::MatrixXd A(n, n); // System dynamics matrix
    Eigen::MatrixXd B(n, n); // Control matrix

    Eigen::MatrixXd X(n, m);  // Final state
    Eigen::MatrixXd X_(n, m); // Previous state
    Eigen::MatrixXd Xp(n, m); // Priori state

    Eigen::MatrixXd P(n, n);  // Final state covariance
    Eigen::MatrixXd P_(n, n); // Previous state covariance
    Eigen::MatrixXd Pp(n, n); // Priori state covariance

    Eigen::MatrixXd U(n, m); // Control signal

    Eigen::MatrixXd Q(n, n); // Noise covariance in the process
    Eigen::MatrixXd K(n, n); // Kalman gain
    Eigen::MatrixXd H(n, n); // Relates the current state to observations with the output
    Eigen::MatrixXd R(n, n); // Noise covariance in the environment
    Eigen::MatrixXd I(n, n); // Identity Matrix
    Eigen::MatrixXd Z(n, m); // Measurement vector from sensors

    float q = 15; // Model uncertainty. If this number is 0, the model is perfect
    float r = 10;  // Sensor variance. We will use it for random gps signal. High value indicate high dispersion
    float p = 0.1; // State variance.

    std::vector<float> xgps, xt, x_filter;
    std::vector<float> ygps, yt, y_filter;
    float random_x;
    float random_y;


    // Discrete LTI projectile motion, measuring position only
    A << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1;
    B << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    X << 0, 0, 0, 0;
    X_ << 0, 0, 0, 0; //(px,py,vx,vy)
    Xp << 0, 0, 0, 0;

    P << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    P_ << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Pp << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    U << 0, 0, 0, 0; // TODO

    Q << q * q, 0, 0, 0, 0, q * q, 0, 0, 0, 0, q * q, 0, 0, 0, 0, q * q;
    K << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    H << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    R << r * r, 0, 0, 0, 0, r * r, 0, 0, 0, 0, r * r, 0, 0, 0, 0, r * r;
    I << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    Z << 0, 0, 0, 0;

    std::cout << "A: \n"<< A << "\n"<< std::endl;
    std::cout << "B: \n"<< B << "\n"<< std::endl;

    //GPS measurement simulation
    for (int i = 0; i < time; i++)
    {

        random_x = r * static_cast <float> (rand()) / static_cast <float> (RAND_MAX); //We are using variance for random number
        random_y = r * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

        //car trayectory , representation of the model(just for representation)

        xt.push_back(i);
        yt.push_back(i);



        //gps
        if(i<time/2){       
            xgps.push_back(i + random_x);
            ygps.push_back(i + random_y);
        }
        else{
            xgps.push_back(i + random_x);
            ygps.push_back(time - i + random_y);
        }
    }

    //Initialization of x(k-1) and P(k-1) JUST ONLY for the first loop

    X_ << xt[2], yt[2], (1 / dt) * (xt[2] - xt[1]), (1 / dt) * (yt[2] - yt[1]); //We start in the second loop because we need the velocity with the difference of the position in the first loop and the second one
    P_ << p * p, 0, 0, 0, 0, p * p, 0, 0, 0, 0, p * p, 0, 0, 0, 0, p * p;

    for (int i = 1; i < time; i++)
    { //We start in the second loop

        //Predict
        Xp = A * X_ + B * U;
        Pp = A * P_ * A.transpose() + Q;

        //Update
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        Z << xgps[i], ygps[i], (1 / dt) * (xgps[i] - xgps[i - 1]), (1 / dt) * (ygps[i] - ygps[i - 1]);
        X = Xp + K * (Z - H * Xp);
        P = (I - K * H) * Pp;

        P_ = P;
        X_ = X;

        //Vectos for final representation
        x_filter.push_back(X.coeff(0, 0));
        y_filter.push_back(X.coeff(1, 0));
    }

    std::ofstream myfile;
    myfile.open("Vectors.txt");

    myfile << "[";
    for (int i = 0; i < time; i++)
    {
        if(i<time-1){
            myfile << xt[i] << ", ";
        }
        else
        {
            myfile << xt[i] <<"]";
        }
        
    }

    myfile << "\n"<< "[";
    for (int i = 0; i < time; i++)
    {
       if(i<time-1){
            myfile << yt[i] << ", ";
        }
        else
        {
            myfile << yt[i] <<"]";
        }
    }

    myfile << "\n"<< "[";
    for (int i = 0; i < time; i++)
    {
       if(i<time-1){
            myfile << xgps[i] << ", ";
        }
        else
        {
            myfile << xgps[i] <<"]";
        }
    }

    myfile << "\n"<< "[";
    for (int i = 0; i < time; i++)
    {
       if(i<time-1){
            myfile << ygps[i] << ", ";
        }
        else
        {
            myfile << ygps[i] <<"]";
        }
    }

    myfile << "\n"<< "[";
    for (int i = 0; i < time; i++)
    {
       if(i<time-1){
            myfile << x_filter[i] << ", ";
        }
        else
        {
            myfile << x_filter[i] <<"]";
        }
    }

    myfile << "\n"<< "[";
    for (int i = 0; i < time; i++)
    {
       if(i<time-1){
            myfile << y_filter[i] << ", ";
        }
        else
        {
            myfile << y_filter[i] <<"]";
        }
    }

    myfile.close();
    return 0;
}