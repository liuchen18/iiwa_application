#include<opencv2/opencv.hpp>
#include<iostream>
#include<iomanip>
#include<cmath>
#include<fstream>
#define PI 3.1415926535897932

double PLimit[7]={PI,2/3.0*PI,PI,2/3.0*PI,PI,2/3.0*PI,PI};

using namespace std;
using namespace cv;


int m3m3(double (&m3m3result)[3][3],double m3a[][3],double m3b[][3]);
int m31m13(double (&m31m13result)[3][3],double m31[],double m13[]);
double norm(double matrix_3[]);
int Inv_Kine(double (&Theta)[7],Mat RPY_p,double gamma,double PreTheta[],int PreSize);
int sign(double x);
//int testpart();
int Test();
int m_transpose(double (&m_trans_result)[3][3],double i_matrix[][3]);
int for_kine(double (&f_kineresult)[4][4],double InTheta[7]);

Mat RPY2r(Mat RPY);