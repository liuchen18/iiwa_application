#include "7_DOF_inv.h"



int main()
{


	Test();

	return 0;
}

int Test()
{
	/*Mat RPY_p = (Mat_<float>(6,1)<<0,610,600,-PI,0,-PI/2);
	double PreTheta[7]={90/180*PI,40/180*PI,0,-67/180*PI,0,73/180*PI,0};
	double Theta[7]={0};
	double Phi = 0;
	    	
	Inv_Kine(Theta,RPY_p,Phi,PreTheta,7);

	cout<<"sel_Theta:"<<endl;
	for (int j=0;j<7;j++)
	{
		cout<<Theta[j]*180/PI<<'/';
	}
	cout<<endl;*/

	//double PreTheta[7]={90/180*PI,40/180*PI,0,-67/180*PI,0,73/180*PI,0};
	double PreTheta[7]={PI/2,PI/4.5,0,-PI/2.9032,0,PI/2.3077,0};
	
	cout<<"pre_Theta"<<endl;
	for (int j=0;j<7;j++)
	{
		cout<<PreTheta[j]*180/PI<<'/';
	}
	cout<<endl;

	double Final[4][4]={0};
	For_kine(Final,PreTheta);	
	
	for (int i=0;i<4;i++)
	{
		for (int j=0;j<4;j++)
		{
			cout<<setw(20)<<Final[i][j];
		}
		cout<<endl;
	}

	Mat R_old=Mat::zeros(3,3,CV_32FC1);
	for (int m=0;m<3;m++){
		for (int n=0;n<3;n++)
			R_old.ptr<float>(m)[n]=Final[m][n];
	}


	Mat RPY_=Mat::zeros(3,1,CV_32FC1);
	RPY_ = r2RPY(R_old);
	cout<<RPY_.ptr<float>(0)[0]<<" "<<RPY_.ptr<float>(0)[1]<<" "<<RPY_.ptr<float>(0)[2]<<" "<<Final[0][3]<<" "<<Final[1][3]<<" "<<Final[2][3]<<" "<<endl;

	/*Mat R_new=Mat::zeros(3,3,CV_32FC1);
	R_new = RPY2r(RPY_);


	double Final_new[4][4]={0};
	for (int m=0;m<3;m++){
		for (int n=0;n<3;n++)
			Final_new[m][n]=R_new.ptr<float>(m)[n];
	}

	for (int i=0;i<4;i++)
	{
		for (int j=0;j<4;j++)
		{
			cout<<setw(5)<<Final_new[i][j];
		}
		cout<<endl;
	}*/


	


	
	return 0;
}

//int testpart()   //������ΪT����ʱ�Ĳ��Գ���
//{
//	ifstream inFile("C:\\Users\\��\\Desktop\\taj.txt");        //�������ϵ�taj.txt�ļ�
//	if(!inFile){
//		cout<<endl<<"fail to open txt";
//		return 1;
//	}
//    double aprime=0;
//	double read_result[2404][4]={0};
//	int i=0,j=0;
//	while(!inFile.eof()){
//		inFile>>aprime;
//		if(inFile.fail())
//			break;
//		if(j>3){
//			i++;
//			j=0;
//		}
//		read_result[i][j]=aprime;
//		j++;
//	}
//
//	double T_taj[4][4]={0};
//	double eight_result[8][7]={0};
//	double PreTheta[7]={0};
//	double Theta[7]={0};
//	double Final[4][4]={0};
//	double gamma=-0.385;
//	ofstream outfile("C:\\Users\\��\\Desktop\\write_test.txt");
//	for (int n=0;n<601;n++)
//	{
//		for (int s=0;s<4;s++)
//			for (int t=0;t<4;t++)
//				T_taj[s][t]=read_result[4*n+s][t];
//
//		Inv_Kine(Theta,T_taj,gamma,PreTheta,7);
//		for (int i=0;i<7;i++)
//		{
//			PreTheta[i]=Theta[i];
//		}
//
//		for_kine(Final,Theta);
//		�ļ�д��
//		/*for (int j=0;j<7;j++)                                //д��������ļ�
//		{
//			outfile<<setw(10)<<OptTheta[j];
//		}
//			outfile<<endl;*/
//		for (int i=0;i<4;i++)                                  //д���������ļ�
//		{
//			for (int j=0;j<4;j++)
//			{
//				outfile<<(Final[i][j]-T_taj[i][j])<<"  ";
//			}
//			outfile<<endl;
//		}
//	}
//	outfile.close();
//	return 0;
//}




