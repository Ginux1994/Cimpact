#ifndef SOLID_ISO_6_H
#define SOLID_ISO_6_H


#include "allInclude.h"


class Solid_Iso_6
{
public:
	Solid_Iso_6();
	assembleMassMatrix();
	calculateD(double sita, double phi, double etha);
	calculateN();
protected:
public:
	vector<Elastic> material;
	vector<Node> nodes;
	//参考《有限元分析的概念与应用》P178
	Array2D<double> D;//原始形函数对自然坐标偏导得到的矩阵，N中的元素从D中重组
	Array2D<double> M;//计算坐标矩阵的二维数组
	Array2D<double> N;
	Array2D<double> H;//是形成应变与位移的几何关系时使用的 0-1 矩阵
	Array1D<double> d;//位移矩阵，区别于M坐标矩阵
	Array1D<double> f;//节点力向量
	Array2D<double> P;//扩展雅克比矩阵逆的矩阵；雅克比矩阵就是自然坐标系对笛卡尔坐标系的偏导，对于等参元，等于自然坐标系对形函数的偏导矩阵*节点坐标矩阵

	vector<Array2D<double>> J;
	Array2D<double> J_inv;
	vector<Array1D<double>> strain;//8个积分点，每个积分点都有一个应变矩阵
	vector<Array1D<double>> dstrain;
	vector<Array1D<double>> stress;
	vector<Array2D<double>> B;

	Array1D<double> xsi;
	Array1D<double> phi;
	Array1D<double> etha;
	Array1D<double> W;

	string type''
		int number_of_integration_points;
	bool NIP_is_set;
	bool Nodes_are_set;
	bool Material_is_set;

};

Solid_Iso_4::Solid_Iso_6(){
	type = "SOLID_ISO_6";

	material = vector<Elastic>(8);
	nodes = vector<Node>(8);

	//三个自然坐标，ζ ξ η 
	xsi = Array1D<double>(8,0.0);
	etha = Array1D<double>(8,0.0);
	phi = Array1D<double>(8,0.0);

	//高斯积分点权系数向量，8个积分点
	W = Array1D<double>(8,0.0);;
	//是形成应变与位移的几何关系时使用的 0-1 矩阵
	H = Array2D<double>(6, 9, 0.0);
	//计算坐标矩阵的二维数组
	M = Array2D<double>(8, 3, 0.0);
	d = Array1D<double>(24, 0.0);
	f = Array1D<double>(24, 0.0);
	//扩展的雅克比矩阵
	P = Array2D<double>(9, 9, 0.0);
	N = Array2D<double>(9，24, 0.0);

	B = vector<Array2D<double>>(8);
	J = vector<Array2D<double>>(8);
	strain = vector<Array1D<double>>(8);
	dstrain = vector<Array1D<double>>(8);
	stress = vector<Array1D<double>>(8);
	for (int i = 0; i < 8;i++)
	{
		B[i] = Array2D<double>(6, 24, 0.0);
		J[i] = Array2D<double>(3, 3, 0.0);
		strain[i] = Array1D<double>(6, 0.0);
		dstrain[i] = Array1D<double>(6, 0.0);
		stress[i] = Array1D<double>(6, 0.0);
	}


}

Solid_Iso_6::assembleMassMatrix(){

	Array2D<double> mass(24, 24, 0.0);
	double total_mass;
	double s;

	for (int i = 0; i < 8;i++)
	{
		M[i][0] = nodes[i].pos[0];
		M[i][1] = nodes[i].pos[1];
		M[i][2] = nodes[i].pos[2];
	}
	calculateD(0.0, 0.0, 0.0);
	calculateN();

	//雅克比矩阵 = 自然坐标对形函数的偏导*笛卡尔坐标
	J[0] = D*M;
	LU<double> temp_LU(J[0]);
	double J_det = temp_LU.det();
	mass = N.transpose()*material[0].density*J_det*8.0;
	//只保留对角项
	for (int i = 0; i < 24;i++)
	{
		for (int j = 0; j < 24;j++)
		{
			mass[i][j] = (i == j) ? mass[i][j] : 0.0;
		}
	}
	
	calculateD(0.0, 0.0, 0.0);
	total_mass = material[0].density*J_det*8.0;

	for (int i = 0; i < 8;i++)
	{
		s += mass[3 * i][0];
	}
	for (int i = 0; i < 24;i++)
	{
		mass[i][i] *= total_mass / s;
	}
	for (int i = 0; i < 24; i++)
	{
		nodes[i].addMass(mass[3 * i][3 * i] / 8.0);
	}
}

Solid_Iso_6::setInitialConditions(){
	double t = 1 / sqrt(3.0);

	for (int i = 0; i < number_of_integration_points; i++) {
		material[i].setInitialCondition();
	}
	// For a single gauss point per element
	if (number_of_integration_points == 1) {
		xsi[0] = 0;
		etha[0] = 0;
		phi[0] = 0;
		W[0] = 2.0;
	}
	else
		// For eight gauss points per element
	{
		xsi[0] = -t;
		xsi[1] = -t;
		xsi[2] = -t;
		xsi[3] = -t;
		xsi[4] = t;
		xsi[5] = t;
		xsi[6] = t;
		xsi[7] = t;

		//
		etha[0] = -t;
		etha[1] = -t;
		etha[2] = t;
		etha[3] = t;
		etha[4] = -t;
		etha[5] = -t;
		etha[6] = t;
		etha[7] = t;

		//
		phi[0] = t;
		phi[1] = -t;
		phi[2] = -t;
		phi[3] = t;
		phi[4] = t;
		phi[5] = -t;
		phi[6] = -t;
		phi[7] = t;

		//
		W[0] = 1.0;
		W[1] = 1.0;
		W[2] = 1.0;
		W[3] = 1.0;
		W[4] = 1.0;
		W[5] = 1.0;
		W[6] = 1.0;
		W[7] = 1.0;
	}

	H[0][0] = 1.0;
	H[1][4] = 1.0;
	H[2][8] = 1.0;
	H[3][1] = 1.0;
	H[3][3] = 1.0;
	H[4][5] = 1.0;
	H[4][7] = 1.0;
	H[5][2] = 1.0;
	H[5][6] = 1.0;

}


//输入参数为三个自然坐标
Solid_Iso_6::calculateD(double sita, double phi, double etha){
	
	/************************************************************************/
	/* 
	N1 = (1-ξ)(1-η)(1+ζ)/8
	N3 = (1-ξ)(1-η)(1-ζ)/8
	N3 = (1-ξ)(1+η)(1-ζ)/8
	...
	*/
	/************************************************************************/

	D[0][0] = -(1 - etha)*(1 + phi) / 8;
	D[0][1] = -(1 - etha)*(1 - phi) / 8;
	D[0][2] = -(1 + etha)*(1 - phi) / 8;
	D[0][3] = -(1 + etha)*(1 + phi) / 8;
	D[0][4] = (1 - etha)*(1 + phi) / 8;
	D[0][5] = (1 - etha)*(1 - phi) / 8;
	D[0][6] = (1 + etha)*(1 - phi) / 8;
	D[0][7] = (1 + etha)*(1 + phi) / 8;

	D[1][0] = -(1 - sita)*(1 + phi) / 8;
	D[1][1] = -(1 - sita)*(1 - phi) / 8;
	D[1][2] = (1 - sita)*(1 - phi) / 8;
	D[1][3] = (1 - sita)*(1 + phi) / 8;
	D[1][4] = -(1 + sita)*(1 + phi) / 8;
	D[1][5] = -(1 + sita)*(1 - phi) / 8;
	D[1][6] = (1 + sita)*(1 - phi) / 8;
	D[1][7] = (1 + sita)*(1 + phi) / 8;


	D[2][0] = (1 - sita)*(1 - phi) / 8;
	D[2][1] = -(1 - sita)*(1 - phi) / 8;
	D[2][2] = -(1 - sita)*(1 + phi) / 8;
	D[2][3] = (1 - sita)*(1 + phi) / 8;
	D[2][4] = (1 + sita)*(1 - phi) / 8;
	D[2][5] = -(1 + sita)*(1 - phi) / 8;
	D[2][6] = -(1 + sita)*(1 + phi) / 8;
	D[2][7] = (1 + sita)*(1 + phi) / 8;

}

Solid_Iso_6::calculateN(){
	N.subarray(0, 2, 0, 0) = D.subarray(0, 2, 0, 0);
	N.subarray(3, 5, 1, 1) = D.subarray(0, 2, 0, 0);
	N.subarray(6, 8, 2, 2) = D.subarray(0, 2, 0, 0);

	N.subarray(0, 2, 3, 3) = D.subarray(0, 2, 1, 1);
	N.subarray(3, 5, 4, 4) = D.subarray(0, 2, 1, 1);
	N.subarray(6, 8, 5, 5) = D.subarray(0, 2, 1, 1);

	N.subarray(0, 2, 9, 9) = D.subarray(0, 2, 3, 3);
	N.subarray(3, 5, 10, 10) = D.subarray(0, 2, 3, 3);
	N.subarray(6, 8, 11, 11) = D.subarray(0, 2, 3, 3);

	N.subarray(0, 2, 12, 12) = D.subarray(0, 2, 4, 4);
	N.subarray(3, 5, 13, 13) = D.subarray(0, 2, 4, 4);
	N.subarray(6, 8, 14, 14) = D.subarray(0, 2, 4, 4);

	N.subarray(0, 2, 15, 15) = D.subarray(0, 2, 5, 5);
	N.subarray(3, 5, 16, 16) = D.subarray(0, 2, 5, 5);
	N.subarray(6, 8, 17, 17) = D.subarray(0, 2, 5, 5);

	N.subarray(0, 2, 18, 18) = D.subarray(0, 2, 6, 6);
	N.subarray(3, 5, 19, 19) = D.subarray(0, 2, 6, 6);
	N.subarray(6, 8, 20, 20) = D.subarray(0, 2, 6, 6);

	N.subarray(0, 2, 21, 21) = D.subarray(0, 2, 7, 7);
	N.subarray(3, 5, 22, 22) = D.subarray(0, 2, 7, 7);
	N.subarray(6, 8, 23, 23) = D.subarray(0, 2, 7, 7);
}


Solid_Iso_6::calculateStrain(double step, int k){

	calculateD(xsi[k], phi[k], etha[k]);
	calculateN();
	for (int i = 0; i < 8; i++)
	{
		M[i][0] = nodes[i].pos[0];
		M[i][1] = nodes[i].pos[1];
		M[i][2] = nodes[i].pos[2];
	}
	for (int i = 0; i < 8; i++)
	{
		d[3 * i][0] = nodes[i].dpl[0] - nodes[i].dpl_old[0];
		d[3 * i + 1][1] = nodes[i].dpl[1] - nodes[i].dpl_old[1];
		d[3 * i + 2][2] = nodes[i].dpl[2] - nodes[i].dpl_old[2];
	}

	J[k] = D*M;
	LU<double> inverse_J(J[k]);
	Array2D<double> E(3, 3, 0.0);
	for (int i = 0; i < 3; i++)
	{
		E[i][i] = 1.0;
	}
	inverse_J = inverse_J.solve(E);
	P.subarray(0, 2, 0, 2) = inverse_J;
	P.subarray(3, 5, 3, 5) = inverse_J;
	P.subarray(6, 8, 6, 8) = inverse_J;

	B[k] = H*P*N;

	dstrain[i] = B[i] * d;
}




























#endif