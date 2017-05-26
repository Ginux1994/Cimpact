#ifndef ELASTIC_H
#define ELASTIC_H


#include "Variable.h"
#include "tnt.h"
#include "tnt_array2d.h"
#include <boost/tokenizer.hpp>




using namespace std;
using namespace TNT;

class Elastic
{
public:
	Elastic();



	void parseFembic(string& thisLine);
	bool isAKeyWord(const string &param);
	void setInitialCondition();
	void checkIndata();
	void calculateStressOneDimensional(Array1D<double>& strain, Array1D<double>& dstrain, Array1D<double>& stress, double timestep);
	void calculateStressTwoDimensional(Array1D<double>& strain, Array1D<double>& dstrain, Array1D<double>& stress, double timestep);
	void calculateStressThreeDimensional(Array1D<double>& strain, Array1D<double>& dstrain, Array1D<double>& stress, double timestep);
	//应力波波速计算
	inline double wavespeedOneDimendinal(){ return sqrt(youngs_moduls / density); }
	inline double wavespeedTwoDimendinal(){ return sqrt(youngs_moduls / (density*(1.0 - nu*nu))); }
	inline double wavespeedThreeDimendinal(){ return sqrt((youngs_moduls*(1.0 - nu) / (density*(1.0 + nu)*(1.0 - 2.0*nu)))); }

public:
	Array2D<double> stiffness_matrix_3d;
	Array2D<double> stiffness_matrix_plane_stress;

	Array2D<double> A;
	Array2D<double> I;//

	string filePath;
	string type;
	string name;
	double factor, eps, eps_vel;
	double youngs_moduls;
	double density;
	double nu;//possion ratio;
	double failure_strain;
	double failure_strss;

	bool E_is_set;//Y model;
	bool NU_is_set;//possion ritio
	bool RHO_is_set;//density
	bool TOL_is_set;//tolerance
	bool failure_strss_is_set;
	bool failure_strain_is_set;
protected:
};

Elastic::Elastic(){
	type = "ELASTIC";
	stiffness_matrix_3d = Array2D<double>(6, 6, 0.0);
	stiffness_matrix_plane_stress = Array2D<double>(6, 6, 0.0);
}

bool Elastic::isAKeyWord(const string &param){
	string keywords[10] = { "TITLE", "CONTROLS", "ELEMENTS",
		"NODES", "LOADS", "CONSTRAINTS", "MATERIALS", "TRACKERS", "GROUPS",
		"GEOMETRY" };
	for (int i = 0; i < 10; i++){
		if (keywords[i] == param)
			return true;
	}
	return false;
}


void Elastic::calculateStressOneDimensional(Array1D<double>& strain, Array1D<double>& dstrain, Array1D<double>& stress, double timestep){
	strain = dstrain;
	stress[0] = strain[0]* youngs_moduls;
}

void Elastic::calculateStressTwoDimensional(Array1D<double>& strain, Array1D<double>& dstrain, Array1D<double>& stress, double timestep){
	stress[0] += stiffness_matrix_plane_stress[0][0] * dstrain[0]
		+ stiffness_matrix_plane_stress[0][1] * dstrain[1];

	stress[1] += stiffness_matrix_plane_stress[1][0] * dstrain[0]
		+ stiffness_matrix_plane_stress[1][1] * dstrain[1];
	/************************************************************************/
	/* 这里怎么会没有stress[2][0]的计算                                                                     */
	/************************************************************************/
	stress[3] += stiffness_matrix_plane_stress[3][3] * dstrain[3];

	stress[4] += stiffness_matrix_plane_stress[4][4] * dstrain[4];

	stress[5] += stiffness_matrix_plane_stress[5][5] * dstrain[5];
	//为了防止单元过薄，更新一下厚度方向的应变（各项同性假设）
	dstrain[2] = -(dstrain[1] + dstrain[0])*(nu / (1 - nu));//如果是塑性材料，就要在应力达到屈服应力，也就是曲阜函数判据，后，以试探法计算塑性应变增量，塑性应变，通过屈服函数得到应力；而厚度方向应变=dstrain[1][0] - dstrain[0][0]
	//update strain
	strain += dstrain;
}

void Elastic::calculateStressThreeDimensional(Array1D<double>& strain, Array1D<double>& dstrain, Array1D<double>& stress, double timestep){
	strain += dstrain;
	stress = stiffness_matrix_3d * strain;//用array2D描述唯一的好处，就是不用再重写一堆矩阵运算的程序了，尤其是二维矩阵一维向量常数之间的运算
}
void Elastic::checkIndata(){
	if (!E_is_set)
	{
		throw runtime_error("未定义杨氏模量");
	}
	if (!RHO_is_set)
	{
		throw runtime_error("密度未定义");
	}
	if (!NU_is_set)
	{
		throw runtime_error("婆宋比未定义");
	}
}
void Elastic::setInitialCondition(){
	checkIndata();
	/************************************************************************/
	/* nu = possion ratio

	E		  1-nu	 nu		0
	D = -------------[ nu   1-nu    0	]
	(1+nu)(1-2nu)   0     0  (1-2nu)/2

	应该是弹性矩阵，而非刚度矩阵，

	还应该有个弹塑性矩阵
	=		  C       [


	*/
	/************************************************************************/


	double C = youngs_moduls / ((1.0 + nu)*(1 - 2.0*nu));
	double C2 = youngs_moduls / (1.0 - nu*nu);
	double G = youngs_moduls / (2.0*(1.0 + nu));
	double nuC = nu*C;
	//stiffness_matrix_3d for plane strain problem
	stiffness_matrix_3d[0][0] = (1.0 - nu)*C;
	stiffness_matrix_3d[0][1] = nu*C;
	stiffness_matrix_3d[0][2] = nu*C;
	//stiffness_matrix_3d[0][3], 04,05 = 0;
	stiffness_matrix_3d[1][0] = nu*C;
	stiffness_matrix_3d[1][1] = (1.0 - nu)*C;
	stiffness_matrix_3d[1][2] = nu*C;
	//stiffness_matrix_3d[1][3], 14,15 = 0;
	stiffness_matrix_3d[2][0] = nuC;
	stiffness_matrix_3d[2][1] = nuC;
	stiffness_matrix_3d[2][2] = (1.0 - nu)*C;
	//stiffness_matrix_3d[2][3], 24,25 = 0;
	stiffness_matrix_3d[3][3] = G;
	//stiffness_matrix_3d[3][0], 31,32,34,35 = 0;
	stiffness_matrix_3d[4][4] = G;
	//stiffness_matrix_3d[4][0], 41,42,43,45 = 0;
	stiffness_matrix_3d[5][5] = G;
	//stiffness_matrix_3d 50,51,52,53,54 = 0;

	//stiffness_matrix_plane_stress for plane stress problem;
	stiffness_matrix_plane_stress[0][0] = C2;
	stiffness_matrix_plane_stress[0][1] = nu*C2;
	//stiffness_matrix_plane_stress 02,03,04,05 = 0;
	stiffness_matrix_plane_stress[1][0] = nu*C2;
	stiffness_matrix_plane_stress[1][1] = C2;
	//stiffness_matrix_plane_stress 12, 13, 14, 15 = 0;
	//stiffness_matrix_plane_stress 20,21,22,23,24,25 = 0;
	stiffness_matrix_plane_stress[3][3] = ((1.0 - nu)*C2) / 2.0;
	//stiffness_matrix_plane_stress 30,31,32,34,35 = 0;
	stiffness_matrix_plane_stress[4][4] = ((5.0 / 6.0)*(1.0 - nu)*C2) / 2.0;
	//stiffness_matrix_plane_stress40, 41 ,42,43,45 = 0;
	stiffness_matrix_plane_stress[5][5] = ((5.0 / 6.0)*(1.0 - nu)*C2) / 2.0;
	//stiffness_matrix_plane_stress 50,51,52,53,54 = 0;
}


void Elastic::parseFembic(string& thisLine){

	using namespace boost;

	if (thisLine.empty()) return;
	//开始对thisLine进行字符扑街
	char_separator<char> sepa("= \t[],");//必须要用制表符作为分词符之一，否则会漏掉很多
	typedef tokenizer<char_separator<char>> Tokenizer;
	Tokenizer tok(thisLine, sepa);
	Tokenizer::iterator position = tok.begin();//first string in thisLine

	if ((*position) == "#") return;//ignore annonatations

	string tempString = *position;
	transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);//change into capital model

	if (isAKeyWord(tempString)){
		return;
	}

	name = *position;
	position++;
	//begin inner loop to parse the line

	while (position != tok.end()){

		//position++;
		tempString = *position;
		transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);

		map<string, int> mapStr = { { "E", 01 }, { "RHO", 02 }, { "NU", 03 }, { "FAILURE_STRESS", 11 }, { "FAILURE_STRAIN", 12 }, { "YIELD_STRESS", 13 }, { "EP", 14 },
		{ "V1", 21 }, { "V2", 22 }, { "V3", 23 }, { "V4", 24 }, { "V5", 25 }, { "V6", 26 }, { "V7", 27 }, { "V8", 28 }, { "V9", 29 }, { "Y1", 31 }, { "Y2", 32 }, { "Y3", 33 }, { "Y4", 34 }, { "Y5", 35 }, { "Y6", 36 },
		{ "Y7", 37 }, { "Y8", 38 }, { "Y9", 39 } };

		switch (mapStr[tempString])
		{
		case 01:
		{
			position++;
			tempString = *position;

			youngs_moduls = stod(tempString);
			E_is_set = true;
			cout << "youngs models is set" << endl;
			position++;
			continue;
		}
		case 02:
		{
			position++;
			density = stod(*position);
			position++;
			cout << "density is set" << endl;
			RHO_is_set = true;
			continue;
		}
		case  03:
		{

			position++;
			tempString = *position;
			nu = stod(*position);
			NU_is_set = true;
			cout << "possion ratio is set " << nu << endl;
			position++;
			continue;
		}
		case 11:
		{
			position++;
			tempString = *position;
			failure_strss = stod(tempString);
			failure_strss_is_set = true;
			position++;
			continue;
		}
		case 12:
		{
			position++;
			tempString = *position;


			failure_strain = stod(tempString);
			failure_strain_is_set = true;
			position++;
			continue;

		}
			break;
		}//switch
	}//while(position!=end())
}//function



#endif