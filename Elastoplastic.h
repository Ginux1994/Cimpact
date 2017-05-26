#ifndef ELASTOPLASTIC_H
#define ELASTOPLASTIC_H


#include "Variable.h"
#include "tnt.h"
#include "tnt_array2d.h"
#include <boost/tokenizer.hpp>

using namespace std;
using namespace TNT;
class Elastoplastic
{
public:
	Elastoplastic();

	//~Elastoplastic();

	void parseFembic(string& thisLine);
	bool isAKeyWord(const string &param);
	void setInitialCondition();
	void checkIndata();
	void calculateStressOneDimensional(Array2D<double>& strain, Array2D<double>& dstrain, Array2D<double>& stress, double timestep);
	void calculateStressTwoDimensional(Array2D<double>& strain, Array2D<double>& dstrain, Array2D<double>& stress, double timestep);
	void calculateStressThreeDimensional(Array2D<double>& strain, Array2D<double>& dstrain, Array2D<double>& stress, double timestep);
	double yieldStress(double plastic_strain, double strain_vel);//��������Ӧ�䣬����Ӧ���ʼ�������Ӧ��
	double yieldStressDerivate(double plastic_strain, double strain_vel);

	inline double wavespeedOneDimendinal(double a, double b){ return sqrt(youngs_moduls / density); }
	inline double wavespeedTwoDimendinal(double a, double b){ return sqrt(youngs_moduls / (density*(1.0 - nu*nu))); }
	inline double wavespeedThreeDimendinal(double a, double b){ return sqrt((youngs_moduls*(1.0 - nu) / (density*(1.0 + nu)*(1.0 - 2.0*nu)))); }
	/************************************************************************/
	/*
	cpp�ļ���д���ͻᱨ��ô������ͷ�Ĵ��󣬾���������Ϊ.h��Ҳ���У�ֻ��ɾ������дһ��
	����	11	error LNK2005: "public: void __thiscall Variable::printV(void)" (?printV@Variable@@QAEXXZ) �Ѿ��� Elastoplasticcpp.obj �ж���
	����	13	error LNK2005: "public: double __thiscall Variable::value(double)" (?value@Variable@@QAENN@Z) �Ѿ��� Elastoplasticcpp.obj �ж���
	����	10	error LNK2005: "public: double __thiscall Variable::derivate(double)" (?derivate@Variable@@QAENN@Z) �Ѿ��� Elastoplasticcpp.obj �ж���
	����	12	error LNK2005: "public: bool __thiscall Variable::status(double)" (?status@Variable@@QAE_NN@Z) �Ѿ��� Elastoplasticcpp.obj �ж���
	����	9	error LNK2005: "public: __thiscall Variable::Variable(void)" (??0Variable@@QAE@XZ) �Ѿ��� Elastoplasticcpp.obj �ж���
	����	8	error LNK2005: "public: __thiscall Variable::Variable(double)" (??0Variable@@QAE@N@Z) �Ѿ��� Elastoplasticcpp.obj �ж���
	����	7	error LNK2005: "public: __thiscall Variable::Variable(class std::vector<class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> >,class std::allocator<class std::basic_string<char,struct std::char_traits<char>,class std::allocator<char> > > > &)" (??0Variable@@QAE@AAV?$vector@V?$basic_string@DU?$char_traits@D@std@@V?$allocator@D@2@@std@@V?$allocator@V?$basic_string@DU?$char_traits@D@std@@V?$allocator@D@2@@std@@@2@@std@@@Z) �Ѿ��� Elastoplasticcpp.obj �ж���
	����	14	error LNK1169: �ҵ�һ���������ض���ķ���	1	1

	*/
	/************************************************************************/

protected:

public://public just for test
	Array2D<double> stiffness_matrix_3d;
	Array2D<double> stiffness_matrix_plane_stress;
	Array1D<double> trial_stress;
	Array2D<double> A;
	Array2D<double> I;//

	//public for test
	vector<Variable> yield_array;

	vector<Variable> V;

	string filePath;
	string type;
	string name;
	Variable yield_stress;
	double factor, eps, eps_vel;
	double density;
	double youngs_moduls;
	double nu;//possion ratio;
	double failure_strain;
	double failure_strss;
	double pressure;
	int Y_is_set;
	int V_is_set;
	bool E_is_set;//Y model;
	bool NU_is_set;//possion ritio
	bool RHO_is_set;//density
	bool YIELD_is_set;//yeild stress;
	bool EP_is_set;//plastic Y model
	bool EP_is_required;
	bool TOL_is_set;//tolerance
	bool failure_strss_is_set;
	bool failure_strain_is_set;
	bool yield_stress_is_set;

};



Elastoplastic::Elastoplastic(){
	type = "ELASTOPLASTIC";
	trial_stress = Array1D<double>(6, 0.0);
	stiffness_matrix_plane_stress = Array2D<double>(6, 6, 0.0);
	stiffness_matrix_3d = Array2D<double>(6, 6, 0.0);
	A = Array2D<double>(6, 6, 0.0);
	I = Array2D<double>(6, 6, 0.0);
	eps = 0.0;
	eps_vel = 0.0;

}
bool Elastoplastic::isAKeyWord(const string &param){
	string keywords[10] = { "TITLE", "CONTROLS", "ELEMENTS",
		"NODES", "LOADS", "CONSTRAINTS", "MATERIALS", "TRACKERS", "GROUPS",
		"GEOMETRY" };
	for (int i = 0; i < 10; i++){
		if (keywords[i] == param)
			return true;
	}
	return false;
}

void Elastoplastic::checkIndata(){
	if (!E_is_set)
	{
		throw runtime_error("δ��������ģ��");
	}
	if (!RHO_is_set)
	{
		throw runtime_error("�ܶ�δ����");
	}
	if (!NU_is_set)
	{
		throw runtime_error("���α�δ����");
	}
}


void Elastoplastic::parseFembic(string& thisLine){

	using namespace boost;

	if (thisLine.empty()) return;
	//��ʼ��thisLine�����ַ��˽�
	char_separator<char> sepa("= \t[],");//����Ҫ���Ʊ����Ϊ�ִʷ�֮һ�������©���ܶ�
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
		case 13:
		{
			vector<string> vecYield_stress;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
				if ((tempString == "E") || (tempString == "RHI") || (tempString == "NU") ||
					(tempString == "FAILURE_STRESS") || (tempString == "FAILURE_STRAIN") || (tempString == "EP") ||
					(tempString == "YIELD_STRESS") || (tempString == "V1") || (tempString == "V2") ||
					(tempString == "V3") || (tempString == "V4") || (tempString == "V5") ||
					(tempString == "V6") || (tempString == "V7") || (tempString == "V8") || (tempString == "V9") ||
					(tempString == "Y1") || (tempString == "Y2") || (tempString == "Y3") || (tempString == "Y4") ||
					(tempString == "Y5") || (tempString == "Y6") || (tempString == "Y7") || (tempString == "Y8") || (tempString == "Y9"))break;
				vecYield_stress.push_back(tempString);
			}
			//Variable Yield_stress(vecYield_stress);
			//yield_stress = Yield_stress;
			//yield_stress(vecYield_stress);
			if (vecYield_stress.size()==1)
			{
				double fuckD = stod(vecYield_stress[0]);
				yield_stress = Variable(fuckD);
			}
			else{
				yield_stress = Variable(vecYield_stress);
			}
			yield_stress_is_set = true;
			cout << "yield stree is set " << endl;
			yield_stress.printV();
			continue;
		}
		case 14:
		{
			position++;
			tempString = *position;
			factor = stod(tempString);
			EP_is_set = true;
			cout << "Plastic Strain vs Plastic Stress andgle of the material is set " << factor << endl;
			position++;
			continue;
		}
		case 21:
		{
			vector<string> vecV1;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "E") || (tempString == "RHI") || (tempString == "NU") ||
					(tempString == "FAILURE_STRESS") || (tempString == "FAILURE_STRAIN") ||
					(tempString == "YIELD_STRESS") || (tempString == "V1") || (tempString == "V2") ||
					(tempString == "V3") || (tempString == "V4") || (tempString == "V5") ||
					(tempString == "V6") || (tempString == "V7") || (tempString == "V8") || (tempString == "V9") ||
					(tempString == "Y1") || (tempString == "Y2") || (tempString == "Y3") || (tempString == "Y4") ||
					(tempString == "Y5") || (tempString == "Y6") || (tempString == "Y7") || (tempString == "Y8") || (tempString == "Y9"))break;
				vecV1.push_back(tempString);
			}
			if (vecV1.size() == 1)
			{
				double fuckD = stod(vecV1[0]);
				V.push_back(Variable(fuckD));
			}
			else{
				V.push_back(Variable(vecV1));
			}
			V_is_set = ((V_is_set < 1) ? 1 : V_is_set);
			continue;
		}
		case 22:
		{
			vector<string> vecV2;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "E") || (tempString == "RHI") || (tempString == "NU") ||
					(tempString == "FAILURE_STRESS") || (tempString == "FAILURE_STRAIN") ||
					(tempString == "YIELD_STRESS") || (tempString == "V1") || (tempString == "V2") ||
					(tempString == "V3") || (tempString == "V4") || (tempString == "V5") ||
					(tempString == "V6") || (tempString == "V7") || (tempString == "V8") || (tempString == "V9") ||
					(tempString == "Y1") || (tempString == "Y2") || (tempString == "Y3") || (tempString == "Y4") ||
					(tempString == "Y5") || (tempString == "Y6") || (tempString == "Y7") || (tempString == "Y8") || (tempString == "Y9"))break;
				vecV2.push_back(tempString);
			}
			if (vecV2.size() == 1)
			{
				double fuckD = stod(vecV2[0]);
				V.push_back(Variable(fuckD));
			}
			else{
				V.push_back(Variable(vecV2));
			}
			V_is_set = ((V_is_set < 2) ? 2 : V_is_set);
			continue;
		}
		case 23:
		{
			vector<string> vecV3;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "E") || (tempString == "RHI") || (tempString == "NU") ||
					(tempString == "FAILURE_STRESS") || (tempString == "FAILURE_STRAIN") ||
					(tempString == "YIELD_STRESS") || (tempString == "V1") || (tempString == "V2") ||
					(tempString == "V3") || (tempString == "V4") || (tempString == "V5") ||
					(tempString == "V6") || (tempString == "V7") || (tempString == "V8") || (tempString == "V9") ||
					(tempString == "Y1") || (tempString == "Y2") || (tempString == "Y3") || (tempString == "Y4") ||
					(tempString == "Y5") || (tempString == "Y6") || (tempString == "Y7") || (tempString == "Y8") || (tempString == "Y9"))break;
				vecV3.push_back(tempString);
			}
			if (vecV3.size() == 1)
			{
				double fuckD = stod(vecV3[0]);
				V.push_back(Variable(fuckD));
			}
			else{
				V.push_back(Variable(vecV3));
			}
			V_is_set = ((V_is_set < 3) ? 3 : V_is_set);
			continue;
		}
		case 24:
		{
			vector<string> vecV4;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "E") || (tempString == "RHI") || (tempString == "NU") ||
					(tempString == "FAILURE_STRESS") || (tempString == "FAILURE_STRAIN") ||
					(tempString == "YIELD_STRESS") || (tempString == "V1") || (tempString == "V2") ||
					(tempString == "V3") || (tempString == "V4") || (tempString == "V5") ||
					(tempString == "V6") || (tempString == "V7") || (tempString == "V8") || (tempString == "V9") ||
					(tempString == "Y1") || (tempString == "Y2") || (tempString == "Y3") || (tempString == "Y4") ||
					(tempString == "Y5") || (tempString == "Y6") || (tempString == "Y7") || (tempString == "Y8") || (tempString == "Y9"))break;
				vecV4.push_back(tempString);
			}
			if (vecV4.size() == 1)
			{
				double fuckD = stod(vecV4[0]);
				V.push_back(Variable(fuckD));
			}
			else{
				V.push_back(Variable(vecV4));
			}
			V_is_set = ((V_is_set < 4) ? 4 : V_is_set);
			continue;
		}
		case 25:
		{
			vector<string> vecV5;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "E") || (tempString == "RHI") || (tempString == "NU") ||
					(tempString == "FAILURE_STRESS") || (tempString == "FAILURE_STRAIN") ||
					(tempString == "YIELD_STRESS") || (tempString == "V1") || (tempString == "V2") ||
					(tempString == "V3") || (tempString == "V4") || (tempString == "V5") ||
					(tempString == "V6") || (tempString == "V7") || (tempString == "V8") || (tempString == "V9") ||
					(tempString == "Y1") || (tempString == "Y2") || (tempString == "Y3") || (tempString == "Y4") ||
					(tempString == "Y5") || (tempString == "Y6") || (tempString == "Y7") || (tempString == "Y8") || (tempString == "Y9"))break;
				vecV5.push_back(tempString);
			}
			if (vecV5.size() == 1)
			{
				double fuckD = stod(vecV5[0]);
				V.push_back(Variable(fuckD));
			}
			else{
				V.push_back(Variable(vecV5));
			}
			V_is_set = ((V_is_set < 5) ? 5 : V_is_set);
			continue;
		}
		case 26:
		{
			vector<string> vecV6;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "E") || (tempString == "RHI") || (tempString == "NU") ||
					(tempString == "FAILURE_STRESS") || (tempString == "FAILURE_STRAIN") ||
					(tempString == "YIELD_STRESS") || (tempString == "V1") || (tempString == "V2") ||
					(tempString == "V3") || (tempString == "V4") || (tempString == "V5") ||
					(tempString == "V6") || (tempString == "V7") || (tempString == "V8") || (tempString == "V9") ||
					(tempString == "Y1") || (tempString == "Y2") || (tempString == "Y3") || (tempString == "Y4") ||
					(tempString == "Y5") || (tempString == "Y6") || (tempString == "Y7") || (tempString == "Y8") || (tempString == "Y9"))break;
				vecV6.push_back(tempString);
			}
			if (vecV6.size() == 1)
			{
				double fuckD = stod(vecV6[0]);
				V.push_back(Variable(fuckD));
			}
			else{
				V.push_back(Variable(vecV6));
			}
			V_is_set = ((V_is_set < 6) ? 6 : V_is_set);
			cout << "V6 IS SET";
			for (auto v : V){
				v.printV();
			}
			continue;
		}
		case 27:
		{
			vector<string> vecV7;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "E") || (tempString == "RHI") || (tempString == "NU") ||
					(tempString == "FAILURE_STRESS") || (tempString == "FAILURE_STRAIN") ||
					(tempString == "YIELD_STRESS") || (tempString == "V1") || (tempString == "V2") ||
					(tempString == "V3") || (tempString == "V4") || (tempString == "V5") ||
					(tempString == "V6") || (tempString == "V7") || (tempString == "V8") || (tempString == "V9") ||
					(tempString == "Y1") || (tempString == "Y2") || (tempString == "Y3") || (tempString == "Y4") ||
					(tempString == "Y5") || (tempString == "Y6") || (tempString == "Y7") || (tempString == "Y8") || (tempString == "Y9"))break;
				vecV7.push_back(tempString);
			}
			if (vecV7.size() == 1)
			{
				double fuckD = stod(vecV7[0]);
				V.push_back(Variable(fuckD));
			}
			else{
				V.push_back(Variable(vecV7));
			}
			V_is_set = ((V_is_set < 7) ? 7 : V_is_set);
			continue;
		}
		case 28:
		{
			vector<string> vecV8;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "E") || (tempString == "RHI") || (tempString == "NU") ||
					(tempString == "FAILURE_STRESS") || (tempString == "FAILURE_STRAIN") ||
					(tempString == "YIELD_STRESS") || (tempString == "V1") || (tempString == "V2") ||
					(tempString == "V3") || (tempString == "V4") || (tempString == "V5") ||
					(tempString == "V6") || (tempString == "V7") || (tempString == "V8") || (tempString == "V9") ||
					(tempString == "Y1") || (tempString == "Y2") || (tempString == "Y3") || (tempString == "Y4") ||
					(tempString == "Y5") || (tempString == "Y6") || (tempString == "Y7") || (tempString == "Y8") || (tempString == "Y9"))break;
				vecV8.push_back(tempString);
			}
			if (vecV8.size() == 1)
			{
				double fuckD = stod(vecV8[0]);
				V.push_back(Variable(fuckD));
			}
			else{
				V.push_back(Variable(vecV8));
			}
			V_is_set = ((V_is_set < 8) ? 8 : V_is_set);
			continue;
		}
		case 29:
		{
			vector<string> vecV9;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "E") || (tempString == "RHI") || (tempString == "NU") ||
					(tempString == "FAILURE_STRESS") || (tempString == "FAILURE_STRAIN") ||
					(tempString == "YIELD_STRESS") || (tempString == "V1") || (tempString == "V2") ||
					(tempString == "V3") || (tempString == "V4") || (tempString == "V5") ||
					(tempString == "V6") || (tempString == "V7") || (tempString == "V8") || (tempString == "V9") ||
					(tempString == "Y1") || (tempString == "Y2") || (tempString == "Y3") || (tempString == "Y4") ||
					(tempString == "Y5") || (tempString == "Y6") || (tempString == "Y7") || (tempString == "Y8") || (tempString == "Y9"))break;
				vecV9.push_back(tempString);
			}
			if (vecV9.size() == 1)
			{
				double fuckD = stod(vecV9[0]);
				V.push_back(Variable(fuckD));
			}
			else{
				V.push_back(Variable(vecV9));
			}
			V_is_set = ((V_is_set < 9) ? 9 : V_is_set);
			continue;
		}
		case 31:
		{
			vector<string> vecY1;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "E") || (tempString == "RHI") || (tempString == "NU") ||
					(tempString == "FAILURE_STRESS") || (tempString == "FAILURE_STRAIN") ||
					(tempString == "YIELD_STRESS") || (tempString == "V1") || (tempString == "V2") ||
					(tempString == "V3") || (tempString == "V4") || (tempString == "V5") ||
					(tempString == "V6") || (tempString == "V7") || (tempString == "V8") || (tempString == "V9") ||
					(tempString == "Y1") || (tempString == "Y2") || (tempString == "Y3") || (tempString == "Y4") ||
					(tempString == "Y5") || (tempString == "Y6") || (tempString == "Y7") || (tempString == "Y8") || (tempString == "Y9"))break;
				vecY1.push_back(tempString);
			}
			if (vecY1.size() == 1)
			{
				double fuckD = stod(vecY1[0]);
				V.push_back(Variable(fuckD));
			}
			else{
				V.push_back(Variable(vecY1));
			}
			Y_is_set = ((Y_is_set < 1) ? 1 : Y_is_set);
			continue;
		}
		case 32:
		{
			vector<string> vecY2;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "E") || (tempString == "RHI") || (tempString == "NU") ||
					(tempString == "FAILURE_STRESS") || (tempString == "FAILURE_STRAIN") ||
					(tempString == "YIELD_STRESS") || (tempString == "V1") || (tempString == "V2") ||
					(tempString == "V3") || (tempString == "V4") || (tempString == "V5") ||
					(tempString == "V6") || (tempString == "V7") || (tempString == "V8") || (tempString == "V9") ||
					(tempString == "Y1") || (tempString == "Y2") || (tempString == "Y3") || (tempString == "Y4") ||
					(tempString == "Y5") || (tempString == "Y6") || (tempString == "Y7") || (tempString == "Y8") || (tempString == "Y9"))break;
				vecY2.push_back(tempString);
			}
			if (vecY2.size() == 1)
			{
				double fuckD = stod(vecY2[0]);
				V.push_back(Variable(fuckD));
			}
			else{
				V.push_back(Variable(vecY2));
			}
			Y_is_set = ((Y_is_set < 2) ? 2 : Y_is_set);
			continue;
		}
		case 33:
		{
			vector<string> vecY3;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "E") || (tempString == "RHI") || (tempString == "NU") ||
					(tempString == "FAILURE_STRESS") || (tempString == "FAILURE_STRAIN") ||
					(tempString == "YIELD_STRESS") || (tempString == "V1") || (tempString == "V2") ||
					(tempString == "V3") || (tempString == "V4") || (tempString == "V5") ||
					(tempString == "V6") || (tempString == "V7") || (tempString == "V8") || (tempString == "V9") ||
					(tempString == "Y1") || (tempString == "Y2") || (tempString == "Y3") || (tempString == "Y4") ||
					(tempString == "Y5") || (tempString == "Y6") || (tempString == "Y7") || (tempString == "Y8") || (tempString == "Y9"))break;
				vecY3.push_back(tempString);
			}
			if (vecY3.size() == 1)
			{
				double fuckD = stod(vecY3[0]);
				V.push_back(Variable(fuckD));
			}
			else{
				V.push_back(Variable(vecY3));
			}
			Y_is_set = ((Y_is_set < 3) ? 3 : Y_is_set);
			continue;
		}
		case 34:
		{
			vector<string> vecY4;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "E") || (tempString == "RHI") || (tempString == "NU") ||
					(tempString == "FAILURE_STRESS") || (tempString == "FAILURE_STRAIN") ||
					(tempString == "YIELD_STRESS") || (tempString == "V1") || (tempString == "V2") ||
					(tempString == "V3") || (tempString == "V4") || (tempString == "V5") ||
					(tempString == "V6") || (tempString == "V7") || (tempString == "V8") || (tempString == "V9") ||
					(tempString == "Y1") || (tempString == "Y2") || (tempString == "Y3") || (tempString == "Y4") ||
					(tempString == "Y5") || (tempString == "Y6") || (tempString == "Y7") || (tempString == "Y8") || (tempString == "Y9"))break;
				vecY4.push_back(tempString);
			}
			if (vecY4.size() == 1)
			{
				double fuckD = stod(vecY4[0]);
				V.push_back(Variable(fuckD));
			}
			else{
				V.push_back(Variable(vecY4));
			}

			Y_is_set = ((Y_is_set < 4) ? 4 : Y_is_set);
			continue;
		}
		case 35:
		{
			vector<string> vecY5;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "E") || (tempString == "RHI") || (tempString == "NU") ||
					(tempString == "FAILURE_STRESS") || (tempString == "FAILURE_STRAIN") ||
					(tempString == "YIELD_STRESS") || (tempString == "V1") || (tempString == "V2") ||
					(tempString == "V3") || (tempString == "V4") || (tempString == "V5") ||
					(tempString == "V6") || (tempString == "V7") || (tempString == "V8") || (tempString == "V9") ||
					(tempString == "Y1") || (tempString == "Y2") || (tempString == "Y3") || (tempString == "Y4") ||
					(tempString == "Y5") || (tempString == "Y6") || (tempString == "Y7") || (tempString == "Y8") || (tempString == "Y9"))break;
				vecY5.push_back(tempString);
			}
			if (vecY5.size() == 1)
			{
				double fuckD = stod(vecY5[0]);
				V.push_back(Variable(fuckD));
			}
			else{
				V.push_back(Variable(vecY5));
			}
			Y_is_set = ((Y_is_set < 5) ? 5 : Y_is_set);
			continue;
		}
		case 36:
		{
			vector<string> vecY6;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "E") || (tempString == "RHI") || (tempString == "NU") ||
					(tempString == "FAILURE_STRESS") || (tempString == "FAILURE_STRAIN") ||
					(tempString == "YIELD_STRESS") || (tempString == "V1") || (tempString == "V2") ||
					(tempString == "V3") || (tempString == "V4") || (tempString == "V5") ||
					(tempString == "V6") || (tempString == "V7") || (tempString == "V8") || (tempString == "V9") ||
					(tempString == "Y1") || (tempString == "Y2") || (tempString == "Y3") || (tempString == "Y4") ||
					(tempString == "Y5") || (tempString == "Y6") || (tempString == "Y7") || (tempString == "Y8") || (tempString == "Y9"))break;
				vecY6.push_back(tempString);
			}
			if (vecY6.size() == 1)
			{
				double fuckD = stod(vecY6[0]);
				V.push_back(Variable(fuckD));
			}
			else{
				V.push_back(Variable(vecY6));
			}
			Y_is_set = ((Y_is_set < 6) ? 6 : Y_is_set);
			continue;
		}
		case 37:
		{
			vector<string> vecY7;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "E") || (tempString == "RHI") || (tempString == "NU") ||
					(tempString == "FAILURE_STRESS") || (tempString == "FAILURE_STRAIN") ||
					(tempString == "YIELD_STRESS") || (tempString == "V1") || (tempString == "V2") ||
					(tempString == "V3") || (tempString == "V4") || (tempString == "V5") ||
					(tempString == "V6") || (tempString == "V7") || (tempString == "V8") || (tempString == "V9") ||
					(tempString == "Y1") || (tempString == "Y2") || (tempString == "Y3") || (tempString == "Y4") ||
					(tempString == "Y5") || (tempString == "Y6") || (tempString == "Y7") || (tempString == "Y8") || (tempString == "Y9"))break;
				vecY7.push_back(tempString);
			}
			if (vecY7.size() == 1)
			{
				double fuckD = stod(vecY7[0]);
				V.push_back(Variable(fuckD));
			}
			else{
				V.push_back(Variable(vecY7));
			}
			Y_is_set = ((Y_is_set < 7) ? 7 : Y_is_set);
			continue;
		}
		case 38:
		{
			vector<string> vecY8;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "E") || (tempString == "RHI") || (tempString == "NU") ||
					(tempString == "FAILURE_STRESS") || (tempString == "FAILURE_STRAIN") ||
					(tempString == "YIELD_STRESS") || (tempString == "V1") || (tempString == "V2") ||
					(tempString == "V3") || (tempString == "V4") || (tempString == "V5") ||
					(tempString == "V6") || (tempString == "V7") || (tempString == "V8") || (tempString == "V9") ||
					(tempString == "Y1") || (tempString == "Y2") || (tempString == "Y3") || (tempString == "Y4") ||
					(tempString == "Y5") || (tempString == "Y6") || (tempString == "Y7") || (tempString == "Y8") || (tempString == "Y9"))break;
				vecY8.push_back(tempString);
			}
			if (vecY8.size() == 1)
			{
				double fuckD = stod(vecY8[0]);
				V.push_back(Variable(fuckD));
			}
			else{
				V.push_back(Variable(vecY8));
			}
			Y_is_set = ((Y_is_set < 8) ? 8 : Y_is_set);
			continue;
		}
		case 39:
		{
			vector<string> vecY9;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "E") || (tempString == "RHI") || (tempString == "NU") ||
					(tempString == "FAILURE_STRESS") || (tempString == "FAILURE_STRAIN") ||
					(tempString == "YIELD_STRESS") || (tempString == "V1") || (tempString == "V2") ||
					(tempString == "V3") || (tempString == "V4") || (tempString == "V5") ||
					(tempString == "V6") || (tempString == "V7") || (tempString == "V8") || (tempString == "V9") ||
					(tempString == "Y1") || (tempString == "Y2") || (tempString == "Y3") || (tempString == "Y4") ||
					(tempString == "Y5") || (tempString == "Y6") || (tempString == "Y7") || (tempString == "Y8") || (tempString == "Y9"))break;
				vecY9.push_back(tempString);
			}
			if (vecY9.size() == 1)
			{
				double fuckD = stod(vecY9[0]);
				V.push_back(Variable(fuckD));
			}
			else{
				V.push_back(Variable(vecY9));
			}
			Y_is_set = ((Y_is_set < 9) ? 9 : Y_is_set);
			continue;
		}
			break;
		}//switch
	}//while(position!=end())




}//function

void Elastoplastic::setInitialCondition(){

	checkIndata();
	/************************************************************************/
	/* nu = possion ratio
	
			  E		  1-nu	 nu		0 	
	D = -------------[ nu   1-nu    0	]
		(1+nu)(1-2nu)   0     0  (1-2nu)/2


						
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
	stiffness_matrix_3d[2][2] = C - nuC;
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

	//// A matrix for material law plane stress
	A[0][0] = 1.0;
	A[0][1] = -0.5;
	A[1][0] = -0.5;
	A[3][3] = 3.0;
	//����ȫ��0��

	//��λ����
	for (int i = 0; i < 6; i++)
	{
		I[i][i] = 1;
	}
}

void Elastoplastic::calculateStressOneDimensional(Array2D<double>& strain, Array2D<double>& dstrain, Array2D<double>& stress, double timestep){
	double yield_function;//��������
	double strain_vel;//Ӧ����
	/************************************************************************/
	/* 1. ����Ӧ�䣬Ӧ������������Ӧ��
	2.���ݱ�����ϵ������Ӧ����������֮������ԭʼӦ��
	3.��������������Ҳ���������о�
	4.�����������Ӧ�����ʣ�Ӧ������/ʱ�䲽
				Ӧ�䣺����Ӧ����������
				���������ڣ�Ӧ��Ӧ���Ķ�Ӧ��ϵ���ߣ���������Ӧ��
	/************************************************************************/
	//update strain
	strain += dstrain;
	//calculate trail stressӦ���仯��ʷ�켣
	stress[0][0] += youngs_moduls*dstrain[0][0];
	//calculate yield function
	yield_function = abs(stress[0][0] - yieldStress(eps, eps_vel));

	//����Ƿ񳬹�����Ӧ��
	if (yield_function>0)
	{
		//���µ�Ч����Ӧ������
		eps_vel = abs(dstrain[0][0] / timestep);//����Ӧ����
		eps += abs(dstrain[0][0]);//����Ӧ��
		//����Ӧ������Ϊһά���⣬epsilonXX ���ǵ�ЧӦ��
		stress[0][0] = yieldStress(eps, eps_vel);
	}


}
//return the yield stress as a function of effctive plastic strain
double Elastoplastic::yieldStress(double plastic_strain, double strain_vel){
	if (yield_stress.is_a_constant)
	{//yield stress is a constant generially; ����Ӧ����Ϊvariableʱ����Ӧ��stepTime��Ӧ�䲻��ʱ�䣬stepValue��Ӧ������Ӧ����Ӧ���Ľױ��ϵ
	//factor������Ӧ��������Ӧ��֮��
		return yield_stress.value(plastic_strain) + factor * plastic_strain;//
	}
	else if (V_is_set==0)
	{
		return yield_stress.value(plastic_strain);
	}
	else return yield_stress.value(plastic_strain, strain_vel);
}

void Elastoplastic::calculateStressTwoDimensional(Array2D<double>& strain, Array2D<double>& dstrain, Array2D<double>& stress, double timestep){
	double yield_function;//��������
	double strain_vel;//Ӧ����
	stress[0][0] += stiffness_matrix_plane_stress[0][0] * dstrain[0][0]
		+ stiffness_matrix_plane_stress[0][1] * dstrain[1][0];

	stress[1][0] += stiffness_matrix_plane_stress[1][0] * dstrain[0][0]
		+ stiffness_matrix_plane_stress[1][1] * dstrain[1][0];

	stress[3][0] += stiffness_matrix_plane_stress[3][3] * dstrain[3][0];

	stress[4][0] += stiffness_matrix_plane_stress[4][4] * dstrain[4][0];

	stress[5][0] += stiffness_matrix_plane_stress[5][5] * dstrain[5][0];

		yield_function = sqrt(
		stress[0][0] * stress[0][0] +
		stress[1][0] * stress[1][0] -
		stress[0][0] * stress[1][0] +
	3.0*stress[3][0] * stress[3][0]);
	
	if (yield_function>yieldStress(eps,eps_vel))
	{
		//��������Ӧ��������Ӧ��
		eps_vel = (yield_function - yieldStress(eps, eps_vel) / (youngs_moduls*timestep));
		eps += eps_vel*timestep;
		//��Ԥ����Ӧ���켣�������������棨Ҳ����ͨ��variable����Ӧ�������Ӧ��Ӧ���ϵ���ߣ�
		stress *= yieldStress(eps, eps_vel) / yield_function;
		//��ȷ����Ӧ������ֻ��������֤�������
		dstrain[2][0] = dstrain[1][0] - dstrain[0][0];
	}
	else{
		//���Խ׶Σ��ں�ȷ����Եĸ���Ӧ������
		dstrain[2][0] = -(dstrain[1][0] + dstrain[0][0])*(nu / (1 - nu));
	}
}

void Elastoplastic::calculateStressThreeDimensional(Array2D<double>& strain, Array2D<double>& dstrain, Array2D<double>& stress, double timestep){

	double yield_function;//��������
	double strain_vel;//Ӧ����

	//���Է�Χ�ڣ�����Ӧ�����������õ���̽Ӧ�������ŵ�Ӧ��������
	stress += (stiffness_matrix_3d*dstrain);
	//������Ӧ������Ӧ��ƫ��
	pressure = -(stress[0][0] + stress[1][0] + stress[2][0]) / 3.0;
	stress[0][0] += pressure;
	stress[1][0] += pressure;
	stress[2][0] += pressure;

	yield_function = sqrt(
	1.5*stress[0][0] * stress[0][0] +
		stress[1][0] * stress[1][0] +
		stress[2][0] * stress[2][0] +
	3.0*stress[3][0] * stress[3][0] +
		stress[4][0] * stress[4][0] +
		stress[5][0] * stress[5][0]);
	if (yield_function > yieldStress(eps, eps_vel))
	{
		//���µ�Ч����Ӧ��
		//eps_vel = abs(dstrain[0][0] / timestep);
		eps_vel = ( (yield_function - yieldStress(eps, eps_vel)) / (((3.0*youngs_moduls*timestep)/(2.0*(1.0+nu)))+yieldStressDerivate(eps,eps_vel)) )/timestep;
		eps += (eps_vel*timestep);
		//��Ԥ����Ӧ���켣�������������棨Ҳ����ͨ��variable����Ӧ�������Ӧ��Ӧ���ϵ���ߣ�
		stress *= yieldStress(eps, eps_vel) / yield_function;
	}

	stress[0][0] -= pressure;
	stress[1][0] -= pressure;
	stress[2][0] -= pressure;

	strain += dstrain;
}

//��Ӧ��������Ϊһ�� ��Ч����Ӧ��� ������ʹ�ã����ʻ�����Variable�ඨ���Derivate��ֵ
double Elastoplastic::yieldStressDerivate(double plastic_strain, double strain_vel){
	if (yield_stress.is_a_constant)
	{//yield stress is a constant generially; ����Ӧ����Ϊvariableʱ����Ӧ��stepTime��Ӧ�䲻��ʱ�䣬stepValue��Ӧ������Ӧ����Ӧ���Ľױ��ϵ
		//factor������Ӧ��������Ӧ��֮��
		return factor;//
	}
	else if (V_is_set == 0)
	{
		return yield_stress.derivate(plastic_strain);
	}
	else return yield_stress.derivate(plastic_strain, strain_vel);
}


#endif