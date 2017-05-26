#ifndef SOLID_ISO_6_H
#define SOLID_ISO_6_H


#include "allInclude.h"
using namespace std;

class Solid_Iso_6
{
public:
	Solid_Iso_6();
	void parse_Fembic(string& thisLine, vector<Elastic>& materialList, vector<Load> loadList, vector<Node>& nodeList, map<int, Node>& nodeTable);
	void checkInData();
	double checkTimeStep(double current_timestep);
	void setInitialConditions();
	void calculateMassMatrix();
	void calculateD(double sita, double phi, double etha);
	void calculateN();
	void calculateStrain(double step, int k);
	void calculateStress(int i, double timestep);
	void calculateNodalForces(int i, double timestep);
	void calculateContactForces();//empty
	void calculateExternalForces();//empty
	void checkIfFailed();
	void updateLocalBaseVector();//�ֲ�����ϵ�ĸ�����Ϊ�˽������ת������������ϵ�У���ʵ�嵥Ԫ�����ļ���������������ϵ������Ҫת��
	inline void deActivated(){ deactivated = true; }
	inline bool hasFailed(){
		return failed;
		/************************************************************************/
		/* 		if (internal_contact_element != null)
		internal_contact_element.deActivate();                                                                     */
		/************************************************************************/
	}
	inline bool isDeactivated(){ return deactivated; }
protected:
public:
	vector<Elastic> material;
	vector<Node> nodes;
	Array2D<double> mass;
	//�ο�������Ԫ�����ĸ�����Ӧ�á�P178
	Array2D<double> D;//ԭʼ�κ�������Ȼ����ƫ���õ��ľ���N�е�Ԫ�ش�D������
	Array2D<double> M;//�����������Ķ�ά����
	Array2D<double> N;
	Array2D<double> H;//���γ�Ӧ����λ�Ƶļ��ι�ϵʱʹ�õ� 0-1 ����
	Array1D<double> d;//λ�ƾ���������M�������
	Array1D<double> f;//�ڵ�������
	Array2D<double> P;//��չ�ſ˱Ⱦ�����ľ����ſ˱Ⱦ��������Ȼ����ϵ�Եѿ�������ϵ��ƫ�������ڵȲ�Ԫ��������Ȼ����ϵ���κ�����ƫ������*�ڵ��������

	vector<Array2D<double>> J;
	Array2D<double> J_inv;
	vector<Array1D<double>> strain;//8�����ֵ㣬ÿ�����ֵ㶼��һ��Ӧ�����
	vector<Array1D<double>> dstrain;
	vector<Array1D<double>> stress;
	vector<Array2D<double>> B;

	Array1D<double> xsi;
	Array1D<double> phi;
	Array1D<double> etha;
	Array1D<double> W;

	string type;
	int number_of_integration_points;
	int element_ID;
	bool NIP_is_set;
	bool Nodes_are_set;
	bool Material_is_set;
	bool failed;
	bool deactivated;
	double total_mass;
};

Solid_Iso_6::Solid_Iso_6(){
	type = "SOLID_ISO_6";

	material = vector<Elastic>(8);
	nodes = vector<Node>(8);
	mass = Array2D<double>(24, 24, 0.0);
	//������Ȼ���꣬�� �� �� 
	xsi = Array1D<double>(8,0.0);
	etha = Array1D<double>(8,0.0);
	phi = Array1D<double>(8,0.0);

	//��˹���ֵ�Ȩϵ��������8�����ֵ�
	W = Array1D<double>(8,0.0);;
	//���γ�Ӧ����λ�Ƶļ��ι�ϵʱʹ�õ� 0-1 ����
	H = Array2D<double>(6, 9, 0.0);
	//�����������Ķ�ά����
	M = Array2D<double>(8, 3, 0.0);
	d = Array1D<double>(24, 0.0);
	f = Array1D<double>(24, 0.0);
	//��չ���ſ˱Ⱦ���
	P = Array2D<double>(9, 9, 0.0);
	N = Array2D<double>(9, 24, 0.0);

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

void Solid_Iso_6::calculateMassMatrix(){

	/************************************************************************/
	/*��������������⣬����WK-LIU��50ҳ���������Ǧ�*N.transpose()*N*J*w(i,j,k)  
	Ҳ����˵���ٳ���һ��N*/
	/************************************************************************/
	
	double s = 0.0;

	for (int i = 0; i < 8;i++)
	{
		M[i][0] = nodes[i].pos[0];
		M[i][1] = nodes[i].pos[1];
		M[i][2] = nodes[i].pos[2];
	}
	calculateD(0.0, 0.0, 0.0);
	calculateN();

	//�ſ˱Ⱦ��� = ��Ȼ������κ�����ƫ��*�ѿ�������
	J[0] = D*M;
	LU<double> temp_LU(J[0]);
	double J_det = temp_LU.det();
	mass = N.transpose()*material[0].density*J_det*8.0;
	//ֻ�����Խ���
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

void Solid_Iso_6::calculateContactForces(){}
void Solid_Iso_6::calculateExternalForces(){}
void Solid_Iso_6::setInitialConditions(){
	double t = 1 / sqrt(3.0);

	for (int i = 0; i < number_of_integration_points; i++) {
		material[i] = material[0];
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


//�������Ϊ������Ȼ����
void Solid_Iso_6::calculateD(double sita, double phi, double etha){
	
	/************************************************************************/
	/* 
	N1 = (1-��)(1-��)(1+��)/8
	N3 = (1-��)(1-��)(1-��)/8
	N3 = (1-��)(1+��)(1-��)/8
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

void Solid_Iso_6::calculateN(){
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


void Solid_Iso_6::calculateStrain(double step, int k){
	//ÿ�����ֵ�Ȩϵ������+1/��3��-1/��3��ע�����ֻ��ֵ����Ȼ���꣬����D��N��ʱ���õ��ǻ��ֵ㣬
	//����B���е�������M�Լ�H��Ϊ�ѿ������ꡢ��Ȼ����
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
		d[3 * i] = nodes[i].dpl[0] - nodes[i].dpl_old[0];
		d[3 * i + 1] = nodes[i].dpl[1] - nodes[i].dpl_old[1];
		d[3 * i + 2] = nodes[i].dpl[2] - nodes[i].dpl_old[2];
	}

	J[k] = D*M;
	LU<double> inverse(J[k]);
	Array2D<double> E(3, 3, 0.0);
	Array2D<double> inverse_J(3, 3, 0.0);
	for (int i = 0; i < 3; i++)
	{
		E[i][i] = 1.0;
	}
	inverse_J = inverse.solve(E);
	P.subarray(0, 2, 0, 2) = inverse_J;
	P.subarray(3, 5, 3, 5) = inverse_J;
	P.subarray(6, 8, 6, 8) = inverse_J;

	B[k] = H*P*N;

	dstrain[k] = B[k] * d;
}

void Solid_Iso_6::calculateStress(int i, double timestep){

	material[i].calculateStressThreeDimensional(strain[i], dstrain[i], stress[i],timestep);

}
void Solid_Iso_6::calculateNodalForces(int i, double timestep){

	LU<double> temp_J(J[i]);
	double J_det = temp_J.det();

	f = B[i].transpose() * stress[i] * J_det*(W[i] * W[i] * W[i]);

	Array1D<double> global_force(3, 0.0);
	for (int j = 0; j < 8;j++)
	{
		global_force = f.subarray(3 * j, 3 * j + 2);
		nodes[j].addInternalForce(global_force*(-1.0));
	}


}

double Solid_Iso_6::checkTimeStep(double current_timestep){

	double critical_length = 1e10;
	double timestep;
	double temp_length;
	for (int i = 1; i < 8;i++)
	{
		temp_length = sqrt(pow((nodes[i].pos[0] - nodes[i - 1].pos[0]), 2) +
			pow((nodes[i].pos[1] - nodes[i - 1].pos[1]), 2) +
			pow((nodes[i].pos[2] - nodes[i - 1].pos[2]), 2));
		if (temp_length>0)
		{
			critical_length = min(temp_length, critical_length);
		}
	}
	for (int i = 7; i < 3; i--)
	{
		temp_length = sqrt(pow((nodes[i].pos[0] - nodes[i - 4].pos[0]), 2) +
			pow((nodes[i].pos[1] - nodes[i - 4].pos[1]), 2) +
			pow((nodes[i].pos[2] - nodes[i - 4].pos[2]), 2));
		if (temp_length > 0)
		{
			critical_length = min(temp_length, critical_length);
		}
	}
	temp_length = sqrt(pow((nodes[7].pos[0] - nodes[4].pos[0]), 2) +
		pow((nodes[7].pos[1] - nodes[4].pos[1]), 2) +
		pow((nodes[7].pos[2] - nodes[4].pos[2]), 2));
	if (temp_length > 0)
	{
		critical_length = min(temp_length, critical_length);
	}
	timestep = critical_length / material[0].wavespeedThreeDimendinal();
	return min(timestep, current_timestep);
}


void Solid_Iso_6::parse_Fembic(string& thisLine, vector<Elastic>& materialList, vector<Load> loadList, vector<Node>& nodeList, map<int, Node>& nodeTable){
	//��ʼ��thisLine�����ַ��˽�
	char_separator<char> sepa("= \t [],");//����Ҫ���Ʊ����Ϊ�ִʷ�֮һ�������©���ܶ�
	typedef tokenizer<char_separator<char>> Tokenizer;
	Tokenizer tok(thisLine, sepa);
	Tokenizer::iterator position = tok.begin();//first string in thisLine

	string tempString;
	position++;
	//transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);//change into capital model
	//if (isdigit(stod(tempString)))
	//{
	//		ElementID(stod(tempString));
	//}
	for (position; position != tok.end(); position++)
	{
		tempString = *position;
		transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
		if (tempString == "NODES"){
			position++;

			for (int i = 0; i < 8;i++)
			{
				position++;
				nodes[i] = nodeTable[(int)stod(*position)];
			}
			Nodes_are_set = true;
		}
		else if (tempString=="NIP")
		{
			position++;
			number_of_integration_points = (int)stod(*position);
			NIP_is_set = true;
		}
		else if (tempString == "MATERIAL")
		{
			position++;
			tempString = *position;
			transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
			//material = Element::findMaterial(*position, materialList);
			material[0] = materialList[0];
			material[0].name = tempString;
			Material_is_set = true;
			cout << "material is set ,it's name is " << tempString << " \n";
			cout << "material.density " << material[0].density << endl;
			cout << "material.failure_strss " << material[0].failure_strss << endl;
			cout << "material.yield_strss \n";
			cout << "material.V is \n";
			/************************************************************************/
			/*
			//for elastoplastic
			material.yield_stress.printV();
			for (auto v:material.V)
			{
			v.printV();
			}
			*/
			/************************************************************************/


		}
		//if contact is true, transfrom to contact le
	}
}

void Solid_Iso_6::checkInData(){
	if (!Material_is_set)
	{
		//throw exception
	}
	if (!Nodes_are_set)
	{
		//throw exception
	}
}

void Solid_Iso_6::checkIfFailed(){

	if (!material[0].failure_strss_is_set)
	{
		failed = false;
		return;
	}

	//�����ЧӦ�������ڶ�Ӧ��������J2��������Ƿ�����ƻ�Ӧ��

	double equivalence_stress=0.0;
	float d = sqrt(2.0) / 2.0;
	Array1D<double> temp_stress(6, 0.0);
	for (int i = 0; i < number_of_integration_points;i++)
	{
		temp_stress = stress[i];
		equivalence_stress += d*sqrt(pow(temp_stress[0] - temp_stress[1], 2)
			+ pow(temp_stress[1] - temp_stress[2], 2)
			+ pow(temp_stress[2] - temp_stress[0], 2)
			+ 6.0*(pow(temp_stress[3], 2) + pow(temp_stress[4], 2) + pow(temp_stress[5], 2)));
	}
	equivalence_stress /= number_of_integration_points;
	if (equivalence_stress > material[0].failure_strss)
	{
		failed = true;
		return;
	}
}

void Solid_Iso_6::updateLocalBaseVector(){

}

















#endif