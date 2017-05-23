#include <iostream>

#include "allInclude.h"
//#include "ModelSmp.h"
#include "BdCondition.h"
#include "jama_lu.h"
//#include "jama_lu.h"
#include <fstream>
#include <string>
#include <map>
//#include "stdafx.h"
using namespace std;
using namespace JAMA;
using namespace TNT;

int main3(){

	string filePath = "E:\\1.txt";

	ofstream outFile;

	outFile.open(filePath);
	if (!outFile.is_open())
	{
		cout << "file to open the file \n";
	}
	map<int, string> outString;

	for (int i = 0; i < 10;i++)
	{
		outString.insert({ i, "abc" });
		outFile << outString[i] << "\t" << i << "\n";

	}
	outFile << flush;//����д��Ӳ����
	outFile.close();
	system("pause");
	return 0;

}



int main2(){




	Array2D<double> inertia(3, 3, 0.0);
	Array2D<double> inv_inertia(3, 3, 0.0);
	Array2D<double> A(3, 3, 0.0);

	Array2D<double> E(3, 3, 0.0);
	for (int i = 0; i < 3; i++)
	{
		E[i][i] = 1.0;
		/************************************************************************/
		/* 
				inertia[i][i] = 0.00000000000000000000000000000000000000000000000000000000
				00000000000000000000000000000000000000000000000000000000000000000000001;
				//���д��С�����126λ���ٶ�ͻ��ڴ滺�����������	34	error C1064: ���������� : ���������ڲ�������	25	1

		*/
		/************************************************************************/
		inertia[i][i] = 0.01;//���д��С�����126λ���ٶ�ͻ��ڴ滺�����������	34	error C1064: ���������� : ���������ڲ�������	25	1

	}


	LU<double> inverse_inertia(inertia);
	inv_inertia = inverse_inertia.solve(E);
	cout << "inverse inertia is \n";
	cout << inv_inertia;


	for (int i = 0; i < 3; i++)
	{
		A[i][i] = 10;
	}

	LU<double> invers_A(A);

	cout << invers_A.getL();
	cout << invers_A.getU();

	cout << invers_A.solve(E);



	cout << A;

	Array1D<double> ax(3,1.0);
	
	Array1D<double> ay(3, 1.0);

	cout << ay.crossProduct(ax);

	system("pause");
	return 0;
}


int main(){

	string fileName = "E:\\Ver_13.in";
//	ModelSmp model(1, fileName);
	Array1D<double> T1(2, 0.0);
	T1[0] = 1.0;
	T1[1] = 2.0;
	Array1D<double> T2(2, 1.0);
	T2[0] = 3.0;
	T2[1] = 4.0;
	Array1D<double> T3(3, 1.0);
	vector<Array1D<double>> test;
	test = {T1,T2,T3};

	for (auto tempArray:test)
	{
		cout<<tempArray;
	}

	Array2D<double> e(2, 2, 1.0);
	LU<double> lu(e);
	cout << lu.det();

//	cout<<e.det();

	//������assembleMassMatrix �����ýڵ��������ֲ������ܸ��ڵ����ó�ʼ����
//	model.assembleMassMatrix();/	model.setInitialContion();

//	model.assembleMassMatrix();
//	model.setInitialContion();
//	model.solve();


	/*
		map<int, string> m = { { 1, "a" }, { 2, "b" } };
	for (auto value : m){
		cout << value.first<<value.second << endl;
	}
	*/


	system("pause");
	return 0;
}


int main1(){


	//string fileName = "E:\\Bullet_AKM.in";
	string fileName = "E:\\test.txt";
	Fembic fem(fileName);

	vector<Node> nodeList;
	map<int, Node> nodeTable;

	vector<Rod_2> elementList;
	map<int, Rod_2> elementTable;

	//����������
	Controlset controlset;
	fem.parseControlset(controlset);

	//����Լ��
	vector<BdCondition> consList;
	fem.parseConstraint(consList);

	//��������
	vector<Elastic> materialList;
	fem.parseMaterial(materialList);

	//��������
	vector<Load> loadList;
	fem.parseLoad(loadList);

	//�����ڵ�,��δ�ڵ���Ϻ���Լ����Ϣ�����γɽڵ��
	int num = fem.parseNodeVectorAndTable(nodeList, nodeTable, consList, loadList);

	//�����˵�Ԫ����Ϊ��Ԫ���ϲ�����Ϣ�����γɵ�Ԫ��
	int numE = fem.parseElementOfRod_2(materialList, nodeList, loadList, nodeTable, elementList, elementTable);


	//���Լ����Ϣ
	//constraints of type boundary_condition
	//moving Az = [0, 5] Ay = [0, 0] Ax = [0, 0]
	for (int i = 1; i <= 20; i++)
	{
		cout << "elementTable[i] = " << i << "  " << elementTable[i].ElementID() << " left node constraint: " << endl;
		cout << "elementTable[i].leftNode->AY.name  " << elementTable[i].leftNode->constraint.name << endl;
		elementTable[i].leftNode->constraint.y_acc.printV();
		cout << " right Node constraint: " << endl;
		cout << "elementTable[i].rightNode->AY.name  " << elementTable[i].rightNode->constraint.name << endl;
		elementTable[i].rightNode->constraint.y_acc.printV();

	}

	cout << "\n \n \n";
	//��������Ϣend_load Fx = [0,0,14,0,14.1,-0.2,16,off,20,off] FY = [0,0,14,0,14.1,-0.2,16,off,20,off] FZ = [0,0,14,0,14.1,-0.2,16,off,20,off]

	for (int i = 1; i <= 20; i++)
	{
		cout << "elementTable[i] = " << i << "  " << elementTable[i].ElementID() << " left node load: " << endl;
		cout << "elementTable[i].leftNode->FZ.name  " << elementTable[i].leftNode->load.name << endl;
		//elementTable[i].leftNode->load.x_force.printV();
		//elementTable[i].leftNode->load.y_force.printV();
		elementTable[i].leftNode->load.z_force.printV();
		cout << " right Node constraint: " << endl;
		cout << "elementTable[i].rightNode->FX.name  " << elementTable[i].rightNode->constraint.name << endl;
		elementTable[i].rightNode->load.x_force.printV();

		//elementTable[i].rightNode->load.y_force.printV();
		//elementTable[i].rightNode->load.z_force.printV();
	}


	cout << "\n \n \n";
	//������e_steel rho = 0.0000078 E = 210 nu = 0.3 yield_stress = [0,0,14,0,14.1,-0.2,16,off,20,off] EP = 0.1 V1 = [0,0,14,0,14.1,-0.2,16,off,20,off]
	/************************************************************************/
	/* 
	�����Բ��ϵ�����
	for (int i = 1; i < 20;i++)
	{
	cout << "elementTable[i] = " << i << "  " << elementTable[i].ElementID() << endl;
	cout << "yield stress :" << endl;
	elementTable[i].material.yield_stress.printV();
	cout << "V1 = : " << endl;
	elementTable[i].material.V[0].printV();

	cout << "diameter = " << elementTable[i].diameter << endl;
	}
	
	*/
	/************************************************************************/




	cout << "controlset.startTime = " << controlset.startTime << endl; 
	cout << "controlset.endTime" << controlset.endTime << endl;

	cout << "model.nodeList[5].pos[0] " << nodeList[5].pos[0] << endl;
	cout << "nodeList[6].pos[0] " << nodeList[6].pos[0] << endl;
	cout << "nodeList[7].pos[0] " << nodeList[7].pos[0] << endl;
	sort(nodeList.begin(), nodeList.end());
	cout << "after sort \n \n \n";
	cout << "nodeList[5].pos[0] " << nodeList[5].pos[0] << endl;
	cout << "nodeList[6].pos[0] " << nodeList[6].pos[0] << endl;
	cout << "nodeList[7].pos[0] " << nodeList[7].pos[0] << endl;





	string t = "3";
	string commas = "OFF";
	t += commas + "777";
	cout << t;


	vector<int> v = { 1, 2, 3, 4 };
	cout << "v[2]= " << v[2] << endl;


	cout << (1 < 2) ? 1 : 2;
	system("pause");
	return 0;
}

