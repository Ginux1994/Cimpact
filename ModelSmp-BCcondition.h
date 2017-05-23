#ifndef MODELSMP_H
#define MODELSMP_H

#include "Fembic.h"
#include "Constraint.h"
#include "Controlset.h"
#include "Elastoplastic.h"
#include "Element.h"
#include "Load.h"
#include "Material.h"
#include "Node.h"
//#include "BdCondition.h"
#include "BCondition.h"
#include "Rod_2.h"
#include "tnt.h"
#include "tnt_array2d.h"
#include "Token.h"
#include "Tracker.h"
#include "TrackWriter.h"
#include "Variable.h"
#include "Writer.h"
#include "Worker.h"
#include <map>
#include <vector>
#include <boost/tokenizer.hpp>
using namespace std;
class ModelSmp
{
public:

	ModelSmp(int, string& );


	void initialize(string& fname);
	void setInitialContion();
	void assembleMassMatrix();
	void solve();

protected:
public:
//public for easy	
	Node temporary_node;
	Controlset controlset;
	Tracker temporary_tracker;
	Writer resultWriter;
	TrackWriter trackWriter;

	vector<Tracker> trackerList;

	map<int, Node> nodeTable;//�ڵ�ӳ���

	vector<Node> nodeList;//�ڵ�����

	vector<Rod_2> elementList;//��Ԫ����

	map<int, Rod_2> elementTable;//��Ԫӳ���

	vector<Elastoplastic> materialList;//��������

	vector<BdCondition> constraintList;//Լ������

	vector<BCondition> BConstraintList;

	vector<Load> loadList;//��������

	vector<Controlset> controlsetList;//���Ƽ�����

	Fembic indataFile;

	Worker worker;

	double currtime, ttemp, timestep, exported_ttmp;
	double total_mass;
	int number_of_elements, number_of_trackers, number_of_nodes, number_of_materials, number_of_controls;
	int number_of_loads, nr_of_CPUs, i;
	bool autostep, failure_is_set = false;
	string filePath;
	const bool DEBUG = true;

};


ModelSmp::ModelSmp(int nCPUs, string& fileName){
	using namespace std;
	nr_of_CPUs = nCPUs;
	filePath = fileName;
	//�����ļ��������ݴ���ڸ���nodeList, nodeTable, elementList, elementTable, loadList, materialList, constraintList, controlset��
	indataFile = Fembic(filePath);
	indataFile.parseControlset(controlset);//����������
	indataFile.parseConstraint(BConstraintList);//����Լ����
	indataFile.parseMaterial(materialList);//�������ϼ�
	indataFile.parseLoad(loadList);//�������ؼ�
	indataFile.parseNodeVectorAndTable(nodeList, nodeTable, BConstraintList, loadList);//�����ڵ㼯������Լ�����ط������ڵ�ĳ�Ա�����У��γɽڵ��������ڵ�ӳ���

	indataFile.parseElementOfRod_2(materialList, nodeList, loadList, nodeTable, elementList, elementTable);//������Ԫ�����������Ͻڵ㼯������Ӧ��Ԫ�ĳ�Ա�����У��γɵ�Ԫ��������Ԫӳ���
	//���ˣ������ļ��е�������Ϣ�����ˣ�����ֱ�����ڼ���

	worker = Worker(currtime, timestep, autostep);
	/************************************************************************/
	/* 
	���Ա�ĳ�ʼ��������������ô�򵥣�����Ҫnew��new��ָ��������
	*/
	/************************************************************************/
	cout << "\n";

}

void ModelSmp::initialize(string& fname){

}



void ModelSmp::setInitialContion(){

	//initialize all elements
	for (auto element : elementList)
	{
		element.setInitialConditions();
	}


	//initialize all nodes, and calculate the total mass
	for (auto tempNode : nodeList)
	{
		//set up node BC, if boundaryCondition_is_set		
		tempNode.setInitialConditions();
		total_mass += tempNode.Mass();
	}
	cout << "total mass is " << total_mass;
	//sort the nodes in ascending x-coordinates order
	sort(nodeList.begin(), nodeList.end());
	cout << "set up node neighbour handles \n";
	//needed for contact algorithm, like link list
	for (int i = 0; i < nodeList.size(); i++)
	{
		if (i < (nodeList.size() - 1))
		{
			nodeList[i].Right_neighbour(&nodeList[i + 1]);
		}
		if (i > 0)
		{
			nodeList[i].Left_neighbour(&nodeList[i - 1]);
		}
		//set cpu number for node
		nodeList[i].Cpu_number(i*nr_of_CPUs / nodeList.size());
	}


	//constraint no need to initialize

	//if autostep is used, loop through all elements and determine the smallest timestep
	/************************************************************************/
	/* 
	
	if (controlset.getTimeStep(0) == 0) {
	timestep = 1E10;
	autostep = true;

	for (auto element : elementList) {
	timestep = element.checkTimeStep(timestep);
	}
	}
	else {
	timestep = controlset.getTimeStep(0);
	autostep = false;
	}
	*/
	/************************************************************************/


	

}

void ModelSmp::assembleMassMatrix(){

	cout << "assmble element \n";

//	nodeList.clear();
//	nodeTable.clear();
	for (int i = 1; i <= elementTable.size();i++)
	{
		elementTable[i].assembleMassMatrix(nodeTable);
		elementList[i-1] = elementTable[i];
	//	nodeList.push_back(*(elementTable[i].leftNode));
	//	nodeList.push_back(*(elementTable[i].rightNode));
		//nodeTable.insert({})
	}
	//�ߣ��Ͳ������а�ˣ���nodeTable����ȥ��mass����û��ֵ����������ζ�ţ��Ժ�ÿ�μ��㣬Ҫ����nodeTable�е�node ��ֵ���ͱ��������θ�ֵ���ӵ�

	cout << "assemble node \n";
	for (auto node:nodeList)
	{
		node.determineMassMatrix();
	}
/************************************************************************/
/*				���� 
*/
/************************************************************************/
}



void ModelSmp::solve(){

	long time_info;
	long time_info_tmp;
	long time_remained;
	int time_h, time_m, time_s;//used for time show
	int number_of_integration_points = 1;//1 for rod_2 element
	string time_str;
	bool autosave = false;

	ttemp = 1e10;
	if (DEBUG)
	{
		cout << "calculating beging \n";
		cout << "loop element \n";
	}


	for (currtime = controlset.startTime; currtime <= controlset.endTime; currtime += timestep)
	{
		for (auto temporary_element:elementList)
		{
			if (!temporary_element.isDeactivated())//ʧЧ�ļ����ں��棬�Ͼ��ʼ�ǿӶ���ʧЧ��
			{
				temporary_element.updateLocalBaseVector();//update local coordinate system
				//��ÿ����˹���ֵ㴦����
				for (int i = 0; i < number_of_integration_points;i++)//����iѭ����0��ʼ������ζ�ų���0�����ֵ㣬����ӦΪ���ֵ㴦��Ӧ��������ֵ�������д洢���������±��0
				{
					temporary_element.calculateStrain(timestep, i);//����Ӧ���õ���ʱ�䲽��ԭ���Ǹ���ʱ�䲽����Ӧ���ʼ���һ����Ӧ�䣬�ɴ˴�����Ӧ�������ҵ���Ӧ������Ӧ��
					temporary_element.calculateStress(timestep, i);
				}
				//ÿһ�����ֵ㴦��Ӧ��Ӧ�䶼����֮�󣬼�������
				for (int i = 0; i < number_of_integration_points;i++)
				{
					temporary_element.calculateNodalForces(timestep, i);
				}
				//��Ԫ������ļ��㣬�����Ǽӵ��˵�Ԫ�ڵ���,���ڵ����������ȫ���ӵ�����force_positive����
				if (DEBUG)
				{
					cout << " temporary_element.elementID " << temporary_element.elementID << endl;

					cout << " temporary_element.leftNode->number " << temporary_element.leftNode->number << endl;
					cout << temporary_element.leftNode->force_positive;
				}
				//�����ⲿ��������������������
				temporary_element.calculateExternalForces(currtime);
				//if contact, calculate contact forc, and �ӵ��������
				temporary_element.calculateContactForces();//�Ӵ��������ǰ����Ŀǰ�����Ԫ���нӴ���/���� ��Ԫ���Ӵ����ж�Ҳ�����Ӵ�Ԫ
				//�Ӵ�Ԫ��ʵ�嵥���ýڵ㣬����ʵ�嵥Ԫ����Ӵ�ʱ��ÿ��8�ڵ�6����ʵ�嵥Ԫ�����������Ӵ�����Ԫ�����ǹ��ýڵ㣬�Ӵ�Ԫ����ĽӴ��ڵ���������Node�����洦��
				
				//update time step if needed
				if (autostep)
				{
					ttemp = temporary_element.checkTimeStep(ttemp);//��ʼttemp�ܴ� �״α�Ȼ�Զ�����ʱ�䲽
				}
			}
		}//end element loop

		exported_ttmp = ttemp;// time step ����ʱ����
		if (DEBUG)
		{
			cout << "remove the failed element - for topo remove the useless element";
		}
		if (failure_is_set)
		{
			for (auto temp_element:elementList)
			{
				temp_element.checkIfFailed();
				if (temp_element.hasFailed())
				{
					temp_element.deActivated();//����Ѿ��ƻ��ˣ��Ͳ����������Ԫ���������ڲ������
				}
			}
		}

		if (DEBUG)
		{
			cout << "loop constraint, and update constraint of local coordinate system";
		}
		for (auto temp_constraint:BConstraintList)
		{
			temp_constraint.update();//�����ϸ��µ��Ǿֲ�����ϵ
		}

		//ѭ���ڵ㣬����λ�ã�Ϊ��һ��Ӧ��Ӧ�����׼��
		if (DEBUG)
		{
			cout << "loop node, update position";
		}
		for (auto temp_node:nodeList)
		{
			//temp_node
		}







	}//for (currtime = controlset.startTime; currtime <= controlset.endTime;time+=timestep)



}








#endif