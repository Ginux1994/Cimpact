#ifndef MODELSMP_H
#define MODELSMP_H

#include "allInclude.h"

using namespace std;
class ModelSmp
{
public:

	ModelSmp(int, string& );

	void migrate(map<int, Rod_2>& elementTable, vector<Node>& nodeList);
	void migrate(map<int, Node>& nodeTable, vector<Node>& nodeList);
	void initialize(string& fname);
	void setInitialContion();
	void assembleMassMatrix();
	void solve();
	void run();

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

	vector<Elastic> materialList;//��������

	vector<BdCondition> constraintList;//Լ������
	vector<Rigid_Body> rigidConstraintList;

	map<int, BdCondition> constraintTable;

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
	const bool DEBUG = false;

};


ModelSmp::ModelSmp(int nCPUs, string& fileName){
	using namespace std;
	nr_of_CPUs = nCPUs;
	filePath = fileName;
	//�����ļ��������ݴ���ڸ���nodeList, nodeTable, elementList, elementTable, loadList, materialList, constraintList, controlset��
	indataFile = Fembic(filePath);
	/************************************************************************/
	/*                         //����������                                             */
	/************************************************************************/
	indataFile.parseControlset(controlset);
	cout << "=====�������ķָ���===��������  ����  ��Ϣ=====\n";

	if (controlset.autoStep){
		cout << "=====ʱ�䲽�� Ϊ �Զ����������ݵ�Ԫ�������ȼ���=====��ţ�ưɣ�������\n";
	}
	else{
		cout << "=====ʱ�䲽�� Ϊ " << controlset.getTimeStep(0) << endl;
	}

	cout << "=====�� " << controlset.startTime << " ��ʼ======�� " << controlset.endTime << " =====����====" << endl;

	/************************************************************************/
	/*            //����Լ����                                                          */
	/************************************************************************/
	indataFile.parseConstraint_BdCondition(constraintList);
	cout << "=====�������ķָ���===�������� Լ�� ��Ϣ======\n";
	for (auto testConstraint : constraintList)
	{
		cout << "��������Լ��Ϊ: " << testConstraint.name << " ���ٶ�Ϊ \n";
		testConstraint.x_vel.printV();
	}
	indataFile.parseConstraint_RigidCondition(rigidConstraintList);
	/************************************************************************/
	/*               //�������ϼ�                                                       */
	/************************************************************************/
	indataFile.parseMaterial(materialList);
	cout << "=====�������ķָ���===�������� ���� ��Ϣ=====\n";
	for (auto testMaterial : materialList)
	{
		cout << "��������ģ��Ϊ " << testMaterial.youngs_moduls << endl;
	}

	/************************************************************************/
	/*               //�������ؼ�                                                       */
	/************************************************************************/
	indataFile.parseLoad(loadList);
	cout << "=====�������ķָ���===�������� ���� ��Ϣ=====\n";
	for (auto testLoad : loadList)
	{
		testLoad.x_acc.printV();
	}

	/************************************************************************/
	/*             //�����ڵ㼯������Լ�����ط������ڵ�ĳ�Ա�����У��γɽڵ��������ڵ�ӳ���                                                         */
	/************************************************************************/
	indataFile.parseNodeVectorAndTable(nodeList, nodeTable, constraintList,rigidConstraintList, loadList);

	for (auto testNode : nodeList)
	{
		cout << "�ڵ��Ϊ " << testNode.number << " �Ľڵ�Լ����ϢΪ \n";
		testNode.getConstraint().x_vel.printV();
		cout << "================�������ķָ���===================\n";
	}



	indataFile.parseElementOfRod_2(materialList, nodeList, loadList, nodeTable, elementList, elementTable);//������Ԫ�����������Ͻڵ㼯������Ӧ��Ԫ�ĳ�Ա�����У��γɵ�Ԫ��������Ԫӳ���
	//���ˣ������ļ��е�������Ϣ�����ˣ�����ֱ�����ڼ���
	cout << "�����ļ�������ڵ㵥Ԫ��Ľ��(=====��Ԫ��=====) \n";

	for (auto testElement : elementList)
	{
		cout << "��Ԫ���Ϊ " << testElement.elementID << " �ĵ�Ԫ���������ϢΪ \n";
		cout << "����ģ���� " << testElement.material.youngs_moduls << endl;
		cout << "�ܶȣ� " << testElement.material.density << endl;
		cout << "ֱ���� " << testElement.diameter << endl;
		cout << "�����ţ� " << testElement.leftNode->number << " Լ����ϢΪ�� \n";
		testElement.leftNode->getConstraint().y_vel.printV();
		cout << "�ҽڵ��ţ� " << testElement.rightNode->number << " Լ����ϢΪ�� \n";
		testElement.rightNode->getConstraint().y_vel.printV();
	}


	cout << "\n\n\n\n\n";
	cout << "�����ļ�������ڵ㵥Ԫ��Ľ��(=====��Ԫ��=====) \n";
	auto testElement = elementTable[1];
	cout << "��Ԫ���Ϊ " << testElement.elementID << " �ĵ�Ԫ���������ϢΪ \n";
	cout << "����ģ���� " << testElement.material.youngs_moduls << endl;
	cout << "�ܶȣ� " << testElement.material.density << endl;
	cout << "ֱ���� " << testElement.diameter << endl;
	cout << "�����ţ� " << testElement.leftNode->number << " Լ����ϢΪ�� \n";
	cout << "X�����ٶ�Ϊ�� \n";
	testElement.leftNode->getConstraint().x_vel.printV();
	cout << "Z������ٶ�Ϊ: \n";
	testElement.leftNode->getConstraint().z_acc.printV();
	cout << "�Žڵ��ڵ�Ԫ�е�Լ����Ϣ\n";
	elementTable[1].rightNode->getConstraint().z_acc.printV();
	cout << "�ҽڵ��ţ� " << testElement.rightNode->number << " Լ����ϢΪ�� \n";
	testElement.rightNode->getConstraint().x_vel.printV();



	//worker = Worker(currtime, timestep, autostep);//�������̲߳��д���
	/************************************************************************/
	/*
	���Ա�ĳ�ʼ��������������ô�򵥣�����Ҫnew��new��ָ��������
	*/
	/************************************************************************/
	cout << "\n";

}

void ModelSmp::assembleMassMatrix(){
	cout << "=====�������ķָ���  ֮  �������Ԫ�ڽ���ϵķֲ�����ֵ����ͬ����Ԫ��Ԫ��===\n";
	for (int i = 1; i <= elementTable.size(); i++)
	{
		elementTable[i].calculateMassMatrix(nodeTable);//ͨ���������������������ֲ������ڽڵ�nodeTable�ϣ�ͬʱҲ���е��˵�Ԫ�����ҽڵ���
		elementList[i - 1] = elementTable[i];
		//��Ԫ�е�Ԫ���е����ҽڵ㶼������ֵ
	}
	cout << "=====�������ķָ���  ֮  �������Ԫ�ڽ���ϵķֲ�==ת������===ֵ===\n";
	for (int i = 1; i <= elementTable.size(); i++){
		elementTable[i].leftNode->determineInertiaMatrix();
		elementTable[i].rightNode->determineInertiaMatrix();
		//����ڵ��в�û��������ֻ�е�Ԫ���е����ҽڵ�������������õ�Ԫ�ڵ���ڵ���γ�ӳ��������Ҫ��
	}

	/************************************************************************/
	/*
	for (int i = 1; i <= nodeTable.size(); i++)
	{
	nodeTable[i].determineInertiaMatrix();
	//����ڵ��в�û��������ֻ�е�Ԫ���е����ҽڵ�������������õ�Ԫ�ڵ���ڵ���γ�ӳ��������Ҫ��
	cout << "============�����ָ���֮���ڵ��ת������Ϊ======\n";
	cout << nodeTable[i].inertia;
	cout << "=============�ڵ����е�����Ϊ=========\n";
	cout << nodeTable[i].mass;
	}
	*/
	/************************************************************************/

	/************************************************************************/
	/*				���� ,���ﻹ�ж�Լ���ĳ�ʼ�������о�ûɶ�ã�
	temp_constraint.determineMassMatrix(nodelist);
	public void determineMassMatrix(RplVector nodelist) {
	int j;

	// This should have ideally been made already in the initialization
	// phase, but constraints are
	// defined first by necessity. This means no nodelist is available at
	// that time, so we do it here instead.
	// Ok, now find the numbers & nodes
	if (axis_is_set) {

	for (j = 0; j < 3; j++) {
	node[j] = super.findNode(super.getNodeNumber(j + 1, nodes),
	nodelist);
	}

	// Now, initialize the local coordinate system
	axis = this.calculateLocalBaseVectors(node[0].getX_pos(),
	node[0].getY_pos(), node[0].getZ_pos(), node[1].getX_pos(),
	node[1].getY_pos(), node[1].getZ_pos(), node[2].getX_pos(),
	node[2].getY_pos(), node[2].getZ_pos());
	}
	}
	}
	*/
	/************************************************************************/
}



void ModelSmp::setInitialContion(){

	//initialize all elements
	for (auto element : elementList)
	{
		//ÿ����Ԫ�ĳ�ʼ������ʵ���ǶԵ�Ԫ�������Եĳ�ʼ������նȾ���D,B,N��
		element.setInitialConditions();
		cout << "=====�������ָ��� ֮ ��Ԫ����=====\n";
		cout << "��Ԫ����洢�ڵ�Ԫ�Ĳ�����; ��ά�ն���" << element.material.stiffness_matrix_3d << endl;
		cout <<"ƽ��Ӧ������ĸն��� ��"<< element.material.stiffness_matrix_plane_stress << endl;
	}
	//migrate(elementTable, nodeList);
	migrate(nodeTable, nodeList);
	//initialize all nodes, and calculate the total mass
	for (auto tempNode : nodeList)
	{
		//��������û�õģ���ʱ�Ľڵ���û��������Ϣ�����assembleMassMatrix�����ڴ�֮ǰ
		//set up node BC, if boundaryCondition_is_set		
		tempNode.setInitialConditions(nodeList);
		total_mass += tempNode.Mass();
	}
	cout << "=====�������ָ��� ֮ ģ����������������������װ�����=====\n";
	cout << "total mass is " << total_mass;
	if (true)
	{
		for (auto tempNode : nodeList)
		{
			cout << "�� " << tempNode.number << " �Žڵ������Ϊ " << tempNode.mass << endl;
		}
		for (auto tempElement : elementList)
		{
			cout << "�� " << tempElement.elementID << " �ŵ�Ԫ������Ϊ " << tempElement.elementMass << endl;
		}
	}
	//sort the nodes in ascending x-coordinates order
	cout << "======�������ָ��� ֮ ���ڵ�������======\n";
	sort(nodeList.begin(), nodeList.end());
	cout << "=====�������ָ��� ֮ ���ڵ������ھӣ�׼���Ӵ�======\n";
	//needed for contact algorithm, like link list
	//����ֻ�Ƕ��ھӽ��г�ʼ���ã���ÿ���ڵ���ھӶ��ǿգ�����Ӧ�ýӴ��㷨ʱ����Ҫ���ż��������λ�ø���֮�����������ھ�
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
	//�ڳ�ʼʱ�̶Խڵ�Ӧ�ñ߽�����
	cout << "=====�������ָ��� ֮ ���ڵ���ӳ�ʼ�߽�����������������������ʼֵ0��=====\n";

	for (auto tempNode : nodeList){
		if (tempNode.constraint_is_set){

			if (tempNode.constraint.type == "RIGID_BODY")
			{
				tempNode.applyAccelerationRigidCondition(currtime);
				tempNode.applyVelocityRigidCondition(currtime);
			}
			else{
				tempNode.applyAccelerationBCondition(currtime);//�Ѿ�ʩ�����ٶȱ߽磬�ͻ���ǰ��ͨ�����������ļ��ٶ�===����ζ�ţ����Ա��εķ������������ĸ����˶�����Ӱ�죬ԭ�����
				tempNode.applyVelocityBCondition(currtime);//���ٶȱ߽�Ļ����ѱ߽���ٶȻ���ȥ��û�߽�Ļ�������	
			}
		}
		else{
			tempNode.vel = 0;
			tempNode.acc = 0;
		}
		tempNode.dpl_old = tempNode.dpl;
		tempNode.internal_energy = 0.0;
		tempNode.external_energy = 0.0;
		tempNode.hourglass_energy = 0.0;
		tempNode.contact_energy = 0.0;
		if (tempNode.mass == 0){
			tempNode.mass = 1e-15;
		}
	}
}



void ModelSmp::solve(){

	long time_info;
	long time_info_tmp;
	long time_remained;
	int time_h, time_m, time_s;//used for time show
	int number_of_integration_points = 1;//1 for rod_2 element
	string time_str;
	bool autosave = false;
	ttemp = 1e10;//big number for auto step time calculate
	if (DEBUG)
	{
		cout << "==========�������ķָ���  ֮ ������������������������������������ ��ʼ���㣤����������������������������n";
		cout << "loop element \n";
	}
	/***
	$$$$$$$$$$$$$$$$$$�������ݴ洢·�����ļ�
	�����������Writer��Tracker
	*****/
	string filePath1 = "E:\\stress_element1.txt";
	string filePath2 = "E:\\stress_element2.txt";
	ofstream cout_dpl, cout_stress;
	cout_dpl.open(filePath1);
	cout_stress.open(filePath2);
	if (!cout_dpl.is_open() || cout_stress.is_open())
	{
		cout << "file to open file1\n";
	}
	cout << "time step = " << controlset.getTimeStep(0);
	timestep = controlset.getTimeStep(0);
	int stepNum = 0;
	//��һ�����̿�����ΪWORK������������ھ���ѧ����������Ҫʱ�������һ�μ���ɹ���ģ̬�����Ļ����Ӹ�����ֵ���㣻����ԽӴ���Ԫ�ԼӴ����Ϳ��Դ�����������ˣ���ʵĦ������һ��������⣬�ೡ��Ͼ��Ƕ����������һ������
	//��������������̻����ڲ��м��㣬ÿ��work��Ϊһ����ⵥԪ����һ�������ĵ�Ԫ�ڵ㣬�Ϳ��Խ��ж��߳�����ˡ�
	for (currtime = controlset.startTime; currtime <= controlset.endTime; currtime += timestep)
	{
		stepNum++;
		cout << "*******************************�Ѿ�������***************�� " << stepNum << " ����\n";
		for (auto temporary_element:elementList)
		{
			if (!temporary_element.isDeactivated())//ʧЧ�ļ����ں��棬�Ͼ��ʼ�ǿӶ���ʧЧ��
			{
				temporary_element.updateLocalBaseVector();//update local coordinate system
				//��ÿ����˹���ֵ㴦����
				for (int i = 0; i < number_of_integration_points;i++)//����iѭ����0��ʼ������ζ�ų���0�����ֵ㣬����ӦΪ���ֵ㴦��Ӧ��������ֵ�������д洢���������±��0
				{
					//���ﵥԪ����Ӧ��Ӧ���������ն���ʵ����������ҽڵ����ˡ����ֻҪ��Ԫ�������ҽڵ�����������Ϣ�����ܼ��㣬����Ҫ��Ԫ�ϵĽڵ���ڵ��һһ��Ӧ
					temporary_element.calculateStrain(timestep, i);//����Ӧ���õ���ʱ�䲽��ԭ���Ǹ���ʱ�䲽����Ӧ���ʼ���һ����Ӧ�䣬�ɴ˴�����Ӧ�������ҵ���Ӧ������Ӧ��
					temporary_element.calculateStress(timestep, i);
				}
				//ÿһ�����ֵ㴦��Ӧ��Ӧ�䶼����֮�󣬼�������
				for (int i = 0; i < number_of_integration_points;i++)
				{
					temporary_element.calculateNodalForces(timestep, i);
				}
				//��Ԫ������ļ��㣬�����Ǽӵ��˵�Ԫ�ڵ���,���ڵ����������ȫ���ӵ�����force_positive����
				if (true)
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

				//���
				//update time step if needed
				if (autostep)
				{
					ttemp = temporary_element.checkTimeStep(ttemp);//��ʼttemp�ܴ� �״α�Ȼ�Զ�����ʱ�䲽
				}
			}

		}//end element loop,���ѭ�����е��Ա��η�����㣬��������˶�����


		/************************************************************************/
		/* ������Ӧ��Ӧ��Ϳ����ж������Ԫ�ǲ����Ѿ��ƻ����������������Ż�Ҳ���������о�           */
		if (failure_is_set)
		{
			for (auto temporary_element : elementList)
			{
				temporary_element.checkIfFailed();
				if (temporary_element.hasFailed())
				{
					temporary_element.deActivated();
				}
			}
		}
		/************************************************************************/


		exported_ttmp = ttemp;// time step ����ʱ����

		if (DEBUG)
		{
			cout << "======�������ķָ���  ֮ ѭ��Լ�������¾ֲ�����ϵ=====";
		}
		for (auto temp_constraint:rigidConstraintList)
		{
//			temp_constraint.update();//�����ϸ��µ��Ǿֲ�����ϵ


		}

		//ѭ���ڵ㣬����λ�ã�Ϊ��һ��Ӧ��Ӧ�����׼��
		if (DEBUG)
		{
			cout << "=====�������ķָ���  ֮ ѭ���ڵ㣬����**λ��**======";
		}


		//���ǳ���֮�ƣ����һ����Ԫ��8���ڵ�͵ö�ÿ���ڵ���в������ϲ�ѭ��
		for (auto temp_element : elementList){
			temp_element.leftNode->calculateNewPosition(timestep, currtime);
//			cout<<"��ǰ�ڵ���" <<temp_element.leftNode->number<<" Zλ�� "<< temp_element.leftNode->pos[2]<<endl;
			cout << "****************����ָ� \n"; 
//			cout << currtime << "\t" << elementList[1].leftNode->pos[2] << "\n";
			temp_element.rightNode->calculateNewPosition(timestep, currtime);
			temp_element.leftNode->checkNeighbours();
			temp_element.rightNode->checkNeighbours();
		}
		if (DEBUG)
		{
			cout << "************************�������ָ��� ֮ �ڵ�λ��*******************************\n";
			cout << "=========================һ�ŵ�Ԫ ���ҽڵ�X����λ��==========================================\n";
			cout << "��ڵ�Xλ��Ϊ = " << elementTable[10].leftNode->dpl[0] << endl;
			cout << "�ҽڵ�Xλ��Ϊ = " << elementTable[10].rightNode->dpl[0] << endl;
			cout << "************************�������ָ��� ֮ д��λ��Ӧ��������ļ���*******************************\n";

		}
		//�ⲿ�ֿ��Է�װ��Track�У�������Ҫ�����Ӧֵ
//		cout << "3�Ž��Zλ�� \n";
		cout_dpl << currtime << "\t" << elementTable[1].stress[0] << "\n";
//		cout << currtime << "\t" << elementTable[3].leftNode->pos[2] << "\n";
//		cout_stress << currtime << "\t" << elementTable[2].stress[0][0] << "\n";
	//	cout << currtime << "\t" << elementTable[3].rightNode->pos[2] << "\n";
		cout_dpl << flush;
		cout_stress << flush;
		//cout_dpl.close();
		//cout_stress.close();


		//����ʱ�䲽
		if (controlset.getTimeStep(currtime)==0.0)
		{
			autostep = true;
			timestep = ttemp;
			ttemp = 1e10;//������Ϊ������������һ�ζԱ�
		}
		else {
			autostep = false;
			timestep = controlset.getTimeStep(currtime);
		}

		if (false)
		{
			cout << "���constrolset���Ƿ���Ҫ���Щ���� \n";
		}
		if (DEBUG)
		{
			cout << "*****�������ָ��� ֮ ���constrolset���Ƿ���Ҫ���Щ����***** \n";
		}

		//���滹Ҫ��Щ�������䡣����������������internal, external, contact, hourglass, force_possive, force
		for (auto temp_node:nodeList)
		{
			temp_node.clearNodalForces();
		}		
	}//for (currtime = controlset.startTime; currtime <= controlset.endTime;time+=timestep)
}

void ModelSmp::run(){
	assembleMassMatrix();
	setInitialContion();
	solve();
}

void ModelSmp::migrate(map<int, Rod_2>& elementTable, vector<Node>& nodeList){
	int node_number_of_element = nodeList.size();
	nodeList.clear();
	nodeList.push_back(*(elementTable[1].leftNode));
	int flag = 0;//������������
	for (auto element : elementTable)
	{
		for (auto node = nodeList.begin(); node != nodeList.end(); node++)
		{

			if (!(element.second.leftNode->number == (*node).number))
			{
				nodeList.push_back(*(element.second.leftNode));
				flag++;
				node++;
			}
			if (!(element.second.rightNode->number == (*node).number))
			{
				nodeList.push_back(*(element.second.rightNode));
				flag++;
				node++;
			}

			if (flag == (node_number_of_element - 1))
			{
				break;
			}
		}

	}

}
void ModelSmp::migrate(map<int, Node>& nodeTable, vector<Node>& nodeList){
	nodeList.clear();
	for (auto node : nodeTable)
	{
		nodeList.push_back(node.second);
	}
}




#endif