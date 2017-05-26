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

	map<int, Node> nodeTable;//节点映射表

	vector<Node> nodeList;//节点向量

	vector<Rod_2> elementList;//单元向量

	map<int, Rod_2> elementTable;//单元映射表

	vector<Elastic> materialList;//材料向量

	vector<BdCondition> constraintList;//约束向量
	vector<Rigid_Body> rigidConstraintList;

	map<int, BdCondition> constraintTable;

	vector<Load> loadList;//荷载向量

	vector<Controlset> controlsetList;//控制集向量

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
	//解析文件，将数据存放在各个nodeList, nodeTable, elementList, elementTable, loadList, materialList, constraintList, controlset中
	indataFile = Fembic(filePath);
	/************************************************************************/
	/*                         //解析控制流                                             */
	/************************************************************************/
	indataFile.parseControlset(controlset);
	cout << "=====华丽丽的分割线===解析到的  控制  信息=====\n";

	if (controlset.autoStep){
		cout << "=====时间步长 为 自动步长，根据单元特征长度计算=====很牛逼吧，啊哈哈\n";
	}
	else{
		cout << "=====时间步长 为 " << controlset.getTimeStep(0) << endl;
	}

	cout << "=====从 " << controlset.startTime << " 开始======到 " << controlset.endTime << " =====结束====" << endl;

	/************************************************************************/
	/*            //解析约束集                                                          */
	/************************************************************************/
	indataFile.parseConstraint_BdCondition(constraintList);
	cout << "=====华丽丽的分割线===解析到的 约束 信息======\n";
	for (auto testConstraint : constraintList)
	{
		cout << "解析到的约束为: " << testConstraint.name << " 其速度为 \n";
		testConstraint.x_vel.printV();
	}
	indataFile.parseConstraint_RigidCondition(rigidConstraintList);
	/************************************************************************/
	/*               //解析材料集                                                       */
	/************************************************************************/
	indataFile.parseMaterial(materialList);
	cout << "=====华丽丽的分割线===解析到的 材料 信息=====\n";
	for (auto testMaterial : materialList)
	{
		cout << "材料杨氏模量为 " << testMaterial.youngs_moduls << endl;
	}

	/************************************************************************/
	/*               //解析荷载集                                                       */
	/************************************************************************/
	indataFile.parseLoad(loadList);
	cout << "=====华丽丽的分割线===解析到的 荷载 信息=====\n";
	for (auto testLoad : loadList)
	{
		testLoad.x_acc.printV();
	}

	/************************************************************************/
	/*             //解析节点集，并将约束荷载放入存入节点的成员变量中，形成节点向量，节点映射表                                                         */
	/************************************************************************/
	indataFile.parseNodeVectorAndTable(nodeList, nodeTable, constraintList,rigidConstraintList, loadList);

	for (auto testNode : nodeList)
	{
		cout << "节点号为 " << testNode.number << " 的节点约束信息为 \n";
		testNode.getConstraint().x_vel.printV();
		cout << "================华丽丽的分割线===================\n";
	}



	indataFile.parseElementOfRod_2(materialList, nodeList, loadList, nodeTable, elementList, elementTable);//解析单元集，并将材料节点集放入相应单元的成员变量中，形成单元向量，单元映射表
	//至此，输入文件中的所有信息都有了，可以直接用于计算
	cout << "解析文件并处理节点单元后的结果(=====单元列=====) \n";

	for (auto testElement : elementList)
	{
		cout << "单元编号为 " << testElement.elementID << " 的单元，其材料信息为 \n";
		cout << "杨氏模量： " << testElement.material.youngs_moduls << endl;
		cout << "密度： " << testElement.material.density << endl;
		cout << "直径： " << testElement.diameter << endl;
		cout << "左结点编号： " << testElement.leftNode->number << " 约束信息为： \n";
		testElement.leftNode->getConstraint().y_vel.printV();
		cout << "右节点编号： " << testElement.rightNode->number << " 约束信息为： \n";
		testElement.rightNode->getConstraint().y_vel.printV();
	}


	cout << "\n\n\n\n\n";
	cout << "解析文件并处理节点单元后的结果(=====单元表=====) \n";
	auto testElement = elementTable[1];
	cout << "单元编号为 " << testElement.elementID << " 的单元，其材料信息为 \n";
	cout << "杨氏模量： " << testElement.material.youngs_moduls << endl;
	cout << "密度： " << testElement.material.density << endl;
	cout << "直径： " << testElement.diameter << endl;
	cout << "左结点编号： " << testElement.leftNode->number << " 约束信息为： \n";
	cout << "X方向速度为： \n";
	testElement.leftNode->getConstraint().x_vel.printV();
	cout << "Z方向加速度为: \n";
	testElement.leftNode->getConstraint().z_acc.printV();
	cout << "号节点在单元中的约束信息\n";
	elementTable[1].rightNode->getConstraint().z_acc.printV();
	cout << "右节点编号： " << testElement.rightNode->number << " 约束信息为： \n";
	testElement.rightNode->getConstraint().x_vel.printV();



	//worker = Worker(currtime, timestep, autostep);//用来多线程并行处理
	/************************************************************************/
	/*
	类成员的初始化方案，就是这么简单，不需要new，new和指针搭配出现
	*/
	/************************************************************************/
	cout << "\n";

}

void ModelSmp::assembleMassMatrix(){
	cout << "=====华丽丽的分割线  之  计算出单元在结点上的分布质量值，并同步单元表单元列===\n";
	for (int i = 1; i <= elementTable.size(); i++)
	{
		elementTable[i].calculateMassMatrix(nodeTable);//通过集中质量法，将质量分布集中在节点nodeTable上，同时也集中到了单元的左右节点上
		elementList[i - 1] = elementTable[i];
		//单元列单元表中的左右节点都有质量值
	}
	cout << "=====华丽丽的分割线  之  计算出单元在结点上的分布==转动惯量===值===\n";
	for (int i = 1; i <= elementTable.size(); i++){
		elementTable[i].leftNode->determineInertiaMatrix();
		elementTable[i].rightNode->determineInertiaMatrix();
		//这里节点列并没有质量，只有单元表中的左右节点有质量，如何让单元节点与节点表形成映射至关重要。
	}

	/************************************************************************/
	/*
	for (int i = 1; i <= nodeTable.size(); i++)
	{
	nodeTable[i].determineInertiaMatrix();
	//这里节点列并没有质量，只有单元表中的左右节点有质量，如何让单元节点与节点表形成映射至关重要。
	cout << "============华丽分割线之，节点的转动惯量为======\n";
	cout << nodeTable[i].inertia;
	cout << "=============节点列中的质量为=========\n";
	cout << nodeTable[i].mass;
	}
	*/
	/************************************************************************/

	/************************************************************************/
	/*				继续 ,这里还有对约束的初始化，但感觉没啥用，
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
		//每个单元的初始化，其实就是对单元材料属性的初始化，如刚度矩阵D,B,N等
		element.setInitialConditions();
		cout << "=====华丽丽分割线 之 单元刚阵：=====\n";
		cout << "单元刚阵存储在单元的材料中; 三维刚度阵" << element.material.stiffness_matrix_3d << endl;
		cout <<"平面应力问题的刚度阵 ："<< element.material.stiffness_matrix_plane_stress << endl;
	}
	//migrate(elementTable, nodeList);
	migrate(nodeTable, nodeList);
	//initialize all nodes, and calculate the total mass
	for (auto tempNode : nodeList)
	{
		//放在这里没用的，此时的节点列没有质量信息，因此assembleMassMatrix必须在此之前
		//set up node BC, if boundaryCondition_is_set		
		tempNode.setInitialConditions(nodeList);
		total_mass += tempNode.Mass();
	}
	cout << "=====华丽丽分割线 之 模型总质量，检验质量阵组装结果：=====\n";
	cout << "total mass is " << total_mass;
	if (true)
	{
		for (auto tempNode : nodeList)
		{
			cout << "第 " << tempNode.number << " 号节点的质量为 " << tempNode.mass << endl;
		}
		for (auto tempElement : elementList)
		{
			cout << "第 " << tempElement.elementID << " 号单元的质量为 " << tempElement.elementMass << endl;
		}
	}
	//sort the nodes in ascending x-coordinates order
	cout << "======华丽丽分割线 之 给节点列排序======\n";
	sort(nodeList.begin(), nodeList.end());
	cout << "=====华丽丽分割线 之 给节点设置邻居，准备接触======\n";
	//needed for contact algorithm, like link list
	//这里只是对邻居进行初始设置，让每个节点的邻居都非空，真正应用接触算法时，还要随着计算迭代，位置更新之后，重新设置邻居
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
	//在初始时刻对节点应用边界条件
	cout << "=====华丽丽分割线 之 给节点添加初始边界条件，并给能量变量赋初始值0：=====\n";

	for (auto tempNode : nodeList){
		if (tempNode.constraint_is_set){

			if (tempNode.constraint.type == "RIGID_BODY")
			{
				tempNode.applyAccelerationRigidCondition(currtime);
				tempNode.applyVelocityRigidCondition(currtime);
			}
			else{
				tempNode.applyAccelerationBCondition(currtime);//已经施加了速度边界，就会冲掉前面通过结点力计算的加速度===》意味着，弹性变形的反力不会对整体的刚体运动产生影响，原来如此
				tempNode.applyVelocityBCondition(currtime);//有速度边界的话，把边界的速度换上去，没边界的话，不变	
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
		cout << "==========华丽丽的分割线  之 ￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥ 开始计算￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥n";
		cout << "loop element \n";
	}
	/***
	$$$$$$$$$$$$$$$$$$设置数据存储路径和文件
	这个后续交给Writer和Tracker
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
	//这一求解过程可以作为WORK分离出来，对于静力学分析，不需要时间迭代，一次计算成功；模态分析的话，加个特征值计算；如果对接触单元稍加处理，就可以处理耦合问题了，其实摩擦就是一个耦合问题，多场耦合就是对这个因子做一个扩充
	//另外抽离出这个过程还便于并行计算，每个work作为一个求解单元分配一定数量的单元节点，就可以进行多线程求解了。
	for (currtime = controlset.startTime; currtime <= controlset.endTime; currtime += timestep)
	{
		stepNum++;
		cout << "*******************************已经迭代到***************第 " << stepNum << " 步了\n";
		for (auto temporary_element:elementList)
		{
			if (!temporary_element.isDeactivated())//失效的检查放在后面，毕竟最开始是坑定不失效的
			{
				temporary_element.updateLocalBaseVector();//update local coordinate system
				//在每个高斯积分点处计算
				for (int i = 0; i < number_of_integration_points;i++)//这里i循环从0开始不是意味着成了0个积分点，而是应为积分点处对应的物理量值在数组中存储，而数组下标从0
				{
					//这里单元计算应力应变力，最终都落实到基层的左右节点上了。因此只要单元表中左右节点是有质量信息，就能计算，不需要单元上的节点与节点表一一对应
					temporary_element.calculateStrain(timestep, i);//计算应变用的是时间步，原因是根据时间步计算应变率及上一步的应变，由此从屈服应力普中找到相应的屈服应力
					temporary_element.calculateStress(timestep, i);
				}
				//每一个积分点处的应力应变都有了之后，计算结点力
				for (int i = 0; i < number_of_integration_points;i++)
				{
					temporary_element.calculateNodalForces(timestep, i);
				}
				//单元结点力的计算，最终是加到了单元节点上,而节点的力，最终全都加到向量force_positive上了
				if (true)
				{
					cout << " temporary_element.elementID " << temporary_element.elementID << endl;

					cout << " temporary_element.leftNode->number " << temporary_element.leftNode->number << endl;
					cout << temporary_element.leftNode->force_positive;
				}
				//计算外部力，如重力体力，其他
				temporary_element.calculateExternalForces(currtime);
				//if contact, calculate contact forc, and 加到结点力中
				temporary_element.calculateContactForces();//接触力计算的前提是目前这个单元里有接触线/三角 单元，接触的判断也交给接触元
				//接触元与实体单共用节点，比如实体单元处理接触时，每个8节点6面体实体单元都会有两个接触三角元，他们共用节点，接触元计算的接触节点力都会在Node类里面处理

				//输出
				//update time step if needed
				if (autostep)
				{
					ttemp = temporary_element.checkTimeStep(ttemp);//初始ttemp很大， 首次必然自动更新时间步
				}
			}

		}//end element loop,这个循环进行弹性变形方面计算，下面进行运动计算


		/************************************************************************/
		/* 更新完应力应变就可以判断这个单元是不是已经破坏可扔了啦，拓扑优化也在这里做判决           */
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


		exported_ttmp = ttemp;// time step 的临时变量

		if (DEBUG)
		{
			cout << "======华丽丽的分割线  之 循环约束，更新局部坐标系=====";
		}
		for (auto temp_constraint:rigidConstraintList)
		{
//			temp_constraint.update();//本质上更新的是局部坐标系


		}

		//循环节点，更新位置，为下一波应力应变计算准备
		if (DEBUG)
		{
			cout << "=====华丽丽的分割线  之 循环节点，更新**位置**======";
		}


		//不是长久之计，如果一个单元有8个节点就得对每个节点进行操作，上层循环
		for (auto temp_element : elementList){
			temp_element.leftNode->calculateNewPosition(timestep, currtime);
//			cout<<"当前节点编号" <<temp_element.leftNode->number<<" Z位置 "<< temp_element.leftNode->pos[2]<<endl;
			cout << "****************计算分割 \n"; 
//			cout << currtime << "\t" << elementList[1].leftNode->pos[2] << "\n";
			temp_element.rightNode->calculateNewPosition(timestep, currtime);
			temp_element.leftNode->checkNeighbours();
			temp_element.rightNode->checkNeighbours();
		}
		if (DEBUG)
		{
			cout << "************************华丽丽分割线 之 节点位移*******************************\n";
			cout << "=========================一号单元 左右节点X方向位移==========================================\n";
			cout << "左节点X位移为 = " << elementTable[10].leftNode->dpl[0] << endl;
			cout << "右节点X位移为 = " << elementTable[10].rightNode->dpl[0] << endl;
			cout << "************************华丽丽分割线 之 写出位移应力结果到文件中*******************************\n";

		}
		//这部分可以封装在Track中，根据需要输出相应值
//		cout << "3号结点Z位置 \n";
		cout_dpl << currtime << "\t" << elementTable[1].stress[0] << "\n";
//		cout << currtime << "\t" << elementTable[3].leftNode->pos[2] << "\n";
//		cout_stress << currtime << "\t" << elementTable[2].stress[0][0] << "\n";
	//	cout << currtime << "\t" << elementTable[3].rightNode->pos[2] << "\n";
		cout_dpl << flush;
		cout_stress << flush;
		//cout_dpl.close();
		//cout_stress.close();


		//更新时间步
		if (controlset.getTimeStep(currtime)==0.0)
		{
			autostep = true;
			timestep = ttemp;
			ttemp = 1e10;//重新置为大数，用于下一次对比
		}
		else {
			autostep = false;
			timestep = controlset.getTimeStep(currtime);
		}

		if (false)
		{
			cout << "检查constrolset看是否需要输出些东东 \n";
		}
		if (DEBUG)
		{
			cout << "*****华丽丽分割线 之 检查constrolset看是否需要输出些东东***** \n";
		}

		//上面还要加些输出的语句。下面清除结点力，如internal, external, contact, hourglass, force_possive, force
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
	int flag = 0;//这里得做个标记
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