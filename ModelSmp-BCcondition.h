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

	map<int, Node> nodeTable;//节点映射表

	vector<Node> nodeList;//节点向量

	vector<Rod_2> elementList;//单元向量

	map<int, Rod_2> elementTable;//单元映射表

	vector<Elastoplastic> materialList;//材料向量

	vector<BdCondition> constraintList;//约束向量

	vector<BCondition> BConstraintList;

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
	const bool DEBUG = true;

};


ModelSmp::ModelSmp(int nCPUs, string& fileName){
	using namespace std;
	nr_of_CPUs = nCPUs;
	filePath = fileName;
	//解析文件，将数据存放在各个nodeList, nodeTable, elementList, elementTable, loadList, materialList, constraintList, controlset中
	indataFile = Fembic(filePath);
	indataFile.parseControlset(controlset);//解析控制流
	indataFile.parseConstraint(BConstraintList);//解析约束集
	indataFile.parseMaterial(materialList);//解析材料集
	indataFile.parseLoad(loadList);//解析荷载集
	indataFile.parseNodeVectorAndTable(nodeList, nodeTable, BConstraintList, loadList);//解析节点集，并将约束荷载放入存入节点的成员变量中，形成节点向量，节点映射表

	indataFile.parseElementOfRod_2(materialList, nodeList, loadList, nodeTable, elementList, elementTable);//解析单元集，并将材料节点集放入相应单元的成员变量中，形成单元向量，单元映射表
	//至此，输入文件中的所有信息都有了，可以直接用于计算

	worker = Worker(currtime, timestep, autostep);
	/************************************************************************/
	/* 
	类成员的初始化方案，就是这么简单，不需要new，new和指针搭配出现
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
	//哼，就不信这个邪了，把nodeTable传进去看mass还有没有值，但这样意味着，以后每次计算，要想让nodeTable中的node 有值，就必须做两次赋值，坑爹

	cout << "assemble node \n";
	for (auto node:nodeList)
	{
		node.determineMassMatrix();
	}
/************************************************************************/
/*				继续 
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
			if (!temporary_element.isDeactivated())//失效的检查放在后面，毕竟最开始是坑定不失效的
			{
				temporary_element.updateLocalBaseVector();//update local coordinate system
				//在每个高斯积分点处计算
				for (int i = 0; i < number_of_integration_points;i++)//这里i循环从0开始不是意味着成了0个积分点，而是应为积分点处对应的物理量值在数组中存储，而数组下标从0
				{
					temporary_element.calculateStrain(timestep, i);//计算应变用的是时间步，原因是根据时间步计算应变率及上一步的应变，由此从屈服应力普中找到相应的屈服应力
					temporary_element.calculateStress(timestep, i);
				}
				//每一个积分点处的应力应变都有了之后，计算结点力
				for (int i = 0; i < number_of_integration_points;i++)
				{
					temporary_element.calculateNodalForces(timestep, i);
				}
				//单元结点力的计算，最终是加到了单元节点上,而节点的力，最终全都加到向量force_positive上了
				if (DEBUG)
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
				
				//update time step if needed
				if (autostep)
				{
					ttemp = temporary_element.checkTimeStep(ttemp);//初始ttemp很大， 首次必然自动更新时间步
				}
			}
		}//end element loop

		exported_ttmp = ttemp;// time step 的临时变量
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
					temp_element.deActivated();//如果已经破坏了，就不激活这个单元，不让他在参与计算
				}
			}
		}

		if (DEBUG)
		{
			cout << "loop constraint, and update constraint of local coordinate system";
		}
		for (auto temp_constraint:BConstraintList)
		{
			temp_constraint.update();//本质上更新的是局部坐标系
		}

		//循环节点，更新位置，为下一波应力应变计算准备
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