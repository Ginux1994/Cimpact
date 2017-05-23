#ifndef WORKER_H
#define WORKER_H



#include "allInclude.h"
#include <vector>
using namespace std;
class Worker
{
public:
	Worker();
	Worker(double time, double timestep, bool autostep);

	inline void addElement(Rod_2& element){ elementList.push_back(element); };
	void removeElement(Rod_2& element);
	void addNode(Node& node){ nodeList.push_back(node); }
	void removeNode(Node& node);
	void run();



protected:
public:
	Rod_2 temporary_element;
	Node temporary_node;
	vector<Rod_2> elementList;
	vector<Node> nodeList;
	double time, timestep, ttemp;
	bool keep_running;
	bool autostep;


};

Worker::Worker(){

}

Worker::Worker(double time, double timestep, bool autostep){
	time = time;
	timestep = timestep;
	autostep = autostep;
}

void Worker::removeElement(Rod_2& element){
	vector<Rod_2>::iterator eraseIt = elementList.begin() + element.elementID;
	elementList.erase(eraseIt);
}
void Worker::removeNode(Node& node){
	vector<Node>::iterator eraseIt = nodeList.begin() + node.number;
	nodeList.erase(eraseIt);
}

void Worker::run(){
	int number_of_intergration_points=temporary_element.numberOfIntefrationPoints;
	int number_of_nodes = nodeList.size();

	if (autostep) ttemp = 1e10;
	//如果多线程处理，下面的循环在一个while循环中，可能会被打断，作为线程停止
	for (int i = 0; i < elementList.size(); i++)
	{
		//已经把单元表中的单元赋给单元列了，也就是说，单元按照编号有序排列
		if (!elementList[i].isDeactivated())
		{
			// update local coordinate systems
			elementList[i].updateLocalBaseVector();
			for (int j = 0; j < number_of_intergration_points; j++)
			{
				//对于杆单元，只有一个积分点，计算时并不用
				number_of_intergration_points = elementList[i].numberOfIntefrationPoints;
				elementList[i].calculateStrain(timestep, j);
				elementList[i].calculateStress(timestep, j);
			}
			//有了应变应力（对于每个积分点上的），就可以计算节点力了
			for (int j = 0; j < number_of_intergration_points; j++)
			{
				elementList[i].calculateNodalForces(timestep, j);
			}
			//计算外力及接触力
			elementList[i].calculateExternalForces(timestep);
			elementList[i].calculateContactForces();
			if (autostep)
			{
				ttemp = elementList[i].checkTimeStep(ttemp);
			}
		}
	}//end for loop
	//清除节点力，准备下一次迭代
	for (int i = 0; i < nodeList.size(); i++)
	{
		if (!nodeList[i].deactivated)
		{
			nodeList[i].clearNodalForces();
		}
	}
}




#endif