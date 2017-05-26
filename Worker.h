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
	//������̴߳��������ѭ����һ��whileѭ���У����ܻᱻ��ϣ���Ϊ�߳�ֹͣ
	for (int i = 0; i < elementList.size(); i++)
	{
		//�Ѿ��ѵ�Ԫ���еĵ�Ԫ������Ԫ���ˣ�Ҳ����˵����Ԫ���ձ����������
		if (!elementList[i].isDeactivated())
		{
			// update local coordinate systems
			elementList[i].updateLocalBaseVector();
			for (int j = 0; j < number_of_intergration_points; j++)
			{
				//���ڸ˵�Ԫ��ֻ��һ�����ֵ㣬����ʱ������
				number_of_intergration_points = elementList[i].numberOfIntefrationPoints;
				elementList[i].calculateStrain(timestep, j);
				elementList[i].calculateStress(timestep, j);
			}
			//����Ӧ��Ӧ��������ÿ�����ֵ��ϵģ����Ϳ��Լ���ڵ�����
			for (int j = 0; j < number_of_intergration_points; j++)
			{
				elementList[i].calculateNodalForces(timestep, j);
			}
			//�����������Ӵ���
			elementList[i].calculateExternalForces(timestep);
			elementList[i].calculateContactForces();
			if (autostep)
			{
				ttemp = elementList[i].checkTimeStep(ttemp);
			}
		}
	}//end for loop
	//����ڵ�����׼����һ�ε���
	for (int i = 0; i < nodeList.size(); i++)
	{
		if (!nodeList[i].deactivated)
		{
			nodeList[i].clearNodalForces();
		}
	}
}




#endif