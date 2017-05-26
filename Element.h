#ifndef ELEMENT_H
#define ELEMENT_H

#include <vector>
#include <map>
#include "Node.h"
#include "Material.h"
#include "Rod_2.h"
class Element
{
public:

	Element() :failed(false){}
//	virtual void assembleMassMatrix();
//	virtual void calculateContactForces();
//	virtual void calculateExternalForces();
//	virtual void calculateNodalForces(int j, double timestep);
//	virtual void calculateStrain(double timestep, int i);
//	virtual void calculateStress(int i, double temestep);
	//virtual void checkTimeStep(double currentTimeStep);
//	virtual void checkIfFailed();
//	virtual void pars_Fembic(string& thisLine, vector<Material>& materialList, vector<Load> loadList, vector<Node>& nodeList, map<int, Node>& nodeTable);// 
//	virtual void checkIndata();
//	virtual string print_Gid(int ctr1, int gpn);
//	virtual string print_Fembin(int ctr1, int gpn);
	//virtual void setInitialConditions();
	//virtual void setInterNodePosition();
	void updateLocalCoordinateSystem();
	void calculateLocalBaseVector(double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3, Array2D<double> &baseVecotrs);

	Material& findMaterial(string name, vector<Material> materialList);
	Node& findNode(int nodeNumber, map<Node, int> nodeTable);
	Element* getElementOfType_Fembic(string& type);

	int getNodeNumber(int nr, string arg);
	

	int Number() const { return number; }
	void Number(int val) { number = val; }
	std::string Type() const { return type; }
	void Type(std::string val) { type = val; }	
	bool Processed() const { return processed; }
	void Processed(bool val) { processed = val; }
	bool hasFiled(){ return failed; }
	int Cpu_number() const { return cpu_number; }
	void Cpu_number(int val) { cpu_number = val; }
	Node * Middle_node() const { return middle_node; }
	void Middle_node(Node * val) { middle_node = val; }
	vector<Node> Nodes() const { return nodes; }
	void Nodes(vector<Node> val) { nodes = val; }
	void deActivated(){ deactivated = true; }
	bool isDeactivated(){ return deactivated; }
public:
	int number;
	string type;
	vector<Node> nodes;
	Node *middle_node;
	int cpu_number;
	bool failed, deactivated;

	static const int MESH = -2, RESULT_HEADER = -1, MESH_HEADER = -3, RESULT_SUB_HEADER = -4;
	static const int RESULT_STRESS_GLOBAL = 0, RESULT_STRESS_LOCAL = 1, RESULT_STRAIN_GLOBAL = 2, RESULT_STRAIN_LOCAL = 3;
	bool processed;
private:
};


void Element::calculateLocalBaseVector(double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3, Array2D<double> &baseVecotrs){
	baseVecotrs = Array2D<double>(3, 3, 0.0);
	//define the local x-axis
	baseVecotrs[0][0] = x2 - x1;
	baseVecotrs[0][1] = y2 - y1;
	baseVecotrs[0][2] = z2 - z1;

	//define the local y-axis
	baseVecotrs[1][0] = x3 - x1;
	baseVecotrs[1][1] = y3 - y1;
	baseVecotrs[1][2] = z3 - z1;

	//calculate the local z-axis; by x crossProduct y
	baseVecotrs[2][0] = baseVecotrs[0][1] * baseVecotrs[1][2] - baseVecotrs[0][2] * baseVecotrs[1][1];
	baseVecotrs[2][1] = baseVecotrs[0][2] * baseVecotrs[1][0] - baseVecotrs[0][0] * baseVecotrs[1][2];
	baseVecotrs[2][2] = baseVecotrs[0][0] * baseVecotrs[1][1] - baseVecotrs[0][1] * baseVecotrs[1][0];

	//normalize: |vector| = vector/length(vector)
	baseVecotrs[1][0] = sqrt(baseVecotrs[2][0] * baseVecotrs[2][0] + baseVecotrs[2][1] * baseVecotrs[2][1] + baseVecotrs[2][2] * baseVecotrs[2][2]);
	baseVecotrs[2][0] /= baseVecotrs[1][0];
	baseVecotrs[2][1] /= baseVecotrs[1][0];
	baseVecotrs[2][2] /= baseVecotrs[1][0];

	baseVecotrs[1][0] = sqrt(baseVecotrs[0][0] * baseVecotrs[0][0] + baseVecotrs[0][1] * baseVecotrs[0][1] + baseVecotrs[0][2] * baseVecotrs[0][2]);
	baseVecotrs[0][0] /= baseVecotrs[1][0];
	baseVecotrs[0][1] /= baseVecotrs[1][0];
	baseVecotrs[0][2] /= baseVecotrs[1][0];

	//calculate y-axis:  cross product
	baseVecotrs[1][0] = baseVecotrs[2][1] * baseVecotrs[0][2] - baseVecotrs[2][2] * baseVecotrs[0][1];
	baseVecotrs[1][1] = baseVecotrs[2][2] * baseVecotrs[0][0] - baseVecotrs[2][0] * baseVecotrs[0][2];
	baseVecotrs[1][2] = baseVecotrs[2][0] * baseVecotrs[0][1] - baseVecotrs[2][1] * baseVecotrs[0][0];

}

Material& Element::findMaterial(string name, vector<Material> materialList){

	for (auto val : materialList)
	{
		if (val.Name() == name)
		{
			return val;
		}
	}
}


/************************************************************************/
/*
Node& Element::findNode(int nodeNumber, map<Node, int> nodeTable){
Node tempNode;
tempNode = nodeTable[nodeNumber];
//if (tempNode!=NULL)
//{
return tempNode;
//}
}

*/
/************************************************************************/



//************************************
// Method:    getElementOfType_Fembic
// FullName:  Element::getElementOfType_Fembic
// Access:    public 
// Returns:   Element&
// Qualifier:
// Parameter: string type
//************************************
/************************************************************************/

/************************************************************************/
/*
Element& Element::getElementOfType_Fembic(string& type){
transform(type.begin(), type.end(),type.begin(), toupper);
if (type=="ROD_2")
{
Rod_2 *rod;
rod = new Rod_2();
return *rod;
}

}


*/
/************************************************************************/







//************************************
// Method:    getNodeNumber
// FullName:  Element::getNodeNumber
// Access:    public 
// Returns:   int
// Qualifier:
// Parameter: int nr
// Parameter: string arg
//************************************
int Element::getNodeNumber(int nr, string arg){

	return 0;

}


#endif