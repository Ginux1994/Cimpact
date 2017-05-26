#ifndef ROD_2
#define ROD_2

#include "Elastoplastic.h"
#include "Elastic.h"
#include "tnt.h"
#include "tnt_array2d.h"
#include "tnt_array1d.h"
#include "Element.h"
#include <boost/tokenizer.hpp>
#include <math.h>
using namespace boost;
using namespace std;

class Rod_2
{
public:
	Rod_2();
	void parse_Fembic(string& thisLine, vector<Elastic>& materialList, vector<Load> loadList, vector<Node>& nodeList, map<int, Node>& nodeTable);
	void setInitialConditions();
	double checkTimeStep(double timestep);
	void checkIfFailed();

	void calculateMassMatrix(map<int, Node>&);
	void calculateLocalBaseVector(double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3, Array2D<double> &baseVecotrs);
	void updateLocalBaseVector();
	void calculateStrain(double timestep, int integration_points);
	void calculateStress(double timestep, int integration_points);
	void calculateNodalForces(double timestep, int integration_points);
	void calculateContactForces();
	void calculateExternalForces(double currtime);


	Elastoplastic& findMaterial(string name, vector<Elastoplastic> materialList);
	int ElementID() const { return elementID; }
	void ElementID(int val) { elementID = val; }

	inline bool isDeactivated(){ return deactivated; }
	inline void deActivated(){ deactivated = true; }
	inline bool hasFailed(){
		return failed;
	/************************************************************************/
	/* 		if (internal_contact_element != null)
			internal_contact_element.deActivate();                                                                     */
	/************************************************************************/
	 }
	std::string Type() const { return type; }
	void Type(std::string val) { type = val; }



public://just for easy
	Node *leftNode, *rightNode;
	Elastic material;

	Array2D<double> inertia, force, global_force;
	Array1D<double> dstrain, strain, stress; 
	Array2D<double> local_coordinate_system;

	//Array1D<double> global_force_1;

	string type;	
	double diameter;
	double initial_cross_section_area, initial_length;
	double cross_section_area,factor, friction;
	double elementMass;
	int elementID;

	const static int DISABLED = 0;
	const static int BASIC = 1;
	int Contact, numberOfIntefrationPoints = 1;
	bool node_set, material_set, factor_set, friction_set, diameter_set;
	bool processed;
	bool deactivated;
	bool failed = false;
protected:
private:

};

Rod_2::Rod_2(){

	type = "ROD_2";
	inertia = Array2D<double>(3, 3,0.0);
	force = Array2D<double>(3, 3, 0.0);
	stress = Array1D<double>(6, 0.0);
	strain = Array1D<double>(6, 0.0);
	dstrain = Array1D<double>(6, 0.0);
	local_coordinate_system = Array2D<double>(3, 3, 0.0);
	processed = false;
	Contact = BASIC;

}


Elastoplastic& Rod_2::findMaterial(string name, vector<Elastoplastic> materialList){

	for (auto val : materialList)
	{
		if (val.name == name)
		{
			return val;
		}
	}
}


void Rod_2::parse_Fembic(string& thisLine, vector<Elastic>& materialList, vector<Load> loadList, vector<Node>& nodeList, map<int, Node>& nodeTable){
	//开始对thisLine进行字符扑街
	char_separator<char> sepa("= \t [],");//必须要用制表符作为分词符之一，否则会漏掉很多
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
			//LeftNode(nodeTable[(int)stod(*position)]);
			leftNode = &(nodeTable[(int)stod(*position)]);
			
			nodeTable[(int)stod(*position)] = *leftNode;

			position++;
			//RightNode(nodeTable[(int)stod(*position)]);
			rightNode = &(nodeTable[(int)stod(*position)]);
			
			nodeTable[(int)stod(*position)] = *rightNode;
			
		}
		else if (tempString=="MATERIAL")
		{
			position++;
			tempString = *position;
			transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
			//material = Element::findMaterial(*position, materialList);
			material = materialList[0];
			material.name = tempString; 
			material_set = true;
			cout << "material is set ,it's name is "<<tempString<<" \n";
			cout << "material.density " << material.density<<endl;
			cout << "material.failure_strss " << material.failure_strss << endl;
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
		else if (tempString=="D")
		{
			position++;
			tempString = *position;
			diameter = stod(tempString);
			diameter_set = true;
			
		}
		else if (tempString == "FRACTOR")
		{
			position++;
			tempString = *position;
			factor = stod(tempString);
			factor_set = true;
		
		}
		else if (tempString == "FRICTION")
		{
			position++;
			tempString = *position;
			friction = stod(tempString);
			friction_set = true;
			
		}
		else if (tempString == "CONTACT")
		{
			position++;
			tempString = *position;
			transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);

			if (tempString=="OFF")
			{
				Contact = DISABLED;
			}
			position++;
			tempString = *position;
			transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
		}

		//if contact is true, transfrom to contact le
	}



}

void Rod_2::setInitialConditions(){
	material.setInitialCondition();
	/************************************************************************/
	/* 
		if (Contact == BASIC) {
		internal_contact_element.setInitialConditions();
	}
	*/
	/************************************************************************/
}


double Rod_2::checkTimeStep(double current_timestep){
	double timestep;
	double critical_length;


	critical_length = sqrt((leftNode->pos[0] - rightNode->pos[0])*(leftNode->pos[0] - rightNode->pos[0])
		+ (leftNode->pos[1] - rightNode->pos[1])*(leftNode->pos[1] - rightNode->pos[1])
		+ (leftNode->pos[2] - rightNode->pos[2])*(leftNode->pos[2] - rightNode->pos[2]));

	timestep = critical_length / material.wavespeedOneDimendinal();
	return (0.9*timestep < current_timestep) ? (0.9*timestep) : current_timestep;
}

void Rod_2::calculateMassMatrix(map<int, Node>& nodeTable){
	//本来单元中的左右节点 应该和 nodeList， nodeTable偶合在一起的，但目前还是实现不了
	double mass;

	initial_cross_section_area = 3.1414926*diameter*diameter / 4;
	cross_section_area = initial_cross_section_area;
	initial_length = sqrt((leftNode->pos[0] - rightNode->pos[0])*(leftNode->pos[0] - rightNode->pos[0])
		+ (leftNode->pos[1] - rightNode->pos[1])*(leftNode->pos[1] - rightNode->pos[1])
		+ (leftNode->pos[2] - rightNode->pos[2])*(leftNode->pos[2] - rightNode->pos[2]));
	mass = material.density*cross_section_area*initial_length;
	elementMass = mass;
	leftNode->mass += mass / 2.0;
	//nodeTable[leftNode->number].mass = mass / 2.0;
	rightNode->mass += mass / 2.0;
	//nodeTable[rightNode->number].mass = mass / 2.0;

	double a = mass*diameter*diameter / 8.0;
	double b = mass*initial_length*initial_length / 12;
	inertia[0][0] = a > b ? a : b;
	inertia[1][1] = inertia[0][0];
	inertia[2][2] = inertia[0][0];

	leftNode->inertia += inertia;
	//nodeTable[leftNode->number].inertia = inertia;
	rightNode->inertia += inertia;
	//nodeTable[rightNode->number].inertia = inertia;

	/************************************************************************/
	/* 
			if (Contact == BASIC) {
			internal_contact_element.assembleMassMatrix();
			}
	*/
	/************************************************************************/

}

void Rod_2::calculateLocalBaseVector(double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3, Array2D<double> &baseVecotrs){
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

void Rod_2::updateLocalBaseVector(){
	calculateLocalBaseVector(0, 0, 0, 
		leftNode->pos[0] - rightNode->pos[0],
		leftNode->pos[1] - rightNode->pos[1], 
		leftNode->pos[2] - rightNode->pos[2], 
		leftNode->pos[2] - rightNode->pos[2],
		rightNode->pos[0] - leftNode->pos[0],
		leftNode->pos[1] - rightNode->pos[1],
		local_coordinate_system);

	/************************************************************************/
	/* 
		if (Contact==BASIC)
		{
		internal_contact_element.uodaptLocalBaseVector();
		}

	*/
	/************************************************************************/
}

void Rod_2::calculateStrain(double timestep, int integration_points){
	//sqrt(dx^2+dy^2+dz^2)
	double new_length = sqrt((leftNode->pos[0] - rightNode->pos[0])*(leftNode->pos[0] - rightNode->pos[0]) +
		(leftNode->pos[1] - rightNode->pos[1])*(leftNode->pos[1] - rightNode->pos[1]) +
		(leftNode->pos[2] - rightNode->pos[2])*(leftNode->pos[2] - rightNode->pos[2]));
	//ln strain
	dstrain[0] = log(1 + (new_length - initial_length) / initial_length) - strain[0];
	// Calculate also the new cross section area (assuming incompressible
	// material)
	cross_section_area = initial_cross_section_area*initial_length / new_length;

}

void Rod_2::calculateStress(double timestep, int integration_points){
	material.calculateStressOneDimensional(strain, dstrain, stress, timestep);
}

//check the element is destroyed or not
void Rod_2::checkIfFailed(){
	if ((!material.failure_strain_is_set)&&(!material.failure_strss_is_set))
	{//说明是个弹性体，没有损伤极限
		failed = false;
		return;
	}
	if (material.failure_strss_is_set)
	{
		if (stress[0]>material.failure_strss)
		{
			failed = true;
		}
	}
	if (material.failure_strain_is_set)
	{
		if (strain[0]>material.failure_strain)
		{
			failed = true;
			return;
		}
	}
	failed = false;
}

void Rod_2::calculateNodalForces(double timestep, int integration_points){
	
	force[0][0] = stress[0] * cross_section_area;
	//transform to global coordinates
	global_force = local_coordinate_system.transpose()*force;//3*3的矩阵，要转化为6*1的，再给global_force_1
	//global_force_1 = Array1D<double>(6, 0.0);
	Array1D<double> global_force_1(6, 0.0), global_force_r(6, 0.0);
	for (int i = 0; i < 3;i++)
	{
		global_force_1[i] = global_force[i][0];
	}
	leftNode->addInternalForce(global_force_1);
	

	for (int i = 0; i < 3; i++)
	{
		global_force_r[i] = global_force_1[i];//-号出现难以置信的结果
	}

	//但问题来了，不加负号合理吗
	//这个地方的设置，对结果的影响至关重要，妈的，参考程序竟然是错的，根本不需要加-号，靠，看来得小心谨慎了
	rightNode->addInternalForce(global_force_r);
}

void Rod_2::calculateExternalForces(double currtime){
	//计算诸如重力，等体力，将其转化到节点上，并嫁给节点
	//如果有接触单元，接触力就作为外力，在这里计算
	/************************************************************************/
	/* 
			if (Contact == BASIC) {
			internal_contact_element.calculateExternalForces(currtime);
			}
	*/
	/************************************************************************/
}
void Rod_2::calculateContactForces(){
	/************************************************************************/
	/*
	if (Contact == BASIC) {
	internal_contact_element.calculateExternalForces(currtime);
	}
	*/
	/************************************************************************/
}



#endif