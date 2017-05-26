#ifndef RIGID_BODY_H
#define RIGID_BODY_H


#pragma once
#include "allInclude.h"
//#include "Node.h"
#include "Variable.h"
#include <string>
#include <vector>

#include <boost/tokenizer.hpp>
#include <iostream>
#include "tnt_array2d.h"

using namespace std;
using namespace TNT;

class Node;

class Rigid_Body
{
public:
	Rigid_Body();
	Rigid_Body::Rigid_Body(const Rigid_Body& bc);
	Rigid_Body& Rigid_Body::operator =(const Rigid_Body& bc);


	void parseFembic(string& thisLine);
	void determinMassMatrix(map<int, Node>& nodeTable);
 	inline void registerNode(Node& node){ nodes.push_back(node); }

	void applyAccelerationCondition(Node* node, double currtime);
	void applyVelocityCondition(Node& node, double currtime);
	Array2D<double>& calculateLocalBaseVectors(double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3);




	//	int test(Node& no);
protected:
public:

	vector<Node> nodes;
	Array2D<double> axis;

	Array1D<double> force, moment, moment_arm;

	//	Node master_node;
	/************************************************************************/
	/*
	妹的，这个类里面竟然歧视Node类，说人家Node未定义，我去你二大爷的！！！！
	又得迂回作战了，把这个直接在Node中实现吧
	*/
	/************************************************************************/

	//	Node nod;
	double x, y, z;

	bool axis_is_set = false;
	bool update_is_set = false;

	bool master_node_number_is_set;
	bool master_node_update;
	int master_node_number;

	string name;
	string type;

	vector<string> nameL;

};

Rigid_Body::Rigid_Body(){

	type = "RIGID_BODY";
	axis = Array2D<double>(3, 3, 0.0);
	master_node_update = false;
	nodes = vector<Node>(3);
	moment_arm = Array1D<double>(6, 0.0);
	force = Array1D<double>(6, 0.0);
	moment = Array1D<double>(6, 0.0);
//	master_node = Node();
}
Rigid_Body::Rigid_Body(const Rigid_Body& bc){

	//	nodes = bc.nodes;


	axis_is_set = bc.axis_is_set;
	update_is_set = bc.update_is_set;
	name = bc.name;
}
Rigid_Body& Rigid_Body::operator =(const Rigid_Body& bc){

	//	nodes = bc.nodes;
	x = bc.x;
	y = bc.y;
	z = bc.z;

	axis_is_set = bc.axis_is_set;
	update_is_set = bc.update_is_set;
	name = bc.name;
	return *this;
}


void Rigid_Body::parseFembic(string& thisLine){
	using namespace boost;

	//开始对thisLine进行字符扑街
	char_separator<char> sepa("= \t[],");//必须要用制表符作为分词符之一，否则会漏掉很多
	typedef tokenizer<char_separator<char>> Tokenizer;
	Tokenizer tok(thisLine, sepa);
	Tokenizer::iterator position = tok.begin();//first string in thisLine

	//	if ((*position) == "#") continue;//ignore annonatations
	string tempString = *position;
	name = tempString;
	position++;
	transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);//change into capital model
	name = tempString;



	//begin inner loop to parse the line
	while (position != tok.end())
	{

		tempString = *position;
		transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
		map<string, int> mapStr = { { "MASTER_NODE", 01 }, { "UPDATE_POSITION", 02 } };
		switch (mapStr[tempString])
		{
		case 01:
		{
			position++;
			master_node_number = (int)stod(tempString);
			position++;
			continue;
		}
		case 02:
		{
			position++;
			tempString = *position;
			transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
			if (tempString == "ON")
			{
				master_node_update = true;
			}
			position++;
			continue;
		}
		}//switch case
	}// while(position!=end)


}

/**
void Rigid_Body::update(){
if (axis_is_set&&update_is_set)
{
axis = calculateLocalBaseVectors(nodes[0].pos[0], nodes[0].pos[1], nodes[0].pos[2],
nodes[1].pos[0], nodes[1].pos[1], nodes[1].pos[2],
nodes[2].pos[0], nodes[2].pos[1], nodes[2].pos[2]);
}
}
*/


Array2D<double>& Rigid_Body::calculateLocalBaseVectors(double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3){

	Array2D<double> base_vector_system(3, 3, 0.0);

	Array1D<double> local_x_axis(3, 0.0);
	Array1D<double> local_y_axis(3, 0.0);
	Array1D<double> local_z_axis(3, 0.0);

	local_x_axis[0] = x2 - x1;
	local_x_axis[1] = y2 - y1;
	local_x_axis[2] = z2 - z1;

	local_y_axis[0] = x3 - x1;
	local_y_axis[1] = y3 - y1;
	local_y_axis[2] = z3 - z1;



	local_z_axis.crossProduct(local_y_axis);
	local_y_axis.crossProduct(local_z_axis);

	double tempLengthX = 1.0 / local_x_axis.length();
	double tempLengthY = 1.0 / local_y_axis.length();
	double tempLengthZ = 1.0 / local_z_axis.length();

	//normalise and set up the base vector system matrix
	for (int i = 0; i < 3; i++)
	{
		local_x_axis[i] = local_x_axis[i] * tempLengthX;
		local_y_axis[i] = local_y_axis[i] * tempLengthY;
		local_z_axis[i] = local_z_axis[i] * tempLengthZ;
	}

	//第一列放X, 第二列放Y, 第三列放Z
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			base_vector_system[j][i] = local_x_axis[j];
		}
	}

	return base_vector_system;
}


#endif