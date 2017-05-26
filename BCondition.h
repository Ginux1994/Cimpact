#ifndef BCONDITION_H
#define BCONDITION_H

#include "Node.h"
#include "Variable.h"
#include <string>
#include <vector>

#include <boost/tokenizer.hpp>
#include <iostream>
#include "tnt_array2d.h"

class Node;

class BCondition
{
public:
	BCondition();
	

	BCondition(const BCondition&);
	BCondition& operator=(const BCondition&);


	void applyAccelerationCondition(Node& node, double currtime);
	void applyVelocityCondition(Node& node, double currtime);
	Array2D<double>& calculateLocalBaseVectors(double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3);
	void update();
	void determinMassMatrix(map<int, Node>& nodeTable);



	void parseFembic(string& thisLine);
	void registerNode(Node& node);
	bool x_acc_is_on(double currentime);
	bool y_acc_is_on(double currentime);
	bool z_acc_is_on(double currentime);
	bool x_rot_acc_is_on(double currentime);
	bool y_rot_acc_is_on(double currentime);
	bool z_rot_acc_is_on(double currentime);
	bool x_vel_is_on(double currentime);
	bool y_vel_is_on(double currentime);
	bool z_vel_is_on(double currentime);
	bool x_rot_vel_is_on(double currentime);
	bool y_rot_vel_is_on(double currentime);
	bool z_rot_vel_is_on(double currentime);


public:
	string name;
	string type;
	Variable x_vel;
	Variable y_vel;
	Variable z_vel;
	Variable x_rot_vel;
	Variable y_rot_vel;
	Variable z_rot_vel;
	Variable x_acc;
	Variable y_acc;
	Variable z_acc;
	Variable x_rot_acc;
	Variable y_rot_acc;
	Variable z_rot_acc;
	Array2D<double> axis;
	double x, y, z;
	vector<Node> nodes;
	Node nod;
	bool x_vel_is_set = false;
	bool y_vel_is_set = false;
	bool z_vel_is_set = false;
	bool z_rot_vel_is_set = false;
	bool y_rot_vel_is_set = false;
	bool x_rot_vel_is_set = false;
	bool x_acc_is_set = false;
	bool z_acc_is_set = false;
	bool y_acc_is_set = false;
	bool x_rot_acc_is_set = false;
	bool z_rot_acc_is_set = false;
	bool y_rot_acc_is_set = false;
	bool axis_is_set = false;
	bool update_is_set = false;

protected:
private:
};




BCondition::BCondition(){
	type = "BOUNDARY_CONDITION";
	axis = Array2D<double>(3, 3, 0.0);
}

BCondition::BCondition(const BCondition& bc){
	x_vel = bc.x_vel;
	y_vel = bc.y_vel;
	z_vel = bc.z_vel;
	x_rot_vel = bc.x_rot_vel;
	y_rot_vel = bc.y_rot_vel;
	z_rot_vel = bc.z_rot_vel;
	x_acc = bc.x_acc;
	y_acc = bc.y_acc;
	z_acc = bc.z_acc;
	x_rot_acc = bc.x_rot_acc;
	y_rot_acc = bc.y_rot_acc;
	z_rot_acc = bc.z_rot_acc;
	axis = bc.axis;
	//	nodes = bc.nodes;
	x = bc.x;
	y = bc.y;
	z = bc.z;
	x_vel_is_set = bc.x_vel_is_set;
	y_vel_is_set = bc.y_vel_is_set;
	z_vel_is_set = bc.z_vel_is_set;
	z_rot_vel_is_set = bc.z_rot_vel_is_set;
	y_rot_vel_is_set = bc.y_rot_vel_is_set;
	x_rot_vel_is_set = bc.x_rot_vel_is_set;
	x_acc_is_set = bc.x_acc_is_set;
	z_acc_is_set = bc.z_acc_is_set;
	y_acc_is_set = bc.y_acc_is_set;
	x_rot_acc_is_set = bc.x_rot_acc_is_set;
	z_rot_acc_is_set = bc.z_rot_acc_is_set;
	y_rot_acc_is_set = bc.y_rot_acc_is_set;
	axis_is_set = bc.axis_is_set;
	update_is_set = bc.update_is_set;
}

BCondition& BCondition::operator =(const BCondition& bc){
	x_vel = bc.x_vel;
	y_vel = bc.y_vel;
	z_vel = bc.z_vel;
	x_rot_vel = bc.x_rot_vel;
	y_rot_vel = bc.y_rot_vel;
	z_rot_vel = bc.z_rot_vel;
	x_acc = bc.x_acc;
	y_acc = bc.y_acc;
	z_acc = bc.z_acc;
	x_rot_acc = bc.x_rot_acc;
	y_rot_acc = bc.y_rot_acc;
	z_rot_acc = bc.z_rot_acc;
	axis = bc.axis;
	//	nodes = bc.nodes;
	x = bc.x;
	y = bc.y;
	z = bc.z;
	x_vel_is_set = bc.x_vel_is_set;
	y_vel_is_set = bc.y_vel_is_set;
	z_vel_is_set = bc.z_vel_is_set;
	z_rot_vel_is_set = bc.z_rot_vel_is_set;
	y_rot_vel_is_set = bc.y_rot_vel_is_set;
	x_rot_vel_is_set = bc.x_rot_vel_is_set;
	x_acc_is_set = bc.x_acc_is_set;
	z_acc_is_set = bc.z_acc_is_set;
	y_acc_is_set = bc.y_acc_is_set;
	x_rot_acc_is_set = bc.x_rot_acc_is_set;
	z_rot_acc_is_set = bc.z_rot_acc_is_set;
	y_rot_acc_is_set = bc.y_rot_acc_is_set;
	axis_is_set = bc.axis_is_set;
	update_is_set = bc.update_is_set;
	return *this;
}

bool BCondition::x_acc_is_on(double currentime){if (!x_acc_is_set) return false; return x_acc.status(currentime);}
bool BCondition::y_acc_is_on(double currentime){ if (!y_acc_is_set) return false; return y_acc.status(currentime); }
bool BCondition::z_acc_is_on(double currentime){ if (!z_acc_is_set) return false; return z_acc.status(currentime); }
bool BCondition::x_rot_acc_is_on(double currentime){ if (!x_rot_acc_is_set) return false; return x_rot_acc.status(currentime); }
bool BCondition::y_rot_acc_is_on(double currentime){ if (!y_rot_acc_is_set) return false; return y_rot_acc.status(currentime); }
bool BCondition::z_rot_acc_is_on(double currentime){ if (!z_rot_acc_is_set) return false; return z_rot_acc.status(currentime); }
bool BCondition::x_vel_is_on(double currentime){ if (!x_vel_is_set) return false; return x_vel.status(currentime); }
bool BCondition::y_vel_is_on(double currentime){ if (!y_vel_is_set) return false; return y_vel.status(currentime); }
bool BCondition::z_vel_is_on(double currentime){ if (!z_vel_is_set) return false; return z_vel.status(currentime); }
bool BCondition::x_rot_vel_is_on(double currentime){ if (!x_rot_vel_is_set) return false; return x_rot_vel.status(currentime); }
bool BCondition::y_rot_vel_is_on(double currentime){ if (!y_rot_vel_is_set) return false; return y_rot_vel.status(currentime); }
bool BCondition::z_rot_vel_is_on(double currentime){ if (!z_rot_vel_is_set) return false; return z_rot_vel.status(currentime); }

void BCondition::registerNode(Node& node){ nodes.push_back(node); }

void BCondition::parseFembic(string& thisLine){
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
		map<string, int> mapStr = { { "VX", 01 }, { "VY", 02 }, { "VZ", 03 }, { "ARX", 11 }, { "ARY", 12 }, { "ARZ", 13 },
		{ "AX", 21 }, { "AY", 22 }, { "AZ", 23 }, { "VRX", 31 }, { "VRY", 32 }, { "VRZ", 33 }, { "P", 41 } };
		switch (mapStr[tempString])
		{
		case 01:
		{
			vector<string> vecVX;
			position++;
			for (position; position != tok.end(); position++)
			{

				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
				if ((tempString == "FY") || (tempString == "FZ") ||
					(tempString == "VRX") || (tempString == "VRY") || (tempString == "VRZ") ||
					(tempString == "AX") || (tempString == "AY") || (tempString == "AZ") ||
					(tempString == "VX") || (tempString == "VY") || (tempString == "VZ") ||
					(tempString == "ARX") || (tempString == "ARY") || (tempString == "ARZ") || (tempString == "P")) {

					break;
				}
				vecVX.push_back(tempString);
			}
			if (vecVX.size() == 1)
			{
				double fuckD = stod(vecVX[0]);
				x_vel = Variable(fuckD);
			}
			else{
				x_vel = Variable(vecVX);
			}
			x_vel_is_set = true;
			continue;
			x_vel.printV();
		}
		case 02:
		{
			vector<string> vecVY;
			for (++position; position != tok.end(); position++)//必须在第一个position上加个++，才能正确判断，并且不把VY放入vecVY
			{

				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
				if ((tempString == "VX") || (tempString == "FZ") ||
					(tempString == "VRX") || (tempString == "VRY") || (tempString == "VRZ") ||
					(tempString == "AX") || (tempString == "AY") || (tempString == "AZ") ||
					(tempString == "VX") || (tempString == "VY") || (tempString == "VZ") ||
					(tempString == "ARX") || (tempString == "ARY") || (tempString == "ARZ") || (tempString == "P"))
					break;
				vecVY.push_back(tempString);
			}
			if (vecVY.size() == 1)
			{
				double fuckD = stod(vecVY[0]);
				y_vel = Variable(fuckD);
			}
			else{
				y_vel = Variable(vecVY);
			}
			y_vel_is_set = true;
			y_vel.printV();
			continue;
		}
		case  03:
		{
			vector<string> vecVZ;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "VX") || (tempString == "VY") ||
					(tempString == "VRX") || (tempString == "VRY") || (tempString == "VRZ") ||
					(tempString == "AX") || (tempString == "AY") || (tempString == "AZ") ||
					(tempString == "VX") || (tempString == "VY") || (tempString == "VZ") ||
					(tempString == "ARX") || (tempString == "ARY") || (tempString == "ARZ") || (*position == "P"))break;
				vecVZ.push_back(tempString);
			}
			if (vecVZ.size() == 1)
			{
				double fuckD = stod(vecVZ[0]);
				z_vel = Variable(fuckD);
			}
			else{
				z_vel = Variable(vecVZ);
			}
			z_vel_is_set = true;
			z_vel.printV();
			continue;
		}
		case 11:
		{
			vector<string> vecARX;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "VX") || (tempString == "VY") ||
					(tempString == "VRX") || (tempString == "VRY") || (tempString == "VRZ") ||
					(tempString == "AX") || (tempString == "AY") || (tempString == "AZ") ||
					(tempString == "VX") || (tempString == "VY") || (tempString == "VZ") ||
					(tempString == "ARX") || (tempString == "ARY") || (tempString == "ARZ") || (*position == "P"))break;
				vecARX.push_back(tempString);
			}
			if (vecARX.size() == 1)
			{
				double fuckD = stod(vecARX[0]);
				x_rot_acc = Variable(fuckD);
			}
			else{
				x_rot_acc = Variable(vecARX);
			}
			x_rot_acc_is_set = true;
			x_rot_acc.printV();
			continue;
		}
		case 12:
		{
			vector<string> vecARY;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "VX") || (tempString == "VY") ||
					(tempString == "VRX") || (tempString == "VRY") || (tempString == "VRZ") ||
					(tempString == "AX") || (tempString == "AY") || (tempString == "AZ") ||
					(tempString == "VX") || (tempString == "VY") || (tempString == "VZ") ||
					(tempString == "ARX") || (tempString == "ARY") || (tempString == "ARZ") || (*position == "P"))break;
				vecARY.push_back(tempString);
			}
			if (vecARY.size() == 1)
			{
				double fuckD = stod(vecARY[0]);
				y_rot_acc = Variable(fuckD);
			}
			else{
				y_rot_acc = Variable(vecARY);
			}
			y_rot_acc_is_set = true;
			y_rot_acc.printV();
			continue;
		}
		case 13:
		{
			vector<string> vecARZ;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "VX") || (tempString == "VY") ||
					(tempString == "VRX") || (tempString == "VRY") || (tempString == "VRZ") ||
					(tempString == "AX") || (tempString == "AY") || (tempString == "AZ") ||
					(tempString == "VX") || (tempString == "VY") || (tempString == "VZ") ||
					(tempString == "ARX") || (tempString == "ARY") || (tempString == "ARZ") || (*position == "P"))break;
				vecARZ.push_back(tempString);
			}
			if (vecARZ.size() == 1)
			{
				double fuckD = stod(vecARZ[0]);
				z_rot_acc = Variable(fuckD);
			}
			else{
				z_rot_acc = Variable(vecARZ);
			}
			z_rot_vel_is_set = true;
			z_rot_acc.printV();
			continue;
		}
		case 21:
		{
			vector<string> vecAx;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "VX") || (tempString == "VY") ||
					(tempString == "VRX") || (tempString == "VRY") || (tempString == "VRZ") ||
					(tempString == "AX") || (tempString == "AY") || (tempString == "AZ") ||
					(tempString == "VX") || (tempString == "VY") || (tempString == "VZ") ||
					(tempString == "ARX") || (tempString == "ARY") || (tempString == "ARZ") || (*position == "P"))break;
				vecAx.push_back(tempString);
			}
			if (vecAx.size() == 1)
			{
				double fuckD = stod(vecAx[0]);
				x_acc = Variable(fuckD);
			}
			else{
				x_acc = Variable(vecAx);
			}
			x_acc_is_set = true;
			x_acc.printV();
			continue;
		}
		case 22:
		{
			vector<string> vecAy;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "VX") || (tempString == "VY") ||
					(tempString == "VRX") || (tempString == "VRY") || (tempString == "VRZ") ||
					(tempString == "AX") || (tempString == "AY") || (tempString == "AZ") ||
					(tempString == "VX") || (tempString == "VY") || (tempString == "VZ") ||
					(tempString == "ARX") || (tempString == "ARY") || (tempString == "ARZ") || (*position == "P"))break;
				vecAy.push_back(tempString);
			}
			if (vecAy.size() == 1)
			{
				double fuckD = stod(vecAy[0]);
				y_acc = Variable(fuckD);
			}
			else{
				y_acc = Variable(vecAy);
			}
			y_acc_is_set = true;
			y_acc.printV();
			continue;
		}
		case 23:
		{
			vector<string> vecAz;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "VX") || (tempString == "VY") ||
					(tempString == "VRX") || (tempString == "VRY") || (tempString == "VRZ") ||
					(tempString == "AX") || (tempString == "AY") || (tempString == "AZ") ||
					(tempString == "VX") || (tempString == "VY") || (tempString == "VZ") ||
					(tempString == "ARX") || (tempString == "ARY") || (tempString == "ARZ") || (*position == "P"))break;
				vecAz.push_back(tempString);
			}
			if (vecAz.size() == 1)
			{
				double fuckD = stod(vecAz[0]);
				z_acc = Variable(fuckD);
			}
			else{
				z_acc = Variable(vecAz);
			}
			z_acc_is_set = true;
			z_acc.printV();
			continue;
		}
			break;

		}//switch case

		if (position == tok.end()) break;


	}// while(position!=end)



}




void BCondition::applyAccelerationCondition(Node& node, double currtime){

	if (axis_is_set)
	{
		//平动部分, 根据约束值，判断是否施加加速度
		if (x_vel_is_on(currtime))
		{
			x = 0;
		}
		else if (x_acc_is_on(currtime))
		{
			x = x_acc.value(currtime);//x y z 方向的加速度，速度，在parse文件时就加入了，如：Az = [0,5] Ay = [0,0] Ax = [0,0]

		}
		else{//acc[0]-x direction; acc[1]-y direction; acc[2]-z direction
			// x= 加速度向量(3*1) 乘以 方向轴矩阵(3*3)中的第一列
			x = node.acc[0] * axis[0][0] + node.acc[1] * axis[1][0] + node.acc[2] * axis[2][0];
		}
		if (y_vel_is_on(currtime))
		{
			y = 0;
		}
		else if (y_acc_is_on(currtime))
		{
			y = y_acc.value(currtime);
		}
		else{// y= 加速度向量(3*1) 乘以 方向轴矩阵(3*3)中的第2列
			y = node.acc[0] * axis[0][1] + node.acc[1] * axis[1][1] + node.acc[2] * axis[2][1];
		}
		if (z_vel_is_on(currtime))
		{
			z = 0;
		}
		else if (z_acc_is_on(currtime))
		{
			z = z_acc.value(currtime);
		}
		else{// z= 加速度向量(3*1) 乘以 方向轴矩阵(3*3)中的第3列
			z = node.acc[0] * axis[0][2] + node.acc[1] * axis[1][2] + node.acc[2] * axis[2][2];
		}

		node.acc[0] = x*axis[0][0] + y*axis[0][1] + z*axis[0][2];
		node.acc[1] = x*axis[1][0] + y*axis[1][1] + z*axis[1][2];
		node.acc[2] = x*axis[2][0] + y*axis[2][1] + z*axis[2][2];

		//旋转部分
		if (x_rot_vel_is_on(currtime))
		{
			x = 0;
		}
		else if (x_rot_acc_is_on(currtime))
		{
			x = x_rot_acc.value(currtime);
		}
		else{//acc[0]-x direction; acc[1]-y direction; acc[2]-z direction
			// x= 加速度向量(3*1) 乘以 方向轴矩阵(3*3)中的第一列
			x = node.acc[3] * axis[0][0] + node.acc[4] * axis[1][0] + node.acc[5] * axis[2][0];
		}
		if (y_rot_vel_is_on(currtime))
		{
			y = 0;
		}
		else if (y_rot_acc_is_on(currtime))
		{
			y = y_rot_acc.value(currtime);
		}
		else{// y= 加速度向量(3*1) 乘以 方向轴矩阵(3*3)中的第2列
			y = node.acc[3] * axis[0][1] + node.acc[4] * axis[1][1] + node.acc[5] * axis[2][1];
		}
		if (z_vel_is_on(currtime))
		{
			z = 0;
		}
		else if (z_rot_acc_is_on(currtime))
		{
			z = z_rot_acc.value(currtime);
		}
		else{// z= 加速度向量(3*1) 乘以 方向轴矩阵(3*3)中的第3列
			z = node.acc[3] * axis[0][2] + node.acc[4] * axis[1][2] + node.acc[5] * axis[2][2];
		}

		node.acc[3] = x*axis[0][0] + y*axis[0][1] + z*axis[0][2];
		node.acc[4] = x*axis[1][0] + y*axis[1][1] + z*axis[1][2];
		node.acc[5] = x*axis[2][0] + y*axis[2][1] + z*axis[2][2];
	}
	else{
		//translate part
		if (x_vel_is_on(currtime))
		{
			node.acc[0] = 0;
		}
		else if (x_acc_is_on(currtime))
		{
			node.acc[0] = x_acc.value(currtime);
		}

		if (y_vel_is_on(currtime))
		{
			node.acc[1] = 0;
		}
		else if (y_acc_is_on(currtime))
		{
			node.acc[1] = y_acc.value(currtime);
		}

		if (z_vel_is_on(currtime))
		{
			node.acc[2] = 0;
		}
		else if (z_acc_is_on(currtime))
		{
			node.acc[2] = z_acc.value(currtime);
		}

		//rot part
		if (x_rot_vel_is_on(currtime))
		{
			node.acc[3] = 0;
		}
		else if (x_rot_vel_is_on(currtime))
		{
			node.acc[3] = x_rot_acc.value(currtime);
		}

		if (y_rot_vel_is_on(currtime))
		{
			node.acc[4] = 0;
		}
		else if (y_rot_vel_is_on(currtime))
		{
			node.acc[4] = y_rot_acc.value(currtime);
		}

		if (z_rot_vel_is_on(currtime))
		{
			node.acc[5] = 0;
		}
		else if (z_rot_vel_is_on(currtime))
		{
			node.acc[5] = z_rot_acc.value(currtime);
		}
	}
	cout << "acc bc has been added" << endl;

}


void BCondition::applyVelocityCondition(Node& node, double currtime){

	if (axis_is_set)
	{
		//平动部分, 根据约束值，判断是否施加加速度
		if (x_vel_is_on(currtime))
		{//此时正施加着速度约束
			x = x_vel.value(currtime);
		}
		else
		{//(vx, vy, vz)*AXIS  (axis第一列)以次后推
			x = node.vel[0] * axis[0][0] + node.vel[1] * axis[1][0] + node.vel[2] * axis[2][0];
		}

		if (y_vel_is_on(currtime))
		{
			y = y_vel.value(currtime);
		}
		else
		{
			y = node.vel[0] * axis[0][1] + node.vel[1] * axis[1][1] + node.vel[2] * axis[2][1];
		}
		if (z_vel_is_on(currtime))
		{
			z = z_vel.value(currtime);
		}
		else
		{
			z = node.vel[0] * axis[0][2] + node.vel[1] * axis[1][2] + node.vel[2] * axis[2][2];
		}

		// vel = AXIS*(x,y,z)^T
		node.vel[0] = x*axis[0][0] + y*axis[0][1] + z*axis[0][2];
		node.vel[1] = x*axis[1][0] + y*axis[1][1] + z*axis[1][2];
		node.vel[2] = x*axis[2][0] + y*axis[2][1] + z*axis[2][2];



		//旋转部分
		//x-direction
		if (x_rot_vel_is_on(currtime))
		{//此时正施加着速度约束
			x = x_rot_vel.value(currtime);
		}
		else
		{
			x = node.vel[3] * axis[0][0] + node.vel[3] * axis[1][0] + node.vel[3] * axis[2][0];
		}
		//y-direction
		if (y_rot_vel_is_on(currtime))
		{
			y = y_rot_vel.value(currtime);
		}
		else
		{
			y = node.vel[4] * axis[0][1] + node.vel[4] * axis[1][1] + node.vel[4] * axis[2][1];
		}
		//z-direction
		if (z_rot_vel_is_on(currtime))
		{
			z = z_rot_vel.value(currtime);
		}
		else
		{
			z = node.vel[5] * axis[0][2] + node.vel[5] * axis[1][2] + node.vel[5] * axis[2][2];
		}

		// vel = AXIS*(x,y,z)^T
		node.vel[3] = x*axis[0][0] + y*axis[0][1] + z*axis[0][2];
		node.vel[4] = x*axis[1][0] + y*axis[1][1] + z*axis[1][2];
		node.vel[5] = x*axis[2][0] + y*axis[2][1] + z*axis[2][2];
	}
	else{

		if (x_vel_is_on(currtime))
		{
			node.vel[0] = x_vel.value(currtime);
		}
		if (y_vel_is_on(currtime))
		{
			node.vel[1] = y_vel.value(currtime);
		}
		if (z_vel_is_on(currtime))
		{
			node.vel[2] = z_vel.value(currtime);
		}

		//rot part
		if (x_rot_vel_is_on(currtime))
		{
			node.vel[3] = x_rot_vel.value(currtime);
		}

		if (y_rot_vel_is_on(currtime))
		{
			node.vel[4] = y_rot_vel.value(currtime);
		}
		if (z_rot_vel_is_on(currtime))
		{
			node.vel[5] = z_rot_vel.value(currtime);
		}
	}


}


void BCondition::update(){
	if (axis_is_set&&update_is_set)
	{
		axis = calculateLocalBaseVectors(nodes[0].pos[0], nodes[0].pos[1], nodes[0].pos[2],
			nodes[1].pos[0], nodes[1].pos[1], nodes[1].pos[2],
			nodes[2].pos[0], nodes[2].pos[1], nodes[2].pos[2]);
	}
}

Array2D<double>& BCondition::calculateLocalBaseVectors(double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3){

	Array2D<double> base_vector_system(3, 3);

	Array1D<double> local_x_axis(3);
	Array1D<double> local_y_axis(3);
	Array1D<double> local_z_axis(3);

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
	local_x_axis *= tempLengthX;
	local_y_axis *= tempLengthY;
	local_z_axis *= tempLengthZ;

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

void BCondition::determinMassMatrix(map<int, Node>& nodeTable){

	if (axis_is_set)
	{
		for (int j = 0; j < 3; j++)
		{
			nodes[j] = nodeTable[j + 1];
		}
		axis = calculateLocalBaseVectors(nodes[0].pos[0], nodes[0].pos[1], nodes[0].pos[2],
			nodes[1].pos[0], nodes[1].pos[1], nodes[1].pos[2],
			nodes[2].pos[0], nodes[2].pos[1], nodes[2].pos[2]);
	}
}














#endif

