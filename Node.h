#ifndef NODE_H
#define NODE_H

#pragma once

#include <iostream>
#include "BdCondition.h"
//#include "Rigid_Body.h"
#include "Rigid_Body.h"
//#include "BCondition.h"
//这个头文件一加上去就会影响BCondition的编译通过，使得很多正常的函数成员发生错误，也就是说，这两个头文件不能相互包含
#include "Load.h"
//#include "Constraint.h"
#include "tnt.h"
#include "tnt_array2d.h"
#include <math.h>
#include "jama_lu.h"



class BdCondition;
class BCondition;
class Rigid_Body;

using namespace TNT;
using namespace std;
using namespace JAMA;
class Node
{
public:
	Node();
	Node(const Node &node);
	Node& operator =(const Node &node);
	//更新力属性
	void addExternalForce(const Array1D<double> &);
	void addInternalForce(const Array1D<double> &);
	void addContactForce(const Array1D<double> &);
	void addHourglassForce(const Array1D<double> &);
	void addInternalMoment(const Array1D<double> &);
	void addExternalMoment(const Array1D<double> &);
	void addHourglassMoment(const Array1D<double> &);
	void clearNodalForces();
	void determineInertiaMatrix();
	void calculateNewPosition(double temestep, double currtime);
	void setInitialConditions(vector<Node> &nodeList);
	void checkNeighbours();
	bool operator<(const Node& nodeToCompare);
	//施加边界条件，本来应该在BCondition中的方法被迫迁移到了node里
	void applyAccelerationBCondition(double currtime);
	void applyVelocityBCondition(double currtime);

	void applyAccelerationRigidCondition(double currtime);
	void applyVelocityRigidCondition(double currtime);

	bool isDeactivated(){ return deactivated; }
	Node * Left_neighbour() const { return left_neighbour; }
	void Left_neighbour(Node * val) { left_neighbour = val; }
	Node * Right_neighbour() const { return right_neighbour; }
	void Right_neighbour(Node * val) { right_neighbour = val; }
	void Deactivated(){ left_neighbour->Right_neighbour(right_neighbour); right_neighbour->Left_neighbour(left_neighbour); deactivated = true; }


	double Mass() const { return mass; }//C++中重构get，set方法是以变量名首字母大写的函数形式来实现的，不需要get set关键词
	void Mass(double val) { mass = val; }
	void addMass(double val){ mass += val; }
	Array2D<double> Inertia() const { return inertia; }
	void Inertia(Array2D<double> val) { inertia = val; }
	void addInertia(Array2D<double> val){ inertia += val; }
	
	Array1D<double> Force() const { return force; }
	void Force(Array1D<double> val) { force = val; }
	Array1D<double> External_force() const { return external_force; }
	void External_force(Array1D<double> val) { external_force = val; }
	Array1D<double> Internal_force() const { return internal_force; }
	void Internal_force(Array1D<double> val) { internal_force = val; }
	Array1D<double> Hourglass_force() const { return hourglass_force; }
	void Hourglass_force(Array1D<double> val) { hourglass_force = val; }
	Array1D<double> Contact_force() const { return contact_force; }
	void Contact_force(Array1D<double> val) { contact_force = val; }
	Array1D<double> Force_positive() const { return force_positive; }
	void Force_positive(Array1D<double> val) { force_positive = val; }
	int Type() const { return type; }
	void Type(int val) { type = val; }
	int Cpu_number() const { return cpu_number; }
	void Cpu_number(int val) { cpu_number = val; }
	int Number() const { return number; }
	void Number(int val) { number = val; }
	Load getLoad() const { return load; }
	void setLoad(Load& val) { load = val; }
	BdCondition getConstraint() const { return constraint; }
	void setConstraint(BdCondition& val) { constraint = val; }
	double Internal_energy() const { return internal_energy; }
	void Internal_energy(double val) { internal_energy = val; }
	double External_energy() const { return external_energy; }
	void External_energy(double val) { external_energy = val; }
	double Contact_energy() const { return contact_energy; }
	void Contact_energy(double val) { contact_energy = val; }
	double Hourglass_energy() const { return hourglass_energy; }
	void Hourglass_energy(double val) { hourglass_energy = val; }
	void setX_pos_orig(double param) { pos_orig[0] = param; pos[0] = param + dpl[0]; }

	void setY_pos_orig(double param){ pos_orig[1] = param; pos[1] = param + dpl[1]; }

	void setZ_pos_orig(double param){ pos_orig[2] = param; pos[2] = param + dpl[2]; }


public:
	int type;
	bool deactivated;
//public just for easy
public:
	Node *left_neighbour;
	Node *right_neighbour;	
	Node *master_node;
	/************************************************************************/
	/* 这里Node不能作为对象的形式成为成员，只能作为指针，否则报错                                                                     */
	/************************************************************************/
	Load load;

	BdCondition constraint;
	bool BdCondition_is_set;
	Rigid_Body rigidConstraint;
	bool rigidCondition_is_set;
//	BCondition bcondition;

	//shoule be private, here for test
	double mass;
	double internal_energy, external_energy, contact_energy, hourglass_energy;
	double halfstep, oldstep, temp_timestep_for_test;
	int number;//node ID
	int cpu_number;//parallel
	static const int NODE = 1;
	static const int INTERNAL_NODE = 2;
	bool x_is_set, Y_is_set, Z_is_set;
	bool M_is_set;
	bool IXX_is_set, IYY_is_set, IZZ_is_set, IXY_is_set, IYZ_is_set, IXZ_is_set;
	bool load_is_set = false;
	bool constraint_is_set = false;
	bool DEBUG = true;
	/************************************************************************/
	/* 由于RIgid的不配和，只好转移到这里                                                                     */
	/************************************************************************/
	bool is_a_master_node;

	Array1D<double> pos, pos_orig;
	Array2D<double> inertia;

	Array1D<double> force, external_force, internal_force, hourglass_force, contact_force, force_positive;
	Array1D<double> external_force_old, internal_force_old, hourglass_force_old, contact_force_old;
	Array1D<double> external_moment, internal_noment;
	Array1D<double> lastload;
	Array2D<double> inv_inertia;
	Array1D<double> acc, vel, dpl, dpl_old;
	Array1D<double> temp_dpl;
	
};

Node::Node(){
	vel = Array1D<double>(6, 0.0);//前三个为平动分量，后三个为转动分量
	//vel(6, 1, 0.0);
	acc = Array1D<double>(6,0.0);
	//acc(6, 1, 0.0);
	dpl = Array1D<double>(6, 0.0);
	dpl_old = Array1D<double>(6, 0.0);
	pos_orig = Array1D<double>(6, 0.0);
	pos = Array1D<double>(6, 0.0);
	force = Array1D<double>(6, 0.0);
	force_positive = Array1D<double>(6, 0.0);
	internal_force = Array1D<double>(6, 0.0);
	internal_force_old = Array1D<double>(6, 0.0);
	external_force= Array1D<double>(6, 0.0);
	external_force_old = Array1D<double>(6, 0.0);
	hourglass_force = Array1D<double>(6, 0.0);
	hourglass_force_old= Array1D<double>(6, 0.0);
	contact_force = Array1D<double>(6,0.0);
	contact_force_old = Array1D<double>(6, 0.0);
	force_positive = Array1D<double>(6, 0.0);
	lastload = Array1D<double>(6, 0.0);
	inertia = Array2D<double>(3, 3, 0.0);
	temp_dpl = Array1D<double>(6, 0.0);
	mass = 0.0;
	type = NODE;
	//load = new Load();

}
Node::Node(const Node &node){
	vel = node.vel;//前三个为平动分量，后三个为转动分量
	//vel(6, 1, 0.0);
	acc = node.acc;
	//acc(6, 1, 0.0);
	dpl = node.dpl;
	dpl_old = node.dpl_old;
	pos_orig = node.pos_orig;
	pos = node.pos;
	force = node.force;
	force_positive = node.force_positive;
	internal_force = node.internal_force;
	internal_force_old = node.internal_force_old;
	external_force = node.external_force;
	external_force_old = node.external_force_old;
	hourglass_force = node.hourglass_force;
	hourglass_force_old = node.hourglass_force_old;
	contact_force = node.contact_force;
	contact_force_old = node.contact_force_old;
	force_positive = node.force_positive;
	lastload = node.lastload;
	inertia = node.inertia;
	temp_dpl = node.temp_dpl;
	mass = node.mass;
}
bool Node::operator <(const Node& nodeToCompare){
	return pos[0] < nodeToCompare.pos[0];
}
Node& Node::operator =(const Node &node){
	vel = node.vel;//前三个为平动分量，后三个为转动分量
	//vel(6, 1, 0.0);
	acc = node.acc;
	//acc(6, 1, 0.0);
	dpl = node.dpl;
	dpl_old = node.dpl_old;
	pos_orig = node.pos_orig;
	pos = node.pos;
	force = node.force;
	force_positive = node.force_positive;
	internal_force = node.internal_force;
	internal_force_old = node.internal_force_old;
	external_force = node.external_force;
	external_force_old = node.external_force_old;
	hourglass_force = node.hourglass_force;
	hourglass_force_old = node.hourglass_force_old;
	contact_force = node.contact_force;
	contact_force_old = node.contact_force_old;
	force_positive = node.force_positive;
	lastload = node.lastload;
	inertia = node.inertia;
	temp_dpl = node.temp_dpl;
	mass = node.mass;
	return *this;
}


void Node::addContactForce(const Array1D<double> &param){
	//contact_force[0] = contact_force[0]+param[0];	
	contact_force[0] += param[0];	
	contact_force[1] += param[1];
	contact_force[2] += param[2];
	force[0] += param[0];
	force[1] += param[1];
	force[2] += param[2];
	if (param[0]>0)
	{
		force_positive[0] += param[0];
	}
	if (param[1] > 0)
	{
		force_positive[1] += param[1];
	}
	if (param[2] > 0)
	{
		force_positive[2] += param[2];
	}
}
void Node::addExternalForce(const Array1D<double> &param){
	external_force[0] += param[0];
	external_force[1] += param[1];
	external_force[2] += param[2];
	force[0] += param[0];
	force[1] += param[1];
	force[2] += param[2];
	if (param[0] > 0)
	{
		force_positive[0] += param[0];
	}
	if (param[1] > 0)
	{
		force_positive[1] += param[1];
	}
	if (param[2] > 0)
	{
		force_positive[2] += param[2];
	}
}
void Node::addInternalForce(const Array1D<double> &param){
	internal_force[0] += param[0];
	internal_force[1] += param[1];
	internal_force[2] += param[2];
	force[0] += param[0];
	force[1] += param[1];
	force[2] += param[2];
	if (param[0] > 0)
	{
		force_positive[0] += param[0];
	}
	if (param[1] > 0)
	{
		force_positive[1] += param[1];
	}
	if (param[2] > 0)
	{
		force_positive[2] += param[2];
	}
}
void Node::addHourglassForce(const Array1D<double> &param){
	hourglass_force[0] += param[0];
	hourglass_force[1] += param[1];
	hourglass_force[2] += param[2];
	force[0] += param[0];
	force[1] += param[1];
	force[2] += param[2];
	if (param[0] > 0)
	{
		force_positive[0] += param[0];
	}
	if (param[1] > 0)
	{
		force_positive[1] += param[1];
	}
	if (param[2] > 0)
	{
		force_positive[2] += param[2];
	}
}

void Node::addExternalMoment(const Array1D<double> &param){
	external_force[3] += param[0];
	external_force[4] += param[1];
	external_force[5] += param[2];
	force[3] += param[0];
	force[4] += param[1];
	force[5] += param[2];
	if (param[0] > 0)
	{
		force_positive[3] += param[0];
	}
	if (param[1] > 0)
	{
		force_positive[4] += param[1];
	}
	if (param[2] > 0)
	{
		force_positive[5] += param[2];
	}
}
void Node::addInternalMoment(const Array1D<double> &param){
	internal_force[3] += param[0];
	internal_force[4] += param[1];
	internal_force[5] += param[2];
	force[3] += param[0];
	force[4] += param[1];
	force[5] += param[2];
	if (param[0] > 0)
	{
		force_positive[3] += param[0];
	}
	if (param[1] > 0)
	{
		force_positive[4] += param[1];
	}
	if (param[2] > 0)
	{
		force_positive[5] += param[2];
	}
}
void Node::addHourglassMoment(const Array1D<double> &param){
	hourglass_force[3] += param[0];
	hourglass_force[4] += param[1];
	hourglass_force[5] += param[2];
	force[3] += param[0];
	force[4] += param[1];
	force[5] += param[2];
	if (param[0] > 0)
	{
		force_positive[3] += param[0];
	}
	if (param[1] > 0)
	{
		force_positive[4] += param[1];
	}
	if (param[2] > 0)
	{
		force_positive[5] += param[2];
	}
}

void Node::clearNodalForces(){
	for (int i = 0; i < 6;i++)
	{
		internal_force[i] = 0;
		external_force[i] = 0;
		contact_force[i] = 0;
		hourglass_force[i] = 0;
		force_positive[i] = 0;
		force[i] = 0;
	}
}
void Node::determineInertiaMatrix(){
	if ((inertia[0][0] == 0 && inertia[1][1] == 0 && inertia[2][2] == 0))
	{
		inertia[0][0] = 0.000000000001;
		inertia[1][1] = 0.000000000001;
		inertia[2][2] = 0.000000000001;
	}
	//calculate the innerted inertia matrix
	/************************************************************************/
	/* 
	*/
	/************************************************************************/
	LU<double> inverse_inertia(inertia);
	Array2D<double> E(3, 3, 0.0);
	for (int i = 0; i < 3;i++)
	{
		E[i][i] = 1.0;
	}
	inv_inertia = inverse_inertia.solve(E);

}


void Node::setInitialConditions(vector<Node> &nodeList){
	//set initial Boundary Condition like acc and vel in the start time;
	dpl_old = dpl;
	if (mass==0.0)
	{
		mass = 1e-15;
	}

	if (BdCondition_is_set)
	{
		this->applyAccelerationBCondition(0);
		this->applyVelocityBCondition(0);
		master_node = NULL;
	}else if (rigidCondition_is_set)
	{
		/************************************************************************/
		/* 
		1.对于受刚性约束的节点，加速度是恒定为0的，速度则与主节点息息相关，而主节点的质量转动惯量则
		完全由从节点凝聚而来，因此可以说两者相互耦合，
		2.另外，在本函数中，如果为刚性约束，则会在这里实例化master_node，因此可以供给后面施加速度边界条件
		直接使用
		3.最后这里巧妙的解决了 Rigid 约束与Node冲突的问题，并能够正确的实现应有的功能
		*/
		/************************************************************************/
		master_node = &nodeList[number];
//		if (number==rigidConstraint.master_node_number)
//		{
		double mass_temp = master_node->mass;
		double vx = 0.0;
		double vy = 0.0;
		double vz = 0.0;
		//计算质量中心，质心
		Array1D<double> pos_temp(6, 0.0);
		Array2D<double> inertia_temp(3, 3, 0.0);
		pos_temp = master_node->pos * master_node->mass;
		
		for (auto node : rigidConstraint.nodes)
		{
			pos_temp += node.pos * node.mass;
			mass_temp += node.mass;
		}
		pos_temp /= mass_temp;//质心

		//通过所有从节点，计算主节点的质量和转动惯量

		for (auto node : rigidConstraint.nodes)
		{
			inertia_temp = node.inertia;
			master_node->addMass(node.mass);
			vx = pos[0] - pos_temp[0];
			vy = pos[1] - pos_temp[1];
			vz = pos[2] - pos_temp[2];
			inertia_temp[0][0] = mass*(vy*vy + vz*vz);//Ixx
			inertia_temp[1][1] = mass*(vx*vx + vz*vz);//Iyy
			inertia_temp[2][2] = mass*(vx*vx + vy*vy);//Izz
			inertia_temp[0][1] = mass*(vx + vy);//Ixy
			inertia_temp[1][0] = mass*(vx + vy);//IYX
			inertia_temp[1][2] = mass*(vz + vy);//Iyz
			inertia_temp[2][1] = mass*(vz + vy);//Izy
			inertia_temp[2][0] = mass*(vx + vz);//Ixz
			inertia_temp[0][2] = mass*(vx + vz);//Izx
			master_node->addInertia(inertia);
		}

		//如果需要，更新主节点的位置到质心位置
		if (rigidConstraint.master_node_update)
		{
			//pos = pos_temp;
			master_node->pos_orig = pos_temp;
		}
		else{
			vx = pos[0] - pos_temp[0];
			vy = pos[1] - pos_temp[1];
			vz = pos[2] - pos_temp[2];
			inertia[0][0] = mass*(vy*vy + vz*vz);//Ixx
			inertia[1][1] = mass*(vx*vx + vz*vz);//Iyy
			inertia[2][2] = mass*(vx*vx + vy*vy);//Izz
			inertia[0][1] = mass*(vx + vy);//Ixy
			inertia[1][0] = mass*(vx + vy);//IYX
			inertia[1][2] = mass*(vz + vy);//Iyz
			inertia[2][1] = mass*(vz + vy);//Izy
			inertia[2][0] = mass*(vx + vz);//Ixz
			inertia[0][2] = mass*(vx + vz);//Izx
			master_node->addInertia(inertia);
		}

//		}
	}
}


void Node::calculateNewPosition(double timestep, double currtime){
	int i;
		
	//save old force information
	dpl_old = dpl;
	external_force_old = external_force;
	internal_force_old = internal_force;
	hourglass_force_old = hourglass_force;
	contact_force_old = contact_force;

	if (load_is_set)
	{
		//加上外部力，这些力是边界条件中的力边界输入的，外力的作用在动力学计算中，不但有形变作用还有加速度速度位移作用 
		lastload = load.getLoad(currtime);
		force += lastload;
	}



	//计算加速度，还是牛顿定律，acc前三个放线加速度，后三个为角速度
	acc[0] = force[0] / mass;
	acc[1] = force[1] / mass;
	acc[2] = force[2] / mass;
	//计算角速度，这里计算的是相对于全局坐标系的，并没有用欧拉角定律
	double temp3 = inertia[0][0] * vel[3] + inertia[0][1] * vel[4] + inertia[0][2] * vel[5];
	double temp4 = inertia[1][0] * vel[3] + inertia[1][1] * vel[4] + inertia[1][2] * vel[5];
	double temp5 = inertia[2][0] * vel[3] + inertia[2][1] * vel[4] + inertia[2][2] * vel[5];

	acc[3] = vel[4] * temp5 - vel[5] * temp4;
	acc[4] = vel[5] * temp3 - vel[3] * temp5;
	acc[5] = vel[3] * temp4 - vel[4] * temp3;

	temp3 = force[3] - acc[3];
	temp4 = force[4] - acc[4];
	temp5 = force[5] - acc[5];

	acc[3] = inv_inertia[0][0] * temp3 + inv_inertia[0][1] * temp4 + inv_inertia[0][2] * temp5;
	acc[4] = inv_inertia[1][0] * temp3 + inv_inertia[1][1] * temp4 + inv_inertia[1][2] * temp5;
	acc[5] = inv_inertia[2][0] * temp3 + inv_inertia[2][1] * temp4 + inv_inertia[2][2] * temp5;

	//如果读入了外荷载（加速度型的），加上去；不过load一般加的都是力,只有个重力这个奇葩是以加速度形式加上的外载
	if (load_is_set)
	{
		acc += load.getAcc(currtime);
	}

	//如果读入了外部加速度方面的边界约束条件，也加上去
	if (constraint_is_set)
	{
		if (constraint.type=="RIGID_BODY")
		{
			this->applyAccelerationRigidCondition(currtime);
			this->applyVelocityRigidCondition(currtime);
//			rigidConstraint.registerNode(*this);
		}
		else{
			this->applyAccelerationBCondition(currtime);//已经施加了速度边界，就会冲掉前面通过结点力计算的加速度===》意味着，弹性变形的反力不会对整体的刚体运动产生影响，原来如此
			this->applyVelocityBCondition(currtime);//有速度边界的话，把边界的速度换上去，没边界的话，不变	
		}
	}

	//更新步长update timestep
	halfstep = 0.5*(oldstep + timestep);

	//remember timestep
	oldstep = timestep;

	//update velocities;更新速度
	for (int i = 0; i < 6;i++)
	{//对于有速度边界的，acc会被冲掉，是的这里的速度不发生变化，保持边界约束
		vel[i] += acc[i]*halfstep;//	vel += tempAcc;直接=，会出问题，TNT的bug
	}
 	for (int i = 0; i < 6;i++)
	{
		dpl[i] += vel[i] * oldstep;//dpl += vel*oldstep;		
	}
	//dpl += vel*oldstep;
	//update position，利用简单的高中知识就可以：X = x0 + V*t + 1\2 * a * t^2
	//dx_n_1 = dx_n + (vel_n_-0.5)*dt + acc_n*dt*dt
	//dpl += vel*timestep;竟然无法读取右边vel*timestep的内存，深入到+=这个运算操作，发现没有进行赋值过程，直接返回空值


	/*
	//dpl += acc*timestep*(timestep / 2.0);注意这里用的是增量模式，不用除以2
	dpl += acc*timestep*timestep;	
	//update velocity;速度更新
	vel = (dpl - dpl_old) / timestep;	
	*/
	//update position,位置更新
	for (int i = 0; i < 6;i++)
	{
		this->pos[i] = pos_orig[i] + dpl[i];
	}
//	pos = pos_orig + dpl;

	//为沙漏模式计算内能及沙漏能
	//还得更新下速度
	if (constraint_is_set)
	{
		if (constraint.type == "RIGID_BODY")
		{
			this->applyVelocityRigidCondition(currtime);
		}
		else{
			this->applyVelocityBCondition(currtime);//有速度边界的话，把边界的速度换上去，没边界的话，不变	
		}
	}

	Array1D<double> tempEnergy(6,0.0);
	tempEnergy = vel*(external_force_old + external_force);
	external_energy += 0.5*timestep*tempEnergy.sum();//sum是对各个分量的直接求和

	tempEnergy = vel*(internal_force_old + internal_force);
	internal_energy -= 0.5 * timestep*tempEnergy.sum();

	tempEnergy = vel*(contact_force_old + contact_force);
	contact_energy += 0.5*timestep*tempEnergy.sum();

	tempEnergy = vel*(hourglass_force_old + hourglass_force);
	hourglass_energy -= 0.5*timestep*tempEnergy.sum();
}

//就应该是这样组织程序的，BC只是一个中间信息，只是为了给constraintList存值，供node辨别使用，而真正的使用权在于node；问题来了，如果是假的是刚性边界条件，方法就得重写了
void Node::applyAccelerationBCondition(double currtime){
	double x, y, z;
	if (constraint.axis_is_set)
	{//平动部分, 根据约束值，判断是否施加加速度
		//for X-direction
		if (constraint.x_vel_is_on(currtime))
		{
			x = 0;
		}
		else if (constraint.x_acc_is_on(currtime))
		{
			x = constraint.x_acc.value(currtime);//x y z 方向的加速度，速度，在parse文件时就加入了，如：Az = [0,5] Ay = [0,0] Ax = [0,0]
		}
		else{//acc[0]-x direction; acc[1]-y direction; acc[2]-z direction
			// x= 加速度向量(3*1) 乘以 方向轴矩阵(3*3)中的第一列
			x = acc[0] * constraint.axis[0][0] + acc[1] * constraint.axis[1][0] + acc[2] * constraint.axis[2][0];
		}

		//for Y-direction
		if (constraint.y_vel_is_on(currtime))
		{
			y = 0;//若此刻速度边界有值，就不加加速度
		}
		else if (constraint.y_acc_is_on(currtime))
		{
			y = constraint.y_acc.value(currtime);//若加速度边界有值，取出此刻的线性插值加速度
		}
		else{// y= 加速度向量(3*1) 乘以 方向轴矩阵(3*3)中的第2列
			y = acc[0] * constraint.axis[0][1] + acc[1] * constraint.axis[1][1] + acc[2] * constraint.axis[2][1];
		}
		//for Z-direction
		if (constraint.z_vel_is_on(currtime))
		{
			z = 0;
		}
		else if (constraint.z_acc_is_on(currtime))
		{
			z = constraint.z_acc.value(currtime);
		}
		else{// z= 加速度向量(3*1) 乘以 方向轴矩阵(3*3)中的第3列
			z = acc[0] * constraint.axis[0][2] + acc[1] * constraint.axis[1][2] + acc[2] * constraint.axis[2][2];
		}
		//(x, y, z)*AXIS^T
		acc[0] = x*constraint.axis[0][0] + y*constraint.axis[0][1] + z*constraint.axis[0][2];
		acc[1] = x*constraint.axis[1][0] + y*constraint.axis[1][1] + z*constraint.axis[1][2];
		acc[2] = x*constraint.axis[2][0] + y*constraint.axis[2][1] + z*constraint.axis[2][2];
		//整个过程等价于(ax, ay, az)*AXIS*(ax, ay, az)^T
		//旋转部分
		if (constraint.x_rot_vel_is_on(currtime))
		{
			x = 0;
		}
		else if (constraint.x_rot_acc_is_on(currtime))
		{
			x = constraint.x_rot_acc.value(currtime);
		}
		else{//acc[0]-x direction; acc[1]-y direction; acc[2]-z direction
			// x= 加速度向量(3*1) 乘以 方向轴矩阵(3*3)中的第一列
			x = acc[3] * constraint.axis[0][0] + acc[4] * constraint.axis[1][0] + acc[5] * constraint.axis[2][0];
		}
		//for Y-direction
		if (constraint.y_rot_vel_is_on(currtime))
		{
			y = 0;
		}
		else if (constraint.y_rot_acc_is_on(currtime))
		{
			y = constraint.y_rot_acc.value(currtime);
		}
		else{// y= 加速度向量(3*1) 乘以 方向轴矩阵(3*3)中的第2列
			y = acc[3] * constraint.axis[0][1] + acc[4] * constraint.axis[1][1] + acc[5] * constraint.axis[2][1];
		}
		//for Z-direction
		if (constraint.z_vel_is_on(currtime))
		{
			z = 0;
		}
		else if (constraint.z_rot_acc_is_on(currtime))
		{
			z = constraint.z_rot_acc.value(currtime);
		}
		else{// z= 加速度向量(3*1) 乘以 方向轴矩阵(3*3)中的第3列
			z = acc[3] * constraint.axis[0][2] + acc[4] * constraint.axis[1][2] + acc[5] * constraint.axis[2][2];
		}

		acc[3] = x*constraint.axis[0][0] + y*constraint.axis[0][1] + z*constraint.axis[0][2];
		acc[4] = x*constraint.axis[1][0] + y*constraint.axis[1][1] + z*constraint.axis[1][2];
		acc[5] = x*constraint.axis[2][0] + y*constraint.axis[2][1] + z*constraint.axis[2][2];
	}
	else{//局部坐标系constraint.axis未定义时（一般不定义）
		//translate part
		if (constraint.x_vel_is_on(currtime))
		{
			acc[0] = 0;
		}
		else if (constraint.x_acc_is_on(currtime))//可见，如果施加速度边界，会冲掉前面通过应力计算节点反力得到的加速度
		{
			acc[0] = constraint.x_acc.value(currtime);
		}

		if (constraint.y_vel_is_on(currtime))
		{
			acc[1] = 0;
		}
		else if (constraint.y_acc_is_on(currtime))
		{
			acc[1] = constraint.y_acc.value(currtime);
		}

		if (constraint.z_vel_is_on(currtime))
		{
			acc[2] = 0;
		}
		else if (constraint.z_acc_is_on(currtime))
		{
			acc[2] = constraint.z_acc.value(currtime);
		}

		//rot part
		if (constraint.x_rot_vel_is_on(currtime))
		{
			acc[3] = 0;
		}
		else if (constraint.x_rot_acc_is_on(currtime))
		{
			acc[3] = constraint.x_rot_acc.value(currtime);
		}

		if (constraint.y_rot_vel_is_on(currtime))
		{
			acc[4] = 0;
		}
		else if (constraint.y_rot_acc_is_on(currtime))
		{
			acc[4] = constraint.y_rot_acc.value(currtime);
		}

		if (constraint.z_rot_vel_is_on(currtime))
		{
			acc[5] = 0;
		}
		else if (constraint.z_rot_acc_is_on(currtime))
		{
			acc[5] = constraint.z_rot_acc.value(currtime);
		}
	}
	cout << "acc bc has been added" << endl;
}

void Node::applyAccelerationRigidCondition(double currtime){
	acc = 0.0;
}

void Node::applyVelocityBCondition(double currtime){
	double x = 0.0, y = 0.0, z = 0.0;
	if (constraint.axis_is_set)
	{
		//平动部分, 根据约束值，判断是否施加加速度
		if (constraint.x_vel_is_on(currtime))
		{//此时正施加着速度约束
			x = constraint.x_vel.value(currtime);
		}
		else
		{//(vx, vy, vz)*AXIS  (axis第一列)以次后推
			x = vel[0] * constraint.axis[0][0] + vel[1] * constraint.axis[1][0] + vel[2] * constraint.axis[2][0];
		}

		if (constraint.y_vel_is_on(currtime))
		{
			y = constraint.y_vel.value(currtime);
		}
		else
		{
			y = vel[0] * constraint.axis[0][1] + vel[1] * constraint.axis[1][1] + vel[2] * constraint.axis[2][1];
		}
		if (constraint.z_vel_is_on(currtime))
		{
			z = constraint.z_vel.value(currtime);
		}
		else
		{
			z = vel[0] * constraint.axis[0][2] + vel[1] * constraint.axis[1][2] + vel[2] * constraint.axis[2][2];
		}

		// vel = AXIS*(x,y,z)^T
		vel[0] = x*constraint.axis[0][0] + y*constraint.axis[0][1] + z*constraint.axis[0][2];
		vel[1] = x*constraint.axis[1][0] + y*constraint.axis[1][1] + z*constraint.axis[1][2];
		vel[2] = x*constraint.axis[2][0] + y*constraint.axis[2][1] + z*constraint.axis[2][2];



		//旋转部分
		//x-direction
		if (constraint.x_rot_vel_is_on(currtime))
		{//此时正施加着速度约束
			x = constraint.x_rot_vel.value(currtime);
		}
		else
		{
			x = vel[3] * constraint.axis[0][0] + vel[3] * constraint.axis[1][0] + vel[3] * constraint.axis[2][0];
		}
		//y-direction
		if (constraint.y_rot_vel_is_on(currtime))
		{
			y = constraint.y_rot_vel.value(currtime);
		}
		else
		{
			y = vel[4] * constraint.axis[0][1] + vel[4] * constraint.axis[1][1] + vel[4] * constraint.axis[2][1];
		}
		//z-direction
		if (constraint.z_rot_vel_is_on(currtime))
		{
			z = constraint.z_rot_vel.value(currtime);
		}
		else
		{
			z = vel[5] * constraint.axis[0][2] + vel[5] * constraint.axis[1][2] + vel[5] * constraint.axis[2][2];
		}

		// vel = AXIS*(x,y,z)^T
		vel[3] = x*constraint.axis[0][0] + y*constraint.axis[0][1] + z*constraint.axis[0][2];
		vel[4] = x*constraint.axis[1][0] + y*constraint.axis[1][1] + z*constraint.axis[1][2];
		vel[5] = x*constraint.axis[2][0] + y*constraint.axis[2][1] + z*constraint.axis[2][2];
	}
	else{

		if (constraint.x_vel_is_on(currtime))
		{
			vel[0] = constraint.x_vel.value(currtime);//if is a constant return stepValue[0]
		}
		if (constraint.y_vel_is_on(currtime))
		{
			vel[1] = constraint.y_vel.value(currtime);
		}
		if (constraint.z_vel_is_on(currtime))
		{
			vel[2] = constraint.z_vel.value(currtime);
		}

		//rot part
		if (constraint.x_rot_vel_is_on(currtime))
		{
			vel[3] = constraint.x_rot_vel.value(currtime);
		}

		if (constraint.y_rot_vel_is_on(currtime))
		{
			vel[4] = constraint.y_rot_vel.value(currtime);
		}
		if (constraint.z_rot_vel_is_on(currtime))
		{
			vel[5] = constraint.z_rot_vel.value(currtime);
		}
	}


}

void Node::applyVelocityRigidCondition(double currtimem){
	/************************************************************************/
	/* master_node已经是Node的成员变量了，可以在每个Node中尽情使用                                                  */
	/************************************************************************/
	rigidConstraint.moment_arm = master_node->pos - pos;
	Array1D<double> moment_arm(6, 0.0);
	moment_arm = master_node->pos - pos;
	vel[0] = master_node->vel[0] + moment_arm[1] * master_node->vel[5] - moment_arm[2] * master_node->vel[4];
	vel[1] = master_node->vel[1] + moment_arm[2] * master_node->vel[3] - moment_arm[0] * master_node->vel[5];
	vel[2] = master_node->vel[2] + moment_arm[0] * master_node->vel[4] - moment_arm[1] * master_node->vel[3];
	
	vel.subarray(3, 5) = master_node->vel.subarray(3, 5);

}

void Node::checkNeighbours(){
	Node *temp_neighbour;
	bool finished = false;
	while (finished)
	{
		finished = true;
		if (left_neighbour!=NULL)
		{
			if (left_neighbour->pos[0]>this->pos[0])
			{
				//左结点的X应该比本this节点小才行，否则就得换位置
				temp_neighbour = left_neighbour->left_neighbour;
				//左结点的右节点就是this节点，但如果让它指向本节点的右节点，就是要把this隔开后，提到左结点之前，形成this-left-right (之前是left-this-right)
				left_neighbour->right_neighbour = this->right_neighbour;
				if (right_neighbour!=NULL)
				{
					right_neighbour->left_neighbour = this->left_neighbour;
				}
				//完成转变，结构变为：this-left-right
				left_neighbour->left_neighbour = this;
				right_neighbour = left_neighbour;

				left_neighbour = temp_neighbour;//保证this节点的左结点是之前左结点的左结点
				if (left_neighbour!=NULL)
				{
					left_neighbour->right_neighbour = this;
				}
				if (DEBUG)
				{
					cout<<"节点 "<<number << " 从left-this-right变为this-left-right \n";
				}
				finished = false;//换位未完成，知道左结点的x比this的小
			}
		}else if (right_neighbour!=NULL)
		{
			if (right_neighbour->pos[0]>this->pos[0])
			{
				temp_neighbour = right_neighbour->right_neighbour;
				right_neighbour->left_neighbour = this->left_neighbour;
				if (left_neighbour!=NULL)
				{
					left_neighbour->right_neighbour = this->right_neighbour;
				}
				right_neighbour->right_neighbour = this;
				left_neighbour = right_neighbour;
				left_neighbour = temp_neighbour;
				if (right_neighbour!=NULL)
				{
					right_neighbour->right_neighbour = this;
				}
				finished = false;
			}
		}
	}
}












#endif