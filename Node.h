#ifndef NODE_H
#define NODE_H

#include <iostream>
#include "BdCondition.h"
//#include "BCondition.h"
//���ͷ�ļ�һ����ȥ�ͻ�Ӱ��BCondition�ı���ͨ����ʹ�úܶ������ĺ�����Ա��������Ҳ����˵��������ͷ�ļ������໥����
#include "Load.h"
//#include "Constraint.h"
#include "tnt.h"
#include "tnt_array2d.h"
#include <math.h>
#include "jama_lu.h"



class BdCondition;
class BCondition;

using namespace TNT;
using namespace std;
using namespace JAMA;
class Node
{
public:
	Node();

	//����������
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
	void setInitialConditions();
	void checkNeighbours();
	bool operator<(const Node& nodeToCompare);
	//ʩ�ӱ߽�����������Ӧ����BCondition�еķ�������Ǩ�Ƶ���node��
	void applyAccelerationCondition(double currtime);
	void applyVelocityCondition(double currtime);

	bool isDeactivated(){ return deactivated; }
	Node * Left_neighbour() const { return left_neighbour; }
	void Left_neighbour(Node * val) { left_neighbour = val; }
	Node * Right_neighbour() const { return right_neighbour; }
	void Right_neighbour(Node * val) { right_neighbour = val; }
	void Deactivated(){ left_neighbour->Right_neighbour(right_neighbour); right_neighbour->Left_neighbour(left_neighbour); deactivated = true; }

	double Mass() const { return mass; }//C++���ع�get��set�������Ա���������ĸ��д�ĺ�����ʽ��ʵ�ֵģ�����Ҫget set�ؼ���
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
	Load load;
	BdCondition constraint;
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
	vel = Array1D<double>(6, 0.0);//ǰ����Ϊƽ��������������Ϊת������
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
bool Node::operator <(const Node& nodeToCompare){
	return pos[0] < nodeToCompare.pos[0];
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


void Node::setInitialConditions(){
	//set initial Boundary Condition like acc and vel in the start time;
	dpl_old = dpl;
	if (mass==0.0)
	{
		mass = 1e-15;
	}
	if (constraint_is_set){
		this->applyAccelerationCondition(0);
		this->applyVelocityCondition(0);
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
		//�����ⲿ������Щ���Ǳ߽������е����߽�����ģ������������ڶ���ѧ�����У��������α����û��м��ٶ��ٶ�λ������ 
		lastload = load.getLoad(currtime);
		force += lastload;
	}



	//������ٶȣ�����ţ�ٶ��ɣ�accǰ�������߼��ٶȣ�������Ϊ���ٶ�
	acc[0] = force[0] / mass;
	acc[1] = force[1] / mass;
	acc[2] = force[2] / mass;
	//������ٶȣ����������������ȫ������ϵ�ģ���û����ŷ���Ƕ���
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

	//�������������أ����ٶ��͵ģ�������ȥ������loadһ��ӵĶ�����,ֻ�и���������������Լ��ٶ���ʽ���ϵ�����
	if (load_is_set)
	{
		acc += load.getAcc(currtime);
	}

	//����������ⲿ���ٶȷ���ı߽�Լ��������Ҳ����ȥ
	if (constraint_is_set)
	{
		this->applyAccelerationCondition(currtime);//�Ѿ�ʩ�����ٶȱ߽磬�ͻ���ǰ��ͨ�����������ļ��ٶ�===����ζ�ţ����Ա��εķ������������ĸ����˶�����Ӱ�죬ԭ�����
		this->applyVelocityCondition(currtime);//���ٶȱ߽�Ļ����ѱ߽���ٶȻ���ȥ��û�߽�Ļ�������	
	}

	//���²���update timestep
	halfstep = 0.5*(oldstep + timestep);

	//remember timestep
	oldstep = timestep;

	//update velocities;�����ٶ�
	for (int i = 0; i < 6;i++)
	{//�������ٶȱ߽�ģ�acc�ᱻ������ǵ�������ٶȲ������仯�����ֱ߽�Լ��
		vel[i] += acc[i]*halfstep;//	vel += tempAcc;ֱ��=��������⣬TNT��bug
	}
 	for (int i = 0; i < 6;i++)
	{
		dpl[i] += vel[i] * oldstep;//dpl += vel*oldstep;		
	}
	//dpl += vel*oldstep;
	//update position�����ü򵥵ĸ���֪ʶ�Ϳ��ԣ�X = x0 + V*t + 1\2 * a * t^2
	//dx_n_1 = dx_n + (vel_n_-0.5)*dt + acc_n*dt*dt
	//dpl += vel*timestep;��Ȼ�޷���ȡ�ұ�vel*timestep���ڴ棬���뵽+=����������������û�н��и�ֵ���̣�ֱ�ӷ��ؿ�ֵ


	/*
	//dpl += acc*timestep*(timestep / 2.0);ע�������õ�������ģʽ�����ó���2
	dpl += acc*timestep*timestep;	
	//update velocity;�ٶȸ���
	vel = (dpl - dpl_old) / timestep;	
	*/
	//update position,λ�ø���
	for (int i = 0; i < 6;i++)
	{
		this->pos[i] = pos_orig[i] + dpl[i];
	}
//	pos = pos_orig + dpl;

	//Ϊɳ©ģʽ�������ܼ�ɳ©��
	//���ø������ٶ�
	if (constraint_is_set)
	{
		this->applyVelocityCondition(currtime);
	}

	Array1D<double> tempEnergy(6,0.0);
	tempEnergy = vel*(external_force_old + external_force);
	external_energy += 0.5*timestep*tempEnergy.sum();//sum�ǶԸ���������ֱ�����

	tempEnergy = vel*(internal_force_old + internal_force);
	internal_energy -= 0.5 * timestep*tempEnergy.sum();

	tempEnergy = vel*(contact_force_old + contact_force);
	contact_energy += 0.5*timestep*tempEnergy.sum();

	tempEnergy = vel*(hourglass_force_old + hourglass_force);
	hourglass_energy -= 0.5*timestep*tempEnergy.sum();
}

//��Ӧ����������֯����ģ�BCֻ��һ���м���Ϣ��ֻ��Ϊ�˸�constraintList��ֵ����node���ʹ�ã���������ʹ��Ȩ����node���������ˣ�����Ǽٵ��Ǹ��Ա߽������������͵���д��
void Node::applyAccelerationCondition(double currtime){
	double x, y, z;
	if (constraint.axis_is_set)
	{//ƽ������, ����Լ��ֵ���ж��Ƿ�ʩ�Ӽ��ٶ�
		//for X-direction
		if (constraint.x_vel_is_on(currtime))
		{
			x = 0;
		}
		else if (constraint.x_acc_is_on(currtime))
		{
			x = constraint.x_acc.value(currtime);//x y z ����ļ��ٶȣ��ٶȣ���parse�ļ�ʱ�ͼ����ˣ��磺Az = [0,5] Ay = [0,0] Ax = [0,0]
		}
		else{//acc[0]-x direction; acc[1]-y direction; acc[2]-z direction
			// x= ���ٶ�����(3*1) ���� ���������(3*3)�еĵ�һ��
			x = acc[0] * constraint.axis[0][0] + acc[1] * constraint.axis[1][0] + acc[2] * constraint.axis[2][0];
		}

		//for Y-direction
		if (constraint.y_vel_is_on(currtime))
		{
			y = 0;//���˿��ٶȱ߽���ֵ���Ͳ��Ӽ��ٶ�
		}
		else if (constraint.y_acc_is_on(currtime))
		{
			y = constraint.y_acc.value(currtime);//�����ٶȱ߽���ֵ��ȡ���˿̵����Բ�ֵ���ٶ�
		}
		else{// y= ���ٶ�����(3*1) ���� ���������(3*3)�еĵ�2��
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
		else{// z= ���ٶ�����(3*1) ���� ���������(3*3)�еĵ�3��
			z = acc[0] * constraint.axis[0][2] + acc[1] * constraint.axis[1][2] + acc[2] * constraint.axis[2][2];
		}
		//(x, y, z)*AXIS^T
		acc[0] = x*constraint.axis[0][0] + y*constraint.axis[0][1] + z*constraint.axis[0][2];
		acc[1] = x*constraint.axis[1][0] + y*constraint.axis[1][1] + z*constraint.axis[1][2];
		acc[2] = x*constraint.axis[2][0] + y*constraint.axis[2][1] + z*constraint.axis[2][2];
		//�������̵ȼ���(ax, ay, az)*AXIS*(ax, ay, az)^T
		//��ת����
		if (constraint.x_rot_vel_is_on(currtime))
		{
			x = 0;
		}
		else if (constraint.x_rot_acc_is_on(currtime))
		{
			x = constraint.x_rot_acc.value(currtime);
		}
		else{//acc[0]-x direction; acc[1]-y direction; acc[2]-z direction
			// x= ���ٶ�����(3*1) ���� ���������(3*3)�еĵ�һ��
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
		else{// y= ���ٶ�����(3*1) ���� ���������(3*3)�еĵ�2��
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
		else{// z= ���ٶ�����(3*1) ���� ���������(3*3)�еĵ�3��
			z = acc[3] * constraint.axis[0][2] + acc[4] * constraint.axis[1][2] + acc[5] * constraint.axis[2][2];
		}

		acc[3] = x*constraint.axis[0][0] + y*constraint.axis[0][1] + z*constraint.axis[0][2];
		acc[4] = x*constraint.axis[1][0] + y*constraint.axis[1][1] + z*constraint.axis[1][2];
		acc[5] = x*constraint.axis[2][0] + y*constraint.axis[2][1] + z*constraint.axis[2][2];
	}
	else{//�ֲ�����ϵconstraint.axisδ����ʱ��һ�㲻���壩
		//translate part
		if (constraint.x_vel_is_on(currtime))
		{
			acc[0] = 0;
		}
		else if (constraint.x_acc_is_on(currtime))//�ɼ������ʩ���ٶȱ߽磬����ǰ��ͨ��Ӧ������ڵ㷴���õ��ļ��ٶ�
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

void Node::applyVelocityCondition(double currtime){
	double x = 0.0, y = 0.0, z = 0.0;
	if (constraint.axis_is_set)
	{
		//ƽ������, ����Լ��ֵ���ж��Ƿ�ʩ�Ӽ��ٶ�
		if (constraint.x_vel_is_on(currtime))
		{//��ʱ��ʩ�����ٶ�Լ��
			x = constraint.x_vel.value(currtime);
		}
		else
		{//(vx, vy, vz)*AXIS  (axis��һ��)�Դκ���
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



		//��ת����
		//x-direction
		if (constraint.x_rot_vel_is_on(currtime))
		{//��ʱ��ʩ�����ٶ�Լ��
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
				//�����XӦ�ñȱ�this�ڵ�С���У�����͵û�λ��
				temp_neighbour = left_neighbour->left_neighbour;
				//������ҽڵ����this�ڵ㣬���������ָ�򱾽ڵ���ҽڵ㣬����Ҫ��this�������ᵽ����֮ǰ���γ�this-left-right (֮ǰ��left-this-right)
				left_neighbour->right_neighbour = this->right_neighbour;
				if (right_neighbour!=NULL)
				{
					right_neighbour->left_neighbour = this->left_neighbour;
				}
				//���ת�䣬�ṹ��Ϊ��this-left-right
				left_neighbour->left_neighbour = this;
				right_neighbour = left_neighbour;

				left_neighbour = temp_neighbour;//��֤this�ڵ��������֮ǰ���������
				if (left_neighbour!=NULL)
				{
					left_neighbour->right_neighbour = this;
				}
				if (DEBUG)
				{
					cout<<"�ڵ� "<<number << " ��left-this-right��Ϊthis-left-right \n";
				}
				finished = false;//��λδ��ɣ�֪�������x��this��С
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