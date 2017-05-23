#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <string>


class Constraint
{
public:
	Constraint();
	std::string name;
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
Constraint::Constraint(){
	//һֱû��д������췽��������һ��ʹ��constraint�ͻᱬ�����ⲿ���ӳ�����
/************************************************************************/
/* ����	1	error LNK2019: �޷��������ⲿ���� "public: __thiscall Constraint::Constraint(void)" (??0Constraint@@QAE@XZ)���÷����ں��� "public: __thiscall Node::Node(void)" (??0Node@@QAE@XZ) �б�����
	����	2	error LNK1120: 1 ���޷��������ⲿ����	1	1
                                                                 */
/************************************************************************/
}



#endif