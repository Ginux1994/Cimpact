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
	//一直没有写这个构造方法，导致一旦使用constraint就会爆出，外部链接出问题
/************************************************************************/
/* 错误	1	error LNK2019: 无法解析的外部符号 "public: __thiscall Constraint::Constraint(void)" (??0Constraint@@QAE@XZ)，该符号在函数 "public: __thiscall Node::Node(void)" (??0Node@@QAE@XZ) 中被引用
	错误	2	error LNK1120: 1 个无法解析的外部命令	1	1
                                                                 */
/************************************************************************/
}



#endif