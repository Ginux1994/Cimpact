#ifndef VARIABLE_H
#define VARIABLE_H



#include <vector>
#include <string>
#include <iostream>


using namespace std;
class Variable
{
public:
	Variable();
	Variable(vector<string> &vecStr);
	Variable(double arg);
//	Variable(Variable& q, vector<Variable>& a);



	double value(double time);
	double value(double x_value, double y_value);
	double derivate(double time);
	double derivate(double x_value, double y_value);
	bool status(double time);
	bool Is_a_constant() const { return is_a_constant; }
	void Is_a_constant(bool val) { is_a_constant = val; }

	void printV();
protected:
public:
	//public just for easy
	vector<double> stepTime, stepValue;
	vector<bool> on;
	vector<Variable> z;
	Variable *v;

	bool is_a_constant;
	int index=0;
	string token;
	double fuck;
};

Variable::Variable(){

}


Variable::Variable(vector<string> &vecStr):is_a_constant(false){
	//0,5,4,-5,8,5,12,-5,12.1,off,20,off
	//stepTime, stepvalue, stepTime, stepvalue, ...

	int i = -1;
	for (auto iter = vecStr.begin(); iter != vecStr.end(); ++iter)
	{

		//X==stepTime ; stepValue = VALUE;
		stepTime.push_back(stod(*iter));
		iter++;
		i++;

		if (*iter == "OFF")
		{
			stepValue.push_back(stepValue[i - 1]);
			on.push_back(false);
			continue;
		}
		else {
			stepValue.push_back(stod(*iter));
			on.push_back(true);
		}
	}
}
Variable::Variable(double arg):is_a_constant(true){
	is_a_constant = true;
	stepValue.push_back(arg);
}

/************************************************************************/
/* 
Variable::Variable(Variable& q, vector<Variable>& a){
	var = q;
	z = a;
}
*/
/************************************************************************/

void Variable::printV(){
	for (auto vel:stepTime)
	{
		cout <<"time "<< vel << endl;
	}
	for (auto vel:stepValue)
	{
		cout << "value " << vel << endl;
	}

}



double Variable::value(double time){
	if (is_a_constant == true)
	{
		return stepValue[0];
	}
	while ((index<stepTime.size())&&(time>stepTime[index+1]))
	{
		index++;
	}

	while ((time < stepTime[index]) && (index>0)){
		index--;
	}

	if (index==stepTime.size())
	{
		return stepValue[index];
	}
	if (index >stepTime.size())
	{
		cout << "out of bounds";
	}
	double tempValue = (((stepValue[index + 1] - stepValue[index]) * (time - stepTime[index])) / (stepTime[index + 1] - stepTime[index]))
		+ stepValue[index];
	return tempValue;

}

double Variable::value(double x_value, double y_value){

	index = (int)v->value(y_value);
	if (index == z.size())
	{
		return z[index - 1].value(x_value);
	}
	//index>z.size()   runtime erro
	if (index > z.size())
	{
		throw runtime_error("index>z.size() Parameter out of bounds");
	}
	double dz = z[index + 1].derivate(x_value) - z[index].derivate(x_value);
	double dv = v->value(y_value) - (int)v->value(y_value);

	return dz * dv + z[index].derivate(x_value);
}


double Variable::derivate(double time){
	if (is_a_constant == true)
	{
		return stepValue[0];
	}
	while ((index<stepTime.size()) && (time>stepTime[index + 1]))
	{
		index++;
	}

	while ((time < stepTime[index]) && (index>0)){
		index--;
	}

	if (index == stepTime.size())
	{
		return stepValue[index];
	}
	if (index > stepTime.size())
	{
		cout << "out of bounds";
	}
	return (stepValue[index + 1] - stepValue[index]) / (stepTime[index + 1] - stepTime[index]);
}

double Variable::derivate(double x_value, double y_value){
	index = (int)v->value(y_value);

	if (index == z.size())
	{
		return z[index - 1].value(x_value);
	}
	//index>z.size()   runtime erro
	if (index > z.size())
	{
		throw runtime_error("index>z->size() Parameter out of bounds");
	}

	double dz = z[index + 1].derivate(x_value) - z[index].derivate(x_value);
	double dv = v->value(y_value) - (int)v->value(y_value);

	return dz * dv + z[index].derivate(x_value);
}

bool Variable::status(double time){
	if (is_a_constant )
	{
		return true;
	}
	while ((index<stepTime.size()) && (time>stepTime[index + 1]))
	{
		index++;
	}

	while ((time < stepTime[index]) && (index>0)){
		index--;
	}

	if (index > stepTime.size())
	{
		cout << "out of bounds";
	}

	return on[index];
}






#endif