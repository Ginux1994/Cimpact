#ifndef LOAD_H
#define LOAD_H

#include "tnt_array2d.h"
#include "Variable.h"
#include "tnt.h"
#include <boost/tokenizer.hpp>


using namespace TNT;
class Load
{
public:
	Load();
	std::string name;//if no use std::, will be error:缺少类型说明符 - 假定为 int。注意:  C++ 不支持默认 int	9	1...

	void parse_Fembic(string& thisLine);
	Array1D<double>& getLoad(double currtime);
	Array1D<double>& getAcc(double currtime);

	Variable x_moment, y_moment, z_moment;
	Variable x_force, y_force, z_force;
	Variable x_acc, y_acc, z_acc;
	Variable x_acc_rot, y_acc_rot, z_acc_rot;
	Variable pressure;

	bool x_force_is_set, y_force_is_set, z_force_is_set;
	bool x_moment_is_set, y_moment_is_set, z_moment_is_set;
	bool x_acc_is_set, y_acc_is_set, z_acc_is_set;
	bool x_acc_rot_is_set, y_acc_rot_is_set, z_acc_rot_is_set;
	bool pressure_is_set;

	Array1D<double> load, acc;
protected:
private:


};


Load::Load(){
	load = Array1D<double>(6,0.0);
	acc = Array1D<double>(6,0.0);


}


void Load::parse_Fembic(string& thisLine){

	using namespace boost;
	//开始对thisLine进行字符扑街
	char_separator<char> sepa("= \t[],");//必须要用制表符作为分词符之一，否则会漏掉很多
	typedef tokenizer<char_separator<char>> Tokenizer;
	Tokenizer tok(thisLine, sepa);
	Tokenizer::iterator position = tok.begin();//first string in thisLine

	//	if ((*position) == "#") continue;//ignore annonatations
	
	string tempString = *position;
	transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);//change into capital model
	name = tempString;
	position++;
	transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);//change into capital model

	//begin inner loop to parse the line
	//for (position; position != tok.end(); position++){
	while (position != tok.end()){

		tempString = *position;
		transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);

		map<string, int> mapStr = { { "FX", 01 }, { "FY", 02 }, { "FZ", 03 }, { "MX", 11 }, { "MY", 12 }, { "MZ", 13 },
		{ "AX", 21 }, { "AY", 22 }, { "AZ", 23 }, { "ARX", 31 }, { "ARY", 32 }, { "ARZ", 33 }, { "P", 41 } };

		switch (mapStr[tempString])
		{
		case 01:
		{
			vector<string> vecFX;
			position++;
			for (position; position != tok.end(); position++)
			{

				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
				if ((tempString == "FX") || (tempString == "FY") || (tempString == "FZ") ||
					(tempString == "ARX") || (tempString == "ARY") || (tempString == "ARZ") ||
					(tempString == "AX") || (tempString == "AY") || (tempString == "AZ") ||
					(tempString == "MX") || (tempString == "MY") || (tempString == "MZ") || (tempString == "P")) {

					break;
				}
				vecFX.push_back(tempString);
			}
			if (vecFX.size() == 1)
			{
				double fuckD = stod(vecFX[0]);
				x_force = Variable(fuckD);
			}
			else{
				x_force = Variable(vecFX);
			}
			x_force_is_set = true;
			x_force.printV();
			continue;
		}
		case 02:
		{
			vector<string> vecFy;
			for (++position; position != tok.end(); position++)//必须在第一个position上加个++，才能正确判断，并且不把VY放入vecVY
			{

				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
				if ((tempString == "FX") || (tempString == "FY") || (tempString == "FZ") ||
					(tempString == "ARX") || (tempString == "ARY") || (tempString == "ARZ") ||
					(tempString == "AX") || (tempString == "AY") || (tempString == "AZ") ||
					(tempString == "MX") || (tempString == "MY") || (tempString == "MZ") || (tempString == "P")) {
					cout << " i should break";
					break;
				}
				vecFy.push_back(tempString);
			}
			if (vecFy.size() == 1)
			{
				double fuckD = stod(vecFy[0]);
				y_force = Variable(fuckD);
			}
			else{
				y_force = Variable(vecFy);
			}
			y_force_is_set = true;
			y_force.printV();
			continue;
		}
		case  03:
		{
			vector<string> vecFz;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "FX") || (tempString == "FY") ||
					(tempString == "ARX") || (tempString == "ARY") || (tempString == "ARZ") ||
					(tempString == "AX") || (tempString == "AY") || (tempString == "AZ") ||
					(tempString == "MX") || (tempString == "MY") || (tempString == "MZ") || (*position == "P"))break;
				vecFz.push_back(tempString);
			}
			if (vecFz.size() == 1)
			{
				double fuckD = stod(vecFz[0]);
				z_force = Variable(fuckD);
			}
			else{
				z_force = Variable(vecFz);
			}
			z_force_is_set = true;
			z_force.printV();
			continue;
		}
		case 11:
		{
			vector<string> vecMx;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "FX") || (tempString == "FY") || (tempString == "FZ") ||
					(tempString == "ARX") || (tempString == "ARY") || (tempString == "ARZ") ||
					(tempString == "AX") || (tempString == "AY") || (tempString == "AZ") ||
					(tempString == "MY") || (tempString == "MZ") || (tempString == "P"))break;
				vecMx.push_back(tempString);
			}
			if (vecMx.size() == 1)
			{
				double fuckD = stod(vecMx[0]);
				x_moment = Variable(fuckD);
			}
			else{
				x_moment = Variable(vecMx);
			}
			x_moment_is_set = true;
			x_moment.printV();
			continue;
		}
		case 12:
		{
			vector<string> vecMy;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "FX") || (tempString == "FY") || (tempString == "FZ") ||
					(tempString == "ARX") || (tempString == "ARY") || (tempString == "ARZ") ||
					(tempString == "AX") || (tempString == "AY") || (tempString == "AZ") ||
					(tempString == "MX") || (tempString == "MZ") || (tempString == "P"))break;
				vecMy.push_back(tempString);
			}
			if (vecMy.size() == 1)
			{
				double fuckD = stod(vecMy[0]);
				y_moment = Variable(fuckD);
			}
			else{
				y_moment = Variable(vecMy);
			}
			y_moment_is_set = true;
			y_moment.printV();
			continue;
		}
		case 13:
		{
			vector<string> vecMz;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "FX") || (tempString == "FY") || (tempString == "FZ") ||
					(tempString == "ARX") || (tempString == "ARY") || (tempString == "ARZ") ||
					(tempString == "AX") || (tempString == "AY") || (tempString == "AZ") ||
					(tempString == "MX") || (tempString == "MY") || (tempString == "P"))break;
				vecMz.push_back(tempString);
			}
			if (vecMz.size() == 1)
			{
				double fuckD = stod(vecMz[0]);
				z_moment = Variable(fuckD);
			}
			else{
				z_moment = Variable(vecMz);
			}
			z_moment_is_set = true;
			z_moment.printV();
			continue;
		}
		case 21:
		{
			vector<string> vecARx;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "FX") || (tempString == "FY") || (tempString == "FZ") ||
					(tempString == "ARY") || (tempString == "ARZ") ||
					(tempString == "AX") || (tempString == "AY") || (tempString == "AZ") ||
					(tempString == "MX") || (tempString == "MY") || (tempString == "MZ") || (tempString == "P"))break;
				vecARx.push_back(tempString);
			}
			if (vecARx.size() == 1)
			{
				double fuckD = stod(vecARx[0]);
				x_acc_rot = Variable(fuckD);
			}
			else{
				x_acc_rot = Variable(vecARx);
			}
			x_acc_rot_is_set = true;
			x_acc_rot.printV();
			continue;
		}
		case 22:
		{
			vector<string> vecARy;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "FX") || (tempString == "FY") || (tempString == "FZ") ||
					(tempString == "ARX") || (tempString == "ARZ") ||
					(tempString == "AX") || (tempString == "AY") || (tempString == "AZ") ||
					(tempString == "MX") || (tempString == "MY") || (tempString == "MZ") || (tempString == "P"))break;
				vecARy.push_back(tempString);
			}
			if (vecARy.size() == 1)
			{
				double fuckD = stod(vecARy[0]);
				y_acc_rot = Variable(fuckD);
			}
			else{
				y_acc_rot = Variable(vecARy);
			}
			y_acc_rot_is_set = true;
			continue;
		}
		case 23:
		{
			vector<string> vecARz;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "FX") || (tempString == "FY") || (tempString == "FZ") ||
					(tempString == "ARX") || (tempString == "ARY") ||
					(tempString == "AX") || (tempString == "AY") || (tempString == "AZ") ||
					(tempString == "MX") || (tempString == "MY") || (tempString == "MZ") || (tempString == "P"))break;
				vecARz.push_back(tempString);
			}
			if (vecARz.size() == 1)
			{
				double fuckD = stod(vecARz[0]);
				z_acc_rot = Variable(fuckD);
			}
			else{
				z_acc_rot = Variable(vecARz);
			}
			z_acc_rot_is_set = true;
			continue;
		}
		case 24:
		{
			vector<string> vecP;
			for (++position; position != tok.end(); position++)
			{
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);


				if ((tempString == "FX") || (tempString == "FY") || (tempString == "FZ") ||
					(tempString == "ARX") || (tempString == "ARY") || (tempString == "ARZ") ||
					(tempString == "AX") || (tempString == "AY") || (tempString == "AZ") ||
					(tempString == "MX") || (tempString == "MY") || (tempString == "MZ"))break;
				vecP.push_back(tempString);
			}
			if (vecP.size() == 1)
			{
				double fuckD = stod(vecP[0]);
				pressure = Variable(fuckD);
			}
			else{
				pressure = Variable(vecP);
			}
			pressure_is_set = true;
			pressure.printV();
			continue;
		}
			break;
		}//switch case
		if (position == tok.end()) break;
	}//while(position!=end)
}

Array1D<double>& Load::getLoad(double currtime){

	if (x_force_is_set)
	{
		load[0] = x_force.status(currtime) ? x_force.value(currtime) : 0.0;
	}
	if (y_force_is_set)
	{
		load[1] = y_force.status(currtime) ? y_force.value(currtime) : 0.0;
	}
	if (z_force_is_set)
	{
		load[2] = z_force.status(currtime) ? z_force.value(currtime) : 0.0;
	}
	if (x_moment_is_set)
	{
		load[3] = x_moment.status(currtime) ? x_moment.value(currtime) : 0.0;
	}
	if (y_moment_is_set)
	{
		load[4] = y_moment.status(currtime) ? y_moment.value(currtime) : 0.0;

	}
	if (z_moment_is_set)
	{
		load[5] = z_moment.status(currtime) ? z_moment.value(currtime) : 0.0;
	}
	return load;
}
Array1D<double>& Load::getAcc(double currtime){

	if (x_acc_is_set)
	{
		acc[0] = x_acc.status(currtime) ? x_acc.value(currtime) : 0.0;
	}
	if (y_acc_is_set)
	{
		acc[1] = y_acc.status(currtime) ? y_acc.value(currtime) : 0.0;
	}
	if (z_acc_is_set)
	{
		acc[2] = z_acc.status(currtime) ? z_acc.value(currtime) : 0.0;
	}
	if (x_acc_rot_is_set)
	{
		acc[3] = x_acc_rot.status(currtime) ? x_acc_rot.value(currtime) : 0.0;
	}
	if (y_acc_rot_is_set)
	{
		acc[4] = y_acc_rot.status(currtime) ? y_acc_rot.value(currtime) : 0.0;

	}
	if (z_acc_rot_is_set)
	{
		acc[5] = z_acc_rot.status(currtime) ? z_acc_rot.value(currtime) : 0.0;
	}
	return acc;
}



#endif