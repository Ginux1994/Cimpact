
#ifndef FEMBIC_H
#define FEMBIC_H






#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

#include <map>
#include <boost/tokenizer.hpp>
#include "Node.h"
#include "Material.h"
#include "Load.h"
#include "Rod_2.h"
#include "Variable.h"
#include "Elastoplastic.h"
#include "Elastic.h"
#include "Controlset.h"

//#include "BdCondition.h"

using namespace std;
using namespace boost;


class Fembic
{
public:

	Fembic();

	Fembic(string &str);


	bool isAKeyWord(const string &param);

	void parseMaterial(vector<Elastic>& materialList);

	int parseNodeVectorAndTable(vector<Node>& nodeList, map<int, Node>& nodeTable, vector<BdCondition>& constraintList, vector<Load>& loadList);

	int parseElementOfRod_2(vector<Elastic>& materialList, vector<Node>& nodeList, vector<Load> loadList, map<int, Node>& nodeTable, vector<Rod_2>& elementList, map<int, Rod_2>& elementTable);

	int parseLoad(vector<Load>& loadList);

	int parseConstraint(vector<BdCondition>& constraintList);


	void parseControlset(Controlset& controlset);





	string filePath;
protected:
private:

};
Fembic::Fembic(){
}

Fembic::Fembic(string &str){
	filePath = str;
}

bool Fembic::isAKeyWord(const string &param){
	string keywords[10] = { "TITLE", "CONTROLS", "ELEMENTS",
		"NODES", "LOADS", "CONSTRAINTS", "MATERIALS", "TRACKERS", "GROUPS",
		"GEOMETRY" };
	for (int i = 0; i < 10; i++){
		if (keywords[i] == param)
			return true;
	}
	return false;
}

void Fembic::parseMaterial(vector<Elastic>& materialList){
	using namespace boost;

	//Elastoplastic material;

	ifstream file;//#include <fstream>
	file.open(filePath);
	if (!file.is_open()){
		cout << "fail to open the file \n";
	}

	int numberOfNodes = 0;
	bool in_block = false;
	string thisLine;
	string current_material_type;

	while (getline(file, thisLine))
	{
		//Material material;
		if (thisLine.empty()) continue;
		//开始对thisLine进行字符扑街
		char_separator<char> sepa("= \t[],");//必须要用制表符作为分词符之一，否则会漏掉很多
		typedef tokenizer<char_separator<char>> Tokenizer;
		Tokenizer tok(thisLine, sepa);
		Tokenizer::iterator position = tok.begin();//first string in thisLine

		if ((*position) == "#") continue;;//ignore annonatations

		string tempString = *position;
		transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);//change into capital model

		if (tempString == "MATERIALS")
		{

			in_block = true;
			position++;

			tempString = *position;
			transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
			if (tempString == ("OF"))
			{
				position++;

				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
				if (tempString == ("TYPE"))
				{
					position++;

					tempString = *position;
					transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
					current_material_type = tempString;
					in_block = true;//get into element block
					cout << "material block has been found" << endl;

					continue;
				}
			}
		}

		if (in_block)
		{
			if (isAKeyWord(tempString)){
				break;
			}
			/************************************************************************/
			/* 这里应该有个工厂，通过curruent_material_type做一个material对象                                                                     */
			/************************************************************************/
			Elastic material;
			material.name = current_material_type;
			material.parseFembic(thisLine);
			materialList.push_back(material);

		}

	}


}

//int Fembic::parseConstraint(vector<Constraint>& constraintList){
int Fembic::parseConstraint(vector<BdCondition>& constraintList){
	string fileName = filePath;
	ifstream file;//#include <fstream>
	file.open(fileName);
	if (!file.is_open()){
		cout << "fail to open the file \n";
	}

	int numberOfNodes = 0;
	bool in_block = false;
	string thisLine;
	string current_constraint_type;

	while (getline(file, thisLine))
	{

		if (thisLine.empty()) continue;
		//开始对thisLine进行字符扑街
		char_separator<char> sepa("= \t[],");//必须要用制表符作为分词符之一，否则会漏掉很多
		typedef tokenizer<char_separator<char>> Tokenizer;
		Tokenizer tok(thisLine, sepa);
		Tokenizer::iterator position = tok.begin();//first string in thisLine

		if ((*position) == "#") continue;;//ignore annonatations

		string tempString = *position;
		transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);//change into capital model

		if (tempString == "CONSTRAINTS")
		{

			in_block = true;
			position++;

			tempString = *position;
			transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
			if (tempString == ("OF"))
			{
				position++;

				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
				if (tempString == ("TYPE"))
				{
					position++;

					tempString = *position;
					transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
					current_constraint_type = tempString;

					/************************************************************************/
					/* 此处应该利用类工厂，生成一个对应的类，用来调用parse                                                                     */
					/************************************************************************/

					in_block = true;//get into element block
					cout << "constraint block has been found" << endl;
					continue;
				}
			}
		}

		if (in_block)
		{
			if (isAKeyWord(tempString)){
				break;
			}
			/************************************************************************/
			/* 此处应该利用类工厂，生成一个对应的类，用来调用parse                                                                     */
			/************************************************************************/
			BdCondition constraint;
			constraint.parseFembic(thisLine);
			constraint.x_vel.printV();
			cout<<"约束名字为" << constraint.name << endl;
			constraintList.push_back(constraint);

		}//while thisLine, if in block

	}//while (getline(file, thisLine))
	return 0;

}

int Fembic::parseLoad(vector<Load>& loadList){
	string fileName = filePath;
	ifstream file;//#include <fstream>
	file.open(fileName);
	if (!file.is_open()){
		cout << "fail to open the file \n";
	}

	int numberOfNodes = 0;
	bool in_block = false;
	string thisLine;

	while (getline(file, thisLine))
	{
		Load load;

		if (thisLine.empty()) continue;
		//开始对thisLine进行字符扑街
		char_separator<char> sepa("= \t[],");//必须要用制表符作为分词符之一，否则会漏掉很多
		typedef tokenizer<char_separator<char>> Tokenizer;
		Tokenizer tok(thisLine, sepa);
		Tokenizer::iterator position = tok.begin();//first string in thisLine

		if ((*position) == "#") continue;;//ignore annonatations

		string tempString = *position;
		transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);//change into capital model

		if (tempString == "LOADS")
		{

			in_block = true;
			continue;
		}

		if (in_block)
		{
			if (isAKeyWord(tempString)){
				break;
			}

			load.parse_Fembic(thisLine);
			loadList.push_back(load);
		}//while(getline(file, thisLine);
	}
	return 0;
}

int Fembic::parseElementOfRod_2(vector<Elastic>& materialList, vector<Node>& nodeList, vector<Load> loadList, map<int, Node>& nodeTable, vector<Rod_2>& elementList, map<int, Rod_2>& elementTable){



	//string fileName ="E:\\test.txt";
	//string fileName;
	ifstream file;//#include <fstream>
	file.open(filePath);
	if (!file.is_open()){
		cout << "fail to open the file \n";
	}
	int numberOfElements = 0;
	bool in_block = false;
	string thisLine;

	string current_element_type;
	while (getline(file, thisLine))
	{
		//some temeporary param
		Rod_2 temporary_element;


		if (thisLine.empty()) continue;

		//开始对thisLine进行字符扑街
		char_separator<char> sepa("= \t[]");//必须要用制表符作为分词符之一，否则会漏掉很多
		typedef tokenizer<char_separator<char>> Tokenizer;
		Tokenizer tok(thisLine, sepa);
		Tokenizer::iterator position = tok.begin();//first string in thisLine

		if ((*position) == "#") continue;;//ignore annonatations

		string tempString = *position;
		transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);//change into capital model


		if (tempString == ("ELEMENTS")) {
			numberOfElements = 0;
			in_block = true;//get into parse this line, and get the data
			position++;

			tempString = *position;
			transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
			if (tempString == ("OF"))
			{
				position++;

				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
				if (tempString == ("TYPE"))
				{
					position++;

					tempString = *position;
					transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
					current_element_type = tempString;
					temporary_element.Type(current_element_type);
					in_block = true;//get into element block
					cout << "element block has been found" << endl;
					continue;
				}
			}
		}//block judge over

		if (in_block)
		{
			if (isAKeyWord(tempString)){
				break;
			}

			///Element ele;
			//temporary_element = ele.getElementOfType_Fembic(current_element_type);
			//temporary_element = new Rod_2();
			++numberOfElements;
			temporary_element.ElementID(numberOfElements);
			temporary_element.parse_Fembic(thisLine, materialList, loadList, nodeList, nodeTable);

			elementList.push_back(temporary_element);
			elementTable.insert({ numberOfElements, temporary_element });
		}

		//加入进度显示变量
	}// 外层w

	return numberOfElements;

}
//使用map时，容器自动对表按照<int, node>的int重排，但可以自定义重排规则
int Fembic::parseNodeVectorAndTable(vector<Node>& nodeList, map<int, Node>& nodeTable, vector<BdCondition>& constraintList, vector<Load>& loadList){


	string fileName = filePath;
	ifstream file;//#include <fstream>
	file.open(fileName);
	if (!file.is_open()){
		cout << "fail to open the file \n";
	}

	int numberOfNodes = 0;
	bool in_block = false;
	string thisLine;


	//双层while循环，外层对整个文件一行一行循环
	//内层对一行中分割好的分词符迭代器迭代循环

	while (getline(file, thisLine)){

		Node temporaryNode;

		BdCondition temp_constraint;
//		Load temp_load;

		if (thisLine.empty()) continue;

		//开始对thisLine进行字符扑街
		char_separator<char> sepa("= \t");//必须要用制表符作为分词符之一，否则会漏掉很多
		typedef tokenizer<char_separator<char>> Tokenizer;
		Tokenizer tok(thisLine, sepa);
		Tokenizer::iterator position = tok.begin();//first string in thisLine

		if ((*position) == "#") continue;;//ignore annonatations

		string tempString = *position;
		transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);//change into capital model

		if (tempString == ("NODES")) {
			numberOfNodes = 0;
			in_block = true;//get into parse this line, and get the data
			continue; //终止最近的循环中的当前迭代，并立即开始下一次迭代，只出现在for，while dowhile中
		}
		if (in_block)//a wonderful design
		{
			if (isAKeyWord(tempString)){
				break;
			}
 			temporaryNode.Number(++numberOfNodes);

			//begin inner loop to parse the line
			for (position; position != tok.end(); position++){
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
				if (tempString == "X"){
					position++;
					temporaryNode.setX_pos_orig(stod(*position));//set x
					temporaryNode.x_is_set = true;
				}
				else if (tempString == "Y"){
					position++;
					temporaryNode.setY_pos_orig(stod(*position));
					temporaryNode.Y_is_set = true;
				}
				else if (tempString == "Z"){
					position++;
					temporaryNode.setZ_pos_orig(stod(*position));
					temporaryNode.Z_is_set = true;
				}
				else if (tempString == "M"){
					position++;
					temporaryNode.mass = stod(*position);
					temporaryNode.M_is_set = true;
				}
				else if (tempString == "IXX"){
					position++;
					temporaryNode.inertia[0][0] = stod(*position);
					temporaryNode.IXX_is_set = true;
				}
				else if (tempString == "IYY"){
					position++;
					temporaryNode.inertia[1][1] = stod(*position);
					temporaryNode.IYY_is_set = true;
				}
				else if (tempString == "IZZ"){
					position++;
					temporaryNode.inertia[2][2] = stod(*position);
					temporaryNode.IZZ_is_set = true;
				}
				else if (tempString == "IXY"){
					position++;
					temporaryNode.inertia[0][1] = stod(*position);
					temporaryNode.inertia[1][0] = stod(*position);
					temporaryNode.IXY_is_set = true;
				}
				else if (tempString == "IYZ"){
					position++;
					temporaryNode.inertia[1][2] = stod(*position);
					temporaryNode.inertia[2][1] = stod(*position);
					temporaryNode.IYZ_is_set = true;
				}
				else if (tempString == "IXZ"){
					position++;
					temporaryNode.inertia[0][2] = stod(*position);
					temporaryNode.inertia[2][0] = stod(*position);
					temporaryNode.IXZ_is_set = true;
				}
				else if (tempString == "CONSTRAINT"){
					position++;
					tempString = *position;
					transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
					for (auto temp_constraint : constraintList)
					{
						if (temp_constraint.name == (tempString))
						{
							temporaryNode.setConstraint(temp_constraint);
							temporaryNode.constraint_is_set = true;

						}
					}
					cout << "node constraint has been set, constraint name is " << temporaryNode.getConstraint().name << endl;
					cout << "temporaryNode.acc.acc is \n";
					temporaryNode.getConstraint().x_acc.printV();
					temporaryNode.getConstraint().y_acc.printV();
					temporaryNode.getConstraint().z_acc.printV();
					cout << "解析临时节点时速度为 \n";
					temporaryNode.getConstraint().x_vel.printV();
					temporaryNode.getConstraint().y_vel.printV();
					temporaryNode.getConstraint().z_vel.printV();
				}
				else if (tempString == "LOAD"){
					position++;
					tempString = *position;
					transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
					/************************************************************************/
					/*
					for (unsigned i = 0; i < loadList.size(); i++){
					temp_load = loadList[i];
					if (temp_load.name == (*position)){
					temporaryNode.setLoad(temp_load);
					break;
					}
					}                                                                      */
					/************************************************************************/
					for (auto temp_load : loadList)
					{
						if (temp_load.name == (tempString)){
							temporaryNode.setLoad(temp_load);
							temporaryNode.load_is_set = true;
							cout << "node load has been set, node ID is " << numberOfNodes << endl;
							cout << "temporaryNode.load.acc is \n";
							temporaryNode.load.x_acc.printV();
							temporaryNode.load.y_acc.printV();
							temporaryNode.load.z_acc.printV();
							cout << "temporaryNode.load.Force is \n";
							temporaryNode.load.x_force.printV();
							temporaryNode.load.y_force.printV();
							temporaryNode.load.z_force.printV();
						}
					}
				}//else{ throw runtime_error("Unidentified node parameter indata in line"); }
			}//内层循环之，解析本行
		}//内层判断之，if in block,完成一个node的读入

		nodeList.push_back(temporaryNode);
		nodeTable.insert(pair<int, Node>(numberOfNodes, temporaryNode));
		//still exit an error: the information of nodeTable can not save ;the error is because foget plus & before the param
		//这里加入进度信息
	}//外层循环结束	
	//记得对异常进行捕捉
	if (!in_block){
		throw runtime_error("no nodes block found of missing nodes");
	}
	if (numberOfNodes == 0){
		throw runtime_error("No nodes found");
	}
	return numberOfNodes;
}

void Fembic::parseControlset(Controlset& controlset){


	//string fileName ="E:\\test.txt";
	//string fileName;
	ifstream file;//#include <fstream>
	file.open(filePath);
	if (!file.is_open()){
		cout << "fail to open the file \n";
	}
	int numberOfElements = 0;
	bool in_block = false;
	string thisLine;

	string current_element_type;
	while (getline(file, thisLine))
	{

		if (thisLine.empty()) continue;

		//开始对thisLine进行字符扑街
		char_separator<char> sepa("= \t[]");//必须要用制表符作为分词符之一，否则会漏掉很多
		typedef tokenizer<char_separator<char>> Tokenizer;
		Tokenizer tok(thisLine, sepa);
		Tokenizer::iterator position = tok.begin();//first string in thisLine

		if ((*position) == "#") continue;;//ignore annonatations

		string tempString = *position;
		transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);//change into capital model


		if (tempString == "CONTROLS"){
			in_block = true;//get into parse this line, and get the data
			
			continue;
		}

		if (in_block)
		{
			if (isAKeyWord(tempString)){
				break;
			}
			controlset.parseControlset_Fembic(thisLine);
		}
	}//while(getline(file, thisLine)


}

















#endif