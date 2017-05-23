#ifndef TOKEN_H
#define TOKEN_H
#include <string>


class Token
{
public:
	string word;
	std::string Word() const { return word; }
	void Word(std::string val) { word = val; }

	double number;
	double Number() const { return number; }
	void Number(double val) { number = val; }
	
	bool is_a_word;
	bool Is_a_word() const { return is_a_word; }
	bool Is_a_num() const { return !is_a_word; }
	void Is_a_word(bool val) { is_a_word = val; }

	Token(string &str);
	Token(double &d);


protected:
private:
};
Token::Token(string &str){

	try{
	number = stod(str);
	}
	catch (std::invalid_argument &ia)
	{
		cerr << "invalid argument " << ia.what() << endl;
		cout << "string to doubleis wrong" << endl;
		word = str;
		is_a_word = true;
		return;
	}
	//it is turely a number
	is_a_word = false;
}
Token::Token(double &d){
	number = d;
	is_a_word = false;
}











#endif