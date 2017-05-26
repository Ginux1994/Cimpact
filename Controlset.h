#ifndef CONTROLSET_H
#define CONTROLSET_H





class Controlset
{
public:
	Controlset();

	void parseControlset_Fembic(string& filePath);
	void setInitialConditions();
	inline double getTimeStep(double time){
		if (timestepV.status(time))  return timestepV.value(time);
		else return 0;
	}

	double startTime;
	double endTime;
	double timeStep;
	double printstep;
	double printstep_tracker;
	int counter;
	int counter_tracker;
	int restore_save = 0;
	const int RESTORE_SAVE_OFF = 0;
	const int RESTORE_SAVE_ALL = 0;
	const int RESTORE_SAVE_PREVIOUS = 0;
	bool timestep_is_set;
	bool run_is_set, print_is_set, track_print_is_set;
	bool restore_save_is_set;
	bool autoStep;

	Variable timestepV;

	string writer;
	string trackWriter;

protected:
private:
};
Controlset::Controlset(){

}


void Controlset::parseControlset_Fembic(string& thisLine){



	//开始对thisLine进行字符扑街
	char_separator<char> sepa("= \t[]");//必须要用制表符作为分词符之一，否则会漏掉很多
	typedef tokenizer<char_separator<char>> Tokenizer;
	Tokenizer tok(thisLine, sepa);
	Tokenizer::iterator position = tok.begin();//first string in thisLine

//	if ((*position) == "#") continue;//ignore annonatations

	string tempString = *position;
	transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);//change into capital model

	for (position; position != tok.end(); position++)
	{

		if (tempString == ("RUN")) {

			position++;

			tempString = *position;
			transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
			if (tempString == ("FROM"))
			{
				position++;
				startTime = stod(*position);
				position++;
				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
				
				if (tempString == ("TO"))
				{
					position++;
					tempString = *position;

					endTime = stod(tempString);
					cout << "start form " << startTime << " to  " << endTime << endl;
					position++;
					if (position==tok.end())
					{
						autoStep = true;
						timestepV = Variable(0.0);//to meet the auto step
						return;
					}
					else{
						tempString = *position;
						transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
						if (tempString == ("STEP"))
						{
							while (position != tok.end())
							{
								vector<string> vecTimeStep;
								for (++position; position != tok.end(); position++)
								{
									tempString = *position;
									vecTimeStep.push_back(tempString);
								}
								if (vecTimeStep.size() == 1)
								{
									double fuckD = stod(vecTimeStep[0]);
									timestepV = Variable(fuckD);
									autoStep = true;
								}
								else{
									timestepV = Variable(vecTimeStep);
									autoStep = false;
								}
								timestep_is_set = true;
								cout << "========华丽丽的分割线 之 步长====== \n";
								cout << "步长为： \n";
								timestepV.printV();
							}
							return;
						}
					}
				}
			}
		}//block judge over
		if (tempString == ("PRINT")) {

			position++;

			tempString = *position;
			transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);
			if (tempString == ("EVERY"))
			{
				position++;

				tempString = *position;
				timeStep = stod(tempString);
				position++;

				tempString = *position;
				transform(tempString.begin(), tempString.end(), tempString.begin(), toupper);

				if (tempString == ("STEP"))
				{
					cout << "print every " << timeStep << endl;
					autoStep = true;
					return;
				}
			}
		}

	}

}

void Controlset::setInitialConditions(){
	if (!track_print_is_set)
	{
		printstep_tracker = printstep;
	}
}






#endif