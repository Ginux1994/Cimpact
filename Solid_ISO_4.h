#ifndef SOLID_ISO_4_H
#define SOLID_ISO_4_H


#include "allInclude.h"


class Solid_Iso_4
{
public:
	Solid_Iso_4();

protected:
public:
	vector<Elastic> material;
	vector<Node> nodes;
	Array2D<double> D;
	Array2D<double> M;
	Array2D<double> N;
	Array2D<double> H;
	Array2D<double> d;
	Array2D<double> f;
	Array2D<double> P;
	vector<Array2D<double>> M;
	vector<Array2D<double>> J;
	vector<Array2D<double>> J_inv;
	vector<Array2D<double>> strain;
	vector<Array2D<double>> stress;
	vector<Array2D<double>> B;

	vector<double> xsi;
	vector<double> phi;
	vector<double> etha;
	vector<double> W;

	string type''
	int number_of_integration_points;
	bool NIP_is_set;
	bool Nodes_are_set;
	bool Material_is_set;

};

Solid_Iso_4::Solid_Iso_4(){
	type = "SOLID_ISO_4";

	material = vector<Elastic>(8);

	xsi = vector<double>(8);
	etha = vector<double>(8);
	phi = vector<double>(8);
	nodes = vector<Node>(8);
	W = vector<double>(8);;

	H = Array2D<double>(6, 9);




}



































#endif