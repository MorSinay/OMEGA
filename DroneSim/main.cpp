/**
* Authors: Oleg Maksimov, Guy Levy, Aviad Fuchs and Mor Sinay, AIM Lab, Bar-Ilan University
*
* The MIT License (MIT)
*
* Copyright (c) 2018 AIM Lab, Bar-Ilan University
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "types.h"
#include "MilitarySimulator.h"

#include "Agents/MosheAgent/MosheSearchModule.h"

#define foreach BOOST_FOREACH

MilitarySimulator* simulatorLoader(const string& inputFileName);
void initForce(vector<Drone>& drones);
void checkAndFixNumberOfCells(vector<cell_id_t>& cellsToSearch);
bool isAllDronesArriveToGoalCell(vector<Drone>& drones);
void arrivedDroneMoveToNeighbors(vector<Drone>& drones);


int main(int argc, char* argv[]) {

	vector<Drone> drones;
	initForce(drones);

	srand ((size_t)time(NULL));

	MilitarySimulator* simulator = MilitarySimulator::simulatorLoader(SimulationManager::getDataDirPath() + "sections_data.csv");

	if (nullptr != simulator) {
		delete simulator;
	}
	
	//MilitarySimulator* simulator = simulatorLoader(path + "small_example.csv");

	// Here will be Mor's algorithm
	//#####################################################################!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//auto sectionMap = simulator->getSectionMap();
	//auto moshe = new MosheSearchModule(sectionMap);
	//moshe->initialize(0);
	//simulator->start(drones);

	//for (size_t i = 0; i < 100; i++)
	//{
	//	moshe->tick(1);
	//}

	//delete moshe;
}


void initForce(vector<Drone>& drones){
	for(int i=0;i<NUMBER_OF_DRONES;i++)
	{
		drones.push_back(Drone(FIRST_CELL_IN_BORDER+(i*GRID_COLS),i));
	}
}

