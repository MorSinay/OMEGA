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

// #include <conio.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <stdlib.h> 
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <termios.h>
#include <unistd.h>
#include <stdio.h>

#include "types.h"
#include <time.h>

#include "MilitarySimulator.h"

#define foreach BOOST_FOREACH

MilitarySimulator::MilitarySimulator() {
	buildCellNeighboursMap();
	this->setIsEnemyFound(false);
}

MilitarySimulator::~MilitarySimulator() {
	
	for (auto& section : sectionsMap_) {
		
		if (nullptr != section.second) {
			
			delete section.second;
			section.second = nullptr;
		}
	}

	sectionsMap_.clear();

	sectionsVec_.clear();
}

int MilitarySimulator::getch(void) {

    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}

cell_id_t MilitarySimulator::start(vector<Drone>& drones) {

    assert(sectionsVec_.size() > 0);

    unsigned int randomSectionNum = rand() % sectionsVec_.size();

    cout << "Start from section id: " << sectionsVec_[randomSectionNum]->getSectionId() << endl;

	size_t id = 1;
    Enemy enemy(id, randomSectionNum, rand() % sectionsVec_.size(),sectionsMap_);

    enemy.gotoGoal();

	setDroneEnemy(enemy,drones);

	while(false == isEnemyFound)
	{
		if (enemy.isReachedTheGoal()) {
            cout << " reached " << endl;
			getch();
            return 0;
        }

		//all drones arrive 
		if(true == isAllDronesArriveToGoalCell(drones)) {

			cellsToSearch.push_back(279);//TEMP
			cellsToSearch.push_back(345);//TEMP
			checkAndFixNumberOfCells(cellsToSearch);
			//set drones goal cell
			for(size_t i=0 ; i<drones.size(); i++)
			{
				drones.at(i).setInNeighbor(false);
				drones.at(i).setIsDroneArrive(false);
				drones.at(i).setGoalCellId(cellsToSearch.at(i));
			}
		}
		else//check if some of the drones arrived
		{
			arrivedDroneMoveToNeighbors(drones);
		}

		for(vector<Drone>::iterator itr=drones.begin(); itr!=drones.end(); ++itr)
		{
			isEnemyFound = itr->tick();
		}
		 enemy.tick();
		
	}

	cout<<"found enemy"<<endl;
	getch();

    return 0;
}

cell_id_t MilitarySimulator::nextStep() {

    if (false == moveEnemyToNextLocation()) {
        // Stop walking
        return 0;
    }

    return currentEnemyCell_;
}

cell_id_t MilitarySimulator::getEntropyAction() {
	return 0;
}

bool MilitarySimulator::moveEnemyToNextLocation() {

    if (NULL == currentEnemySection_) {
        cout << "moveEnemyToNextLocation: there is a problem: currentEnemySection_ is null!" << endl;
        return false;
    }

    currentEnemySection_ = getSectionById(currentEnemySection_->getLikelyNextSectionId());

    currentEnemyCell_ = currentEnemySection_->getSectionCellId();

    return true;
}
void MilitarySimulator::buildCellNeighboursMap(){

	for(int row=0; row<GRID_ROWS; row++)
	{
		for(int col=0; col<GRID_COLS; col++)
		{
			vector<cell_id_t> cellNeighbours;

			if(col > 0)//left
			{
				cellNeighbours.push_back( (GRID_COLS * row + col) - 1);
			}
			if(col < GRID_COLS-1)//right
			{
				cellNeighbours.push_back( (GRID_COLS * row + col) + 1);
			}
			if(row > 0)//up
			{
				cellNeighbours.push_back( (GRID_COLS * row + col)  - GRID_COLS);
			}
			if(row < GRID_ROWS-1)//down
			{
				cellNeighbours.push_back( (GRID_COLS * row + col)  + GRID_COLS);
			}
			if(row > 0 && col > 0)// left up corner
			{
				cellNeighbours.push_back( (GRID_COLS * row + col)  - GRID_COLS - 1);
			}
			if(row > 0 && col < GRID_COLS-1)//right up corner
			{
				cellNeighbours.push_back( (GRID_COLS * row + col)  - GRID_COLS + 1);
			}
			if(row < GRID_ROWS-1  && col >0)//left down corner
			{
				cellNeighbours.push_back( (GRID_COLS * row + col)  + GRID_COLS - 1);
			}
			if(row < GRID_ROWS-1 && col < GRID_COLS-1)//right down corner
			{
				cellNeighbours.push_back( (GRID_COLS * row + col)  + GRID_COLS + 1);
			}

			setCellNeighbours()[GRID_COLS * row + col] = cellNeighbours;
		}
	}
}
void MilitarySimulator::printCellNeighboursMap(){

	for(map_cell_neighbours_t::const_iterator itr = getCellNeighbours().begin(); itr!=getCellNeighbours().end(); ++itr)
	{
		cout<<"cell "<< itr->first<<":"<<endl;
		for(vector<cell_id_t>::const_iterator itrr=itr->second.begin();itrr!=itr->second.end();++itrr)
		{
			cout<<*itrr<<",";
		}
		cout<<endl;
	}
}

bool MilitarySimulator::isAllDronesArriveToGoalCell(vector<Drone>& drones)
{
	for(vector<Drone>::iterator itr=drones.begin(); itr!=drones.end(); ++itr)
	{
		if(true == itr->isNavigating() && false == itr->getIsDroneArrive())
		{
			return false;	
		}
	}
	return true;
}

void MilitarySimulator::checkAndFixNumberOfCells(vector<cell_id_t>& cellsToSearch){
	bool addAbove=false;
	if(NUMBER_OF_DRONES > cellsToSearch.size())
	{
		int cellsToAdd = NUMBER_OF_DRONES - cellsToSearch.size();
		while(cellsToAdd != 0)
		{
			int row = cellsToSearch.at(cellsToSearch.size()-1) / GRID_COLS;
			if(false == addAbove && row < GRID_ROWS)
			{
				cellsToSearch.push_back(cellsToSearch.at(cellsToSearch.size()-1) + GRID_COLS);
				cellsToAdd--;
			}
			else
			{
				addAbove=true;
			}
			if(true == addAbove && row > 0)
			{
				cellsToSearch.push_back(cellsToSearch.at(0) - GRID_COLS);
				cellsToAdd--;
			}
		}
	}
}

void MilitarySimulator::arrivedDroneMoveToNeighbors(vector<Drone>& drones)
{
	for(vector<Drone>::iterator itr = drones.begin(); itr!=drones.end(); ++itr)
	{
		if(true == itr->getIsDroneArrive() && false == itr->getInNeighbor())
		{
			if( (itr->getCurrentCellId() % GRID_COLS)/*col*/ > 0)
			itr->setGoalCellId(itr->getCurrentCellId()-1);
			itr->setInNeighbor(true);
		}
	}
}

void MilitarySimulator::setDroneEnemy(Enemy& enemy,vector<Drone>& drones)
{
	for(auto itr=drones.begin(); itr != drones.end(); ++itr)
	{
		itr->setEnemy(&enemy);
	}

}

bool MilitarySimulator::LoadBorderCells(const string& borderCellsFileName)
{
	if (borderCellsFileName.compare("") == 0) return false;

	ifstream input(borderCellsFileName.c_str());
	string line;
	
	if (input.is_open())
	{
		getline(input, line);
		vector<string> values;
		boost::split(values, line, boost::is_any_of(","));
		for (string value : values)
		{
			borderCells_.push_back(std::stoi(value));
		}
		input.close();
	}
	

	return borderCells_.size() > 0 ? true : false;
}

bool MilitarySimulator::LoadCitiesCells(const string& citiesCellsFileName)
{
	if (citiesCellsFileName.compare("") == 0) return false;
	ifstream input(citiesCellsFileName.c_str());
	string line;

	if (input.is_open())
	{
		getline(input, line);
		vector<string> values;
		boost::split(values, line, boost::is_any_of(","));
		for (string value : values)
		{
			citiesCells_.push_back(std::stoi(value));
		}
		input.close();
	}

	return citiesCells_.size() > 0 ? true : false;
}

size_t MilitarySimulator::getCitiesSize() {
	
	return citiesCells_.size();
}

vector<section_id_t> MilitarySimulator::GetInitEnemiesPoint(const unsigned int& enemyCount)
{
	vector<section_id_t> initPointsVector;

	for (int i = 0; i < enemyCount; i++)
	{
		int r = rand() % borderCells_.size();
		cell_id_t initCell = borderCells_[r];

		section_id_t goalSection;
		for (auto section : sectionsMap_)
		{
			if (section.second->getSectionCellId() == initCell)
			{
				initPointsVector.push_back(section.second->getSectionId());
				break;
			}
		}
	}
	return initPointsVector;
}

vector<section_id_t> MilitarySimulator::GetGoalEnemiesPoint(const unsigned int& enemyCount, const vector<int>& goalsVector)
{

	vector<section_id_t> goalPointsVector;

	for (int i = 0; i < enemyCount; i++)
	{
		cell_id_t initCell;// = citiesCells_[r];

		if (goalsVector.size() > i) {

			initCell = citiesCells_[goalsVector[i]];
		}
		else {

			int r = rand() % citiesCells_.size();

			initCell = citiesCells_[r];
		}


		//vector<section_id_t> sectionsInCell;
		section_id_t goalSection;

		for (auto section : sectionsMap_) {
			
			if (section.second->getSectionCellId() == initCell) {
				goalPointsVector.push_back(section.second->getSectionId());
				break;
			}
		}
	}
	
	return goalPointsVector;
}

//This function take the sections probs from the grid and aggregate this values with uniform probability by alpha parameter that determine the wights for 'grid probs' and uniform brobs
vector<double> MilitarySimulator::AggregateAndGetNextSectionProb(vector<double> probsSectionsList, double alpha)
{
	double uniform = 1 / boost::lexical_cast<double>(probsSectionsList.size());

	vector<double> probList;

	for (double nextSectionProb : probsSectionsList)
	{
		probList.push_back(alpha * nextSectionProb + (1 - alpha) * uniform);
	}

	return probList;
}

void MilitarySimulator::findCellsNearCities()
{
	for (cell_id_t city : citiesCells_)
	{
		for (int i = -2; i < 2; i++)
		{
			for (int j = -2; j < 3; j++){
				int newCell = city + i + j * GRID_COLS;
				if (newCell <= 0 || newCell > GRID_COLS*GRID_ROWS) {
					continue;
				}

				if (!(std::find(cellNearCities_.begin(), cellNearCities_.end(), newCell) != cellNearCities_.end())) {
					cellNearCities_.push_back(newCell);
				}
			}
		}
	}
}

MilitarySimulator* MilitarySimulator::simulatorLoader(const string& inputFileName) {
	return simulatorLoader(inputFileName, "", "", 1);
}

MilitarySimulator* MilitarySimulator::simulatorLoader(const string& inputFileName, const string& borderCellsFileName = "", 
	const string& citiesCellsFileName = "", double alpha = 1.0) {

	// auto startTime = clock();
	MilitarySimulator* simulator = new MilitarySimulator();

	bool loadBorderCellsSucceed = simulator->LoadBorderCells(borderCellsFileName);
	bool loadCitiesCellsSucceed = simulator->LoadCitiesCells(citiesCellsFileName);
	simulator->findCellsNearCities();

	string line;

	ifstream input(inputFileName.c_str());
	
	if (input.is_open()) {
		// Throw the first line
		getline(input, line);	

		vector<string> gridDim;
		boost::split(gridDim, line, boost::is_any_of(","));
		int RowsDim = boost::lexical_cast<int>(gridDim[1]);

		int ColsDim = boost::lexical_cast<int>(gridDim[3]);
		
		int GridCellSizeMeters = boost::lexical_cast<float>(gridDim[5]);

		if ((RowsDim != GRID_ROWS) || (ColsDim != GRID_COLS) || (GridCellSizeMeters != GRID_CELL_SIZE_METERS)){
			throw invalid_argument("Incompatible grid dimensions, please update grid dimensions according to sections_data.csv generation file.");
		}

		while (getline(input, line)) {

			try
			{
				
			vector<string> columns;
			boost::split(columns, line, boost::is_any_of(","));

			if ("" == line) { // nextSectionsIdsNum [0..999]
				continue;
			}

			section_id_t ordinalSectionId = boost::lexical_cast<int>(columns[0]);     // 0   columns[0]:  ordinalSectionId
			cell_id_t    myCellId = boost::lexical_cast<int>(columns[2]);     // 279	columns[2]: myCellId
			float length = boost::lexical_cast<float>(columns[4]);   // 71.07		columns[4]: length

																	 // Parsing the nextSectionsIdsProbs
			string probsString = columns[7]; // [0.00;1.00;0.00;0.00]		columns[7]: nextSectionsIdsProbs

			boost::replace_all(probsString, "[", "");
			boost::replace_all(probsString, "]", "");

			vector<string> probsStrList;
			boost::split(probsStrList, probsString, boost::is_any_of(";")); // 0.00;1.00;0.00;0.00

			vector<double> nextSectionsProbs;
			if (boost::lexical_cast<int>(columns[6]) != 0) {
				foreach(string sectionProbabilityStr, probsStrList) {
					nextSectionsProbs.push_back(boost::lexical_cast<double>(sectionProbabilityStr));
				}
			}

			vector<double> aggregationNextSectionsProbs = simulator->AggregateAndGetNextSectionProb(nextSectionsProbs, alpha);

			// Parsing the nextSectionsOrdinalIds
			string nextSectionsIdsSingleStr = columns[9];  // [1;2;35615;35626]		columns[9]: nextSectionsOrdinalIds

			boost::replace_all(nextSectionsIdsSingleStr, "[", "");
			boost::replace_all(nextSectionsIdsSingleStr, "]", "");

			vector<string> nextSectionsIdsStrings;
			boost::split(nextSectionsIdsStrings, nextSectionsIdsSingleStr, boost::is_any_of(";")); // 0.00;1.00;0.00;0.00

			vector<section_id_t> nextSectionsIds;
			if (boost::lexical_cast<int>(columns[6]) != 0) {
				foreach(string sectionIdStr, nextSectionsIdsStrings) {
					nextSectionsIds.push_back(boost::lexical_cast<section_id_t>(sectionIdStr));
				}
			}


			simulator->addSection(ordinalSectionId, myCellId, length, aggregationNextSectionsProbs, nextSectionsIds);

			}
			catch (exception e)
			{
				int y = 0;
			}

			// Not used fields
			// double sectionId 				= boost::lexical_cast<double>(columns[1]);  // -124465.00 
			// long int enemyPassCounter 		= boost::lexical_cast<int>(columns[3]);     // 3100 
			// int roadId 						= boost::lexical_cast<int>(columns[5]);     // 16129 

			// vector<unsigned int> nextSectionsIdsCounters = (columns[8]);  // [1;1643232;1;5414] 
			// vector<double> nextSectionsIds 				 = (columns[10]); // [-124464.00;-124463.00;-79188.00;-79177.00] 

		} // for each line
		input.close();
	}
	
	// auto elapsedTime = ((float)(clock() - startTime)) / CLOCKS_PER_SEC;
	// cout << "Loading Section Completed in " << elapsedTime << " seconds." << endl;
	return simulator;
}
