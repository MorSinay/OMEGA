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

#pragma once
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <algorithm>

#include "BSection.h"

#include "Managers\RoutesLibrary\VirtualGrid.h"

#include "GraphGridDrone\AdvModel.h"


using namespace std;

//#define DETECTION_PROBABILITY 0.9
// #define MINOR_PROBABILITY 0.00001
#define PROB_PRECISION 10000000



typedef map<int, VirtualCell> cells_dict_t;

typedef map<double /*section id*/, int /*junction id*/> junctions_dict_t;//junction id is double

class SearchModule
{

public:
	//SearchModule(const unsigned int& numberOfDrones){numberOfDrones_=numberOfDrones;};
    SearchModule(const vector<BRoad>& bRoads,VirtualGrid& vgrid, const unsigned int& numberOfDrones);

	
	SearchModule& operator=(const SearchModule& other) // copy assignment
	{
		if (this != &other) { // self-assignment check expected

				this->cellsDict_=other.cellsDict_;
				vgrid_.sectionDict_=other.vgrid_.sectionDict_;
				this->junctionDict_=other.junctionDict_;
		}
		return *this;
	}

	SearchModule(SearchModule& obj);

	vector<int> setPenetrationPoint(const double& originalSectionId, const unsigned int& numOfDrones, const int& junctionId, const double& cellChooseProbThreshold);
	vector<int> updateListOfCells(const vector<int> cellList, const unsigned int& numOfDrones);

	void printSectionsProbability();
    
    void nextP();

	vector<int>& getCellsIdWithProb()
	{
		return cellsIdsWithProb_;
	}

private:
    cells_dict_t cellsDict_;
    
    VirtualGrid& vgrid_;

    junctions_dict_t junctionDict_;

    int numberOfDrones_;

    vector<int> cellsIdsWithProb_;

   // SearchModule();
    
    void cleanMinorProbabilities();
    void resetJunctions();
    void initialize();
    void resetNextProbability();
    void printCurrProbability();
    void printNextProbability();
	void updateProbRecursive(const double& sectionId, const int& junctionId, const double& prob, const double& length);
    AdvModel getAdversaryModel(const double& sectionId, const int& junctionId);
    void updateNextProbability();
    void setNextProbabilityToCurr();
    int setNextProbabilityToCurrTempCheck();
    void updateSectionProbabilityList(const vector<double>& sections);
    void updateCellListProbability(const vector<int> cells);
    double entropy(const bool& currFlag = false);
    double currEntropy();
    double newCellsEntropy(const vector<int>& cells);
    int getMaxProbCellId(const vector<int>& cells, double& maxProbCellProb);
    vector<int> maxIG_kDrone(const unsigned int& k);
    
    void printCurrAndNextProbs();
    
    void getCombinations(const vector<int>& cellIds, int K, vector<vector<int> >& combinationsList);
};

