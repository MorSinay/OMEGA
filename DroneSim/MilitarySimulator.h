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

#ifndef MILITARY_SIMULATOR_H
#define MILITARY_SIMULATOR_H

#include <stdlib.h>
#include <assert.h>
#include <iostream>
#include <sstream>

#include "types.h"
#include "defines.h"

#include "Entities/Enemy.h"
#include "Entities/Drone.h"
#include "Entities/UGV.h"

#include "Structures/Section.h"
#include <set>


class MilitarySimulator {
    
public:

    MilitarySimulator();
    ~MilitarySimulator();
    
	static MilitarySimulator* simulatorLoader(const string& inputFileName, const string& borderCellsFileName, const string& citiesCellsFileName, double alpha);
	static MilitarySimulator* simulatorLoader(const string& inputFileName);
	int getch(void);
	cell_id_t start(vector<Drone>& drones);
    cell_id_t nextStep();
    cell_id_t getEntropyAction();
	vector<section_id_t> GetInitEnemiesPoint(const unsigned int& enemyCount);
	vector<section_id_t> GetGoalEnemiesPoint(const unsigned int& enemyCount, const vector<int>& goalsVector);
	vector<cell_id_t> GetInitCellsPenetrationPoints() { return initBorderCells_; }


	size_t getCitiesSize();

//Preprocess methods
    inline void addSection(const section_id_t& sectionId, const cell_id_t& myCellId, const float& sectionLength, const vector<double>& nextSectionsProbs, const vector<section_id_t>& nextSectionsIds) {

        Section* section = new Section(sectionId, myCellId, sectionLength, nextSectionsProbs, nextSectionsIds);

        sectionsVec_.push_back(section);
        sectionsMap_[sectionId] = section;
    }

	inline Section* getCurrentEnemySection() const { return currentEnemySection_; }

	inline cell_id_t getCurrentEnemyCell() const { return currentEnemyCell_; }

	inline cell_id_t getCurrentDroneCell() const { return currentDroneCell_; }

	inline const bool& getIsEnemyFound() const { return this->isEnemyFound; }

	inline void setIsEnemyFound(bool data) { this->isEnemyFound = data; }

	inline const map<section_id_t, Section*> getSectionMap() const { return sectionsMap_; }

	inline const map_cell_neighbours_t& getCellNeighbours() const { return this->cellNeighbours_; }

	inline map_cell_neighbours_t& setCellNeighbours() { return this->cellNeighbours_; }

	inline const vector<cell_id_t>& getCellsToSearch() const { return this->cellsToSearch; }

	inline vector<cell_id_t>& setCellsToSearch() { return this->cellsToSearch; }

	inline vector<cell_id_t>& getCellNearCities() { return cellNearCities_; }

	bool LoadBorderCells(const string& borderCellsFileName);

	bool LoadCitiesCells(const string& citiesCellsFileName);
		
private:

    bool moveEnemyToNextLocation();

    inline Section* getSectionById(const section_id_t& sectionId) {

        // TODO: add try catch or existence checks - only for debug. In release will run without them

        return sectionsMap_[sectionId];
    }

	void buildCellNeighboursMap();

	void printCellNeighboursMap();

	bool isAllDronesArriveToGoalCell(vector<Drone>& drones);

	//check if number of cells we got from mor algo is fit
	void checkAndFixNumberOfCells(vector<cell_id_t>& cellsToSearch);

	//check if part of the drone group arrive to destinion and send them to neighbors cells
	void arrivedDroneMoveToNeighbors(vector<Drone>& drones);

	//set all drones enemy
	void setDroneEnemy(Enemy& enemy,vector<Drone>& drones);

	//Aggregate between grid next section prob and uniform prob by alpha
	vector<double> AggregateAndGetNextSectionProb(vector<double> probsStrList, double alpha);

private:
    // Holds current enemy section
    Section* currentEnemySection_;

    // Holds updated value of current enemy's cell (according to it's current section)
    cell_id_t currentEnemyCell_;

    // Holds updated value of current drone's cell (according to it's current section)
    cell_id_t currentDroneCell_;

    // The vector that holds all sections
    vector<Section*> sectionsVec_;

    // The map that holds all sections when the section's id is the value
    map<section_id_t, Section*> sectionsMap_;

    // The map that holds all sections when the section's id is the value
    map<cell_id_t, double /* cell's probability */> cellsProbs_;

	//map that contain for each cell is Neighbours
    map_cell_neighbours_t cellNeighbours_;

	//cell to search, we get them from mor algo
	vector<cell_id_t> cellsToSearch;

	//flag if we found enemy
	bool isEnemyFound;

	//Optional border cells
	vector<cell_id_t> borderCells_;

	//initialaized border cells
	vector<cell_id_t> initBorderCells_;

	//Optional cities cells
	vector<cell_id_t> citiesCells_;

	vector<cell_id_t> cellNearCities_;

	void findCellsNearCities();

};

#endif /* MILITARY_SIMULATOR_H */
