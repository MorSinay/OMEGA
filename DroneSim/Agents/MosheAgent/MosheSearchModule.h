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

#ifndef MOSHESEARCHMODULE_H
#define MOSHESEARCHMODULE_H

#include <algorithm>
#include "../../Structures/Section.h"
#include "../../Entities/Enemy.h"
#include "../../types.h"
#include "../../defines.h"
#include <unordered_set>
#include <iostream>
#include <fstream>
#include <string>

#include "../../SimulationManager.hpp"

#include "../../Entities/EnemyEntity.h"
#include <set>

class MosheSearchModule {

public:
	~MosheSearchModule() {
		for (auto enemy : enemies_) {

			enemy.reset();
			// if (nullptr != enemy) {
			// 	delete enemy;
			// }
		}

		for (auto pair : enemyEntitiesMap_) {
			pair.second.reset();
			// if (nullptr != pair.second) {
			// 	delete pair.second;
			// }
		}
	}

	MosheSearchModule(const map<section_id_t, Section*>& sectionsMap, 
			const unsigned int& enemyCount, const double& detectionProbability, 
			vector<cell_id_t>& planBCells, char algorithmType, double alphaThr, double minorProb);

	void initialize(const vector<section_id_t> sectionIdInitVector, const vector<section_id_t> sectionIdGoalVector, const double& cellChooseProbThreshold, char algorithmType, const vector<int>& enemyPaths);

	void tick(double time, double intervals);

	void tick(double time);

	inline vector<boost::shared_ptr<Enemy> > getEnemies() { return enemies_; }

	map<cell_id_t, double> UpdateCellProbablities(EnemyEntity* sectionIds);

	//void UpdateCellProbablities(const unordered_set<section_id_t>& sectionIds);
	void UpdateCellProbablities();
	//void setPenetrationPoint(const section_id_t& sectionId);
	void setPenetrationPoint(const vector<section_id_t> sectionIdInitVector, const vector<section_id_t> sectionIdGoalVector, const double& cellChooseProbThreshold, char algorithmType, const vector<int>& enemyPaths);

	void updateBeliefProbability(double time);
	inline cell_id_t getMaxProbability() { return maxCellId_; }
	inline vector<boost::shared_ptr<Enemy> > getActiveEnemies() { return activeEnemies_; }
	bool isAnyEnemyReachedGoal();
	void updateProbRecursive(const Section& rootSection, const double & prob, const double & sectionLength, const double & maxLength, const double & minLength);

	double GetDistance(const cell_id_t& from, const cell_id_t& to);

	bool isActive(boost::shared_ptr<Enemy> enemy);

	void printStatus();

	vector<cell_id_t> performDetection(const vector<cell_id_t>& detectCellIds, const double& detectionProbability);

	void getCombinations(const unordered_set<cell_id_t>& cellIds, int K, vector<vector<cell_id_t>>& combinationsList);
	double getEntropyGain(map<cell_id_t, double> nonZeroProbCells, const vector<cell_id_t>& permutationCellIds, const double& detectionProbability);
	void Entropy(const unsigned int & k, double maxThreshold, vector<cell_id_t>& resultCellIds, const double& detectionProbability);
	void Entropy(const unsigned int & k, vector<cell_id_t>& resultsCellIds, const double& detectionProbability);
	void Entropy_(const unsigned int& k, double maxThreshold, vector<cell_id_t>& resultCellIds, const double& detectionProbability);
	void SCOUT(const unsigned int & k, vector<cell_id_t>& resultCellIds, const double& detectionProbability, const double& cellChooseProbThreshold);
	vector<cell_id_t> getNextCells(const Algo & algo, const unsigned int & k, const double & detectionProbability, const double & cellChooseProbThreshold, bool planB = false);
	void getMaxProb(const unsigned int& k, double maxThreshold, vector<cell_id_t>& resultCellIds);
	void GetRandProb(const unsigned int& k, vector <cell_id_t>& resultCells);
	int FindMaxPerEnemyAndFixResults(const unsigned int& k, double maxThreshold, vector<cell_id_t>& resultCellIds);

	map<cell_id_t, double>  GetNonZeroProbCells();	
private:

private:
	unsigned int enemyCount_;
	cell_id_t maxCellId_;
	map<section_id_t, Section*> sectionsMap_;
	vector<boost::shared_ptr<Enemy> > enemies_;
	vector<boost::shared_ptr<Enemy> > activeEnemies_;

	map<cell_id_t, double>  nonZeroProbCells__;
	unordered_set<section_id_t> nonZeroProbSectionsIds_;
	unordered_set<section_id_t> tmpNonZeroProbSectionsIds_;

	//Dictionary that hold all data about the enemy entities
	map<int, boost::shared_ptr<EnemyEntity> > enemyEntitiesMap_;

	//Set with all cell with non zero prob from all enemies
	unordered_set<cell_id_t> nonZeroProbCells_;

	vector<vector<section_id_t> > LoadPaths(const vector<int>& enemyPaths, int enemyCount);
	vector<string> getPathsListFromDirectory(int enemyCount);

	vector<cell_id_t> planBCells_;
	vector<cell_id_t> GetPlanBCells(int uavCount);

	char algorithmType_;

	double alphaThr_;

	double minorProb_;
};


#endif /* MOSHESEARCHMODULE_H */
