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

#include "MosheSearchModule.h"
#include <boost/lexical_cast.hpp>

MosheSearchModule::MosheSearchModule(const map<section_id_t, Section*>& sectionsMap, 
		const unsigned int& enemyCount, const double& detectionProbability, 
		vector<cell_id_t>& planBCells, char algorithmType, double alphaThr, double minorProb) :

	sectionsMap_(sectionsMap),
	enemyCount_(enemyCount),
	planBCells_(planBCells),
	algorithmType_(algorithmType),
	alphaThr_(alphaThr),
	minorProb_(minorProb)
{
}

void MosheSearchModule::initialize(const vector<section_id_t> sectionIdInitVector, const vector<section_id_t> sectionIdGoalVector, const double& cellChooseProbThreshold, char algorithmType, const vector<int>& enemyPaths) { 

	setPenetrationPoint(sectionIdInitVector, sectionIdGoalVector, cellChooseProbThreshold, algorithmType, enemyPaths); 
}


void MosheSearchModule::tick(double time, double intervals)
{
	tick(time / intervals);
}

void MosheSearchModule::tick(const double time)
{
	for (auto enemy : enemies_) enemy->tick(time);
	updateBeliefProbability(time);
}

void MosheSearchModule::UpdateCellProbablities()
{
	for (auto enemy : activeEnemies_)
	{
		enemyEntitiesMap_[enemy->id_]->updateCellProbablities();
	}

	nonZeroProbCells_.clear();
	for (auto enemy_entity : enemyEntitiesMap_)
	{
		for (auto nonZeroCellId : enemy_entity.second->getNonZeroProbCells())
		{
			nonZeroProbCells_.insert(nonZeroCellId.first);
		}
	}
}

double MosheSearchModule::GetDistance(const cell_id_t& from, const cell_id_t& to)
{
	auto x0 = from / GRID_COLS;
	auto y0 = from % GRID_COLS;
	auto x1 = to / GRID_COLS;
	auto y1 = to % GRID_COLS;
	cout << "From: " << from << "-->" << to << "\t";
	cout << "(" << x0 << ", " << y0 << ") --> (" << x1 << ", " << y1 << ")" << endl;
	auto dist = sqrt((y1 - y0) * (y1 - y0) + (x1 - x0) * (x1 - x0)) * GRID_CELL_SIZE_METERS;
	return dist;
}

inline bool MosheSearchModule::isActive(boost::shared_ptr<Enemy> enemy) { return std::find(activeEnemies_.begin(), activeEnemies_.end(), enemy) != activeEnemies_.end(); }

void MosheSearchModule::printStatus()
{
	cout << "Enemies: ";
	for (size_t i = 0; i < enemies_.size(); i++)
		cout << "\t" << i << "\t" << enemies_[i]->getCurrentSectionId() << "\t" << enemies_[i]->getCurrentCellId();
	cout << endl;

	cout << "Cells: ";
	for (auto pair : nonZeroProbCells__)
		cout << "\t" << pair.first << "\t" << pair.second << endl;
	cout << endl;
}

inline bool IsContains(const vector<cell_id_t>& items, const cell_id_t& item) { return std::find(items.begin(), items.end(), item) != items.end(); }

vector<cell_id_t> MosheSearchModule::performDetection(const vector<cell_id_t>& detectCellIds, const double& detectionProbability)
{
	vector<cell_id_t> detectedEnemiesCells;
	for (int i = activeEnemies_.size() - 1; i >= 0; i--)
	{
		auto enemy = activeEnemies_[i];
		auto enemyCellId = sectionsMap_[enemy->getCurrentSectionId()]->getSectionCellId();


		auto isEnemyCellDetected = IsContains(detectCellIds, enemyCellId);

		//cerr << detectionProbability << endl;
		double randNum = (double)(rand() % 10000) / 10000;
		if (isEnemyCellDetected && randNum < detectionProbability)
		{
			if (VERBOSE_LOG) cout << "Detected: " << enemy->id_ << "\tCell:\t" << enemyCellId << endl;
			// enemy was detected with prob detectionProbability and is no longer an active enemy
			activeEnemies_.erase(activeEnemies_.begin() + i);
			for (auto e : enemies_)
			{
				if (e->id_ == enemy->id_) e->setIsDetectedTrue();
			}

			enemyEntitiesMap_.erase(enemy->id_);
			detectedEnemiesCells.push_back(enemyCellId);
		}
	}

	// If there are enemies left, we need to update the probablities of the cells
	auto allDetected = activeEnemies_.size() == 0;
	if (!allDetected)
	{
		for (auto enemyEntity : enemyEntitiesMap_)
		{
			auto enemy_entity = enemyEntity.second;
			enemy_entity->updateProbabilitiesAfterDetection(detectCellIds, detectionProbability);
		}

		UpdateCellProbablities();
	}

	return detectedEnemiesCells;
}

vector<string> MosheSearchModule::getPathsListFromDirectory(int enemyCount)
{
	int maxNum = 4;
	// string dirPath = SimulationManager::getDataDirPath() + "general_paths/path";
	string dirPath = SimulationManager::getDataDirPath() + "splited_students/students_merged_all/";
	std:vector<int> numFiles;
	for (int i = 0; i < maxNum; i++)
	{
		numFiles.push_back(i);
	}

	std::random_shuffle(numFiles.begin(), numFiles.end());

	vector<string> paths;

	for(int i = 0; i < enemyCount; i++)
	{
		auto it1 = std::next(numFiles.begin(), i);

		string random = std::to_string(it1[0]);

		std::stringstream ss;
		ss << dirPath << random << ".txt";

		paths.push_back(ss.str());
	}
	
	return paths;
}

vector<vector<section_id_t> > MosheSearchModule::LoadPaths(const vector<int>& enemyPaths, int enemyCount) {

	//vector<string> pathsList = getPathsListFromDirectory(enemyCount);
	// string pathsDir = SimulationManager::getDataDirPath() + "general_paths/path";
	string pathsDir = SimulationManager::getDataDirPath() + "splited_students/students_merged_all/";

	vector<string> pathsList;

	// for (int i = 0; i < enemyCount; i++) {
	
	for (int pathNumber : enemyPaths) {

		// cerr << "Loading path " << pathNumber << endl;

		pathsList.push_back(pathsDir + std::to_string(pathNumber) + ".txt");
	}
	
	vector<vector<section_id_t> > returnValue;

	for (string pathFile : pathsList) {

		ifstream input(pathFile.c_str());
		string line;

		vector<section_id_t> path;

		while (getline(input, line)) {
			
			try {
				path.push_back(boost::lexical_cast<int>(line));
			}
			catch(...){}
		}
		
		input.close();
		returnValue.push_back(path);
	}

	return returnValue;
}

void MosheSearchModule::setPenetrationPoint(const vector<section_id_t> sectionIdInitVector, const vector<section_id_t> sectionIdGoalVector, const double& cellChooseProbThreshold, char algorithmType, const vector<int>& enemyPaths)
{
	// delete old enemy
	for (auto enemy : enemies_) {
		enemy.reset();
	}
	enemies_.clear();
	activeEnemies_.clear();
	// Reset probabilities
	for (auto pair : sectionsMap_)
	{
		pair.second->setProbability(0.0);
		pair.second->setTempProb(0.0);
	}
	enemyEntitiesMap_.clear();
	// Create enemyCount_ enemies at point
	if (!enemyPaths.empty()) {//permutations

		vector<vector<section_id_t> > paths = LoadPaths(enemyPaths, enemyCount_);

		if (paths.size() < enemyCount_) {

			cerr << " Error: paths.size() = " << paths.size() << " is less than enemyCount_ = " << enemyCount_ << endl;
			return;
		}

		for (size_t i = 0; i < enemyCount_; i++) {
			
			enemyEntitiesMap_[i].reset();
			enemyEntitiesMap_[i] = boost::shared_ptr<EnemyEntity>(new EnemyEntity(i, paths.at(i).at(0), sectionsMap_, algorithmType));

			auto enemy = boost::shared_ptr<Enemy>(new Enemy(i, sectionsMap_, paths.at(i)));
			enemy->gotoGoal();
			activeEnemies_.push_back(enemy);
			enemies_.push_back(enemy);
		}

//Random old students:
		// vector<vector<section_id_t> > paths = LoadPaths(enemyPaths, enemyCount_);

		// if (paths.size() < enemyCount_) {

		// 	cerr << " Error: paths.size() = " << paths.size() << " is less than enemyCount_ = " << enemyCount_ << endl;
		// 	return;
		// }

		// for (size_t i = 0; i < enemyCount_; i++) {
			
		// 	enemyEntitiesMap_[i].reset();
			
		// 	enemyEntitiesMap_[i] = boost::shared_ptr<EnemyEntity>(new EnemyEntity(i, paths.at(i).at(0), sectionsMap_, algorithmType));

		// 	auto enemy = boost::shared_ptr<Enemy>(new Enemy(i, sectionsMap_, paths.at(i)));
		// 	enemy->gotoGoal();
		// 	activeEnemies_.push_back(enemy);
		// 	enemies_.push_back(enemy);
		// }
	}
	else {//our

		if (sectionIdInitVector.size() < enemyCount_) {

			cerr << "ERROR: sectionIdInitVector.size() = " << sectionIdInitVector.size() << " when enemyCount_ = " << enemyCount_ << endl;
			
			return;
		}

		for (size_t i = 0; i < enemyCount_; i++) {

			enemyEntitiesMap_[i].reset();

			enemyEntitiesMap_[i] = boost::shared_ptr<EnemyEntity>(new EnemyEntity(i, sectionIdInitVector.at(i), sectionsMap_, algorithmType));
			auto enemy = boost::shared_ptr<Enemy>(new Enemy(i, sectionIdInitVector.at(i), sectionIdGoalVector.at(i), sectionsMap_));
			enemy->gotoGoal();
			activeEnemies_.push_back(enemy);
			enemies_.push_back(enemy);
		}
	}



	// else {
	// 	for (size_t i = 0; i < enemyCount_; i++) {
			
	// 		enemyEntitiesMap_[i].reset();
			
	// 		enemyEntitiesMap_[i] = boost::shared_ptr<EnemyEntity>(new EnemyEntity(i, paths.at(i).at(0), sectionsMap_, algorithmType));

	// 		auto enemy = boost::shared_ptr<Enemy>(new Enemy(i, sectionsMap_, loadpath(rand(1,400)));
	// 		enemy->gotoGoal();
	// 		activeEnemies_.push_back(enemy);
	// 		enemies_.push_back(enemy);
	// 	}
	// }
}

void MosheSearchModule::updateBeliefProbability(double time)
{
	auto stepLength = ENEMY_SPEED * time;

	for (auto enemyNonZeroProbSectionList : enemyEntitiesMap_)
	{
		enemyNonZeroProbSectionList.second->updateEnemyLocationList(stepLength, alphaThr_, minorProb_);
	}

	UpdateCellProbablities();

}

bool MosheSearchModule::isAnyEnemyReachedGoal()
{
	for (auto enemy : activeEnemies_) if (enemy->isReachedTheGoal()) return true;
	return false;
}

/////**** there is no all combinations here, why?*/
void MosheSearchModule::getCombinations(const unordered_set<cell_id_t>& cellIds, int K, vector<vector<cell_id_t>>& combinationsList)
{
	string bitmask(K, 1); // K leading 1's
	bitmask.resize(cellIds.size(), 0); // cellIds.size()-K trailing 0's // print integers and permute bitmask

	do
	{
		vector<cell_id_t> combination;
		int i = 0;
		for (auto probByCellId : cellIds)
		{
			if (bitmask[i]) combination.push_back(probByCellId);
			i++;
		}

		combinationsList.push_back(combination);

	} while (std::prev_permutation(bitmask.begin(), bitmask.end()));
}


double MosheSearchModule::getEntropyGain(map<cell_id_t, double> nonZeroProbCells, const vector<cell_id_t>& permutationCellIds, const double& detectionProbability)
{
	map<cell_id_t, double> probByCellId;

	// Copy current to temp
	for (auto&& pair : nonZeroProbCells) probByCellId[pair.first] = pair.second;

	// In this permutation we assume that the detection process has failed.
	// Therefore, the probablities for an enemy to be in a detecting cell decreases by 1 - detectionProbability
	for (auto cellId : permutationCellIds) {
		if (probByCellId.find(cellId) != probByCellId.end())
		{
			probByCellId[cellId] *= (1.0 - detectionProbability);
		}
		//TODO Mor
		/*else
		{
		probByCellId[cellId] = 0;
		}*/
	}

	// Sum up probablities
	auto probSum = 0.0;
	for (auto pair : probByCellId) probSum += pair.second;

	// Normalize and calc entropy
	auto entropy = 0.0;
	for (auto pair : nonZeroProbCells)
	{
		auto prob = probByCellId[pair.first] /= probSum;
		entropy += (prob == 1 || prob == 0) ? 0 : prob * log10(prob) / log10(2.0);
	}

	// The entropy coefficient is the multiplication (pi) of (1 - DetectionProb * OriginalCellProb)
	auto piOneMinusProbProbDet = 1.0;
	for (auto cellId : permutationCellIds)
	{
		piOneMinusProbProbDet *= (1 - detectionProbability * nonZeroProbCells[cellId]);
	}
	return -piOneMinusProbProbDet * -entropy;
}

bool sortbysec(const tuple<cell_id_t, double>& a,
	const tuple<cell_id_t, double>& b)
{
	return (get<1>(a) > get<1>(b));
}

int MosheSearchModule::FindMaxPerEnemyAndFixResults(const unsigned int& k, double maxThreshold, vector<cell_id_t>& resultCellIds)
{
	int uavCount = k;

	map<cell_id_t, double> tempCellMap;

	//Find the max probability per enemy
	for (auto enemyEntityElement : enemyEntitiesMap_)
	{
		auto enemEntity = enemyEntityElement.second;
		tuple<cell_id_t, double> maxCellPerEnemy = enemEntity->getMaxProbCell();

		if (tempCellMap.find(get<0>(maxCellPerEnemy)) != tempCellMap.end())
		{
			tempCellMap[get<0>(maxCellPerEnemy)] += get<1>(maxCellPerEnemy);
		}
		else
		{
			if (get<1>(maxCellPerEnemy) > maxThreshold)
			{
				tempCellMap[get<0>(maxCellPerEnemy)] = get<1>(maxCellPerEnemy);
			}
		}
	}

	auto tempCellList = vector<tuple<cell_id_t, double>>(tempCellMap.begin(), tempCellMap.end());

	sort(tempCellList.begin(), tempCellList.end(), sortbysec);

	//Add to resultCellIds the cells with the high probability
	for (auto cell : tempCellList)
	{
		resultCellIds.push_back(get<0>(cell));
		nonZeroProbCells_.erase(get<0>(cell));
		uavCount = std::max(0, --uavCount);
		if (uavCount == 0) return uavCount;
	}

	//Edge Cases:

	int j = uavCount;
	//For case that the cells count with probability small than free UAV, so if there is cells in resultCellIds so free UAV get random from them, 
	//otherwise free UAV get random from all grid
	if (nonZeroProbCells_.size() < j)
	{
		bool randomFromAllGrid = resultCellIds.empty();
		for (int i = 0; i < j - nonZeroProbCells_.size(); i++) {

			if (!randomFromAllGrid) {
				int random = rand() % resultCellIds.size();
				resultCellIds.push_back(resultCellIds[random]);
			}
			else {
				resultCellIds.push_back(rand() % (GRID_COLS * GRID_ROWS));
			}

			uavCount = std::max(0, --uavCount);
		}
	}
	if (nonZeroProbCells_.size() == 0) return uavCount;

	//For case the the cells count with probability equal to free UAV count so all UAV get different cell from nonZeroProbCells_
	if (nonZeroProbCells_.size() == uavCount)
	{
		for (cell_id_t cell : nonZeroProbCells_)
		{
			resultCellIds.push_back(cell);
			uavCount = std::max(0, --uavCount);
		}
		return uavCount;
	}

	return uavCount;
}

void MosheSearchModule::Entropy(const unsigned int& k, double maxThreshold, vector<cell_id_t>& resultCellIds, const double& detectionProbability)
{
	resultCellIds.clear();
	int uavCount = FindMaxPerEnemyAndFixResults(k, maxThreshold, resultCellIds);

	if (uavCount == 0) return;

	vector<vector<cell_id_t>> combinations;
	getCombinations(nonZeroProbCells_, uavCount, combinations);

	auto maxEntropyGain = -std::numeric_limits<double>::max();
	unsigned int maxEntropyGainIndex = 0;
	const unsigned int combinationsSize = combinations.size();

	for (unsigned int i = 0; i < combinationsSize; i++)
	{
		auto sumGain = 0.0;
		for (auto enemy_entity : enemyEntitiesMap_)
		{
			sumGain += getEntropyGain(enemy_entity.second->getNonZeroProbCells(), combinations[i], detectionProbability);
		}

		if (sumGain > maxEntropyGain)
		{
			maxEntropyGain = sumGain;
			maxEntropyGainIndex = i;
		}
	}

	for (auto cellId : combinations[maxEntropyGainIndex]) {
		resultCellIds.push_back(cellId);
	}
}

void MosheSearchModule::Entropy_(const unsigned int& k, double maxThreshold, vector<cell_id_t>& resultCellIds, const double& detectionProbability)
{
	resultCellIds.clear();

	int uavCount = k;
	for (auto enemyEntityElement : enemyEntitiesMap_)
	{
		auto enemEntity = enemyEntityElement.second;
		tuple<cell_id_t, double> maxCellPerEnemy = enemEntity->getMaxProbCell();
		if (get<1>(maxCellPerEnemy) > maxThreshold)
		{
			resultCellIds.push_back(get<0>(maxCellPerEnemy));
			nonZeroProbCells_.erase(get<0>(maxCellPerEnemy));
			uavCount = std::max(0, --uavCount);
		}
		if (uavCount == 0) return;
	}

	int j = uavCount;
	if (nonZeroProbCells_.size() < j)
	{
		for (int i = 0; i < j - nonZeroProbCells_.size(); i++) {

			if (!resultCellIds.empty()) {
				int random = rand() % resultCellIds.size();
				resultCellIds.push_back(resultCellIds[random]);
			}
			else {
				resultCellIds.push_back(rand() % (GRID_COLS * GRID_ROWS));
			}

			uavCount = std::max(0, --uavCount);
		}
	}
	if (nonZeroProbCells_.size() == 0) return;
	if (nonZeroProbCells_.size() == uavCount)
	{
		for (cell_id_t cell : nonZeroProbCells_)
		{
			resultCellIds.push_back(cell);
			uavCount = std::max(0, --uavCount);
		}
		return;
	}

	vector<vector<cell_id_t>> combinations;
	getCombinations(nonZeroProbCells_, uavCount, combinations);

	auto maxEntropyGain = -std::numeric_limits<double>::max();
	unsigned int maxEntropyGainIndex = 0;
	const unsigned int combinationsSize = combinations.size();

	for (unsigned int i = 0; i < combinationsSize; i++)
	{
		auto sumGain = 0.0;
		for (auto enemy_entity : enemyEntitiesMap_)
		{
			sumGain += getEntropyGain(enemy_entity.second->getNonZeroProbCells(), combinations[i], detectionProbability);
		}

		if (sumGain > maxEntropyGain)
		{
			maxEntropyGain = sumGain;
			maxEntropyGainIndex = i;
		}
	}

	for (auto cellId : combinations[maxEntropyGainIndex]) {
		resultCellIds.push_back(cellId);
	}

}

inline void MosheSearchModule::Entropy(const unsigned int& k, vector<cell_id_t>& resultsCellIds, const double& detectionProbability) {
	Entropy(k, 1, resultsCellIds, detectionProbability);
}

inline void MosheSearchModule::SCOUT(const unsigned int& k, vector<cell_id_t>& resultsCellIds, const double& detectionProbability, const double& cellChooseProbThreshold) { Entropy(k, cellChooseProbThreshold, resultsCellIds, detectionProbability); }

vector<cell_id_t> MosheSearchModule::getNextCells(const Algo& algo, const unsigned int& k, const double& detectionProbability, const double& cellChooseProbThreshold, bool planB)
{
	if (planB)
	{
		return GetPlanBCells(k);
	}

	vector <cell_id_t> resultCells;
	switch (algo)
	{
	case Algo::SCOUT:
		SCOUT(k, resultCells, detectionProbability, cellChooseProbThreshold);
		break;
	case Algo::OMEGA_MAX:

		if (k < activeEnemies_.size()) {

			Entropy(k, resultCells, detectionProbability);
		}
		else {

			SCOUT(k, resultCells, detectionProbability, cellChooseProbThreshold);
		}
		break;
	case Algo::OMEGA_MIN:

		if (k < activeEnemies_.size()) {

			SCOUT(k, resultCells, detectionProbability, cellChooseProbThreshold);
		}
		else {

			Entropy(k, resultCells, detectionProbability);
		}
		break;
	case Algo::Entropy:
		Entropy(k, resultCells, detectionProbability);
		break;
	case Algo::Rand:
		GetRandProb(k, resultCells);
		break;
	case Algo::MaxProb:
	default:
		getMaxProb(k, cellChooseProbThreshold, resultCells);
		break;
	}
	return resultCells;
}

void MosheSearchModule::GetRandProb(const unsigned int& k, vector<cell_id_t>& resultCells)
{
	resultCells.clear();

	int uavCount = k;
	vector<cell_id_t> nonZeroProbCellsVector;
	for (auto it = nonZeroProbCells_.begin(); it != nonZeroProbCells_.end(); ++it)
	{
		nonZeroProbCellsVector.push_back(*it);
	}

	int j = uavCount;
	for (size_t i = 0; i < j; i++)
	{
		if (nonZeroProbCellsVector.size() < 1) {
			break;
		}

		int randNum = rand() % nonZeroProbCellsVector.size();
		resultCells.push_back(nonZeroProbCellsVector.at(randNum));
		nonZeroProbCellsVector.erase(nonZeroProbCellsVector.begin() + randNum);
		uavCount = std::max(0, --uavCount);
	}

	if (uavCount > 0)
	{
		for (int i = 0; i < uavCount; i++)
		{
			if (!resultCells.empty()) {
				int random = rand() % resultCells.size();
				resultCells.push_back(resultCells[random]);
			}
			else {
				cerr << " ERROR: no items in resultCellIds " << endl;
			}
		}
	}
}

void MosheSearchModule::getMaxProb(const unsigned int& k, double maxThreshold, vector<cell_id_t>& resultCellIds)
{
	resultCellIds.clear();
	int uavCount = FindMaxPerEnemyAndFixResults(k, maxThreshold, resultCellIds);

	if (uavCount == 0) return;
	
	map<cell_id_t, double> generalNonProbCells = GetNonZeroProbCells();

	int j = uavCount;
	for (int i = 0; i < j; i++)
	{
		if (generalNonProbCells.size() < 1) break;
		double maxValue = 0;
		cell_id_t maxProbCell;

		for (auto probCell : generalNonProbCells)
		{
			if (probCell.second > maxValue)
			{
				maxProbCell = probCell.first;
				maxValue = probCell.second;
			}
		}
		resultCellIds.push_back(maxProbCell);
		generalNonProbCells.erase(maxProbCell);
		uavCount = std::max(0, --uavCount);
	}

	//For case that we stayed with free UAV are left 
	if (!resultCellIds.empty()) {
		for (int i = 0; i < uavCount; i++) {
			int random = rand() % resultCellIds.size();
			resultCellIds.push_back(resultCellIds[random]);
		}
	}
	else {
		for (int i = 0; i < uavCount; i++) {
			resultCellIds.push_back(rand() % (GRID_COLS * GRID_ROWS));
		}
	}
}

map<cell_id_t, double>  MosheSearchModule::GetNonZeroProbCells()
{
	map<cell_id_t, double> nonZeroProbCellsMap;

	for (cell_id_t nonZeroProbCell : nonZeroProbCells_)
	{
		nonZeroProbCellsMap[nonZeroProbCell] = 0;
		for (auto enemyEntity : enemyEntitiesMap_)
		{
			auto nonZeroProbCellsMapPerEnemy = enemyEntity.second->getNonZeroProbCells();
			if (nonZeroProbCellsMapPerEnemy.find(nonZeroProbCell) != nonZeroProbCellsMapPerEnemy.end())
			{
				nonZeroProbCellsMap[nonZeroProbCell] += nonZeroProbCellsMapPerEnemy[nonZeroProbCell];
			}
		}
		nonZeroProbCellsMap[nonZeroProbCell] = nonZeroProbCellsMap[nonZeroProbCell] / activeEnemies_.size();
	}

	return nonZeroProbCellsMap;
}

vector<cell_id_t> MosheSearchModule::GetPlanBCells(int uavCount)
{
	vector <cell_id_t> resultCells;
	set<cell_id_t> usedCells;

	for (int i = 0; i < uavCount; i++)
	{
		while (true) {
			int random = rand() % planBCells_.size();

			cell_id_t cell = planBCells_[random];
			if (!(usedCells.find(cell) != usedCells.end())) {
				resultCells.push_back(cell);
				usedCells.insert(cell);
				break;
			}
		}
	}
	return resultCells;
}
