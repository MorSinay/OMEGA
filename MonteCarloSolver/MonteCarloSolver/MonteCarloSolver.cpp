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

// MonteCarloSolver.cpp : Defines the entry point for the console application.
//

#include "Console.h"
// #include "Solver.h"
#include <iomanip>      // std::put_time
#include <ctime>   

#include <iostream>
#include <fstream>
#include <string>
#include <time.h>


#include "../../DroneSim/types.h"
#include "../../DroneSim/defines.h"
#include "../../DroneSim/MilitarySimulator.h"
#include "../../DroneSim/SimulationManager.hpp"

#include "../../DroneSim/Agents/MosheAgent/MosheSearchModule.h"


#include <boost/lexical_cast.hpp>
#include <numeric>

#ifdef DRAW_GRID
#include "DrawGrid.h"
#endif

#pragma warning(disable : 4996) //_CRT_SECURE_NO_WARNINGS

string filePrefix_ = "";

static ofstream myfile;
static unsigned int enemyCount, uavCount;

using namespace std;

struct Pparameters {
	unsigned int currentIteration;
	unsigned int amountOfIterations;
	unsigned int captureTime;
	unsigned int enemyCount;
	unsigned int uavCount;
	unsigned int delay;
};

string getCurrentTimeStr() {

	auto mytime = time(NULL);
	auto tm = *std::localtime(&mytime);

	std::stringstream ss;
	ss << put_time(&tm, "%d-%m-%Y %H:%M:%S");
	return ss.str();
}
string getFileIndexDateTime() {

	auto mytime = time(NULL);
	auto tm = *std::localtime(&mytime);

	std::stringstream ss;
	ss << put_time(&tm, "%d_%m_%Y_%H_%M_%S");
	return ss.str();
}
string simulationResultsFilePath(const string& fileSuffix) {
	
	stringstream ss;

	ss << SimulationManager::getResultsDirPath();

	ss << "run_" << filePrefix_ << "_" << fileSuffix << ".csv";

	return ss.str();
}

struct ScenarioData {

	string algoType;
	string isOur;
	double alpha;
	string isTracking;
};

void writeResultsLogFileTitle(const string& fileSuffix) {

	ofstream* simulationResultsFile = new ofstream(simulationResultsFilePath(fileSuffix), ios::out | ios::app);

	*simulationResultsFile << "Algorithm";
	*simulationResultsFile << ",";
	*simulationResultsFile << "simulationStartTime (unique id)";
	*simulationResultsFile << ",";
	*simulationResultsFile << "isLastCycle";
	*simulationResultsFile << ",";
	*simulationResultsFile << "currentIteration / amountOfIterations";
	*simulationResultsFile << ",";
	*simulationResultsFile << "duration";
	*simulationResultsFile << ",";
	*simulationResultsFile << "captureTime";
	*simulationResultsFile << ",";
	*simulationResultsFile << "detectedEnemies";
	*simulationResultsFile << ",";
	*simulationResultsFile << "initialEnemyCount";
	*simulationResultsFile << ",";
	*simulationResultsFile << "currentUavCount";
	*simulationResultsFile << ",";
	*simulationResultsFile << "initialUavCount";
	*simulationResultsFile << ",";
	*simulationResultsFile << "delay";
	*simulationResultsFile << ",";
	*simulationResultsFile << "DETECTION_PROBABILITY";
	*simulationResultsFile << ",";
	*simulationResultsFile << "CELL_CHOOSE_PROB_THRESHOLD";
	*simulationResultsFile << ",";

	*simulationResultsFile << "algoType";
	*simulationResultsFile << ",";
	*simulationResultsFile << "isOur";
	*simulationResultsFile << ",";
	*simulationResultsFile << "alpha";
	*simulationResultsFile << ",";
	*simulationResultsFile << "isTracking";
	*simulationResultsFile << ",";

	*simulationResultsFile << "eventStr";
	*simulationResultsFile << ",";
	*simulationResultsFile << "|,";
	*simulationResultsFile << "system time miliseconds";
	*simulationResultsFile << ",";
	*simulationResultsFile << "current time";
	*simulationResultsFile << "\n";

	simulationResultsFile->close();
	delete simulationResultsFile;
}

void writeResultsShortLogFileTitle(const string& fileSuffix) {

	ofstream* simulationResultsFile = new ofstream(simulationResultsFilePath(fileSuffix), ios::out | ios::app);

	*simulationResultsFile << "Algorithm";
	*simulationResultsFile << ",";
	*simulationResultsFile << "duration";
	*simulationResultsFile << ",";
	*simulationResultsFile << "initialEnemyCount";
	*simulationResultsFile << ",";
	*simulationResultsFile << "initialUavCount";
	*simulationResultsFile << ",";
	*simulationResultsFile << "delay";
	*simulationResultsFile << ",";
	*simulationResultsFile << "DETECTION_PROBABILITY";
	*simulationResultsFile << ",";
	*simulationResultsFile << "CELL_CHOOSE_PROB_THRESHOLD";
	*simulationResultsFile << ",";

	*simulationResultsFile << "algoType";
	*simulationResultsFile << ",";
	*simulationResultsFile << "isOur";
	*simulationResultsFile << ",";
	*simulationResultsFile << "alpha";
	*simulationResultsFile << ",";
	*simulationResultsFile << "isTracking";
	*simulationResultsFile << ",";

	*simulationResultsFile << "success";
	*simulationResultsFile << ",";
	*simulationResultsFile << "current time";
	*simulationResultsFile << "\n";

	simulationResultsFile->close();
	delete simulationResultsFile;
}

void writeResultsToLogFile(string fileSuffix, int algorithmId, unsigned int currentIteration, unsigned int amountOfIterations, unsigned int duration, unsigned int captureTime, unsigned int detectedEnemies, unsigned int initialEnemyCount, unsigned int currentUavCount, unsigned int initialUavCount, unsigned int delay, time_t simulationStartTime, const string& eventStr, vector<cell_id_t> penetrationPoints, vector<cell_id_t> detectedEnemiesCells, const string& isLastCycle, const double& detectionProbability, const double& cellChooseProbThreshold, const ScenarioData& scenarioData) {

	ofstream* simulationResultsFile = new ofstream(simulationResultsFilePath(fileSuffix), ios::out | ios::app);

	switch (algorithmId) {
	case Algo::SCOUT:
		if (cellChooseProbThreshold < 0.99) {
			*simulationResultsFile << "SCOUT";
		}
		else {
			*simulationResultsFile << "Entropy";
		}
		break;
	case Algo::MaxProb:
		*simulationResultsFile << "MaxProb";
		break;
	case Algo::Rand:
		*simulationResultsFile << "Rand";
		break;
	case Algo::Entropy:
		*simulationResultsFile << "Entropy";
		break;
	default:
		*simulationResultsFile << to_string(Algo::SCOUT);
		break;
	}

	*simulationResultsFile << ",";
	*simulationResultsFile << simulationStartTime;
	*simulationResultsFile << ",";
	*simulationResultsFile << isLastCycle;
	*simulationResultsFile << ",";
	*simulationResultsFile << currentIteration << "/" << amountOfIterations;
	*simulationResultsFile << ",";
	*simulationResultsFile << duration;
	*simulationResultsFile << ",";
	*simulationResultsFile << captureTime;
	*simulationResultsFile << ",";
	*simulationResultsFile << detectedEnemies;
	*simulationResultsFile << ",";
	*simulationResultsFile << initialEnemyCount;
	*simulationResultsFile << ",";
	*simulationResultsFile << currentUavCount;
	*simulationResultsFile << ",";
	*simulationResultsFile << initialUavCount;
	*simulationResultsFile << ",";
	*simulationResultsFile << delay;
	*simulationResultsFile << ",";
	*simulationResultsFile << detectionProbability;
	*simulationResultsFile << ",";
	*simulationResultsFile << cellChooseProbThreshold;
	*simulationResultsFile << ",";

	*simulationResultsFile << scenarioData.algoType;
	*simulationResultsFile << ",";
	*simulationResultsFile << scenarioData.isOur;
	*simulationResultsFile << ",";
	*simulationResultsFile << scenarioData.alpha;
	*simulationResultsFile << ",";
	*simulationResultsFile << scenarioData.isTracking;
	*simulationResultsFile << ",";

	*simulationResultsFile << eventStr;
	*simulationResultsFile << "|,";
	*simulationResultsFile << ",";
	*simulationResultsFile << clock();
	*simulationResultsFile << ",";
	*simulationResultsFile << getCurrentTimeStr();

	*simulationResultsFile << ",penetration points -> ";

	for (section_id_t point : penetrationPoints)
	{
		*simulationResultsFile << ",";
		*simulationResultsFile << point;
	}

	if (detectedEnemiesCells.size() > 0)
	{
		*simulationResultsFile << ",enemies detected cells points -> ";

		for (section_id_t point : detectedEnemiesCells)
		{
			*simulationResultsFile << ",";
			*simulationResultsFile << point;
		}
	}
	else
	{
		*simulationResultsFile << ",Enemy reached to goal in cell:";
	}

	*simulationResultsFile << "\n";

	simulationResultsFile->close();
	delete simulationResultsFile;
}

void writeResultsToShortLogFile(string fileSuffix, int algorithmId, unsigned int currentIteration, unsigned int amountOfIterations, unsigned int duration, unsigned int captureTime, unsigned int detectedEnemies, unsigned int initialEnemyCount, unsigned int currentUavCount, unsigned int initialUavCount, unsigned int delay, time_t simulationStartTime, const string& eventStr, vector<cell_id_t> penetrationPoints, vector<cell_id_t> detectedEnemiesCells, const string& isLastCycle, const double& detectionProbability, const double& cellChooseProbThreshold, const ScenarioData& scenarioData) {

	ofstream* simulationResultsFile = new ofstream(simulationResultsFilePath(fileSuffix), ios::out | ios::app);

	switch (algorithmId) {
	case Algo::SCOUT:
		if (cellChooseProbThreshold < 0.99) {
			*simulationResultsFile << "SCOUT";
		}
		else {
			*simulationResultsFile << "Entropy";
		}
		break;
	case Algo::MaxProb:
		*simulationResultsFile << "MaxProb";
		break;
	case Algo::Rand:
		*simulationResultsFile << "Rand";
		break;
	case Algo::Entropy:
		*simulationResultsFile << "Entropy";
		break;
	default:
		*simulationResultsFile << to_string(Algo::SCOUT);
		break;
	}

	*simulationResultsFile << ",";
	*simulationResultsFile << duration;
	*simulationResultsFile << ",";
	*simulationResultsFile << initialEnemyCount;
	*simulationResultsFile << ",";
	*simulationResultsFile << initialUavCount;
	*simulationResultsFile << ",";
	*simulationResultsFile << delay;
	*simulationResultsFile << ",";
	*simulationResultsFile << detectionProbability;
	*simulationResultsFile << ",";
	*simulationResultsFile << cellChooseProbThreshold;
	*simulationResultsFile << ",";

	*simulationResultsFile << scenarioData.algoType;
	*simulationResultsFile << ",";
	*simulationResultsFile << scenarioData.isOur;
	*simulationResultsFile << ",";
	*simulationResultsFile << scenarioData.alpha;
	*simulationResultsFile << ",";
	*simulationResultsFile << scenarioData.isTracking;
	*simulationResultsFile << ",";

	*simulationResultsFile << eventStr;
	*simulationResultsFile << ",";
	*simulationResultsFile << getCurrentTimeStr();

	*simulationResultsFile << "\n";

	simulationResultsFile->close();
	delete simulationResultsFile;
}

void writeSectionsFromCellsIntoFile(map<section_id_t, Section*> sectionMap)
{
	vector<cell_id_t> cellIdVector = { 62,152,242,362,452 };
	vector<section_id_t> sectionIdVector;
	
	for (cell_id_t cell : cellIdVector)
	{
		for (auto section : sectionMap)
		{
			if (section.second->getSectionCellId() == cell)
			{
				sectionIdVector.push_back(section.first);
				break;
			}
		}
	}

	ofstream simulationResultsFile;
	simulationResultsFile.open(SimulationManager::getDataDirPath() + "experience_generate\\file.csv", ios::out | ios::app);

	for (section_id_t section : sectionIdVector)
	{
		simulationResultsFile << section;
		simulationResultsFile << ",";
	}

	simulationResultsFile.close();
	
}

void writeZeroCellsCountToFile(vector<int> counterOfEnemyInZeroProbCell, vector<int> numOfSections, double alpha, double minorProb, string startTime, string endTime)
{
	float averageEnemyInZeroProb = accumulate(counterOfEnemyInZeroProbCell.begin(), counterOfEnemyInZeroProbCell.end(), 0.0) / counterOfEnemyInZeroProbCell.size();

	float averageSections = accumulate(numOfSections.begin(), numOfSections.end(), 0.0) / numOfSections.size();


	ofstream simulationResultsFile;

	simulationResultsFile.open(SimulationManager::getDataDirPath() + "file.csv", ios::out | ios::app);

	simulationResultsFile << alpha;
	simulationResultsFile << ",";
	simulationResultsFile << minorProb;
	simulationResultsFile << ",";
	simulationResultsFile << averageEnemyInZeroProb;
	simulationResultsFile << ",";
	simulationResultsFile << averageSections;
	simulationResultsFile << ",";
	simulationResultsFile << startTime;
	simulationResultsFile << ",";
	simulationResultsFile << endTime;
	simulationResultsFile << "\n";
	for (int element : numOfSections)
	{
		simulationResultsFile << element;
		simulationResultsFile << ",";
	}
	simulationResultsFile << "\n";
	simulationResultsFile << "\n";

	simulationResultsFile.close();
}

MilitarySimulator* LoadSimulator(float alpha)
{
	string path = SimulationManager::getDataDirPath() + "sections_data.csv";

	string bolderCellsFilePath = SimulationManager::getDataDirPath() + "K_DRONE_Grid/BorderCells.txt";
	string citiesCellsFilePath = SimulationManager::getDataDirPath() + "K_DRONE_Grid/CitiesCell.txt";

	auto simulator = MilitarySimulator::simulatorLoader(path, bolderCellsFilePath, citiesCellsFilePath, alpha);
	return simulator;
}

void MCTS()
{
	auto a = .00001;
	auto b = .00001;
	auto c = a*b;
}

void Simulate(Algo algo, const unsigned int& initialUavCount, const unsigned int& initialEnemyCount,
	const unsigned int& delay, const unsigned int& currentIteration, const unsigned int& amountOfIterations,
	time_t startTime, const double& detectionProbability, const double& cellChooseProbThreshold, MilitarySimulator* simulator, char algorithmType, bool isTracking, double alphaThr, double minorProb, const vector<int>& enemyPaths, const vector<int>& goalsVector)//TODO: delay not used yet
{	
	ScenarioData scenarioData;
	scenarioData.algoType = algorithmType;
	scenarioData.isOur = enemyPaths.empty() ? "our" : "students"; // == our
	scenarioData.alpha = 1.0;
	scenarioData.isTracking = isTracking ? "1" : "0";

	auto sectionMap = simulator->getSectionMap();

	auto scout = MosheSearchModule(sectionMap, initialEnemyCount, detectionProbability, simulator->getCellNearCities(), algorithmType, alphaThr, minorProb);
	
	scout.initialize(simulator->GetInitEnemiesPoint(initialEnemyCount), simulator->GetGoalEnemiesPoint(initialEnemyCount, goalsVector), cellChooseProbThreshold, algorithmType, enemyPaths);

	
	auto enemies = scout.getEnemies();

	auto uavCellId = vector<cell_id_t>(initialUavCount, DRONES_INIT_CELL);

	unsigned int currentUavCount = initialUavCount;

	vector<cell_id_t> initPointsVector = simulator->GetInitCellsPenetrationPoints();

	string startTimeStr = getCurrentTimeStr();

	
	// wait until drone reches grid
	scout.tick(delay, TICK);


	vector<int> counterOfEnemyInZeroProbCell;
	vector<int> numOfSections;


	auto duration = delay;

	int prevDetectedEnemies = 0;
	bool planBMode = false;

	while (!scout.isAnyEnemyReachedGoal()) {

		auto zeroProb_cells = scout.GetNonZeroProbCells();
		auto enemies = scout.getEnemies();

		for (auto enemy : enemies)
		{
			if (zeroProb_cells.find(enemy->getCurrentCellId()) == zeroProb_cells.end())
			{
				counterOfEnemyInZeroProbCell.push_back(0);
			}
			else
			{
				counterOfEnemyInZeroProbCell.push_back(1);
			}
		}
		numOfSections.push_back(zeroProb_cells.size());

		//update if enemies detected then uav stay with him
		int detectedEnemies = scout.getEnemies().size() - scout.getActiveEnemies().size();

		if (isTracking) {
			currentUavCount = std::max(0, (int)initialUavCount - detectedEnemies);
			if (currentUavCount == 0) break;

			while (uavCellId.size() > currentUavCount) {

				uavCellId.pop_back();
			}
		}
		
		if (zeroProb_cells.size() == 0) {
			planBMode = true;
		}

		auto nextCellIds = scout.getNextCells(algo, currentUavCount, detectionProbability, cellChooseProbThreshold, planBMode);
		auto dist = 0.0;

		for (size_t i = 0; i < uavCount; i++) {
			dist = max(dist, scout.GetDistance(uavCellId[i], nextCellIds[i]));
		}

		//If the drones not need to move, we are want to give to enemy 10 sec to continue.
		auto timeToCell = dist / UAV_SPEED > 10 ? dist / UAV_SPEED : 10;

		if (timeToCell > 0) {
			duration += timeToCell;
			scout.tick(timeToCell, TICK);
		}
		
#ifdef DRAW_GRID
		//set drawGrid information
		drawGrid->SetParametersAndDraw(scout.GetNonZeroProbCells(), scout.getEnemies(), uavCellId, nextCellIds);
#endif

		uavCellId = nextCellIds;

		//Do actual work
		vector<cell_id_t> detectedEnemiesCells = scout.performDetection(uavCellId, detectionProbability);

		detectedEnemies = scout.getEnemies().size() - scout.getActiveEnemies().size();

		if (detectedEnemies == initialEnemyCount) {

			writeResultsToShortLogFile("short", algo, currentIteration, amountOfIterations, duration, clock() - startTime, detectedEnemies, initialEnemyCount, currentUavCount, initialUavCount, delay, startTime, "1", initPointsVector, detectedEnemiesCells, "1", detectionProbability, cellChooseProbThreshold, scenarioData);
			break;
		}
		
		prevDetectedEnemies = detectedEnemies;
	}


	int detectedEnemies = scout.getEnemies().size() - scout.getActiveEnemies().size();

	if (isTracking) {
		currentUavCount = initialUavCount - detectedEnemies;
	}

	if (scout.isAnyEnemyReachedGoal() || (scout.getActiveEnemies().size() > currentUavCount)) {

		vector<cell_id_t> detectedEnemiesCells;
		// writeResultsToLogFile("full", algo, currentIteration, amountOfIterations, duration, clock() - startTime, detectedEnemies, initialEnemyCount, currentUavCount, initialUavCount, delay, startTime, "OnEnemyArrive", initPointsVector, detectedEnemiesCells, "1", detectionProbability, cellChooseProbThreshold);
		writeResultsToShortLogFile("short", algo, currentIteration, amountOfIterations, duration, clock() - startTime, detectedEnemies, initialEnemyCount, currentUavCount, initialUavCount, delay, startTime, "0", initPointsVector, detectedEnemiesCells, "1", detectionProbability, cellChooseProbThreshold, scenarioData);

#ifdef DRAW_GRID
		drawGrid->DrawConclusion(false);
#endif
	}
	else
	{
#ifdef DRAW_GRID
		drawGrid->DrawConclusion(true);
#endif
	}

	// cout << endl << "======================================================================" << endl;

#ifdef DRAW_GRID
	drawGrid->CloseWindow();
#endif

	return;
}

void runScountAlgorithm(MilitarySimulator* simulatorInstance_p, char algorithmType, const size_t& iterations, const unsigned int& enemyCount, const unsigned int& uavCount, const unsigned int& delay, bool isTracking, const double& alphaThr, double minorProb, const vector<int>& enemyPaths, const vector<int>& goalsVector) {

	int timeSuffix = 1;

	// SCOUT

	double detectionProbability = 0.7;
	double cellChooseProbThreshold = 0.0;

	time_t startTime = time(0) * 1000 + timeSuffix;
	Simulate(Algo::SCOUT, uavCount, enemyCount, delay, 0, iterations, startTime, detectionProbability, cellChooseProbThreshold, simulatorInstance_p, algorithmType, isTracking, alphaThr, minorProb, enemyPaths, goalsVector);
	++timeSuffix;


	// OMEGA_MIN

	detectionProbability = 0.0;

	startTime = time(0) * 1000 + timeSuffix;
	Simulate(Algo::OMEGA_MIN, uavCount, enemyCount, delay, 0, iterations, startTime, detectionProbability, cellChooseProbThreshold, simulatorInstance_p, algorithmType, isTracking, alphaThr, minorProb, enemyPaths, goalsVector);
	++timeSuffix;


	// OMEGA_MAX

	startTime = time(0) * 1000 + timeSuffix;
	Simulate(Algo::OMEGA_MAX, uavCount, enemyCount, delay, 0, iterations, startTime, detectionProbability, cellChooseProbThreshold, simulatorInstance_p, algorithmType, isTracking, alphaThr, minorProb, enemyPaths, goalsVector);
	++timeSuffix;


	//MaxProb

	startTime = time(0) * 1000 + timeSuffix;
	Simulate(Algo::MaxProb, uavCount, enemyCount, delay, 0, iterations, startTime, detectionProbability, cellChooseProbThreshold, simulatorInstance_p, algorithmType, isTracking, alphaThr, minorProb, enemyPaths, goalsVector);
	++timeSuffix;


	//Rand

	startTime = time(0) * 1000 + timeSuffix;
	Simulate(Algo::Rand, uavCount, enemyCount, delay, 0, iterations, startTime, detectionProbability, cellChooseProbThreshold, simulatorInstance_p, algorithmType, isTracking, alphaThr, minorProb, enemyPaths, goalsVector);
	++timeSuffix;



	// Entropy

	double cellChooseProbThreshold = 1.0;

	startTime = time(0) * 1000 + timeSuffix;
	Simulate(Algo::SCOUT, uavCount, enemyCount, delay, 0, iterations, startTime, detectionProbability, cellChooseProbThreshold, simulatorInstance_p, algorithmType, isTracking, alphaThr, minorProb, enemyPaths, goalsVector);
	++timeSuffix;


}

void runMaxAndRandAlgorithms(MilitarySimulator* simulatorInstance_p, char algorithmType, const size_t& iterations, const unsigned int& enemyCount, const unsigned int& uavCount, const unsigned int& delay, bool isTracking, const double& alphaThr, double minorProb, const vector<int>& enemyPaths, const vector<int>& goalsVector) {

	int timeSuffix = 1;

	double detectionProbability = 0.7;

	time_t startTime = time(0) * 1000 + timeSuffix;
	Simulate(Algo::MaxProb, uavCount, enemyCount, delay, 0, iterations, startTime, detectionProbability, 0.0, simulatorInstance_p, algorithmType, isTracking, alphaThr, minorProb, enemyPaths, goalsVector);
	++timeSuffix;


	startTime = time(0) * 1000 + timeSuffix;
	Simulate(Algo::Rand, uavCount, enemyCount, delay, 0, iterations, startTime, detectionProbability, 0.0, simulatorInstance_p, algorithmType, isTracking, alphaThr, minorProb, enemyPaths, goalsVector);
	++timeSuffix;
}

vector<vector<int> > allPossibleSubsets(const vector<int>& paths, int groupSize) {

	vector<vector<int> > subGroups;

	size_t n = paths.size();

	int  count = pow(2, n);

	// The outer for loop will run 2^n times to print all subset .
	// Here variable i will act as a binary counter
	for (int i = 0; i < count; i++) {

		// The inner for loop will run n times , As the maximum number of elements a set can have is n
		// This loop will generate a subset

		vector<int> currentSubset = {};

		for (int j = 0; j < n; j++) {

			// This if condition will check if jth bit in binary representation of  i  is set or not
			// if the value of (i & (1 << j)) is greater than 0 , include arr[j] in the current subset
			// otherwise exclude arr[j]
			if ((i & (1 << j)) > 0) {
				currentSubset.push_back(paths[j]);
			}
		}

		if (currentSubset.size() == groupSize) {
			subGroups.push_back(currentSubset);
		}
	}

	return subGroups;
}

int main(int argc, char** argv) {

	if (argc < 8) {

		cerr << "Required parameters not specified:" << endl;
		cerr << "algorithmType" << endl;
		cerr << "iterations" << endl;
		cerr << "enemyCount" << endl;
		cerr << "uavCount" << endl;
		cerr << "delay" << endl;
		cerr << "alphaThr" << endl;
		cerr << "minorProb" << endl;
		cerr << "isTracking" << endl;
		// cerr << "enemyPaths" << endl;
		Console::ReadLine();
		return 0;
	}

	srand((unsigned int)time(NULL));

	string currentScenarioUniqueId = argv[1];
	
	string algorithmType = argv[2];
	unsigned int iterations = boost::lexical_cast<unsigned int>(argv[3]);
	unsigned int enemyCount = boost::lexical_cast<unsigned int>(argv[4]);
	unsigned int uavCount = boost::lexical_cast<unsigned int>(argv[5]);
	unsigned int delay = boost::lexical_cast<unsigned int>(argv[6]);
	double alphaThr = boost::lexical_cast<double>(argv[7]);
	double minorProb = boost::lexical_cast<double>(argv[8]);

	bool isTracking = false;

	if (strcmp("false", argv[9]) == 0 || strcmp("true", argv[9]) == 0) {
		
		isTracking = strcmp("true", argv[9]) == 0;
	}
	else {

		isTracking = boost::lexical_cast<bool>(argv[9]);
	}

	int useStudentPaths = 0;

		useStudentPaths = boost::lexical_cast<int>(argv[10]);

	cout << "" << endl;
	cout << "*********" << endl;
	cout << "Running the simulation with following parameters:" << endl;
	cout << "currentScenarioUniqueId " << currentScenarioUniqueId << endl;
	cout << "algorithmType " << algorithmType << endl;
	cout << "iterations " << iterations << endl;
	cout << "enemyCount " << enemyCount << endl;
	cout << "uavCount " << uavCount << endl;
	cout << "delay " << delay << endl;
	cout << "alphaThr " << alphaThr << endl;
	cout << "minorProb " << minorProb << endl;
	cout << "isTracking " << isTracking << endl;
	cout << "useStudentPaths " << useStudentPaths << endl;
	cout << "*********" << endl;
	cout << "" << endl;


	// a - Aviad
	// m - Mor
	// h - hybrid
	char algorithmTypeChar = 'h';

	if ("mor" == algorithmType) {

		algorithmTypeChar = 'm';
	}
	else if ("aviad" == algorithmType) {
		
		algorithmTypeChar = 'a';
	}


	MilitarySimulator* simulatorInstance_p = LoadSimulator(1.0);

	for (size_t iter = 1; iter <= iterations; iter++) {

		vector<vector<int> > subGroups;

		if (1 == useStudentPaths) {//TODO

			subGroups.resize(1);

			for (size_t i = 0; i < enemyCount; ++i) {
				
				subGroups[0].push_back(1 + rand() % 10000);
			}
		}
		else {
			subGroups = {{}};
		}


		for (size_t i = 0; i < subGroups.size(); ++i) {


			stringstream ss;

			ss << "__";
			ss << "name_" << currentScenarioUniqueId << "_";
			ss << "" << algorithmType << "_";
			ss << "i" << iterations << "_";
			ss << "e" << enemyCount << "_";
			ss << "u" << uavCount << "_";
			ss << "d" << delay << "_";
			ss << "a" << alphaThr << "_";
			ss << "mp" << minorProb << "_";

			if (subGroups[i].empty()) {

				ss << "our";
			}
			
			ss << "pths";

			ss << "_";

			if (isTracking) {

				ss << "t" << "_";
			}
			else {

				ss << "nt" << "_";
			}

			ss << "_";

			filePrefix_ = ss.str();

			std::ifstream infile(simulationResultsFilePath("short").c_str());

			if (!infile.good()) {

				writeResultsShortLogFileTitle("short");
			}

			vector<int> goalsVector;

			for (size_t enemyNum = 0; enemyNum < enemyCount; ++enemyNum) {

				int randomCity = rand() % simulatorInstance_p->getCitiesSize();

				goalsVector.push_back(randomCity);
			}

			try {
				runScountAlgorithm(simulatorInstance_p, algorithmTypeChar, iterations, enemyCount, uavCount, delay, isTracking, alphaThr, minorProb, subGroups[i], goalsVector);
			}
			catch(const std::exception &e) { cerr << " Exception occured at runnint SCOUT algorithm " << e.what() << endl; }
			catch (...) {
				cerr << " Exception occured at runnint SCOUT algorithm " << endl;
			}
		}//subgroup

	}//iter
	

	return 0;
}
