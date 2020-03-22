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

#include "Enemy.h"


Enemy::Enemy(const unsigned int& id, const section_id_t& initialSectionId, const section_id_t& goalSectionId, const map<section_id_t, Section*>& sectionsMap) :
	id_(id),
	sectionsMap_(sectionsMap),
	currentSectionId_(initialSectionId),
	goalSectionId_(goalSectionId),
	pathDistance_(0.0),
	currentPathDistance_(0.0),
	isStarted_(false),
	isReachedTheGoal_(false)
{
	if (sectionsMap.end() != sectionsMap.find(initialSectionId) && NULL != sectionsMap.find(initialSectionId)->second) {
		currentCellId_ = sectionsMap.find(initialSectionId)->second->getSectionCellId();
	}
	else {
		currentCellId_ = 0;//TODO:!!
		cerr << "initialSectionId" << initialSectionId << " not found OR sectionsMap.find(initialSectionId)->second is NULL in Enemy::Enemy" << endl;
	}

	try {
		pathDistance_ = calculatePathToGoal();
	} catch (...) {
		cerr << "calculatePathToGoal failed in Enemy::Enemy. " << "initialSectionId" << initialSectionId << endl;
	}
}

Enemy::Enemy(const unsigned int& id, const map<section_id_t, Section*>& sectionsMap, vector<section_id_t> preparedPath) :
	id_(id),
	sectionsMap_(sectionsMap),
	pathDistance_(0.0),
	currentPathDistance_(0.0),
	isStarted_(false),
	isReachedTheGoal_(false)
{
	currentSectionId_ = preparedPath.at(0);
	goalSectionId_ = preparedPath.back();

	if (sectionsMap.end() != sectionsMap.find(currentSectionId_) && NULL != sectionsMap.find(currentSectionId_)->second) {
		currentCellId_ = sectionsMap.find(currentSectionId_)->second->getSectionCellId();
	}
	else {
		currentCellId_ = 0;//TODO:!!
		cerr << "initialSectionId" << currentSectionId_ << " not found OR sectionsMap.find(initialSectionId)->second is NULL in Enemy::Enemy" << endl;
	}

	try {
		pathDistance_ = calculatePreparedPathToGoal(preparedPath);
	}
	catch (...) {
		cerr << "calculatePathToGoal failed in Enemy::Enemy. " << "initialSectionId" << currentSectionId_ << endl;
	}
}

void Enemy::tick(const double& time)
{
	if (!isStarted_ || isReachedTheGoal_ || isDetected_) return;

	currentPathDistance_ += (float)(ENEMY_SPEED * time);

 	isReachedTheGoal_ = (currentPathDistance_ >= pathDistance_);

	updateCurrentState();

	if (VERBOSE_LOG)
		cout << "Enemy: " << currentCellId_ << "  SectionId: " << currentSectionId_ << " Time:" << time << " Dist: " << currentPathDistance_ << endl;
}


void Enemy::gotoGoal() { isStarted_ = true; }

void Enemy::updateCurrentState() {

	assert(!pathToGoal_.empty());

    Section* currentSection = pathToGoal_[0].first;

    float currentSectionDistanceFromStart = 0;

	for (vector<pair<Section*, float> >::iterator it = pathToGoal_.begin(); it != pathToGoal_.end(); ++it) {

		if (it->second >= currentPathDistance_) {
			
			currentSection = it->first;
			currentSectionDistanceFromStart = it->second;

			currentSectionId_ = currentSection->getSectionId();
			currentCellId_ = currentSection->getSectionCellId();

			// TODO: Mor, find in sec -- If is needed:
			currentSectionPassedRate_ = (currentPathDistance_ - currentSectionDistanceFromStart) / currentSection->getSectionLength();
			break;
		}
	}
}

float Enemy::calculatePreparedPathToGoal(vector<section_id_t> preparedPath) {

	float currentDistance = 0.0;
	for (section_id_t sectionId : preparedPath)
	{
		currentDistance += sectionsMap_.at(sectionId)->getSectionLength();
		pathToGoal_.push_back(make_pair(sectionsMap_.at(sectionId), currentDistance));
	}
	return currentDistance;
}

float Enemy::calculatePathToGoal() {
	
	initDijkstra();
	int vv = sectionsMap_.size();
	dijkstra(currentSectionId_, goalSectionId_, sectionsMap_.size());

	float currentDistance = 0.0;
	for (int sectionId : shortestPath_)
	{
		currentDistance += sectionsMap_.at(sectionId)->getSectionLength();
		pathToGoal_.push_back(make_pair(sectionsMap_.at(sectionId), currentDistance));
	}
	return currentDistance;
}

vector<cell_id_t> Enemy::GetCellsPathToGoal() {
	vector<cell_id_t> cellsList;
	
	cell_id_t lastCell = 0;
	for (pair<Section*, float> section : pathToGoal_) {
		if (section.first->getSectionCellId() != lastCell) {
			cellsList.push_back(section.first->getSectionCellId());
		}
		lastCell = section.first->getSectionCellId();
	}

	return cellsList;
}

vector<cell_id_t> Enemy::GetCellsPathFromStartToCurrent(cell_id_t currentCellId) {
	vector<cell_id_t> cellsList;

	cell_id_t lastCell = 0;
	for (pair<Section*, float> section : pathToGoal_) {
		if (section.first->getSectionCellId() != lastCell) {
			cellsList.push_back(section.first->getSectionCellId());
		}
		lastCell = section.first->getSectionCellId();
		if (lastCell == currentCellId) break;
	}

	return cellsList;
}

//Dijkstra Function's
void Enemy::initDijkstra()
{
	//init graphMatrix with zero's
	vector<int> vec;
	for (int col = 0; col < sectionsMap_.size(); col++)
	{
		vec.push_back(0);
	}

	for (int row = 0; row < sectionsMap_.size(); row++)
	{
		graphMatrix_.push_back(vec);
	}

	for (auto itrSections = sectionsMap_.begin(); itrSections != sectionsMap_.end(); ++itrSections)
	{
		for (int i = 0; i < (*itrSections).second->getNextSectionIds().size(); i++)
		{
			graphMatrix_[(*itrSections).second->getSectionId()].at((*itrSections).second->getNextSectionIds().at(i)) = (*itrSections).second->getSectionLength();
		}
	}
}

int Enemy::minDistance(const int dist[], const bool sptSet[], int vertexNum)
{
	// Initialize min value
	int min = std::numeric_limits<int>::max(), min_index;

	for (int v = 0; v < vertexNum; v++) {
		if (sptSet[v] == false && dist[v] <= min) {
			min = dist[v], min_index = v;
		}
	}

	return min_index;
}

// Function to print shortest path from source to j
// using parent array
void Enemy::createDijkstraPath(const int parent[], const int& j, const int& src)
{
	// Base Case : If j is source
	//if (parent[j]==-1)
	if (parent[j] == parent[src])
	{
		return;
	}
	createDijkstraPath(parent, parent[j], src);
	//pathDistance_ += sectionsMap_.at(j)->getSectionLength();
	shortestPath_.push_back(j);
	// printf("%d ", j);
}

// A utility function to print the constructed distance
// array
void Enemy::findSolution(const int dist[], const int& n, const int parent[], const int& src, const int& tar)
{

	// printf("Vertex\t  Distance\tPath");
	// for (int i = 1; i < DEFAULT_VERTEX_NUM; i++)
	// {
	// printf("\n%d -> %d \t\t %d\t\t%d ", src, i, dist[i], src);
	//pathDistance_ += sectionsMap_.at(src)->getSectionLength();
	shortestPath_.push_back(src);
	createDijkstraPath(parent, tar, src);
	//  }
}

// Funtion that implements Dijkstra's single source shortest path
// algorithm for a graph represented using adjacency matrix
// representation

void Enemy::dijkstra(const int& src, const int& tar, const int& vertexNum) {
	shortestPath_.clear();


	int *dist = new int[vertexNum];  // The output array. dist[i] will hold
									 // the shortest distance from src to i

									 // sptSet[i] will true if vertex i is included / in shortest
									 // path tree or shortest distance from src to i is finalized
	bool *sptSet = new bool[vertexNum];

	// Parent array to store shortest path tree
	int *parent = new int[vertexNum];;

	// Initialize all distances as INFINITE and stpSet[] as false
	for (int i = 0; i < vertexNum; i++)
	{
		parent[0] = -1;
		dist[i] = std::numeric_limits<int>::max();
		sptSet[i] = false;
	}

	// Distance of source vertex from itself is always 0
	dist[src] = 0;

	// Find shortest path for all vertices
	for (int count = 0; count < vertexNum - 1; count++)
	{
		// Pick the minimum distance vertex from the set of
		// vertices not yet processed. u is always equal to src
		// in first iteration.
		int u = minDistance(dist, sptSet, vertexNum);

		// Mark the picked vertex as processed
		sptSet[u] = true;

		// Update dist value of the adjacent vertices of the
		// picked vertex.
		for (int v = 0; v < vertexNum; v++) {

			// Update dist[v] only if is not in sptSet, there is
			// an edge from u to v, and total weight of path from
			// src to v through u is smaller than current value of
			// dist[v]
			if (!sptSet[v] && graphMatrix_[u][v] &&
				dist[u] + graphMatrix_[u][v] < dist[v])
			{
				parent[v] = u;
				dist[v] = dist[u] + graphMatrix_[u][v];
			}
		}
	}

	// print the constructed distance array
	findSolution(dist, vertexNum, parent, src, tar);

	if (nullptr != dist) {
		delete dist;
	}
	
	if (nullptr != sptSet) {
		delete sptSet;
	}
	
	if (nullptr != parent) {
		delete parent;
	}
	
}
