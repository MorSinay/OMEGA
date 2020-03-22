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

#ifndef ENEMY_H
#define ENEMY_H

#include <iostream>
#include <algorithm>
#include "../Structures/Section.h"

#include <limits>

#include "../types.h"
#include "../defines.h"


class Enemy {

public:
    Enemy(const unsigned int& id, const section_id_t& initialSectionId, const section_id_t& goalSectionId, const map<section_id_t, Section*>& sectionsMap);

	Enemy(const unsigned int& id, const map<section_id_t, Section*>& sectionsMap, vector<section_id_t> preparedPath);

    void moveToSection(const section_id_t& sectionId);

	inline section_id_t getCurrentSectionId() const { return currentSectionId_; }

	inline cell_id_t getCurrentCellId() const { return currentCellId_; }

	inline double getCurrentSectionPassedRate() const { return currentSectionPassedRate_; }

    inline bool isReachedTheGoal() const {
        return isReachedTheGoal_;
    }

    void tick(const double& time = 1);
    void gotoGoal();

	unsigned int id_;

	vector<cell_id_t> GetCellsPathToGoal(); 
	vector<cell_id_t> GetCellsPathFromStartToCurrent(cell_id_t currentCellId);

	section_id_t GetgoalCellId() { return sectionsMap_.at(goalSectionId_)->getSectionCellId(); }
	void setIsDetectedTrue() { isDetected_ = true; }
	bool isDetected() { return isDetected_; }

private:
    
    void updateCurrentState();
	float calculatePathToGoal();
	float calculatePreparedPathToGoal(vector<section_id_t> preparedPath);

private:

    const map<section_id_t, Section*>& sectionsMap_;

    // Holds current enemy section
    Section* currentEnemySection_;

    section_id_t currentSectionId_;
	section_id_t goalSectionId_;
    float currentPathDistance_;


    double currentSectionPassedRate_;

    cell_id_t currentCellId_;

    bool isStarted_;
    bool isReachedTheGoal_;
	bool isDetected_ = false;

    float pathDistance_;
    vector<pair<Section*, float /* this section len */ > > pathToGoal_;

	//Dijkstra
	vector<int> shortestPath_;
	vector<vector<int>> graphMatrix_;
	void initDijkstra();
	int minDistance(const int dist[], const bool sptSet[], int vertexNum);
	void createDijkstraPath(const int parent[], const int& j, const int& src);
	void findSolution(const int dist[], const int& n, const int parent[], const int& src, const int& tar);
	void dijkstra(const int& src, const int& tar, const int& vertexNum);
};


#endif /* ENEMY_H */
