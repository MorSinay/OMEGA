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

#include "Dijkstra.h"


Dijkstra::Dijkstra(const map<section_id_t, Section*>& sectionsMap):
matrixDimension_(sectionsMap.size()),
sectionsMap_(sectionsMap)
{
    buildMatrix();
}


void Dijkstra::buildMatrix() {
    //m * n is the size of the matrix
    unsigned int m = sectionsMap_.size();
    unsigned int n = sectionsMap_.size();

    //Grow rows by m
    dijkstraGraph_.resize(m);

    for(int i = 0 ; i < m ; ++i) {
        //Grow Columns by n
        dijkstraGraph_[i].resize(n);
    }

    for(int a=0;a<m;a++) {
        for(int b=0;b<n;b++) {
            dijkstraGraph_[a][b]=0;
        }
    }

    int row,col,nh;

    for(row=0;row<m;row++){

        vector<section_id_t> neighbors = sectionsMap_[row]->getNeighboursIds();

        for (vector<section_id_t>::iterator it = neighbors.begin() ; it != neighbors.end(); ++it) {

            dijkstraGraph_[row][*it] = sectionsMap_[row]->getSectionLength();
        }
    }
}


int Dijkstra::minDistance(int dist[], bool sptSet[]) {
	// Initialize min value
	int min = std::numeric_limits<int>::max(), min_index;
 
	for (int v = 0; v < matrixDimension_; v++) {
		if (sptSet[v] == false && dist[v] <= min) {
			min = dist[v], min_index = v;
        }
    }
 
	return min_index;
}
 
// Function to print shortest path from source to j
// using parent array
void Dijkstra::buildThePath(int parent[], int j,int src) {
	if (parent[j]==parent[src]) {
		return;
	}

	buildThePath(parent, parent[j], src);
	shortestPath_.push_back(j);
}
  
// Funtion that implements Dijkstra's single source shortest path
// algorithm for a graph represented using adjacency matrix
// representation
vector<section_id_t> Dijkstra::getShortestPath(int src, int tar) {

	int dist[matrixDimension_];  // The output array. dist[i] will hold
				  // the shortest distance from src to i
 
	// sptSet[i] will true if vertex i is included / in shortest
	// path tree or shortest distance from src to i is finalized
	bool sptSet[matrixDimension_];
 
	// Parent array to store shortest path tree
	int parent[matrixDimension_];
 
	// Initialize all distances as INFINITE and stpSet[] as false
	for (int i = 0; i < matrixDimension_; i++) {
		parent[0] = -1;
		dist[i] = std::numeric_limits<int>::max();
		sptSet[i] = false;
	}
 
	// Distance of source vertex from itself is always 0
	dist[src] = 0;
 
	// Find shortest path for all vertices
	for (int count = 0; count < matrixDimension_-1; count++) {
		// Pick the minimum distance vertex from the set of
		// vertices not yet processed. u is always equal to src
		// in first iteration.
		int u = minDistance(dist, sptSet);
 
		// Mark the picked vertex as processed
		sptSet[u] = true;
 
		// Update dist value of the adjacent vertices of the
		// picked vertex.
		for (int v = 0; v < matrixDimension_; v++) {
 
			// Update dist[v] only if is not in sptSet, there is
			// an edge from u to v, and total weight of path from
			// src to v through u is smaller than current value of
			// dist[v]
			if (!sptSet[v] && dijkstraGraph_[u][v] && dist[u] + dijkstraGraph_[u][v] < dist[v]) {
				parent[v]  = u;
				dist[v] = dist[u] + dijkstraGraph_[u][v];
			}  
        }
	}
 
    shortestPath_.push_back(src);
    buildThePath(parent, tar, src);

    return shortestPath_;
}
