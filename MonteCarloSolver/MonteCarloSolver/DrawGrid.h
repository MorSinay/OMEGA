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
#include "../../DroneSim/Structures/Section.h"
#include "../../DroneSim/Entities/Enemy.h"
#include<tuple>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Audio.hpp>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <stdlib.h> 
#include <chrono>

namespace sf {
	class RenderWindow;
}

class MilitarySimulator;

class DrawGrid
{
private:
	static const int lineWidth = 1;
	static const int RECTANGLE_WIDTH = 50;
	static const int RECTENGLE_LENGTH = 50;
	static const int delayMilliseconds = 1000;

public:
	static const int ROW_EXT = RECTANGLE_WIDTH + lineWidth * 2;
	static const int COL_EXT = RECTENGLE_LENGTH + lineWidth * 2;

private:
	MilitarySimulator* simulator_;
	int gridSize_;
	double heatmapGrid_[GRID_COLS][GRID_ROWS];
	vector<cell_id_t> lastUavsLocation_;
	vector<cell_id_t> currentUavsLocation_;
	vector<Enemy*> enemies_;
	sf::RenderWindow* window_;

private:
	tuple<int, int> GetCellIndexes(int cell);
	sf::Color DrawGrid::DetermineColor(int x, int y);
	void clearHeatmap();

public:
	DrawGrid(MilitarySimulator* simulate, sf::RenderWindow* window);
	DrawGrid();
	void SetCellsPrediction(map<cell_id_t, double> NonZeroProbCells);
	void SetEnemies(vector<Enemy*> enemies);
	void SetLastUavsLocation(vector<cell_id_t> lastUavsLocation);
	void SetCurrentUavsLocation(vector<cell_id_t> currentUavsLocation);
	void Draw();
	void SetParametersAndDraw(map<cell_id_t, double> NonZeroProbCells, vector<Enemy*> enemies, vector<cell_id_t> lastUavsLocation, vector<cell_id_t> currentUavsLocation);
	void DrawConclusion(bool IsDronesWin);
	void CloseWindow();
};
