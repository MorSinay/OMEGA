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

#include "DrawGrid.h"	
#include <list>
#include <chrono>
#include <thread>


DrawGrid::DrawGrid(MilitarySimulator* simulator, sf::RenderWindow* window)
{
	window_ = window;
	simulator_ = simulator;
	clearHeatmap();
}

DrawGrid::DrawGrid()
{

}

/*	row - y axis
 *	col - x axis
 */
void DrawGrid::clearHeatmap()
{
	for (int row = 0; row < GRID_ROWS; row++)
	{
		for (int col = 0; col < GRID_COLS; col++)
		{
			heatmapGrid_[col][row] = 0;
		}
	}
}

void DrawGrid::SetParametersAndDraw(map<cell_id_t, double> NonZeroProbCells, vector<Enemy*> enemies, vector<cell_id_t> lastUavsLocation, vector<cell_id_t> currentUavsLocation) {
	if (!DRAW_GRID) return;
	SetCellsPrediction(NonZeroProbCells);
	SetEnemies(enemies);
	SetLastUavsLocation(lastUavsLocation);
	SetCurrentUavsLocation(currentUavsLocation);
	Draw();
}

void DrawGrid::SetCellsPrediction(map<cell_id_t, double> NonZeroProbCells)
{
	clearHeatmap();
	for (tuple<cell_id_t, double> prob_cell : NonZeroProbCells)
	{
		tuple<int, int> indexCell = GetCellIndexes(get<0>(prob_cell));
		heatmapGrid_[get<0>(indexCell)][get<1>(indexCell)] = get<1>(prob_cell);
	}
}

void DrawGrid::SetEnemies(vector<Enemy*> enemies)
{
	enemies_ = enemies;
}

void DrawGrid::SetLastUavsLocation(vector<cell_id_t> lastUavsLocation)
{
	lastUavsLocation_ = lastUavsLocation;
}

void DrawGrid::SetCurrentUavsLocation(vector<cell_id_t> currentUavsLocation)
{
	currentUavsLocation_ = currentUavsLocation;
}

void DrawGrid::Draw()
{
	window_->clear();
	sf::Font font;
	font.loadFromFile(MONOFONTO);

	//Draw grid
	for (int col = 0; col < GRID_COLS; col++)
	{
		for (int row = 0; row < GRID_ROWS; row++)
		{
			sf::RectangleShape rectangle;
			rectangle.setSize(sf::Vector2f(RECTANGLE_WIDTH, RECTENGLE_LENGTH));
			rectangle.setOutlineColor(sf::Color::Black);
			rectangle.setOutlineThickness(lineWidth);
			rectangle.setPosition(col * ROW_EXT, row * COL_EXT);
			rectangle.setFillColor(DetermineColor(col, row));
			window_->draw(rectangle);
		}
	}

	//Draw Enemies 
	for (Enemy* enemy : enemies_)
	{
		//Draw history path
		for (cell_id_t cellId : enemy->GetCellsPathFromStartToCurrent(enemy->getCurrentCellId())) {
			tuple<int, int> indexPoint = GetCellIndexes(cellId);
			sf::CircleShape circleEnemy;
			circleEnemy.setRadius(RECTANGLE_WIDTH / 4);
			circleEnemy.setPosition(get<0>(indexPoint) * ROW_EXT + ROW_EXT / 2, get<1>(indexPoint) * ROW_EXT);
			circleEnemy.setFillColor(sf::Color(128,128,128));
			window_->draw(circleEnemy);

			sf::Text text(std::to_string(enemy->id_ + 1), font, ROW_EXT / 3);
			text.setPosition(get<0>(indexPoint) * ROW_EXT + ROW_EXT / 6 + ROW_EXT / 2, get<1>(indexPoint) * ROW_EXT);
			text.setFillColor(sf::Color::Red);
			window_->draw(text);
		}

		//Draw current location
		tuple<int, int> indexPoint = GetCellIndexes(enemy->getCurrentCellId());

		sf::CircleShape circleEnemy;
		circleEnemy.setRadius(RECTANGLE_WIDTH / 4);
		circleEnemy.setPosition(get<0>(indexPoint) * ROW_EXT, get<1>(indexPoint) * ROW_EXT);
		circleEnemy.setFillColor(sf::Color::Black);
		if (enemy->isDetected()) {
			circleEnemy.setOutlineThickness(ROW_EXT / 9);
			circleEnemy.setOutlineColor(sf::Color::Red);
		}
		window_->draw(circleEnemy);

		sf::Text text(std::to_string(enemy->id_ + 1), font, ROW_EXT / 3);
		text.setPosition(get<0>(indexPoint) * ROW_EXT + ROW_EXT / 6, get<1>(indexPoint) * ROW_EXT);
		text.setFillColor(sf::Color::Red);
		window_->draw(text);

		//Draw cities
		tuple<int, int> indexGoalPoint = GetCellIndexes(enemy->GetgoalCellId());
		sf::CircleShape goalPoint;
		goalPoint.setPosition(get<0>(indexGoalPoint) * ROW_EXT, get<1>(indexGoalPoint) * ROW_EXT);
		goalPoint.setRadius(RECTANGLE_WIDTH / 4);
		goalPoint.setFillColor(sf::Color::Yellow);
		window_->draw(goalPoint);
		
		sf::Text goalText(std::to_string(enemy->id_ + 1), font, ROW_EXT / 3);
		goalText.setPosition(get<0>(indexGoalPoint) * ROW_EXT + ROW_EXT / 6, get<1>(indexGoalPoint) * ROW_EXT);
		goalText.setFillColor(sf::Color::Red);
		window_->draw(goalText);
	}

	int i = 1;
	//Draw last uav location
	for (cell_id_t uav : lastUavsLocation_)
	{
		tuple<int, int> indexPoint = GetCellIndexes(uav);

		sf::RectangleShape rectUav;
		rectUav.setSize(sf::Vector2f(RECTANGLE_WIDTH / 2, RECTENGLE_LENGTH / 2));
		rectUav.setPosition(get<0>(indexPoint) * ROW_EXT + RECTANGLE_WIDTH / 2, get<1>(indexPoint) * COL_EXT + RECTENGLE_LENGTH / 2);
		rectUav.setFillColor(sf::Color::Blue);
		window_->draw(rectUav);

		sf::Text text(std::to_string(i), font, ROW_EXT / 3);
		text.setPosition(get<0>(indexPoint) * ROW_EXT + (RECTANGLE_WIDTH * 3) / 4, get<1>(indexPoint) * COL_EXT + RECTENGLE_LENGTH / 2);
		text.setFillColor(sf::Color::Red);
		window_->draw(text);
		i++;
	}

	i = 1;
	//Draw current uav location
	for (cell_id_t uav : currentUavsLocation_)
	{
		tuple<int, int> indexPoint = GetCellIndexes(uav);

		sf::RectangleShape rectUav;
		rectUav.setSize(sf::Vector2f(RECTANGLE_WIDTH / 2, RECTENGLE_LENGTH / 2));
		rectUav.setPosition(get<0>(indexPoint) * ROW_EXT + RECTANGLE_WIDTH / 2, get<1>(indexPoint) * COL_EXT + RECTENGLE_LENGTH / 2);
		rectUav.setFillColor(sf::Color::Green);
		window_->draw(rectUav);

		sf::Text text(std::to_string(i), font, ROW_EXT / 3);
		text.setPosition(get<0>(indexPoint) * ROW_EXT + (RECTANGLE_WIDTH * 3)/ 4, get<1>(indexPoint) * COL_EXT + RECTENGLE_LENGTH / 2);
		text.setFillColor(sf::Color::Red);
		window_->draw(text);
		i++;
	}


	i = 1;
	for (int row = 0; row < GRID_ROWS; row++)
	{
		for (int col = 0; col < GRID_COLS; col++)
		{
			sf::Text text(std::to_string(i), font, ROW_EXT/3);
			text.setPosition(col * ROW_EXT + ROW_EXT / 4, row * COL_EXT + ROW_EXT / 4);
			text.setFillColor(sf::Color::Black);
			window_->draw(text);
			i++;
		}
	}

	window_->display();
 	std::this_thread::sleep_for(std::chrono::milliseconds(delayMilliseconds));

}

sf::Color DrawGrid::DetermineColor(int x, int y)
{
	sf::Color returnColor;
	if (heatmapGrid_[x][y] == 0)
	{
		returnColor = sf::Color(255, 255, 255);
	}
	else
	{
		int relativeRed = 205 - heatmapGrid_[x][y] * 255;
		if (relativeRed < 0) relativeRed = 0;
		returnColor = sf::Color(255, relativeRed, relativeRed);
	}

	return returnColor;
}

tuple<int, int> DrawGrid::GetCellIndexes(int cellNum)
{
	int raw = cellNum / GRID_COLS;
	int col = cellNum % GRID_COLS - 1;

	return make_tuple(col, raw);
}

void DrawGrid::DrawConclusion(bool IsDronesWin)
{
	string drawString;
	if (IsDronesWin)
	{
		drawString = "DRONES WIN!!!";
	}
	else
	{
		drawString = "ENEMIES WIN!!!";
	}
	sf::Font font;
	font.loadFromFile(MONOFONTO);

	sf::Text text(drawString, font, ROW_EXT * GRID_COLS / 8);
	text.setPosition(ROW_EXT * GRID_COLS / 4, ROW_EXT * GRID_ROWS / 4);
	text.setFillColor(sf::Color::Black);
	window_->draw(text);
	window_->display();
	std::this_thread::sleep_for(std::chrono::milliseconds(delayMilliseconds + 2000));
}

void DrawGrid::CloseWindow()
{
	window_->close();
}
