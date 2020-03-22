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

#include "Drone.h"


Drone::Drone(const cell_id_t& initialCell, int id) {
	setCurrentCellId(initialCell);
	setGoalCellId(initialCell);
	setInNeighbor(false);
	setId(id);
	cout<<"Drone "<<getId()<<"  currentCell: "<<initialCell<<endl;
}
bool Drone::tick()
{
	if(true == isNavigating())
	{
		moveToCell(getGoalCellId());
	}

	return isEnemyDetected();
}
void Drone::moveToCell(const cell_id_t& cellId) {
	bool aboveMe=false,underMe=false,leftToMe=false,rightToMe=false; // is the new cell to move is right/left/up/down from me
	int row = cellId / GRID_COLS;
	int col = cellId % GRID_COLS;
	int curRow = getCurrentCellId() / GRID_COLS;
	int curCol = getCurrentCellId() % GRID_COLS;

	goalCellId_ = cellId;

	if(curRow < row)
	{
		underMe = true;
	}
	if(curRow > row)
	{
		aboveMe = true;
	}
	if(curCol < col)
	{
		rightToMe = true;
	}
	if(curCol > col)
	{
		leftToMe = true;
	}


	//check the combination of the positions flags
	if(underMe && leftToMe)
	{
		setCurrentCellId((GRID_COLS * curRow + curCol)  + GRID_COLS - 1);
		cout<<"Drone "<<getId()<<" Move To Cell: "<<(GRID_COLS * curRow + curCol)  + GRID_COLS - 1<<endl;
		return;
	}
	if(underMe && rightToMe)
	{
		setCurrentCellId((GRID_COLS * curRow + curCol)  + GRID_COLS + 1);
			cout<<"Drone "<<getId()<<" Move To Cell: "<<(GRID_COLS * curRow + curCol)  + GRID_COLS + 1<<endl;
		return;
	}
	if(underMe)
	{
		setCurrentCellId((GRID_COLS * curRow + curCol)  + GRID_COLS);
			cout<<"Drone "<<getId()<<" Move To Cell: "<<(GRID_COLS * curRow + curCol)  + GRID_COLS<<endl;
		return;
	}
	if(aboveMe && leftToMe)
	{
		setCurrentCellId((GRID_COLS * curRow + curCol)  - GRID_COLS - 1);
			cout<<"Drone "<<getId()<<" Move To Cell: "<<(GRID_COLS * curRow + curCol)  - GRID_COLS - 1<<endl;
		return;
	}
	if(aboveMe && rightToMe)
	{
		setCurrentCellId((GRID_COLS * curRow + curCol)  - GRID_COLS + 1);
			cout<<"Drone "<<getId()<<" Move To Cell: "<<(GRID_COLS * curRow + curCol)  - GRID_COLS + 1<<endl;
		return;
	}
	if(aboveMe)
	{
		setCurrentCellId((GRID_COLS * curRow + curCol)  - GRID_COLS);
			cout<<"Drone "<<getId()<<" Move To Cell: "<<(GRID_COLS * curRow + curCol)  - GRID_COLS<<endl;
		return;
	}
	if(leftToMe)
	{
		setCurrentCellId((GRID_COLS * curRow + curCol)  - 1);
			cout<<"Drone "<<getId()<<" Move To Cell: "<<(GRID_COLS * curRow + curCol)  - 1<<endl;
		return;
	}
	if(rightToMe)
	{
		setCurrentCellId((GRID_COLS * curRow + curCol)  + 1);
			cout<<"Drone "<<getId()<<" Move To Cell: "<<(GRID_COLS * curRow + curCol)  + 1<<endl;
		return;
	}


	//we need to update map prob

}

bool Drone::isEnemyDetected() {
   // for(auto itr = enemies.begin(); itr!=enemies.end(); ++itr)
	//{
	if(this->getCurrentCellId() == this->getEnemy()->getCurrentCellId())
	{
		return true;
	}
	//}
	return false;
	
}

