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

#ifndef DRONE_H
#define DRONE_H

#include <iostream>
#include "../types.h"
#include "../defines.h"
#include "Enemy.h"

using namespace std;

class Drone {

public:
    Drone(const cell_id_t& initialCell, int id);
	
	bool tick();

	inline const cell_id_t & getCurrentCellId() const
	{
		 return currentCellId_;
	}
	inline void setCurrentCellId(const cell_id_t & data)
	{
		 currentCellId_ = data;
	}
	inline const cell_id_t & getGoalCellId() const
	{
		 return goalCellId_;
	}
	inline void setGoalCellId(const cell_id_t & data)
	{
		 goalCellId_ = data;
	}
	inline const bool  getIsDroneArrive() const
	{
		return isArived;
	}
	inline void setIsDroneArrive(const bool & data)
	{
		isArived = data;
	}
	inline const bool  getInNeighbor() const
	{
		return inNeighbor_;
	}
	inline void setInNeighbor(const bool & data)
	{
		inNeighbor_ = data;
	}

	inline bool isNavigating() {
        if(currentCellId_ != goalCellId_)
		{
			return true;
		}
		else
		{
			isArived=true;
			return false;
		}
    }

	inline const Enemy* getEnemy() const
	{
		return this->enemy_;
	}

	inline void setEnemy(Enemy* data)
	{
		this->enemy_ = data;
	}
	

	inline const int& getId() const
	{
		 return this->id_;
	}
	inline void setId(const int data) 
	{
		  this->id_= data;
	}

private:
	
    void moveToCell(const cell_id_t& cellId);
    bool isEnemyDetected();
	bool inNeighbor_;
	Enemy* enemy_;
	int id_;

private:
    cell_id_t currentCellId_;
    cell_id_t goalCellId_;
	bool isArived;
};


#endif /* DRONE_H */
