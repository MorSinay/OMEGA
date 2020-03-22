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

#ifndef SECTION_H
#define SECTION_H

#include <stdlib.h>
#include <assert.h>
using namespace std;

#include <vector>
#include <map>

#include "../types.h"
#include "../defines.h"

class Section {

public:
	Section(const section_id_t& sectionId, const cell_id_t& myCellId, const float& sectionLength, const vector<double>& nextSectionsProbs, const vector<section_id_t>& nextSectionsIds);

	section_id_t getLikelyNextSectionId();

	inline section_id_t getSectionId() const {
		return sectionId_;
	}

	inline cell_id_t getSectionCellId() const {
		return myCellId_;
	}

	inline float getSectionLength() const { return sectionLength_; }

	inline vector<section_id_t> getNeighboursIds() { return nextSectionsIds_; }

	inline void setTempProb(const double& value) { sectionNextProbability_ = value; }
	inline double getTempProb() { return sectionNextProbability_; }
	inline void setProbability(const double& value) { probability_ = value; }
	inline double getProbability() { return probability_; }
	inline double copyProbablityFromTemp() { return probability_ = sectionNextProbability_; }

	const inline vector<double> getNextSectionProbs() const { return nextSectionsProbs_; }
	const inline vector<section_id_t> getNextSectionIds() const { return nextSectionsIds_; }

private:
	section_id_t sectionId_;
	cell_id_t myCellId_;
	float sectionLength_;

	double probability_;
	double sectionNextProbability_;

	vector<double> nextSectionsProbs_;
	vector<section_id_t> nextSectionsIds_;


	// todo: make const
	size_t nextSectionsArrSize_;
};

#endif /* SECTION_H */
