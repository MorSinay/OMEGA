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

#include "SearchModule.h"
#include "Managers/RoutesLibrary/BGraph.h"
//#include "BGraph.h"

//need to create SearchModel in pathManager
SearchModule::SearchModule(const vector<BRoad>& bRoads,VirtualGrid& vgrid, const unsigned int& numberOfDrones):
numberOfDrones_(numberOfDrones),
vgrid_(vgrid)
{
    for (int Index = 0; Index < bRoads.size() ; Index++)
    {
        for (int j = 0; j < bRoads.at(Index).sections_.size() ; j++)
        {
            junctionDict_.insert(make_pair<double,int>(bRoads.at(Index).sections_.at(j)->sectionId,bRoads.at(Index).sourceJunction_));
        }
    }

    cout << " vgrid_.grid_.size() = " << vgrid_.grid_.size() << endl;

    for (int Index = 0; Index < vgrid_.grid_.size(); Index++) {

        cellsDict_[vgrid_.grid_[Index].id_] = vgrid_.grid_[Index];
    }

    resetJunctions();
}
SearchModule::SearchModule(SearchModule& obj):
vgrid_(obj.vgrid_)
{
    *this=obj;
}
void SearchModule::resetJunctions() {

    initialize();
}

/*System Initialization - reset all probabilities*/
void SearchModule::initialize() {
 cout << "initialize" << endl;

    for(sections_dict_t::iterator it = vgrid_.sectionDict_.begin(); it != vgrid_.sectionDict_.end(); ++it) {
        
        it->second->nextProb_ = 0.0;
        it->second->currProb_ = 0.0;

        junctionDict_[it->first] = -1;
    }
}

/*Reset all the next probabilities*/
void SearchModule::resetNextProbability() {
    for(sections_dict_t::iterator it = vgrid_.sectionDict_.begin(); it != vgrid_.sectionDict_.end(); ++it) {
        it->second->nextProb_ = 0.0;
    }
}

void SearchModule::printCurrProbability() {
    for(sections_dict_t::const_iterator it = vgrid_.sectionDict_.begin(); it != vgrid_.sectionDict_.end(); ++it) {
        cout << it->second->sectionId <<  "[" << it->second->currProb_ << "] ," << endl;
    }
   // cout << "\n" << endl;
}

void SearchModule::printNextProbability() {
    for(sections_dict_t::const_iterator it = vgrid_.sectionDict_.begin(); it != vgrid_.sectionDict_.end(); ++it) {
        cout << it->second->sectionId <<  "[" << it->second->nextProb_ << "] ," << endl;
    }
   // cout << "\n" << endl;
}

void SearchModule::updateProbRecursive(const double& sectionId, const int& junctionId, const double& prob, const double& sectionLength) {

    const double sectionLength_inner = sectionLength;
    
    /*The adversary model includes the option for not moving*/
    AdvModel adversaryModel = getAdversaryModel(sectionId, junctionId);
 
    for (int i = 0; i < adversaryModel.model_.size(); i++) {

        double next_section_id = adversaryModel.model_[i].sectionId_;
        double next_prop = adversaryModel.model_[i].probability_;
        int next_junction_id = adversaryModel.model_[i].junctionId_;

        if (sectionLength_inner + vgrid_.sectionDict_[next_section_id]->length_ >= ADVERSARY_MAX_PATH_LENGTH_IN_TICK_M) {

            // TODO: to look again
            double lengthStopPoint = min(ADVERSARY_MAX_PATH_LENGTH_IN_TICK_M - sectionLength_inner, ADVERSARY_MAX_PATH_LENGTH_IN_TICK_M-ADVERSARY_MIN_PATH_LENGTH_IN_TICK_M);

            if (lengthStopPoint < 0.0) {
                lengthStopPoint = ADVERSARY_MAX_PATH_LENGTH_IN_TICK_M - ADVERSARY_MIN_PATH_LENGTH_IN_TICK_M;
            }
                        
            vgrid_.sectionDict_[next_section_id]->nextProb_ += next_prop * prob * (lengthStopPoint / (ADVERSARY_MAX_PATH_LENGTH_IN_TICK_M - ADVERSARY_MIN_PATH_LENGTH_IN_TICK_M));

            continue;
        }
        else {
            
            updateProbRecursive(next_section_id, next_junction_id, next_prop * prob, sectionLength_inner + vgrid_.sectionDict_[next_section_id]->length_);

            if (sectionLength_inner + vgrid_.sectionDict_[next_section_id]->length_ >= ADVERSARY_MIN_PATH_LENGTH_IN_TICK_M) {

                if (sectionLength_inner > ADVERSARY_MIN_PATH_LENGTH_IN_TICK_M) {
                    vgrid_.sectionDict_[next_section_id]->nextProb_ += next_prop * prob * ((vgrid_.sectionDict_[next_section_id]->length_) / (ADVERSARY_MAX_PATH_LENGTH_IN_TICK_M - ADVERSARY_MIN_PATH_LENGTH_IN_TICK_M));
                }
                else {
                    vgrid_.sectionDict_[next_section_id]->nextProb_ += next_prop * prob * ((sectionLength_inner + vgrid_.sectionDict_[next_section_id]->length_ - ADVERSARY_MIN_PATH_LENGTH_IN_TICK_M)  / (ADVERSARY_MAX_PATH_LENGTH_IN_TICK_M - ADVERSARY_MIN_PATH_LENGTH_IN_TICK_M));
                }
            }
        }
    }
}

AdvModel SearchModule::getAdversaryModel(const double& sectionId, const int& junctionId) {

    boost::shared_ptr<BSection> section = vgrid_.sectionDict_[sectionId];

   /*The adversary model is a tripel of [section.getId(), junctionId, prob]*/
    AdvModel model = AdvModel();

    double curJunctionCountersSum = 0.0;

    if (section->nextSectionsId.size() == 0) {
        cout << "this is a dead end. returning." << endl;
        return model;
    }

    for (int k = 0; k < section->nextSectionsId.size(); ++k) {
        curJunctionCountersSum += vgrid_.sectionDict_[section->nextSectionsId[k]]->enemyPassCounter_;
    }

    if (0.0 == curJunctionCountersSum) {
        cout << " ******************************************************** " << endl;
        cout << " curJunctionCountersSum is 0! " << endl;
        cout << " ******************************************************** " << endl;
        return model;
    }

    size_t numEdges = section->nextSectionsId.size();
    
    for (int i = 0; i < numEdges; i++) {
        
        double childSectionId = section->nextSectionsId[i];

        boost::shared_ptr<BSection> childSection = vgrid_.sectionDict_[childSectionId];

        if (childSection->enemyPassCounter_ == 0) {
            continue;
        }

        model.append(childSectionId, childSection->sourceJunction_, (childSection->enemyPassCounter_) / curJunctionCountersSum);
    }

   /*The probability to stay in the same location*/
    // model.append(section->sectionId, junctionId, 1.0 / numEdges);

    return model;
}

/*Compute the next t+1 probability
    the model we are using now is the Uniform distribution to stay
    in the edge and to move to the neighboring edges*/
void SearchModule::updateNextProbability() {

    resetNextProbability();

    for(sections_dict_t::const_iterator it = vgrid_.sectionDict_.begin(); it != vgrid_.sectionDict_.end(); ++it) {

        if (it->second->currProb_ - MINOR_PROBABILITY < 0.0) {
            continue;
        }

        updateProbRecursive(it->second->sectionId, junctionDict_[it->second->sectionId], it->second->currProb_, 0);
    }
}

/*Apply the probability of t+1 to be the curr probability*/
void SearchModule::setNextProbabilityToCurr() {
    for(sections_dict_t::iterator it = vgrid_.sectionDict_.begin(); it != vgrid_.sectionDict_.end(); ++it) {
        it->second->setNextProb();
    }
}

int SearchModule::setNextProbabilityToCurrTempCheck() {

    int counter = 0;

    for(sections_dict_t::iterator it = vgrid_.sectionDict_.begin(); it != vgrid_.sectionDict_.end(); ++it) {
        counter += it->second->setNextProbTempCheck();
    }
    
    return counter;
}

void SearchModule::printCurrAndNextProbs() {

    for (sections_dict_t::iterator it = vgrid_.sectionDict_.begin(); it != vgrid_.sectionDict_.end(); ++it)
    {
        //curr_sum += it->second->currProb_;
    }
}

void SearchModule::nextP()
{
    updateNextProbability();

    cleanMinorProbabilities();

    setNextProbabilityToCurr();
    
    for (int aa = 0; aa < cellsIdsWithProb_.size(); aa++) {

        double currCellProb = 0.0;
        double nextCellProb = 0.0;
        cellsDict_[cellsIdsWithProb_[aa]].cellProbabilities(currCellProb, nextCellProb);
       // cout << " CellId = " << cellsIdsWithProb_[aa] << " currProb = " << currCellProb << " nextProb = " << nextCellProb << endl;
    }
}

void SearchModule::printSectionsProbability()
{
    for(sections_dict_t::iterator it = vgrid_.sectionDict_.begin(); it != vgrid_.sectionDict_.end(); ++it) {
        if(it->second->currProb_!=0)
        {            
            bool found=false;

            for(int j=0;j<vgrid_.grid_.size() && found==false ;j++)
            {
                for(int k=0;k<vgrid_.grid_.at(j).inCellSections_.size() && found==false ;k++)
                {
                    if(it->first==vgrid_.grid_.at(j).inCellSections_.at(k)->sectionId)
                    {
                        found=true;
                    }
                }
            }
        }   
    }
}

/*Update the cell probability in case we didn't detect the intruder*/
void SearchModule::updateSectionProbabilityList(const vector<double>& sections) {

    // the case we could not find the adversary
    double ni = 1.0;
    double tempSum = 0.0;


    for (int i = 0; i < sections.size(); i++) {
       
        ni -= vgrid_.sectionDict_[sections[i]]->currProb_ * DETECTION_PROBABILITY;//vgrid_.sectionDict_[sections[i]]->currProb_ Values are invalid
    }
    
    for(sections_dict_t::iterator it = vgrid_.sectionDict_.begin(); it != vgrid_.sectionDict_.end(); ++it) {
        // If droneLocation==vehicleLocation than the new probability would be vehicle probability

        if (find(sections.begin(), sections.end(), it->second->sectionId) != sections.end())
        {
            it->second->nextProb_ = it->second->currProb_ * (1.0 - DETECTION_PROBABILITY) / ni;
        }
        else {
            it->second->nextProb_ = it->second->currProb_ / ni;           
        }
        
        tempSum += it->second->nextProb_;

        if (tempSum > 1.0001) {
            cout << "problem. tempSum is bigger than 1.0. tempSum = " << tempSum << endl;
        }
    }

    cleanMinorProbabilities();
}

/*Update the next list of cells probability in case we didn't detect the intruder*/
void SearchModule::updateCellListProbability(const vector<int> cells) {

    vector<double> sections;

    for (int i = 0; i < cells.size(); i++) {

       // cout << " cell in update cell list prob = " << cells[i] << endl;

        for (int j = 0; j < cellsDict_[cells[i]].inCellSections_.size(); j++) {

			if (0.0 == cellsDict_[cells[i]].inCellSections_[j]->currProb_) {
				continue;
			}

            double section_id = cellsDict_[cells[i]].inCellSections_[j]->sectionId;

            if (find(sections.begin(), sections.end(), section_id) == sections.end() ) {
                sections.push_back(section_id);
            }

        }
    }
    updateSectionProbabilityList(sections);
}

/*Compute the entropy*/
double SearchModule::entropy(const bool& currFlag /*= false*/) {

    double ent = 0.0;

    for (int Index = 0; Index < vgrid_.grid_.size(); Index++) {

        double currCellProb = 0.0;
        double nextCellProb = 0.0;

        cellsDict_[vgrid_.grid_[Index].id_].cellProbabilities(currCellProb, nextCellProb);
        
        if (currFlag) {

            if (currCellProb < MINOR_PROBABILITY) {
                continue;
            }
            else if (currCellProb > 1.0 - MINOR_PROBABILITY){
                ent = 0.0;
                break;
            }

            ent += currCellProb * log10(currCellProb) / log10(2.0);
        }
        else {

            if (nextCellProb < MINOR_PROBABILITY) {
                continue;
            }
            else if (nextCellProb > 1.0 - MINOR_PROBABILITY){
                ent = 0.0;
                break;
            }

            ent += nextCellProb * log10(nextCellProb) / log10(2.0);
        }
    }
    return -1.0 * ent;
}

/*Compute the current entropy*/
double SearchModule::currEntropy()
{
    return entropy(true);
}

/*Compute the next probability and return the entropy*/
double SearchModule::newCellsEntropy(const vector<int>& cells) {
    updateCellListProbability(cells);
    return entropy(false);
}

int SearchModule::getMaxProbCellId(const vector<int>& cells, double& maxProbCellProb) {

    maxProbCellProb = -1.0;

    int maxCellId = -1;

    for (int i = 0; i < cells.size(); i++) {
        
        if (maxProbCellProb < cellsDict_[cells[i]].cellCurrProbability()) {
            maxProbCellProb = cellsDict_[cells[i]].cellCurrProbability();
            maxCellId = cells[i];
        }
    }

   // cout << "maxProbCellProb = " << maxProbCellProb << endl;
   // cout << "maxCellId = " << maxCellId << endl;

    return maxCellId;
}

/*Compute the max IG*/
vector<int> SearchModule::maxIG_kDrone(const unsigned int& k, const double& cellChooseProbThreshold) {

    if (k > numberOfDrones_) {
        cout << "Error maxIG_kDrone" << endl;
        return vector<int>();
    }
    
    // Filter cell by their probabilities
    // TODO: filter here 20 cells
    vector<int> cellIds;

    cellIds = cellsIdsWithProb_;
   // cout << "cellIds.size() = " << cellIds.size() << endl;

    //for (int aa = 0; aa < cellIds.size(); aa++) {
       // cout <<"cellIds: "<< cellIds[aa] << endl;
    //}

    vector<vector<int> > combinations;

    getCombinations(cellIds, k , combinations);

    //mor removed debug ++++++++++++++ 
    double maxProbCellProb = 0.0;
    int maxProbCellId = getMaxProbCellId(cellIds, maxProbCellProb);
    bool badCombination = false;

    double minIg = 10000.0;
    vector<int> minIgCellList;

   // cout << "combinations.size() = " << combinations.size() << endl;


    //cout << "  " << endl;
    //cout << "  " << endl;
    //cout << " --- " << endl;

    for (int i = 0; i < combinations.size(); i++) {
        
       if (-1 != maxProbCellId && maxProbCellProb > cellChooseProbThreshold) {
            
            bool found = false;
            
            //cout << "combinations[i].size() = " << combinations[i].size() << endl;
            for (int j = 0; j < combinations[i].size(); j++) {
                if (maxProbCellId == combinations[i][j]) {
                    found = true;
                    break;
                }
            }

            if (false == found) {
                badCombination = true;
            }
        }
        
        if (badCombination) {
            badCombination = false;
            continue;
        }
        
        double notDetectP = 1.0;
        for (int j = 0; j < combinations[i].size(); j++) {

            double currCellProb = 0.0;
            double nextCellProb = 0.0;
            cellsDict_[combinations[i][j]].cellProbabilities(currCellProb, nextCellProb);

           // cout << "Cell " << combinations[i][j] << " currCellProb = " << currCellProb << endl;



            notDetectP *= (1.0 - currCellProb * DETECTION_PROBABILITY);

           // cout << "Cell " << combinations[i][j] << "notDetectP = " << notDetectP << endl;
        }

        double gain = notDetectP * newCellsEntropy(combinations[i]);

      //  cout << " newCellsEntropy(combinations[i]) = " << newCellsEntropy(combinations[i]) << endl;
       // cout << " gain = " << gain << endl;
        
        if (minIg > gain) {
            minIg = gain;
            minIgCellList = combinations[i];
        }

      //  cout << " minIg = " << minIg << endl;
    }

  //  cout << " --- " << endl;
    
   // cout << "  " << endl;
   // cout << "  " << endl;
   // cout << " minIgCellList = " << minIgCellList[0] << endl;
   // cout << "  " << endl;
   // cout << "  " << endl;

    return minIgCellList;
}

std::vector<int> SearchModule::setPenetrationPoint(const double& originalSectionId, const unsigned int& numOfDrones, const int& junctionId, const double& cellChooseProbThreshold) {

    resetNextProbability();

    double sectionId = vgrid_.sectionIdsDict_[originalSectionId];

    vgrid_.sectionDict_[sectionId]->currProb_ = 1.0;

    cellsIdsWithProb_.push_back(vgrid_.sectionDict_[sectionId]->myCellId_);
   // cout << "originalSectionId = " << originalSectionId << endl;
   // cout << "sectionId = " << sectionId << endl;
   // cout << "vgrid_.sectionDict_[sectionId]->myCellId_ = " << vgrid_.sectionDict_[sectionId]->myCellId_ << endl;
   // cout << "cellsIdsWithProb_.size() = " << cellsIdsWithProb_.size() << endl;

    junctionDict_[sectionId] = junctionId;

    return maxIG_kDrone(numOfDrones, cellChooseProbThreshold);
}

// TODO:
void SearchModule::cleanMinorProbabilities() {

    double tempSum = 0.0;

    double max = -1.0;
    double* probOfMaxSection = NULL;

    cellsIdsWithProb_.clear();

    for(sections_dict_t::iterator it = vgrid_.sectionDict_.begin(); it != vgrid_.sectionDict_.end(); ++it) {
        if(it->second->nextProb_ != 0.0) {

            if (it->second->nextProb_ < MINOR_PROBABILITY) {                

            //    cout << "smaller than minor: " << it->second->myCellId_ << " prob: " << it->second->nextProb_ << endl;
               it->second->nextProb_ = 0.0;
            }
            else
            {
                
                it->second->nextProb_ = floor(it->second->nextProb_ * PROB_PRECISION) / PROB_PRECISION;

                tempSum += it->second->nextProb_;

                if (tempSum > 1.0) {
                    cout << "tempSum is bigger than 1 !!" << endl;
                }
                
                if (max < it->second->nextProb_) {
                    max = it->second->nextProb_;
                    probOfMaxSection = &it->second->nextProb_;
                }
                
                if (find(cellsIdsWithProb_.begin(), cellsIdsWithProb_.end(), it->second->myCellId_) == cellsIdsWithProb_.end() ) {
                    cellsIdsWithProb_.push_back(it->second->myCellId_);
                }
            }
        }   
    }

    if (NULL != probOfMaxSection) {
        if (tempSum - 1.0 > 0.0) {

            *probOfMaxSection -= tempSum - 1.0;
        }
        else {

            *probOfMaxSection += 1.0 - tempSum;
        }
    }
    else {
        cout << "Problem with probOfMaxSection" << endl;
    }

}

std::vector<int> SearchModule::updateListOfCells(const vector<int> cellList, const unsigned int& numOfDrones, const double& cellChooseProbThreshold) {

    //cout << " from a " << endl;
    updateCellListProbability(cellList);

    setNextProbabilityToCurr();
	
    return maxIG_kDrone(numOfDrones, cellChooseProbThreshold);
}

void SearchModule::getCombinations(const vector<int>& cellIds, int K, vector<vector<int> >& combinationsList) {

    std::string bitmask(K, 1); // K leading 1's
    bitmask.resize(cellIds.size(), 0); // cellIds.size()-K trailing 0's

    // print integers and permute bitmask
    do {
        vector<int> combination;

        for (int i = 0; i < cellIds.size(); ++i) // [0..N-1] integers
        {
            if (bitmask[i]) {
                combination.push_back(cellIds[i]);
            }
        }

        combinationsList.push_back(combination);

    } while (std::prev_permutation(bitmask.begin(), bitmask.end()));
}
