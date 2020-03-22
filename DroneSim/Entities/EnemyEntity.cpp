#include "EnemyEntity.h"


EnemyEntity::EnemyEntity(const int& id, section_id_t penetrationSection, const map<section_id_t, Section*>& sectionsMap, 
		char algorithmType) :
enemyId_(id),
sectionsMap_(sectionsMap),
algorithmType_(algorithmType)
{
	boost::shared_ptr<SectionProg> sectionProg = boost::shared_ptr<SectionProg>(new SectionProg(penetrationSection, 0, 1));

	enemySectionProgList_.push_back(sectionProg);
	tuple<section_id_t, int> sectionInSection;
	sectionInSection = make_tuple(penetrationSection, 0);
	enemySectionProgMap_[sectionInSection] = sectionProg;
}

EnemyEntity::~EnemyEntity() {

	// for (auto it = sectionsMap_.begin(); it != sectionsMap_.end(); ++it) {
		
	// 	if (nullptr != it->second) {
	// 		std::cerr << it->second << std::endl;
	// 		delete it->second;
	// 	}
	// }

	// for (auto it = enemySectionProgMap_.begin(); it != enemySectionProgMap_.end(); ++it) {

	// 	if (nullptr != it->second) {
	// 		std::cerr << it->second << std::endl;
	// 		delete it->second;
	// 	}
	// }

	for (auto it = enemySectionProgList_.begin(); it != enemySectionProgList_.end(); ++it) {

		it->reset();
	}
}

void EnemyEntity::normalizeSectionProgList(std::list<boost::shared_ptr<SectionProg>>& sectionProgList)
{
	double meanValue = 0;

}

void EnemyEntity::updateEnemyLocationList(const double& stepLength, const double& alphaThr, double minorProb)
{
	map<int, int> cellSectionMap;
	for (auto element : sectionsMap_)
	{
		if (cellSectionMap.find(element.second->getSectionCellId()) != cellSectionMap.end())
		{
			cellSectionMap[element.second->getSectionCellId()] += 1;
		}
		else
		{
			cellSectionMap[element.second->getSectionCellId()] = 1;
		}
	}

	const double minStepLength = stepLength * ENEMY_WAY_INTERVAL_RATIO;
	map<tuple<section_id_t, int>, boost::shared_ptr<SectionProg> > newEnemySectionProgMap;

	for (auto sectionProgTuple : enemySectionProgMap_)
	{
		boost::shared_ptr<SectionProg> sectionProg = sectionProgTuple.second;
		updateProbRecursive(sectionProg, stepLength, minStepLength, newEnemySectionProgMap, algorithmType_, alphaThr, minorProb);
	}
	enemySectionProgMap_ = newEnemySectionProgMap;
}

void EnemyEntity::updateProbRecursive(const boost::shared_ptr<SectionProg> sectionProg, const double& stepLength, const double& minStepLength,
	map<tuple<section_id_t, int>, boost::shared_ptr<SectionProg> >& newEnemySectionProgMap, char algorithmKind, double alphaThr, double minorProb) {

	float totalSectionLength = sectionsMap_[sectionProg->sectionId]->getSectionLength();
	float inSectionLength = sectionProg->inSection;

	double totalInSectionDif = totalSectionLength - inSectionLength;

	//Don't add new sectionProg for enemy entity with low probability
	if (sectionProg->prob < minorProb || sectionProg->prob != sectionProg->prob)
	{
		return;
	}

	//Stop condition when the enemy stay on the current section(the progress is just into the section)
	if (inSectionLength + stepLength < totalSectionLength)
	{
		tuple<section_id_t, int> sectionInSection;
		sectionInSection = make_tuple(sectionProg->sectionId, inSectionLength + stepLength);
		double untilProb = 0;
		if (newEnemySectionProgMap.find(sectionInSection) != newEnemySectionProgMap.end())
		{
			untilProb = newEnemySectionProgMap[sectionInSection]->prob;
		}
		
		newEnemySectionProgMap[sectionInSection].reset();
		
		newEnemySectionProgMap[sectionInSection] = boost::shared_ptr<SectionProg>(new SectionProg(sectionProg->sectionId, inSectionLength + stepLength, sectionProg->prob + untilProb));
		return;
	}
	
	Section* section = sectionsMap_[sectionProg->sectionId];

	auto nextSectionsProbs = section->getNextSectionProbs();
	auto nextSectionsIds = section->getNextSectionIds();

	float random = (rand() % 1000 + 1) / (float)1000;

	for (size_t i = 0; i < nextSectionsProbs.size(); i++) {

		if (nextSectionsIds.size() == 0) {
			continue;
		}

		auto nextSectionId = nextSectionsIds[i];
		auto nextSectionProb = nextSectionsProbs[i];
		
		switch (algorithmKind)
		{
		case 'a':
			if (nextSectionProb == 0.0) {
				random = (rand() % 1000 + 1) / (float)1000;
				if (random > alphaThr) {
					nextSectionProb = 1.0 / nextSectionsIds.size();
				}
			}
			break;
		case 'm':
			if (random > alphaThr) {
				nextSectionProb = 1.0 / nextSectionsIds.size();
			}
			break;
		case 'h':
			if (random > alphaThr) {
				if (nextSectionProb == 0.0) {
					nextSectionProb = 1.0 / nextSectionsIds.size();
				}
			}
			break;
		}
		if (nextSectionProb == 0) continue;

		boost::shared_ptr<SectionProg> newSectionProg = boost::shared_ptr<SectionProg>(new SectionProg(nextSectionId, 0, sectionProg->prob * nextSectionProb));
		
		updateProbRecursive(newSectionProg, stepLength - totalInSectionDif, minStepLength - totalInSectionDif, newEnemySectionProgMap, algorithmKind, alphaThr, minorProb);

		//Check if the low way that enemy can go is lower then the total section length(minus the in section length)
		if ((inSectionLength + minStepLength < totalSectionLength)&&(minStepLength > 0))
		{
			tuple<section_id_t, int> sectionInSection;
			sectionInSection = make_tuple(sectionProg->sectionId, inSectionLength + stepLength);
			double untilProb = 0;
			if (newEnemySectionProgMap.find(sectionInSection) != newEnemySectionProgMap.end())
			{
				untilProb = newEnemySectionProgMap[sectionInSection]->prob;
			}

			newEnemySectionProgMap[sectionInSection].reset();

			newEnemySectionProgMap[sectionInSection] = boost::shared_ptr<SectionProg>(new SectionProg(sectionProg->sectionId, inSectionLength + stepLength, sectionProg->prob + untilProb));
		}
	}
}

void EnemyEntity::updateProbabilitiesAfterDetection(const vector<cell_id_t>& detectCellIds, const double& detectionProbability)
{
	for (auto section_prog_record : enemySectionProgMap_)
	{
		boost::shared_ptr<SectionProg> section_prog = section_prog_record.second;
		auto section = sectionsMap_[section_prog->sectionId];
		auto prob = section_prog->prob;
		if (IsContains(detectCellIds, section->getSectionCellId()))
		{
			section_prog->prob *= (1 - detectionProbability);
		}
	}

	auto sumProb = 0.0;
	for (auto section_prog_record : enemySectionProgMap_)
	{
		boost::shared_ptr<SectionProg> section_prog = section_prog_record.second;
		sumProb += section_prog->prob;
	}

	for (auto section_prog_record : enemySectionProgMap_)
	{
		boost::shared_ptr<SectionProg> section_prog = section_prog_record.second;
		section_prog->prob = section_prog->prob / sumProb;
	}
}

void EnemyEntity::updateCellProbablities()
{
	auto maxProb = -1.0;
	nonZeroProbCells_.clear();

	// Sum up the probabilities for normelization
	auto probSum = 0.0;
	for (auto sectionProgTuple : enemySectionProgMap_)
	{
		boost::shared_ptr<SectionProg> sectionProg = sectionProgTuple.second;
		probSum += sectionProg->prob;
	}

	for (auto sectionProgTuple : enemySectionProgMap_)
	{
		boost::shared_ptr<SectionProg> sectionProg = sectionProgTuple.second;
		auto section = sectionsMap_[sectionProg->sectionId];

		auto prob = sectionProg->prob / probSum;
		auto cellId = section->getSectionCellId();

		if (nonZeroProbCells_.find(cellId) != nonZeroProbCells_.end())
			nonZeroProbCells_[cellId] += prob;
		else
			nonZeroProbCells_[cellId] = prob;
	}
}

tuple<cell_id_t, double> EnemyEntity::getMaxProbCell()
{
	cell_id_t maxProbCell;
	double maxProb = 0;
	for (auto cell : nonZeroProbCells_)
	{
		if (cell.second > maxProb)
		{
			maxProbCell = cell.first;
			maxProb = cell.second;
		}
	}

	return std::make_tuple(maxProbCell, maxProb);
}