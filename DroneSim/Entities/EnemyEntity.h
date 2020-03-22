#ifndef ENEMY_ENTITY_H
#define ENEMY_ENTITY_H
#include <list>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <algorithm>
#include "../Structures/Section.h"


class EnemyEntity
{

public:
	EnemyEntity(const int& id, section_id_t penetrationSection, const map<section_id_t, Section*>& sectionsMap, char algorithmType);
	~EnemyEntity();
	int getEnemyId() { return enemyId_; }
	void updateEnemyLocationList(const double& stepLength, const double& alphaThr, double minorProb);
	void updateCellProbablities();
	
	struct SectionProg
	{
		section_id_t sectionId;
		float inSection;
		double prob;

		SectionProg(section_id_t id, float inTheSection, double probability)
		{
			sectionId = id;
			inSection = inTheSection;
			prob = probability;
		}
	};

	std::list<boost::shared_ptr<SectionProg> > getSectionProgList() { return enemySectionProgList_; }
	void updateProbabilitiesAfterDetection(const vector<cell_id_t>& detectCellIds, const double& detectionProbability);
	map<cell_id_t, double> getNonZeroProbCells() { return nonZeroProbCells_; }
	tuple<cell_id_t, double> getMaxProbCell();

private:
	inline bool IsContains(const vector<cell_id_t>& items, const cell_id_t& item) { return std::find(items.begin(), items.end(), item) != items.end(); }
	const int enemyId_;
	//This list hold all sections that there is a probability that this enemy arrive them(float), with the progress in them(double)
	std::list<boost::shared_ptr<SectionProg> > enemySectionProgList_;
	map<section_id_t, Section*> sectionsMap_;
	map<cell_id_t, double> nonZeroProbCells_;
	map<tuple<section_id_t, int>, boost::shared_ptr<SectionProg> > enemySectionProgMap_;

	//get uniform prob just for prob 0
	void updateProbRecursive(const boost::shared_ptr<SectionProg>  sectionProg, const double& stepLength, const double& minStepLength,
		map<tuple<section_id_t, int>, boost::shared_ptr<SectionProg> >& newEnemySectionProgMap, char algorithmKind, double alphaThr, double minorProb);
	
	void normalizeSectionProgList(std::list<boost::shared_ptr<SectionProg> >& sectionProgList);

	char algorithmType_;
	 
};

#endif /* ENEMY_H */