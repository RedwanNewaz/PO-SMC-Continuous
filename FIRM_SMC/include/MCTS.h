//
// Created by redwan on 3/12/19.
//

#ifndef BSPAPP_MCTS_H
#define BSPAPP_MCTS_H

#include "StatisticalModelChecking.h"
#include "Planner/FIRM.h"
#include "Visualization/Visualizer.h"
#include "Utils/FIRMUtils.h"


using namespace ModelChecking;
using namespace std;

class MCTS :public FIRM{
public:
    MCTS(const firm::SpaceInformation::SpaceInformationPtr &si, const SMCptr &model_checker_,  bool debugMode = false);

    virtual ~MCTS();

    void loadParametersFromFile(const std::string &pathToFile) override;

    void run();

private:
    SMCptr model_checker_;
    unsigned int minNodes_;
    unsigned int maxNodes_;
    ompl::base::State *kidnappedState_;
protected:
    void SEARCH();
    void SIMULATE();
    void ROLLOUT();
    void BACKUP();

    void loadRoadMapFromFile(const std::string &pathToFile);

};


#endif //BSPAPP_MCTS_H
