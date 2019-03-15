//
// Created by redwan on 3/12/19.
//

#ifndef BSPAPP_MCTS_H
#define BSPAPP_MCTS_H

#include "StatisticalModelChecking.h"
#include "Planner/FIRM.h"
#include "Visualization/Visualizer.h"
#include "Utils/FIRMUtils.h"
#include <functional>
#include <memory>
#define foreach BOOST_FOREACH

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
    double nEpsilonForQVnodeMerging_;


    ompl::base::State *cstartState;
    ompl::base::State *cendState;
    ompl::base::State *goalState;
    ompl::base::State *tempTrueStateCopy;
    ompl::base::State *kidnappedState_;
protected:
    Vertex SEARCH(Vertex &currentVertex,  const Vertex &targetNode);
    void SIMULATE();
    void ROLLOUT();
    void BACKUP();
    bool EXECUTE(const Edge &e, const Vertex targetNode,  int kStepOfEdgeController);

    void loadRoadMapFromFile(const std::string &pathToFile);
    bool updateQVnodeBeliefOnTree(const ompl::base::State* givenBelief, const ompl::base::State* evolvedBelief,const Vertex selectedChildQnode, Vertex& evolvedVertex, const bool reset);
    void pruneTreeFrom(const Vertex rootVertex);
    void pruneNode(const Vertex rootVertex);

    FIRM::EdgeControllerType& getEdgeControllerOnTree(const Edge& edge);

};


#endif //BSPAPP_MCTS_H
