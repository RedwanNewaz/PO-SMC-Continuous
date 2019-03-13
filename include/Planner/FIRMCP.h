/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Texas A&M University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Texas A&M University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Authors: Sung Kyun Kim, Ali-akbar Agha-mohammadi, Saurav Agarwal */

#ifndef FIRMCP_PLANNER_
#define FIRMCP_PLANNER_

#include "FIRM.h"

class FIRMCP : public FIRM
{

public:

    /** \brief Constructor */
    FIRMCP(const firm::SpaceInformation::SpaceInformationPtr &si, bool debugMode=false);

    virtual ~FIRMCP(void);

    /** \brief Executes the Partially Observable Monte Carlo Planning (POMCP) on top of FIRM graph */
    void executeFeedbackWithPOMCP(void);

    /** \brief Load planner parameters specific to this planner. */
    virtual void loadParametersFromFile(const std::string &pathToFile);


protected:

    /** \brief Generate the POMCP policy */
    virtual Edge generatePOMCPPolicy(const Vertex currentVertex, const FIRM::Vertex goal);

    double pomcpSimulate(const Vertex currentVertex, const int currentDepth, const Edge& selectedEdgePrev, int& collisionDepth);

    double pomcpRollout(const Vertex currentVertex, const int currentDepth, const Edge& selectedEdgePrev, int& collisionDepth, const bool isNewNodeExpanded=false);

    FIRM::Vertex addQVnodeToPOMCPTree(ompl::base::State *state);

    bool expandQnodesOnPOMCPTreeWithApproxCostToGo(const Vertex m, const bool isNewNodeExpanded=false);

    FIRMWeight lazilyAddEdgeToPOMCPTreeWithApproxCost(const FIRM::Vertex a, const FIRM::Vertex b, bool &edgeAdded);

    FIRM::EdgeControllerType& getEdgeControllerOnPOMCPTree(const Edge& edge);

    FIRMWeight addEdgeToPOMCPTreeWithApproxCost(const FIRM::Vertex a, const FIRM::Vertex b, bool &edgeAdded, const Edge* pEdge = NULL);

    FIRMWeight generateEdgeNodeControllerWithApproxCost(const FIRM::Vertex a, const FIRM::Vertex b, EdgeControllerType &edgeController);

    double computeApproxEdgeCost(const FIRM::Vertex a, const FIRM::Vertex b);
    double computeApproxTransitionCost(const FIRM::Vertex a, const FIRM::Vertex b);
    double computeApproxStabilizationCost(const FIRM::Vertex a, const FIRM::Vertex b);

    double getCostToGoWithApproxStabCost(const Vertex vertex);
    bool updateCostToGoWithApproxStabCost(const Vertex current);

    bool executeSimulationFromUpto(const int kStep, const int numSteps, const ompl::base::State *startState, const Edge& selectedEdge, ompl::base::State* endState, double& executionCost);

    bool updateQVnodeBeliefOnPOMCPTree(const Vertex currentVertex, const Vertex selectedChildQnode, const ompl::base::State* evolvedBelief, Vertex& evolvedVertex, const bool reset=false);
    bool updateQVnodeValuesOnPOMCPTree(const Vertex currentVertex, const Vertex selectedChildQnode, const bool executionStatus, const double executionCost, const double selectedChildQVmincosttogo, double& thisQVmincosttogoUpdated, const bool isNewNodeExpanded=false);

    void prunePOMCPTreeFrom(const Vertex rootVertex);
    void prunePOMCPNode(const Vertex rootVertex);

    virtual Edge generateRolloutPolicy(const Vertex currentVertex, const FIRM::Vertex goal);


    /** \brief A table that stores the cost-to-go updated with approximate stabilization cost along the feedback path*/
    std::map <Vertex, double> costToGoWithApproxStabCost_;


    // parameters

    int numPOMCPParticles_;
    int maxPOMCPDepth_;
    int maxFIRMReachDepth_;
    double nSigmaForPOMCPParticle_;

    double cExplorationForSimulate_;

    double cExploitationForRolloutOutOfReach_;
    double cExploitationForRolloutWithinReach_;
    double costToGoRegulatorOutOfReach_;
    double costToGoRegulatorWithinReach_;
    double nEpsilonForRolloutIsReached_;

    double heurPosStepSize_;
    double heurOriStepSize_;
    double heurCovStepSize_;
    double covConvergenceRate_;

    int scaleStabNumSteps_;

    double nEpsilonForQVnodeMerging_;

    int inflationForApproxStabCost_;
};


#endif
