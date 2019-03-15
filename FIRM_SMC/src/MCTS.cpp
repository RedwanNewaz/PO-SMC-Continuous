//
// Created by redwan on 3/12/19.
//

#include <MCTS.h>

#include "MCTS.h"

MCTS::MCTS(const firm::SpaceInformation::SpaceInformationPtr &si,  const SMCptr &model_checker_ , bool debugMode) : FIRM(si, debugMode),
                                                                                          model_checker_(
                                                                                                  model_checker_) {
    OMPL_INFORM("Model Checker parameters are");
    cout<<*this->model_checker_<<endl;
    cstartState = siF_->allocState();
    cendState = siF_->allocState();
    tempTrueStateCopy = siF_->allocState();
}

MCTS::~MCTS() {
    // free the memory
    siF_->freeState(cstartState);
    siF_->freeState(cendState);
    siF_->freeState(tempTrueStateCopy);
}

void MCTS::loadParametersFromFile(const std::string &pathToFile) {
    TiXmlDocument doc(pathToFile);
    bool loadOkay = doc.LoadFile();
    if ( !loadOkay )
    {
        printf( "Could not load setup file in planning problem. Error='%s'. Exiting.\n", doc.ErrorDesc() );

        exit( 1 );
    }
    TiXmlNode* node = 0;

    TiXmlElement* itemElement = 0;
    // load planning parameters
    node = doc.FirstChild( "PlanningProblem" );
    assert( node );
    // read planning time
    auto child  = node->FirstChild("FIRMNodes");
    assert( child );

    itemElement = child->ToElement();
    assert( itemElement );

    int minNodeNum = 0;
    itemElement->QueryIntAttribute("minNodes", &minNodeNum) ;
    minNodes_ = minNodeNum;

    int maxNodeNum = 0;
    itemElement->QueryIntAttribute("maxNodes", &maxNodeNum) ;
    maxNodes_ = maxNodeNum;
    maxFIRMNodes_ = maxNodeNum;
    //TODO - double nEpsilonForQVnodeMerging_;

    // STEP - check if use previous roadmap
    // Read the roadmap filename
    child  = node->FirstChild("RoadMap");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );

    int usermap = 0;
    itemElement->QueryIntAttribute("useRoadMap", &usermap);

    std::string tempPathStr;
    itemElement->QueryStringAttribute("roadmapFile", &tempPathStr);
    if(usermap==1)loadRoadMapFromFile(tempPathStr);
}


void MCTS::loadRoadMapFromFile(const std::string &pathToFile)
{
    OMPL_INFORM("Loading RoadMap from file ");
    cout<<pathToFile<<endl;

    std::vector<std::pair<int, arma::colvec> > FIRMNodePosList;
    std::vector<std::pair<int, arma::mat> > FIRMNodeCovarianceList;

    boost::mutex::scoped_lock _(graphMutex_);

    if(FIRMUtils::readFIRMGraphFromXML(pathToFile,  FIRMNodePosList, FIRMNodeCovarianceList , loadedEdgeProperties_))
    {

        loadedRoadmapFromFile_ = true;

        this->setup();

        for(int i = 0; i < FIRMNodePosList.size() ; i++)
        {
            ompl::base::State *newState = siF_->allocState();

            arma::colvec xVec = FIRMNodePosList[i].second;
            arma::mat     cov = FIRMNodeCovarianceList[i].second;

            newState->as<FIRM::StateType>()->setArmaData(xVec);
            newState->as<FIRM::StateType>()->setCovariance(cov);

            //Vertex v = addStateToGraph(siF_->cloneState(newState));

            Vertex m;

            m = boost::add_vertex(g_);

            stateProperty_[m] = newState;

            NodeControllerType nodeController;

            generateNodeController(newState, nodeController); // Generate the node controller

            nodeControllers_[m] = nodeController; // Add it to the list

            // Initialize to its own (dis)connected component.
            disjointSets_.make_set(m);

            nn_->add(m);

            policyGenerator_->addFIRMNodeToObservationGraph(newState);

            addStateToVisualization(newState);

            assert(m==FIRMNodePosList[i].first && "IDS DONT MATCH !!");
        }

        bool unite = true;

        for(int i=0; i<loadedEdgeProperties_.size(); i++)
        {
            EdgeControllerType edgeController;

            Vertex a = loadedEdgeProperties_[i].first.first;
            Vertex b = loadedEdgeProperties_[i].first.second;

            ompl::base::State* startNodeState = siF_->cloneState(stateProperty_[a]);
            ompl::base::State* targetNodeState = stateProperty_[b];

            // Generate the edge controller for given start and end state
            generateEdgeController(startNodeState,targetNodeState,edgeController);

            const FIRMWeight weight = loadedEdgeProperties_[i].second;

            const unsigned int id = maxEdgeID_++;

            const Graph::edge_property_type properties(weight, id);

            // create an edge with the edge weight property
            std::pair<Edge, bool> newEdge = boost::add_edge(a, b, properties, g_);

            edgeControllers_[newEdge.first] = edgeController;

            if(unite)
                uniteComponents(a, b);

            unite = !unite;

            Visualizer::addGraphEdge(stateProperty_[a], stateProperty_[b]);

            // free the memory
            siF_->freeState(startNodeState);
        }

    }
}


void MCTS::run() {
    // STEP Initialization executeFeedbackWithPOMCP



    const Vertex start = startM_[0];
    const Vertex goal = goalM_[0];

    assert((start != goal) && "start and goal is the same");
    Vertex currentVertex = start;
    Vertex targetNode;

    goalState = stateProperty_[goal];
    // make a copy of ompl variables
    siF_->copyState(cstartState, stateProperty_[start]);
    siF_->setTrueState(stateProperty_[start]);
    siF_->setBelief(stateProperty_[start]);

    Visualizer::setMode(Visualizer::VZRDrawingMode::RolloutMode);
    Visualizer::clearRobotPath();
    sendMostLikelyPathToViz(start, goal);

    Visualizer::doSaveVideo(doSaveVideo_);
    siF_->doVelocityLogging(true);
    nodeReachedHistory_.push_back(std::make_pair(currentTimeStep_, numberofNodesReached_) );


    //double averageTimeForRolloutComputation = 0;
    int numberOfRollouts = 0;

    // local variables for robust connection to a desirable (but far) FIRM nodes during rollout
    Edge e = feedback_.at(currentVertex);
    targetNode = boost::target(e, g_);

    // counter of the number of executions of the same edge controller in a row
    int kStepOfEdgeController = 0;
    Edge e_prev;  // do not set this to e from the beginning


    OMPL_INFORM("MCTS: Running MCTS on top of FIRM");
    ompl::base::State* currentBelief = stateProperty_[currentVertex];
    auto getCoordinate = [&](Vertex b){
        auto x=stateProperty_[b]->as<FIRM::StateType>()->getX();
        auto y=stateProperty_[b]->as<FIRM::StateType>()->getY();
        auto yaw=stateProperty_[b]->as<FIRM::StateType>()->getYaw();
        return make_tuple(x,y,yaw);
    };

    float x,y,yaw;
    std::tie(x,y,yaw) = getCoordinate(currentVertex);
    cout<<model_checker_->minCollisionDistance(x, y, yaw)<<endl;

    //STEP Monte carlo simulation

    // if want/do not want to show monte carlo sim
    siF_->showRobotVisualization(ompl::magic::SHOW_MONTE_CARLO);

    do{
        Vertex evolvedVertex = SEARCH(currentVertex, targetNode);
        OMPL_INFORM(" search node %u", evolvedVertex);
        // update currentVertex for next iteration
        Vertex previousVertex = currentVertex;  // backup for POMCP tree pruning
        currentVertex = evolvedVertex;
        // if want/do not want to show monte carlo sim
        siF_->showRobotVisualization(ompl::magic::SHOW_MONTE_CARLO);
        //TODO - implement e = generatePOMCPPolicy(currentVertex, goal);
        targetNode = boost::target(e, g_);

        // if the edge controller of the last execution is being used again now, apply the kStep'th open-loop control of the edge controller
        if (e == e_prev)
        {
            kStepOfEdgeController++;
        }
        else
        {
            kStepOfEdgeController = 0;
            pruneNode(previousVertex);  // since the edge controllers of this node will no longer be used
        }
        e_prev = e;
        numberOfRollouts++;
        if(!EXECUTE(e, targetNode, kStepOfEdgeController)){
            OMPL_ERROR("Cannot Execute action");
            break;
        }

        // check if cendState after execution has reached targetNode, just for logging
        if(stateProperty_[targetNode]->as<FIRM::StateType>()->isReached(cendState))
        {
            OMPL_INFORM("FIRMCP: Reached FIRM Node: %u", targetNode);
            numberofNodesReached_++;
            nodeReachedHistory_.push_back(std::make_pair(currentTimeStep_, numberofNodesReached_) );
        }


    }while(!goalState->as<FIRM::StateType>()->isReached(cstartState, true));





}


Vertex MCTS::SEARCH(Vertex &currentVertex,  const Vertex &targetNode){
    /*
     * DEPENDENCY:  updateQVnodeBeliefOnTree, pruneTreeFrom and pruneNode
     */
    // retrieve the execution result from previous iteration
    ompl::base::State* currentBelief = stateProperty_[currentVertex];  // latest start state that is already executed
    ompl::base::State* evolvedBelief = cstartState;                    // current start state to be executed
    Vertex selectedChildQnode = targetNode;  // target node of last execution
    // update the evolved node belief after adding it if it is new to the POMCP tree
    Vertex evolvedVertex;
    bool reset = true;  // NOTE to clean up noisy beliefs from previous simulations (possibly with a larger nSigmaForPOMCPParticle_)
    if (!updateQVnodeBeliefOnTree(currentBelief, evolvedBelief, selectedChildQnode, evolvedVertex, reset))
    {
        OMPL_ERROR("Failed to updateQVnodeBeliefOnPOMCPTree()!");
        exit(-1);
    }
    // prune the old tree to free the memory
    const std::vector<Vertex>& childQnodes = currentBelief->as<FIRM::StateType>()->getChildQnodes();
    for (const auto& childQnode : childQnodes)
    {
        const std::vector<Vertex>& childQVnodes = currentBelief->as<FIRM::StateType>()->getChildQVnodes(childQnode);
        for (const auto& childQVnode : childQVnodes)
        {
            if (childQVnode != evolvedVertex)
            {
                pruneTreeFrom(childQVnode);
            }
        }
    }

    OMPL_INFORM("FIRMCP: Moved from Vertex %u (%2.3f, %2.3f, %2.3f, %2.6f) to %u (%2.3f, %2.3f, %2.3f, %2.6f)", currentVertex, evolvedVertex,
                stateProperty_[currentVertex]->as<FIRM::StateType>()->getX(),
                stateProperty_[currentVertex]->as<FIRM::StateType>()->getY(),
                stateProperty_[currentVertex]->as<FIRM::StateType>()->getYaw(),
                arma::trace(stateProperty_[currentVertex]->as<FIRM::StateType>()->getCovariance()),
                stateProperty_[evolvedVertex]->as<FIRM::StateType>()->getX(),
                stateProperty_[evolvedVertex]->as<FIRM::StateType>()->getY(),
                stateProperty_[evolvedVertex]->as<FIRM::StateType>()->getYaw(),
                arma::trace(stateProperty_[evolvedVertex]->as<FIRM::StateType>()->getCovariance()));

    return evolvedVertex;
}

bool MCTS::updateQVnodeBeliefOnTree(const ompl::base::State* givenBelief, const ompl::base::State* evolvedBelief,const Vertex selectedChildQnode, Vertex& evolvedVertex, const bool reset){
    ompl::base::State* currentBelief = siF_->cloneState(givenBelief);
    std::map<double, Vertex> reachedChildQVnodesByDistance;  // this is sorted by distance in increasing order
    const std::vector<Vertex>& selectedChildQVnodes = currentBelief->as<FIRM::StateType>()->getChildQVnodes(selectedChildQnode);
    // check for coincident nodes on the POMCP tree
    for (const auto& childQVnode : selectedChildQVnodes)
    {
        // NOTE need to check for both directions since there is no from-to relationship between these selectedChildQVnodes
        // HACK larger/smaller nEpsilonForQVnodeMerging_ for looser/tighter threshold for merging
        if (stateProperty_[childQVnode]->as<FIRM::StateType>()->isReachedWithinNEpsilon(evolvedBelief, nEpsilonForQVnodeMerging_))
        {
            if (evolvedBelief->as<FIRM::StateType>()->isReachedWithinNEpsilon(stateProperty_[childQVnode], nEpsilonForQVnodeMerging_))
            {
                //OMPL_WARN("Coinciding with existing childQVnodes!");

                // compute distance (weighted sum of position and orientation distances)
                // NOTE similarity in covariance is not considered in sorting coincident nodes
                double dist = evolvedBelief->as<FIRM::StateType>()->getStateDistanceTo(stateProperty_[childQVnode]);

                // add this to the candidate list
                reachedChildQVnodesByDistance.insert(std::pair<double, Vertex>(dist, childQVnode));
            }
        }
    }
    // if there are some coincident nodes, merge the evolved belief into the closest one
    if (reachedChildQVnodesByDistance.size() > 0)
    {
        //OMPL_WARN("There are %d reachedChildQVnodes!", reachedChildQVnodes.size());

        // pick the closest coincident node
        Vertex closestChildQVnode = reachedChildQVnodesByDistance.begin()->second;
        evolvedVertex = closestChildQVnode;

        if (!reset)
        {
            // update the matching belief state
            // NOTE this function should be called before incrementing N(h) by +1
            stateProperty_[evolvedVertex]->as<FIRM::StateType>()->mergeBeliefIntoThis(evolvedBelief);
        }
        else
        {
            // NOTE to clean up noisy beliefs from previous simulations (possibly with a larger nSigmaForPOMCPParticle_)
            siF_->copyState(stateProperty_[evolvedVertex], evolvedBelief);
        }
    }
        // otherwise, add a new node to the POMCP tree
    else
    {
        //OMPL_INFORM("A new childQVnode after execution!");


        boost::mutex mutex;
        mutex.lock();
        // add the given belief state to graph as FIRM node
        Vertex m;
        m = boost::add_vertex(g_);
        mutex.unlock();

        stateProperty_[m] = siF_->cloneState(evolvedBelief);
        evolvedVertex = m;


        currentBelief->as<FIRM::StateType>()->addChildQVnode(selectedChildQnode, evolvedVertex);
    }

    return true;

}

void MCTS::pruneTreeFrom(const Vertex rootVertex)
{
    /*
     * DEPENDENCY: pruneNode
     */
    ompl::base::State* rootState = stateProperty_[rootVertex];

    // recursively call prunePOMCPTreeFrom() to destruct the descendent nodes starting from the leaves
    if (rootState->as<FIRM::StateType>()->getChildQexpanded())
    {
        const std::vector<Vertex>& childQnodes = rootState->as<FIRM::StateType>()->getChildQnodes();
        for (const auto& childQnode : childQnodes)
        {
            // const Vertex childQVnode = rootState->as<FIRM::StateType>()->getChildQVnode(childQnode);
            // if (childQVnode != ompl::magic::INVALID_VERTEX_ID)
            // {
            //     prunePOMCPTreeFrom(childQVnode);
            // }
            const std::vector<Vertex>& childQVnodes = rootState->as<FIRM::StateType>()->getChildQVnodes(childQnode);
            for (const auto& childQVnode : childQVnodes)
            {
                pruneTreeFrom(childQVnode);
            }
        }
    }

    // prune this node on POMCP tree
    pruneNode(rootVertex);
}

void MCTS::pruneNode(const Vertex rootVertex)
{
    //const Vertex start = startM_[0];
    if (rootVertex != startM_[0])
    {
        // free the memory of controller
        // NOTE there is no node controller generated for POMCP tree nodes during rollout execution
        foreach(Edge edge, boost::out_edges(rootVertex, g_))
        {
            if (edgeControllers_.find(edge) != edgeControllers_.end())
            {
                edgeControllers_.at(edge).freeSeparatedController();
                //edgeControllers_.at(edge).freeLinearSystems();
                edgeControllers_.erase(edge);
            }
        }

        // free the memory of state
        siF_->freeState(stateProperty_[rootVertex]);

        // remove the node/edges from POMCP tree
        boost::clear_vertex(rootVertex, g_);     // remove all edges from or to rootVertex
        //boost::remove_vertex(rootVertex, g_);  // remove rootVertex  // NOTE commented this to avoid confusion from vertex IDs
        //stateProperty_.erase(rootVertex);
    }
}


bool MCTS::EXECUTE( const Edge &e, const Vertex targetNode,  int kStepOfEdgeController) {

    /** brief:
     * 1) Configure Edge controller and Node controller
     * 2) NodeController will be invoked after executing EdgeController for the given rolloutSteps_ steps
     * 3) Penalize uncertainty (covariance) and path length (time steps) in the cost
     * 4) Update the cstartState for next iteration
     * DEPENDENCY: getEdgeControllerOnTree(e)
     */
    ompl::base::Cost costCov;
    int stepsExecuted = 0;
    int stepsToStop = 0;
    bool edgeControllerStatus;
    bool nodeControllerStatus;

    EdgeControllerType& edgeController = getEdgeControllerOnTree(e);
    edgeController.setSpaceInformation(policyExecutionSI_);
    auto edgeMovement = std::bind(&EdgeControllerType::executeFromUpto, &edgeController,kStepOfEdgeController, rolloutSteps_, std::cref(cstartState),
                                  std::placeholders::_1,std::placeholders::_2,std::placeholders::_3, false);

    NodeControllerType& nodeController = nodeControllers_.at(targetNode);
    nodeController.setSpaceInformation(policyExecutionSI_);
    auto nodeMovement = std::bind(&NodeControllerType::StabilizeUpto, &nodeController,rolloutSteps_,std::cref(cstartState),
                                  std::placeholders::_1,std::placeholders::_2,std::placeholders::_3, false);

    if(!edgeController.isTerminated(cstartState, 0))
        edgeControllerStatus = edgeMovement(std::ref(cendState), std::ref(costCov),std::ref(stepsExecuted));
    else
        nodeControllerStatus = nodeMovement(std::ref(cendState), std::ref(costCov),std::ref(stepsExecuted));

    // NOTE how to penalize uncertainty (covariance) and path length (time steps) in the cost
    //*1) cost = wc * sum(trace(cov_k))  + wt * K  (for k=1,...,K)
    // 2) cost = wc * trace(cov_f)       + wt * K
    // 3) cost = wc * mean(trace(cov_k)) + wt * K
    // 4) cost = wc * sum(trace(cov_k))
    currentTimeStep_ += stepsExecuted;

    executionCostCov_ += costCov.value() - ompl::magic::EDGE_COST_BIAS;    // 1,2,3,4) costCov is actual execution cost but only for covariance penalty (even without weight multiplication)

    executionCost_ = informationCostWeight_*executionCostCov_ + timeCostWeight_*currentTimeStep_;    // 1)
    //executionCost_ = informationCostWeight_*executionCostCov_/(currentTimeStep_==0 ? 1e-10 : currentTimeStep_) + timeCostWeight_*currentTimeStep_;    // 3)
    //executionCost_ = informationCostWeight_*executionCostCov_;    // 4)

    costHistory_.push_back(std::make_tuple(currentTimeStep_, executionCostCov_, executionCost_));


    // this is a secondary (redundant) collision check for the true state
    siF_->getTrueState(tempTrueStateCopy);
    if(!siF_->isValid(tempTrueStateCopy))
    {
        OMPL_INFORM("Robot Collided :(");
        return false;
    }

    // update the cstartState for next iteration
    siF_->copyState(cstartState, cendState);

    return edgeControllerStatus && nodeControllerStatus;

}

FIRM::EdgeControllerType& MCTS::getEdgeControllerOnTree(const Edge& edge)
{
    // TODO - implement addEdgeToPOMCPTreeWithApproxCost
    if (edgeControllers_.find(edge) == edgeControllers_.end())
    {
        Vertex a = boost::source(edge, g_);
        Vertex b = boost::target(edge, g_);
        bool forwardEdgeAdded=false;

        // generate an edge controller
        //FIXME - addEdgeToPOMCPTreeWithApproxCost(a, b, forwardEdgeAdded, &edge);
        if (!forwardEdgeAdded)
        {
            OMPL_ERROR("Failed to addEdgeToPOMCPTreeWithApproxCost()!");
            exit(0);  // for debug
        }
    }

    return edgeControllers_.at(edge);
}

