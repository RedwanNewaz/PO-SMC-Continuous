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
}

MCTS::~MCTS() {

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

void MCTS::run() {
    // STEP Initialization
    bool edgeControllerStatus;
    bool nodeControllerStatus;


    const Vertex start = startM_[0];
    const Vertex goal = goalM_[0];
    Vertex currentVertex = start;
    Vertex targetNode;

    ompl::base::State *cstartState = siF_->allocState();
    ompl::base::State *cendState = siF_->allocState();
    ompl::base::State *goalState = stateProperty_[goal];
    ompl::base::State *tempTrueStateCopy = siF_->allocState();

    // make a copy of ompl variables
    siF_->copyState(cstartState, stateProperty_[start]);
    siF_->setTrueState(stateProperty_[start]);
    siF_->setBelief(stateProperty_[start]);
    // FIXME - add visualizer

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
    auto x=stateProperty_[currentVertex]->as<FIRM::StateType>()->getX();
    auto y=stateProperty_[currentVertex]->as<FIRM::StateType>()->getY();
    auto yaw=stateProperty_[currentVertex]->as<FIRM::StateType>()->getYaw();

    cout<<model_checker_->minCollisionDistance(x, y, yaw)<<endl;





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