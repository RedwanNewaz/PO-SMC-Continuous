//
// Created by redwan on 3/12/19.
//

#ifndef BSPAPP_TWODPOINTROBOTSMCSETUP_H
#define BSPAPP_TWODPOINTROBOTSMCSETUP_H

#include "Setup/TwoDPointRobotFIRMCPSetup.h"
#include "StatisticalModelChecking.h"
#include "MCTS.h"

class TowDPointRobotSMCSetup: public TwoDPointRobotSetup
{


public:

    TowDPointRobotSMCSetup() : TwoDPointRobotSetup()
    {
        setup_ = false;
    }

    virtual ~TowDPointRobotSMCSetup(void)
    {
    }

    void setup() override {
        /* configure statistical model checker with additional parameters and create a shared pointer
         * then feed this pointer to the MCTS planner class
         */
        using namespace ModelChecking;
        if(!setup_){
            this->loadParameters();
            if(pathToSetupFile_.length() == 0)
            {
                throw ompl::Exception("Path to setup file not set!");
            }

            if(!hasEnvironment() || !hasRobot())
            {
                throw ompl::Exception("Robot/Environment mesh files not setup!");
            }
            // STEP - create model checker

            std::shared_ptr<SMC> smc = std::make_shared<SMC>(pathToSetupFile_);

            auto smc_shared_instance = smc->getPtr();
            // STEP- default planner setup
            ss_->as<SE2BeliefSpace>()->setBounds(inferEnvironmentBounds());

            // STEP - Create an FCL state validity checker and assign to space information
            const ompl::base::StateValidityCheckerPtr &fclSVC = this->allocStateValidityChecker(siF_, getGeometricStateExtractor(), false);
            siF_->setStateValidityChecker(fclSVC);

            // STEP- provide the observation model to the space
            ObservationModelMethod::ObservationModelPointer om(new HeadingBeaconObservationModel(siF_, pathToSetupFile_.c_str()));
            siF_->setObservationModel(om);

            // STEP- Provide the motion model to the space
            // We use the omnidirectional model because collision checking requires SE2
            MotionModelMethod::MotionModelPointer mm(new OmnidirectionalMotionModel(siF_, pathToSetupFile_.c_str()));
            siF_->setMotionModel(mm);

            ompl::control::StatePropagatorPtr prop(ompl::control::StatePropagatorPtr(new OmnidirectionalStatePropagator(siF_)));
            statePropagator_ = prop;
            siF_->setStatePropagator(statePropagator_);
            siF_->setPropagationStepSize(0.1); // this is the duration that a control is applied
            siF_->setStateValidityCheckingResolution(0.005);
            siF_->setMinMaxControlDuration(1,100);

            if(!start_ || goalList_.size() == 0)
            {
                throw ompl::Exception("Start/Goal not set");
            }

            pdef_->setStartAndGoalStates(start_, goalList_[0], 1.0);

            //TODO setup viusalizer

            // STEP- Setup visualizer because it is needed while loading roadmap and visualizing it
            Visualizer::updateSpaceInformation(this->getSpaceInformation());
            Visualizer::updateRenderer(*dynamic_cast<const ompl::app::RigidBodyGeometry*>(this), this->getGeometricStateExtractor());



            // TODO - feed this model checker to MCTS planner
            ompl::base::PlannerPtr plnr(new MCTS(siF_, smc_shared_instance, false));
            planner_ = plnr;
            planner_->setProblemDefinition(pdef_);
            planner_->as<MCTS>()->loadParametersFromFile(pathToSetupFile_.c_str());



        }
        setup_ = true;
    }
    virtual void executeSolution(int choice=0){
        OMPL_INFORM("Executing planner");
        planner_->as<MCTS>()->run();
    }
    virtual void updateEnvironmentMesh(int obindx = 0)
    {
        OMPL_ERROR("Not implemented");

    }

    virtual void saveRoadmap()
    {
        OMPL_ERROR("Not implemented");
    }

};

#endif //BSPAPP_TWODPOINTROBOTSMCSETUP_H
