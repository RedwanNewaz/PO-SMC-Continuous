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

/* Authors: Ali-akbar Agha-mohammadi, Saurav Agarwal, Aditya Mahadevan */

#ifndef FIRM_OMPL_VISUALIZER_H
#define FIRM_OMPL_VISUALIZER_H

#include <X11/X.h>
#include <X11/Xlib.h>
#include <GL/gl.h>
#include <GL/glx.h>
#include <GL/glu.h>
#include <pthread.h>

#ifdef Success
#undef Success
#endif

#include <armadillo>
#include <list>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include "Spaces/SE2BeliefSpace.h"
#include "SpaceInformation/SpaceInformation.h"
#include <omplapp/graphics/RenderGeometry.h>

class Visualizer
{

    public:

        Visualizer(){}

        ~Visualizer(){}

        /** \brief Helps distinguish between states while drawing */
        enum VZRStateType
        {
            TrueState,
            BeliefState,
            GraphNodeState
        };

        enum VZRDrawingMode
        {
            NodeViewMode,
            FeedbackViewMode,
            PRMViewMode,
            FIRMMode,
            FIRMCPMode,
            RolloutMode,
            MultiModalMode
        };

        struct VZRFeedbackEdge
        {
            ompl::base::State *source;
            ompl::base::State *target;
            double cost;
        };

        /** \brief Add the landmarks in the environment */
        static void addLandmarks(const std::vector<arma::colvec>& landmarks)
        {
            boost::mutex::scoped_lock sl(drawMutex_);
            landmarks_.insert(landmarks_.end(), landmarks.begin(), landmarks.end());
        }

        /** \brief Add the states i.e. graph nodes to be drawn*/
        static void addState(const ompl::base::State *state)
        {
            boost::mutex::scoped_lock sl(drawMutex_);
            assert(state);
            states_.push_back(si_->cloneState(state));
        }


        /** \brief Clear the state container */
        static void clearStates()
        {
            boost::mutex::scoped_lock sl(drawMutex_);
            states_.clear();
        }

        /** \brief Add state to belief mode list */
        static void addBeliefMode(ompl::base::State *state)
        {
            boost::mutex::scoped_lock sl(drawMutex_);
            assert(state);
            beliefModes_.push_back(si_->cloneState(state));
        }

        /** \brief Clear belief Modes */
        static void clearBeliefModes()
        {
            boost::mutex::scoped_lock sl(drawMutex_);
            beliefModes_.clear();
        }

        /** \brief Add a Roadmap Graph edge to the visualization */
        static void addGraphEdge(const ompl::base::State *source, const ompl::base::State *target)
        {
            boost::mutex::scoped_lock sl(drawMutex_);

            std::pair<const ompl::base::State*, const ompl::base::State*> edge;

            edge = std::make_pair(si_->cloneState(source),si_->cloneState(target));

            graphEdges_.push_back(edge);
        }

        static void addFeedbackEdge(ompl::base::State *source, ompl::base::State *target, double cost)
        {
            boost::mutex::scoped_lock sl(drawMutex_);
            VZRFeedbackEdge edge;

            edge.source = source;

            edge.target = target;

            edge.cost = cost;

            feedbackEdges_.push_back(edge);
        }

        /** \brief Add a rollout connection to the visualization */
        static void addRolloutConnection(const ompl::base::State *source, const ompl::base::State *target)
        {
            boost::mutex::scoped_lock sl(drawMutex_);

            std::pair<ompl::base::State*, ompl::base::State*> edge;

            edge = std::make_pair(si_->cloneState(source),si_->cloneState(target));

            rolloutConnections_.push_back(edge);
        }

         /** \brief Add a rollout connection to the visualization */
        static void addMostLikelyPathEdge(const ompl::base::State *source, const ompl::base::State *target)
        {
            boost::mutex::scoped_lock sl(drawMutex_);

            std::pair<ompl::base::State*, ompl::base::State*> edge;

            edge = std::make_pair(si_->cloneState(source),si_->cloneState(target));

            mostLikelyPath_.push_back(edge);
        }

        static void setChosenRolloutConnection(const ompl::base::State *source, const ompl::base::State *target)
        {
            boost::mutex::scoped_lock sl(drawMutex_);

            if (chosenRolloutConnection_)
            {
                if (chosenRolloutConnection_->first)
                    si_->freeState(chosenRolloutConnection_->first);
                if (chosenRolloutConnection_->second)
                    si_->freeState(chosenRolloutConnection_->second);
            }

            boost::optional<std::pair<ompl::base::State*, ompl::base::State*> > edge(std::make_pair(si_->cloneState(source),si_->cloneState(target)));

            chosenRolloutConnection_ = edge;
        }

        static void drawRobotPath();

        static void setMode(VZRDrawingMode mode)
        {
            mode_ = mode;
        }

        static void ClearFeedbackEdges()
        {
            boost::mutex::scoped_lock sl(drawMutex_);
            feedbackEdges_.clear();
        }

        static void clearRolloutConnections()
        {
            boost::mutex::scoped_lock sl(drawMutex_);

            // free the memory for cloned states
            for (const auto edge : rolloutConnections_)
            {
                si_->freeState(edge.first);
                si_->freeState(edge.second);
            }

            rolloutConnections_.clear();
        }

        static void clearMostLikelyPath()
        {
            boost::mutex::scoped_lock sl(drawMutex_);

            // free the memory for cloned states
            for (const auto edge : mostLikelyPath_)
            {
                si_->freeState(edge.first);
                si_->freeState(edge.second);
            }

            mostLikelyPath_.clear();
        }

        static void addOpenLoopRRTPath(const ompl::geometric::PathGeometric path)
        {
            boost::mutex::scoped_lock sl(drawMutex_);
            openLoopRRTPaths_.push_back(path);
        }

        static void clearOpenLoopRRTPaths()
        {
            boost::mutex::scoped_lock sl(drawMutex_);
            openLoopRRTPaths_.clear();
        }

        /** \brief update the robot's true state for drawing */
        static void updateTrueState(const ompl::base::State *state)
        {
            if(si_)
            {
                boost::mutex::scoped_lock sl(drawMutex_);
                if(!trueState_) trueState_ = si_->allocState();
                si_->copyState(trueState_,state);
            }
        }

        /** \brief update the robot's belief for drawing */
        static void updateCurrentBelief(const ompl::base::State *state)
        {
            boost::mutex::scoped_lock sl(drawMutex_);
            if(!currentBelief_) currentBelief_ = si_->allocState();
            si_->copyState(currentBelief_,state);
        }

        /** \brief Copy the space information pointer */
        static void updateSpaceInformation(const firm::SpaceInformation::SpaceInformationPtr si)
        {
            boost::mutex::scoped_lock sl(drawMutex_);

            si_ = si;

            trueState_ = si_->allocState();

            currentBelief_ = si->allocState();
        }

        static void updateRenderer(ompl::app::RenderGeometry *renderer)
        {
            boost::mutex::scoped_lock sl(drawMutex_);
            renderGeom_ = renderer;
        }

        static void updateRenderer(const ompl::app::RigidBodyGeometry &rbg, const ompl::app::GeometricStateExtractor &se)
        {
            boost::mutex::scoped_lock sl(drawMutex_);
            //ompl::app::RenderGeometry rd(new ompl::app::RenderGeometry(rbg,se));
            renderGeom_ = new ompl::app::RenderGeometry(rbg,se);
        }

        static void clearRobotPath()
        {
            boost::mutex::scoped_lock sl(drawMutex_);

            // free the memory for cloned states
            for (const auto state : robotPath_)
            {
                si_->freeState(state);
            }

            robotPath_.clear();
        }

        /** \brief Draw a single landmark that is passed to this function */
        static void drawLandmark(arma::colvec& landmark);

        /** \brief Draw the true robot rendering*/
        static void drawRobot(const ompl::base::State *state);

        /** \brief Draw a single state that is passed to this function */
        static void drawState(const ompl::base::State* state, VZRStateType stateType);

        /** \brief Draw an edge that belongs to the Roadmap graph */
        static void drawEdge(const ompl::base::State* source, const ompl::base::State* target);

        /** \brief Draw the stored graph nodes */
        static void drawGraphBeliefNodes();

        /** \brief Draw belief modes. */
        static void drawBeliefModes();

        /** \brief Draw the edges in the roadmap graph */
        static void drawGraphEdges();

        /** \brief Draw graph feedback edges */
        static void drawFeedbackEdges();

        /** \brief Draw rollout connections */
        static void drawRolloutConnections();

        /** \brief Draw the most likely path under FIRM */
        static void drawMostLikelyPath();

        /** \brief Draw the X&Y bounds*/
        static void drawEnvironment();

        static void drawObstacle();

        /** \brief Draw geometric path.*/
        static void drawGeometricPath(const ompl::geometric::PathGeometric path);

        static void drawOpenLoopRRTPaths();

        /** \brief Refresh the drawing and show latest scenario*/
        static void refresh();

        static bool saveVideo()
        {
            return saveVideo_;
        }

        static void doSaveVideo(bool flag)
        {
            saveVideo_ = flag;
        }

        static void printRobotPathToFile(std::string path);


    private:

        /** \brief A list that stores the graph nodes*/
        static std::list<ompl::base::State*> states_;

        /** \brief visualizer thread mutex, allows us to place a lock so that class state changes while drawing */
        static boost::mutex drawMutex_;

        /** \brief Store the robots true state */
        static ompl::base::State* trueState_;

        /** \brief Store the robots belief state */
        static ompl::base::State* currentBelief_;

        /** \brief Store the belief modes for multi-modal operation */
        static std::list<ompl::base::State*> beliefModes_;

        /** \brief Store the landmarks */
        static std::vector<arma::colvec> landmarks_;

        /** \brief A pointer to the space information, because the visualizer must know what space its drawing */
        static firm::SpaceInformation::SpaceInformationPtr si_;

        /** \brief Store the Roadmap graph edges */
        static std::vector<std::pair<const ompl::base::State*, const ompl::base::State*> > graphEdges_;

        /** \brief Store the Rollout connections */
        static std::vector<std::pair<ompl::base::State*, ompl::base::State*> > rolloutConnections_;

        /** \brief Store the Rollout connections */
        static std::vector<std::pair<ompl::base::State*, ompl::base::State*> > mostLikelyPath_;

        /** \brief The rollout connection that gets chosen as the next target*/
        static boost::optional<std::pair<ompl::base::State*, ompl::base::State*> >chosenRolloutConnection_;

        /** \brief Store the feedback edges */
        static std::vector<VZRFeedbackEdge> feedbackEdges_;

        /** \brief stores the sequence of states of the real robot */
        static std::vector<ompl::base::State*> robotPath_;

        /** \brief Container for RRT paths generated as candidates during Multi-Modal operation */
        static std::vector<ompl::geometric::PathGeometric> openLoopRRTPaths_;

        /** \brief Visualizer drawing mode setting */
        static VZRDrawingMode mode_;

        /** \brief The render geometry utility in ompl::app */
        static ompl::app::RenderGeometry *renderGeom_;

        /** \brief The gllistindex of the environment rendering*/
        static int envIndx_;

        /** \brief The gllistindex of the robot rendering */
        static int robotIndx_;

        static bool saveVideo_;


};
#endif // FIRM_OMPL_VISUALIZER_H
