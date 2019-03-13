/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Texas A&M University
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

/* Authors: Saurav Agarwal, Ali-akbar Agha-mohammadi */

#include "ompl/base/SpaceInformation.h"
#include "ompl/tools/config/MagicConstants.h"
#include "Samplers/GaussianValidBeliefSampler.h"
#include "Filters/dare.h"
#include "LinearSystem/LinearSystem.h"

GaussianValidBeliefSampler::GaussianValidBeliefSampler(const ompl::base::SpaceInformation *si) :
    ValidStateSampler(si), sampler_(si->allocStateSampler()), stddev_(si->getMaximumExtent() * ompl::magic::STD_DEV_AS_SPACE_EXTENT_FRACTION)
{
    name_ = "gaussian";
    params_.declareParam<double>("standard_deviation",
                                 std::bind(&GaussianValidBeliefSampler::setStdDev, this, std::placeholders::_1),
                                 std::bind(&GaussianValidBeliefSampler::getStdDev, this));
}

bool GaussianValidBeliefSampler::sample(ompl::base::State *state)
{
    bool result = false;
    unsigned int attempts = 0;
    ompl::base::State *temp = si_->allocState();
    do
    {
        sampler_->sampleUniform(state);
        bool v1 = si_->isValid(state) ;
        sampler_->sampleGaussian(temp, state, stddev_);
        bool v2 = si_->isValid(temp) ;
        if (v1 != v2)
        {
            if (v2 && isObservable(temp)) // observability check must also be performed
                si_->copyState(state, temp);
            result = true;
        }
        ++attempts;
    } while (!result && attempts < attempts_);
    si_->freeState(temp);

    return result;
}

bool GaussianValidBeliefSampler::sampleNear(ompl::base::State *state, const ompl::base::State *near, const double distance)
{
    bool result = false;
    unsigned int attempts = 0;
    ompl::base::State *temp = si_->allocState();
    do
    {
        sampler_->sampleUniformNear(state, near, distance);
        bool v1 = si_->isValid(state) ;
        sampler_->sampleGaussian(temp, state, distance);
        bool v2 = si_->isValid(temp) ;
        if (v1 != v2)
        {
            if (v2 && isObservable(temp)) // state must be observable to be a valid FIRM node
                si_->copyState(state, temp);
            result = true;
        }
        ++attempts;
    } while (!result && attempts < attempts_);
    si_->freeState(temp);
    return result;
}

/*
 Checks whether a sampled state is observable.
 It might be better to have the observability check in filter
 instead of observation model and call it from here.
*/

bool GaussianValidBeliefSampler::isObservable(ompl::base::State *state)
{
    /*
    LinearSystem ls(si_, state, motionModel_->getZeroControl(), observationModel_->getObservation(state, false), motionModel_, observationModel_);

    arma::mat S;

    bool systemStable = dare (trans(ls.getA()),trans(ls.getH()),ls.getG() * ls.getQ() * trans(ls.getG()),
                                ls.getM() * ls.getR() * trans(ls.getM()), S );
    */
    return true;

}

