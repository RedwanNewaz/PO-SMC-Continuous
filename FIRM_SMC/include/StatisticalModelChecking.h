//
// Created by redwan on 3/12/19.
//

#ifndef BSPAPP_STATISTICALMODELCHECKING_H
#define BSPAPP_STATISTICALMODELCHECKING_H

#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <tinyxml.h>
#include <assert.h>

#include <fcl/config.h>
#include <fcl/math/math_details.h>
#include <fcl/math/constants.h>
#include <fcl/math/transform.h>
#include <fcl/collision.h>
#include <fcl/collision_object.h>
#include <fcl/collision_node.h>
#include <fcl/distance.h>

using namespace std;
using namespace fcl;
typedef unsigned long int 	Vertex;
typedef BVHModel<OBBRSS> Model;

namespace ModelChecking{
    class StatisticalModelChecking;
}
typedef ModelChecking::StatisticalModelChecking SMC;
typedef shared_ptr<SMC> SMCptr;

namespace ModelChecking{
    class StatisticalModelChecking: public enable_shared_from_this<SMC> {
    public:
        StatisticalModelChecking(string file);
        SMCptr getPtr();
        void loadOBJFile(const char* filename, std::vector<Vec3f>& points, std::vector<Triangle>& triangles);
        void yawToMatrix(FCL_REAL c, fcl::Matrix3f& R, FCL_REAL a =0.0, FCL_REAL b=0.0);
        double minCollisionDistance(float x, float y, float yaw);
        friend ostream &operator<<(ostream &os, const StatisticalModelChecking &checking);

    private:
        double safety_thredhold_;
        double reachability_threshold_;
        std::shared_ptr<Model> env_model_;
        std::shared_ptr<Model> robot_model_;
        CollisionObject *env_obj_, *robot_obj_;
        // set the distance request structure, here we just use the default setting
        DistanceRequest request_;
        // result will be returned via the collision result structure
        DistanceResult result_;

    protected:
        double computeSaftyProb(std::vector< Vertex > &childQnodes_);
        double computeReachabilityProb(std::vector< Vertex > &childQnodes_);
        void loadParmeter(string &file);
        void setupCollisionDetector(string &env_objFile, string &robot_objFile);

    };
}




#endif //BSPAPP_STATISTICALMODELCHECKING_H
