//
// Created by redwan on 3/12/19.
//

#include <StatisticalModelChecking.h>

#include "StatisticalModelChecking.h"

ModelChecking::StatisticalModelChecking::StatisticalModelChecking(string file) {
    /* Default safety threshold is 0.2
     * and reachability threshold is 0.8
     */
    safety_thredhold_ = 0.0 ;
    reachability_threshold_ = 0.0;
    loadParmeter(file);
}

double ModelChecking::StatisticalModelChecking::computeSaftyProb(std::vector<Vertex> &childQnodes_) {
    /** \brief
     * 1) compute minimum distance from obstacles while considering orientation
     * 2) check how many nodes violates safety tolerance - minimum safe distance
     * 3) return the probability of satisfying safety_thredhold_
     */
    return 0;
}

double ModelChecking::StatisticalModelChecking::computeReachabilityProb(std::vector<Vertex> &childQnodes_) {
    /** \brief
     * 1) compute distance from the dijkstart path while considering orientation
     * 2) check how many nodes violates reachability tolerance
     * 3) return the probability of satisfying reachability_threshold_
     */
    return 0;
}

SMCptr ModelChecking::StatisticalModelChecking::getPtr() {
    return shared_from_this();
}

void ModelChecking::StatisticalModelChecking::loadParmeter(string &file) {
    TiXmlDocument doc(file);
    bool loadOkay = doc.LoadFile();
    if(!loadOkay){
        printf( "Could not load setup file. Error='%s'. Exiting.\n", doc.ErrorDesc() );
        exit( 1 );
    }
    TiXmlNode* node = 0;
    TiXmlElement* itemElement = 0;

    node = doc.FirstChild( "ModelChecking" );
    assert( node );
    // safty threshold
    auto child1 = node->FirstChild("saftyThreshold");
    assert( child1 );
    itemElement = child1->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("saftyThreshold", &safety_thredhold_);

    // reachability threshold
    auto child2 = node->FirstChild("reachabilityThreshold");
    assert( child2 );
    itemElement = child2->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("reachabilityThreshold", &reachability_threshold_);
    //TODO create collision detector from the mesh files

    // STEP Read the env mesh file
    node = doc.FirstChild( "PlanningProblem" );
    auto child = node->FirstChild("Environment");
    assert( child );

    itemElement = child->ToElement();
    assert( itemElement );

    std::string environmentFilePath;
    itemElement->QueryStringAttribute("environmentFile", &environmentFilePath);

    // Read the robot mesh file
    child  = node->FirstChild("Robot");
    assert( child );

    itemElement = child->ToElement();
    assert( itemElement );

    std::string robotFilePath;
    itemElement->QueryStringAttribute("robotFile", &robotFilePath);
    setupCollisionDetector(environmentFilePath, robotFilePath);


}




void ModelChecking::StatisticalModelChecking::setupCollisionDetector(string &env_objFile, string &robot_objFile) {

    // read mesh file and restore them as vectors
    std::vector<Vec3f> ev, rv;
    std::vector<Triangle> et, rt;
    loadOBJFile(env_objFile.c_str(), ev, et);
    loadOBJFile(robot_objFile.c_str(), rv, rt);

    // configure models

    env_model_ = shared_ptr<Model>(new Model());
    robot_model_ = shared_ptr<Model>(new Model());

    // populate model with mesh data
    env_model_->beginModel();
    env_model_->addSubModel(ev, et);
    env_model_->endModel();

    robot_model_->beginModel();
    robot_model_->addSubModel(rv, rt);
    robot_model_->endModel();

    env_obj_ = new CollisionObject(env_model_);

    cout<<"collision detector is configured"<<endl;

}


void ModelChecking::StatisticalModelChecking::yawToMatrix(FCL_REAL c, fcl::Matrix3f& R, FCL_REAL a , FCL_REAL b)
{
    /** brief: c = yaw, a = roll, b = pitch, R= 3x3 Rotational matrix
     *  Given a yaw value, this function will write Rotational Matrix
     */
    FCL_REAL c1 = cos(a);
    FCL_REAL c2 = cos(b);
    FCL_REAL c3 = cos(c);
    FCL_REAL s1 = sin(a);
    FCL_REAL s2 = sin(b);
    FCL_REAL s3 = sin(c);
    R.setValue(c1 * c2, - c2 * s1, s2,
               c3 * s1 + c1 * s2 * s3, c1 * c3 - s1 * s2 * s3, - c2 * s3,
               s1 * s3 - c1 * c3 * s2, c3 * s1 * s2 + c1 * s3, c2 * c3);
}



void ModelChecking::StatisticalModelChecking::loadOBJFile(const char* filename, std::vector<Vec3f>& points, std::vector<Triangle>& triangles)
{
    FILE* file = fopen(filename, "rb");
    if(!file)
    {
        std::cerr << "file not exist" << std::endl;
        return;
    }

    bool has_normal = false;
    bool has_texture = false;
    char line_buffer[2000];
    while(fgets(line_buffer, 2000, file))
    {
        char* first_token = strtok(line_buffer, "\r\n\t ");
        if(!first_token || first_token[0] == '#' || first_token[0] == 0)
            continue;

        switch(first_token[0])
        {
            case 'v':
            {
                if(first_token[1] == 'n')
                {
                    strtok(nullptr, "\t ");
                    strtok(nullptr, "\t ");
                    strtok(nullptr, "\t ");
                    has_normal = true;
                }
                else if(first_token[1] == 't')
                {
                    strtok(nullptr, "\t ");
                    strtok(nullptr, "\t ");
                    has_texture = true;
                }
                else
                {
                    float x = (float)atof(strtok(nullptr, "\t "));
                    float y = (float)atof(strtok(nullptr, "\t "));
                    float z = (float)atof(strtok(nullptr, "\t "));
                    points.emplace_back(x, y, z);
                }
            }
                break;
            case 'f':
            {
                Triangle tri;
                char* data[30];
                int n = 0;
                while((data[n] = strtok(nullptr, "\t \r\n")) != nullptr)
                {
                    if(strlen(data[n]))
                        n++;
                }

                for(int t = 0; t < (n - 2); ++t)
                {
                    if((!has_texture) && (!has_normal))
                    {
                        tri[0] = atoi(data[0]) - 1;
                        tri[1] = atoi(data[1]) - 1;
                        tri[2] = atoi(data[2]) - 1;
                    }
                    else
                    {
                        const char *v1;
                        for(int i = 0; i < 3; i++)
                        {
                            // vertex ID
                            if(i == 0)
                                v1 = data[0];
                            else
                                v1 = data[t + i];

                            tri[i] = atoi(v1) - 1;
                        }
                    }
                    triangles.push_back(tri);
                }
            }
        }
    }
}


ostream &ModelChecking::operator<<(ostream &os, const ModelChecking::StatisticalModelChecking &checking) {
    os <<  "safety_thredhold_: "
       << checking.safety_thredhold_ << " reachability_threshold_: " << checking.reachability_threshold_;
    return os;
}

double ModelChecking::StatisticalModelChecking::minCollisionDistance(float x, float y, float yaw) {
    // TODO min Collision distance linking Function

    Vec3f robot_pos{x, y, 0};
    fcl::Matrix3f R; // robot rotational matrix
    yawToMatrix(yaw,R);
    Transform3f RobotPose(R, robot_pos);
    robot_obj_ = new CollisionObject(robot_model_, RobotPose);
    distance(env_obj_, robot_obj_, request_, result_);

//    printf("cord {%f, %f, %f} collision ",x,y,yaw);
//    cout<<result_.min_distance<<endl;
    //FIXME - is robot_obj_ memory leaking?
    return result_.min_distance;

}
