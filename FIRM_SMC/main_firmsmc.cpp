//
// Created by redwan on 3/12/19.
//
#include <iostream>
#include <cassert>
#include <QtCore>
#include <QTextStream>
#include <QApplication>
#include "TwoDPointRobotSMCSetup.h"



#include <armadillo>
#include "Visualization/Visualizer.h"

#define debug(x) std::cout<<x<<std::endl
using namespace std;

void plan(const std::string &setupFilePath){
    TowDPointRobotSMCSetup *mySetup(new TowDPointRobotSMCSetup);

    mySetup->setPathToSetupFile(setupFilePath.c_str());

    mySetup->setup();
    Visualizer::setMode(Visualizer::VZRDrawingMode::PRMViewMode);


    int keepTrying = 1;

    while(keepTrying)
    {
        if(mySetup->solve())
        {
            mySetup->Run();
            OMPL_INFORM("Plan Executed.");
            Visualizer::doSaveVideo(false);
            sleep(1);
            keepTrying = 0;

        }
        else
        {
            OMPL_INFORM("Unable to find Solution in given time. Either more nodes are needed, or DP could not converge.");

            break;

            //std::cin>>keepTrying;
        }

    }

    delete mySetup;
    OMPL_INFORM("Execution Terminated.");

//    QApplication::quit();

    return;
}


int main(int argc, char *argv[]){
    assert(argc==2 && "SetupFile missing -takes only one additional argument");
    debug("Setup File "<< argv[1]);
    arma::arma_rng::set_seed_random();

    QApplication app(argc, argv);

    MyWindow window;

    window.resize(window.sizeHint());

    window.showMaximized();

    window.resetCamera();

    boost::thread solveThread(plan,argv[1]);


    app.exec();

    solveThread.join();


    return 0;
}