/*-*- Mode++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-   */
/*ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:*/

/*
  $Id: mtsHybridForcePosition.h 3181 2011-11-15 15:41:28Z sleonar7 $

  Author(s):  Simon Leonard
  Created on: 2013-07-22

  (C) Copyright 2012 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnPath.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaGetTime.h>

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmEventButton.h>

#include <sawJR3ForceSensor/osaJR3ForceSensor.h>
#include "osaHybridForcePosition.h"

#include <cisstRobot/robLinearSE3.h>
#include <sawControllers/osaGravityCompensation.h>

#include <cisstNumerical/nmrLSMinNorm.h>
#include <cisstNumerical/nmrSavitzkyGolay.h>

#include <sawBarrett/osaWAM.h>
#include "RLSestimator.h"

#include <sstream>
#include <iostream>
#include <fstream>
class mtsHybridForcePosition : public mtsTaskPeriodic {

private:

  // robot stuff
  robManipulator robot;  // kin
  robManipulator tool;
  robLinearSE3* traj;
  double timer;
  vctFrame4x4<double> Rtwtsold;    // previous command to the slave
  vctFrame4x4<double> Rtwtsoldtrj; // previous interpolated from the master
  vctFrame4x4<double> Rtwtsoldcmd; // previous command from the master

  vctMatrixRotation3<double> Rts;

  // positions of tool wrt WAM link 7
  vctFrame4x4<double> Rtnt;

  // JR3 sensor
  osaJR3ForceSensor* jr3;

  // WAM
  osaWAM* wam;
  
  //--------------- For RLS --------------------
  RLSestimator* rls;
  std::ofstream ofsForceData;
  double startTime;
  bool failstate;
  bool isMoving;
  double prevTime;


  vctFrame4x4<double> prevPos;// record the last position manipulator is in
  vctDynamicVector<double> prevJointPos;

  std::vector< std::vector<double> > jointPoses;
  std::vector<double> timeStamps;
  int avgNum;// number of joint position measurement to take in calculation of velocity

  // Estimated quantities from RLSestimator, xesti = [mu, Fc], Festi is the estimated tangential force
  vctFixedSizeVector<double,2> xesti;
  double Festi;

 //--------------------------------------------
  osaGravityCompensation* gc;
  osaHybridForcePosition* hfp;
  
  // interfaces
  mtsInterfaceRequired* slave;
  mtsInterfaceRequired* control;

  // Control
  vctDynamicVector<double> qready;
  vctDynamicVector<double> qsold;
  vctDynamicVector<double> tauold;

  enum State{ DONOTHING, IDLE, RESET, ENABLE, HYBRID, MOVE};
  State state;
  bool enable;

  double fz;

  // used to record PID input/output
  std::list<double> dt;
  std::ofstream ofsq, ofsqs;

  // Slave side
  //! Read the joint positions
  mtsFunctionRead  GetPosition;
  //! Write the joint torques
  mtsFunctionWrite SetPosition;


  // Master side 
  prmPositionCartesianGet prmCommandSE3;    // Cartesian command (master side)
  prmPositionCartesianGet prmTelemetrySE3;  // Cartesian telemetry
  prmPositionJointGet     prmTelemetryRn;   // Joint telemetry

  void HybridControl();
  void MoveToReady();
  void Idle(); 
  void MoveTraj();
  vctDynamicVector<double> sg;
  std::list< osaJR3ForceSensor::Wrench > stdft;  


    
 public:

  void Hybrid(){ state = HYBRID; }
  void Move(){state = MOVE;};
  void ToIdle(){state = IDLE; std::cout<< "start Idle!"<<std::endl;};
  bool IsEnabled(){ return enable; }

 public:
  
  mtsHybridForcePosition(  const std::string& name,
			   double period,
			   const std::string& robotfilename, 
			   const vctFrame4x4<double>& Rtw0,
			   const vctFrame4x4<double>& Rtnt,
			   const vctDynamicVector<double>& qinit,
			   const vctDynamicVector<double>& qready,

			   osaJR3ForceSensor* jr3,
			   osaGravityCompensation* gc,
			   osaHybridForcePosition* hfp );
 
  // --------------------- For RLS -----------------------------------
  void PrintTime(){
      std::cout<< "current time is: "<< osaGetTime() - startTime <<std::endl;
  }


  void CalcVectorAverage(const std::vector<double>& vect, double& avg){

      double temp = 0;
      for(int i = 0; i < vect.size(); i++){
            temp = temp + vect.at(i);
      }

      avg = temp/vect.size();
  }


void CalcAverageVelocity(vctDynamicVector<double>& currJointPos, double& currTime, vctDynamicVector<double>& avgVel){

    std::vector<double> temp;

    for(int i = 0; i < 7; i++){
        temp.push_back(currJointPos[i]);
    }

         jointPoses.push_back(temp);
         timeStamps.push_back(currTime);
        // process only the most recent avgNum data
        if(timeStamps.size() > avgNum){
            jointPoses.erase(jointPoses.begin());
            timeStamps.erase(timeStamps.begin());
        }


        vctDynamicVector<double> currPos( 7 , jointPoses.at(avgNum-1).at(0),jointPoses.at(avgNum-1).at(1),jointPoses.at(avgNum-1).at(2),jointPoses.at(avgNum-1).at(3),jointPoses.at(avgNum-1).at(4),jointPoses.at(avgNum-1).at(5),jointPoses.at(avgNum-1).at(6));

        vctDynamicVector<double> pastPos( 7 , jointPoses.at(0).at(0),jointPoses.at(0).at(1),jointPoses.at(0).at(2),jointPoses.at(0).at(3),jointPoses.at(0).at(4),jointPoses.at(0).at(5),jointPoses.at(0).at(6));


        vctDynamicVector<double> jointPosdiff = (currPos - pastPos).Abs();
        
        double timediff = (timeStamps.at(avgNum-1) - timeStamps.at(0));
        avgVel = jointPosdiff / timediff;

}

bool WAMIsNotMoving( vctDynamicVector<double>& currJointPos, double& currTime){
      
    vctDynamicVector<double> jointAvgVel;
    CalcAverageVelocity(currJointPos, currTime, jointAvgVel);
      

            double motionThreshold = 0.03;

        if(jointAvgVel[0] < motionThreshold && jointAvgVel[1] < motionThreshold && jointAvgVel[2] < motionThreshold && jointAvgVel[3] < motionThreshold && jointAvgVel[4] < motionThreshold && jointAvgVel[5] < motionThreshold && jointAvgVel[6] < motionThreshold){

            //std::cout<<jointAvgVel<<std::endl;
            return true;
        }
        else{

            //std::cout<<jointAvgVel<<std::endl;
            return false;
        }
}
  //----------------------------------------------------------------
 
  
  void Configure( const std::string& );
  void Startup();
  void Run();
  
};

