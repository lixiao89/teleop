#ifndef _mtsGravityCompensation_h
#define _mtsGravityCompensation_h

#include <sawControllers/mtsController.h>
#include <sawControllers/sawControllersExport.h>


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
#include <sawControllers/osaGravityCompensation.h>
#include "osaHybridForcePosition.h"

#include <cisstRobot/robLinearSE3.h>

#include <cisstNumerical/nmrLSMinNorm.h>
#include <cisstNumerical/nmrSavitzkyGolay.h>


class CISST_EXPORT mtsNewHybridForcePosition : public mtsController {

 private:
 // robot stuff
  robManipulator robot;  // kin
  robManipulator tool;
  robLinearSE3* traj;
  double timer;
  vctFrame4x4<double> Rtwtsold;    // previous command to the slave
  vctFrame4x4<double> Rtwtsoldtrj; // previous interpolated from the master
  vctFrame4x4<double> Rtwtsoldcmd; // previous command from the master


 
  // JR3 sensor
  osaJR3ForceSensor* jr3;

  //
  osaGravityCompensation* gc;
  osaHybridForcePosition* hfp;
  
  // interfaces
  mtsInterfaceRequired* control;

  

  // Control
  vctDynamicVector<double> qready;
  vctDynamicVector<double> qsold;
  vctDynamicVector<double> tauold;

  enum State{ IDLE, RESET, ENABLE, TEST, HYBRID };
  State state;
  bool enable;

  double fz;

  // used to record PID input/output
  std::list<double> dt;
  std::ofstream ofsq, ofsqs;


  //! Read the joint positions
  mtsFunctionRead  GetPositions;

  //! Write the joint torques
  mtsFunctionWrite SetTorques;

 //! Write the joint torques
  mtsFunctionWrite SetPosition;

 void HybridControl();
  void MoveToReady();
  
  vctDynamicVector<double> sg;
  std::list< osaJR3ForceSensor::Wrench > stdft;    

 public:

  void Test(){ state = TEST; }
  void Hybrid(){ state = HYBRID; }
  void Reset(){ state = RESET; }
  void Force(){ fz -= 5.0; }
  void Enable(){ enable = !enable; }

  bool IsEnabled(){ return enable; }

 public:
  
  mtsNewHybridForcePosition(  const std::string& name,
			   double period,
                           
			   const std::string& robotfilename, 
			   const vctFrame4x4<double>& Rtw0,
			   const vctFrame4x4<double>& Rtnt,
			   const vctDynamicVector<double>& qinit,
			   const vctDynamicVector<double>& qready,

			   osaJR3ForceSensor* jr3,
			   osaGravityCompensation* gc,
			   osaHybridForcePosition* hfp );

  
  void Configure( const std::string& );
  void Startup();
  void Run();
  
  void Cleanup();      

};

#
