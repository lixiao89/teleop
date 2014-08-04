#include <cisstCommon/cmnPath.h>
#include <sawBarrett/osaWAM.h>
#include <sawJR3ForceSensor/osaJR3ForceSensor.h>
#include <sawKeyboard/mtsKeyboard.h>
#include <sawCANBus/osaSocketCAN.h>
#include <cisstOSAbstraction/osaCPUAffinity.h>
#include <cisstParameterTypes/prmPositionJointGet.h>

#include <cisstCommon/cmnGetChar.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaGetTime.h>
//#include <sawControllers/mtsGravityCompensation.h>
#include <sawBarrett/mtsWAM.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include "mtsHybridForcePosition.h"
#include "mtsPIDAntiWindup.h"
//#include "RLSestimator.h"
//================== MAIN =========================

int main(int argc, char** argv){

    cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
    cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
    cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );
    
    mtsTaskManager* taskManager = mtsTaskManager::GetInstance();

      
  osaSocketCAN can( argv[1], osaCANBus::RATE_1000 );
  if( can.Open() != osaCANBus::ESUCCESS ){
    CMN_LOG_RUN_ERROR << argv[0] << "Failed to open " << argv[1] << std::endl;
    return -1;
  }

  // Rotate the base
  vctMatrixRotation3<double> Rw0(  0.0,  0.0, -1.0,
                                   0.0,  1.0,  0.0,
                                   1.0,  0.0,  0.0 );
  vctFixedSizeVector<double,3> tw0(0.0);
  vctFrame4x4<double> Rtw0( Rw0, tw0 );
 
  

  //-------------- Setting up Hybrid Control ------------------------
  
    mtsKeyboard kb;
    kb.SetQuitKey( 'q' );

    kb.AddKeyVoidEvent( 'i', "Control", "ToIdle" );
    kb.AddKeyVoidEvent( 'm', "Control", "Move" );
    kb.AddKeyVoidEvent( 't', "Control", "PrintTime");
    kb.AddKeyVoidEvent( 'm', "GC", "MovePID");
    kb.AddKeyVoidEvent( 'G', "GC", "GravityCompensation");
   
    taskManager->AddComponent( &kb );

 // initial joint position
    vctDynamicVector<double> qinit( 7, 0.0 );
    qinit[1] = -cmnPI_2;
    qinit[3] =  cmnPI;  
    qinit[5] = -cmnPI_2;

    // ready joint position
    vctDynamicVector<double> qready( qinit );
    qready[3] =  cmnPI_2;  

 // orientation of the tool wrt FT sensor (18 degrees about +Y)
 // Change this??
    vctMatrixRotation3<double> Rst( 0.9511,  0.0000,  -0.3090,
                                    0.0000,  1.0000,   0.0000,
                                    0.3090,  0.0000,   0.9511,
                                    VCT_NORMALIZE );

   /* vctMatrixRotation3<double> Rst( 1.0000,  0.0000,   0.0000,
                                    0.0000,  1.0000,   0.0000,
                                    0.0000,  0.0000,   1.0000,
                                    VCT_NORMALIZE );*/
 
    // position of the tool wrt FT sensor (9cm along +z)
    vctFixedSizeVector<double,3> tst( 0.0, 0.0, 0.09 );
    vctFrame4x4<double> Rtst( Rst, tst );

    // Tool to attach to the WAM
    // transform of the tool wrt WAM link 7 (18 deg about +Y)
    vctMatrixRotation3<double> R7t( Rst );
    // position of the TCP wrt WAM link 7
   // vctFixedSizeVector<double,3> t7t( 0.0, 0.0, 0.15 );
   vctFixedSizeVector<double,3> t7t( 0.0, 0.0, 0.12 );
    vctFrame4x4<double> Rt7t( R7t, t7t ); 

    // this is used to evaluate the kinematics and Jacobian
   // robManipulator* robWAM = new robManipulator( argv[2], Rtw0 );
 robManipulator* robWAM = new robManipulator( "/home/lixiao/src/wvu-jhu/models/WAM/wam7cutter.rob",Rtw0);
   // Create a tool and attach it to the WAM
    robManipulator* robtool = new robManipulator( Rt7t );

    // mass and center of the tool (measured)
    double mass = 0.145;
    vctFixedSizeVector<double,3> com( 0.0, 0.0, 0.03 );

    // JR3 FT sensor/
    osaJR3ForceSensor::Wrench zero( 0.0 );
    osaJR3ForceSensor* jr3 = new osaJR3ForceSensor( "/dev/comedi0",
                                                    osaJR3ForceSensor::METRIC,
                                                    zero,
                                                    Rtst, 
                                                    mass, 
                                                    com );
    jr3->Open();
    // zero the JR3 accounting for the mass of the tool
    jr3->Zero( robWAM->ForwardKinematics( qinit, 7 ) );

    // attach the tool
    robWAM->Attach( robtool );


    // gains (proportional, integral, derivative, anti-windup)
    vctDynamicVector<double> Kp(7,4000.0,3000.0,2000.0,1500.0,180.0,180.0,20.0);
    vctDynamicVector<double> Ki(7,3200.0,3200.0,2200.0,3200.0,200.0,200.0,30.0);
    vctDynamicVector<double> Kd(7,   8.0,   8.0,   8.0,   5.0,  0.5,  0.5, 0.05);
    vctDynamicVector<double> Kt(7, 5.0);
    vctDynamicVector<double> limits(7,120.0,110.0,110.0, 50.0, 15.0, 15.0, 5.5);

    osaPIDAntiWindup* osapid = NULL;
    osapid = new osaPIDAntiWindup( Kp, Ki, Kd, Kt, limits, qinit );

    osaWAM* wam = new osaWAM( &can, osaWAM::WAM_7DOF );
    wam->Initialize();
    wam->SetPositions( qinit );
    
    // controller part
    osaHybridForcePosition::Mask mask( osaHybridForcePosition::POSITION );
    vctFixedSizeVector<double,6> K( .3, .3, .3, .1, .1, .1 );
    osaHybridForcePosition* hfp = NULL;
    hfp = new osaHybridForcePosition( mask, robWAM, K );
    
    osaGravityCompensation* gc = NULL;
    //gc = new osaGravityCompensation( argv[2], Rtw0 );
    gc = new osaGravityCompensation( "/home/lixiao/src/wvu-jhu/models/WAM/wam7cutter.rob", Rtw0 );

    mtsHybridForcePosition* ctrl = NULL;
    // Control is a required interface
    //ctrl = new mtsHybridForcePosition( "Control", 1.0/500.0,
    //                                   argv[2], Rtw0, Rt7t, qinit, qready,
    //                                   jr3, gc, hfp );
    ctrl = new mtsHybridForcePosition( "Control", 1.0/500.0,
                                       "/home/lixiao/src/wvu-jhu/models/WAM/wam7cutter.rob", Rtw0, Rt7t, qinit, qready,jr3, gc, hfp );
    //ctrl->Startup();


                                      
    taskManager->AddComponent( ctrl );

    mtsPIDAntiWindup* mtspid = new mtsPIDAntiWindup( "PID", 1.0/900.0, wam, osapid, gc );
    taskManager->AddComponent( mtspid );

// ------------------- Connecting ---------------------------

 
//----------- connecting keyboard to hybrid controller------------
 if( !taskManager->Connect( ctrl->GetName(), "Control",
                            kb.GetName(), "Control") ){
    std::cout << "Failed to connect: "
	      << kb.GetName() << "::Control to "
	      << ctrl->GetName() << "::Control" << std::endl;
    return -1;
  }

if( !taskManager->Connect( mtspid->GetName(), "GC",
                            kb.GetName(), "GC") ){
    std::cout << "Failed to connect: "
	      << kb.GetName() << "::GC to "
	      << mtspid->GetName() << "::GC" << std::endl;
    return -1;
  }


// connecting pid antiwindup to hybrid controller
// Slave in mtspid is a provided interface
// Slave in ctrl is required interface
 if( !taskManager->Connect( ctrl->GetName(), "Slave", 
                            mtspid->GetName(), "Slave") ){
    std::cout << "Failed to connect: "
	      << mtspid->GetName() << ":Slave to "
	      << ctrl->GetName() << ":Slave" << std::endl;
    return -1;
  }
 
  taskManager->CreateAll();
  taskManager->StartAll();
  pause();

    //osaJR3ForceSensor jr3("/dev/comedi0");
    //jr3.Open();

    //osaJR3ForceSensor::Wrench ft;
    //jr3.Read(ft);
    //std::cout<<ft<<std::endl;

    //jr3.Close();

    return 0;
}
