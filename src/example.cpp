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
#include <sawControllers/mtsGravityCompensation.h>
#include <sawBarrett/mtsWAM.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include "mtsHybridForcePosition.h"
#include "mtsPIDAntiWindup.h"
#include "WAMprobe.h"

//================== MAIN =========================

int main(int argc, char** argv){

    cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
    cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
    cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );
    
   /* if( argc != 4 ){
        std::cout << "Usage: " << argv[0] << " GCM robfile can0-1" <<std::endl;
        return -1;
    }*/

    mtsTaskManager* taskManager = mtsTaskManager::GetInstance();

    mtsKeyboard kb;
    kb.SetQuitKey( 'q' );
    // Add key 'C' to enable gravity compensation 
    // GCEnable is a required interface created by AddKeyWriteFunction
    kb.AddKeyWriteFunction( 'C', "GCEnable", "Enable", true );


   
  osaSocketCAN can( argv[1], osaCANBus::RATE_1000 );
  if( can.Open() != osaCANBus::ESUCCESS ){
    CMN_LOG_RUN_ERROR << argv[0] << "Failed to open " << argv[1] << std::endl;
    return -1;
  }

  mtsWAM WAM( "WAM", &can, osaWAM::WAM_7DOF, OSA_CPU4, 80 );
  WAM.Configure();
  WAM.SetPositions( vctDynamicVector<double>(7, 
  					     0.0, -cmnPI_2, 0.0, cmnPI, 
  					     0.0, -cmnPI_2, 0.0 ) );
  taskManager->AddComponent( &WAM );


 // cmnPath path;
 // path.AddRelativeToCisstShare("/models/WAM");

  // Rotate the base
  vctMatrixRotation3<double> Rw0(  0.0,  0.0, -1.0,
                                   0.0,  1.0,  0.0,
                                   1.0,  0.0,  0.0 );
  vctFixedSizeVector<double,3> tw0(0.0);
  vctFrame4x4<double> Rtw0( Rw0, tw0 );

  // instantiate and initialize GC controller
  mtsGravityCompensation GC( "GC", 
			     0.002, 
			    // path.Find( "wam7.rob" ), 
                "/home/lixiao/src/wvu-jhu/models/WAM/wam7cutter.rob",
			     Rtw0,
			     OSA_CPU3 );
  taskManager->AddComponent( &GC );


  //-------------- Setting up Hybrid Control ------------------------
  
    
   // mtsKeyboard kb;
   // kb.SetQuitKey( 'q' );
   // AddKeyVoidEvent creates provided interfaces
    kb.AddKeyVoidEvent( 'e', "Control", "Enable" );
    kb.AddKeyVoidEvent( 'r', "Control", "Reset" );
    kb.AddKeyVoidEvent( 't', "Control", "Test" );
    kb.AddKeyVoidEvent( 'f', "Control", "Force" );

    kb.AddKeyVoidEvent( 'h', "Control", "Hybrid" );
    kb.AddKeyVoidEvent( 's', "Setqr", "Setqready");
 
    taskManager->AddComponent( &kb );


 // initial joint position
    vctDynamicVector<double> qinit( 7, 0.0 );
    qinit[1] = -cmnPI_2;
    qinit[3] =  cmnPI;  
    qinit[5] = -cmnPI_2;

// Setting up keyboard to acquire ready joint position "qready"

    WAMprobe* wamprobe = new WAMprobe();

    taskManager->AddComponent( wamprobe );

   if( !taskManager->Connect( wamprobe->GetName(), "Setqr", kb.GetName(), "Setqr")){
    std::cout << "Failed to connect: "
	      << wamprobe->GetName() << "::Control to "
	      << kb.GetName() << "::Control" << std::endl;
    return -1;
  }


if( !taskManager->Connect( wamprobe->GetName(), "Input",WAM.GetName(),  "Output" ) ){
        std::cout << "Failed to connect: "
              << wamprobe->GetName() << "::Input to "
              << WAM.GetName()  << "::Output" << std::endl;
        return -1;
      }


    
    // ready joint position
   // vctDynamicVector<double> qready( qinit );
   // qready[3] =  cmnPI_2;  
      vctDynamicVector<double> qready( wamprobe->qr);

 // orientation of the tool wrt FT sensor (18 degrees about +Y)
 // Change this??
    vctMatrixRotation3<double> Rst( 0.9511,  0.0000,  -0.3090,
                                    0.0000,  1.0000,   0.0000,
                                    0.3090,  0.0000,   0.9511,
                                    VCT_NORMALIZE );
    // position of the tool wrt FT sensor (9cm along +z)
    vctFixedSizeVector<double,3> tst( 0.0, 0.0, 0.09 );
    vctFrame4x4<double> Rtst( Rst, tst );

    // Tool to attach to the WAM
    // transform of the tool wrt WAM link 7 (18 deg about +Y)
    vctMatrixRotation3<double> R7t( Rst );
    // position of the TCP wrt WAM link 7
    vctFixedSizeVector<double,3> t7t( 0.0, 0.0, 0.15 );
    vctFrame4x4<double> Rt7t( R7t, t7t ); 

    // this is used to evaluate the kinematics and Jacobian
    robManipulator* robWAM = new robManipulator( argv[2], Rtw0 );

   // Create a tool and attach it to the WAM
    robManipulator* robtool = new robManipulator( Rt7t );

    // mass and center of the tool (measured)
    double mass = 0.145;
    vctFixedSizeVector<double,3> com( 0.0, 0.0, 0.03 );

    // JR3 FT sensor
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

    osaWAM* wam = new osaWAM( &can );
    wam->Initialize();
    wam->SetPositions( qinit );
    
    // controller part
    osaHybridForcePosition::Mask mask( osaHybridForcePosition::POSITION );
    vctFixedSizeVector<double,6> K( .3, .3, .3, .1, .1, .1 );
    osaHybridForcePosition* hfp = NULL;
    hfp = new osaHybridForcePosition( mask, robWAM, K );
    
    osaGravityCompensation* gc = NULL;
    gc = new osaGravityCompensation( argv[2], Rtw0 );

    mtsHybridForcePosition* ctrl = NULL;
    // Control is a required interface
    ctrl = new mtsHybridForcePosition( "Control", 1.0/500.0,
                                       argv[2], Rtw0, Rt7t, qinit, qready,
                                       jr3, gc, hfp );
    taskManager->AddComponent( ctrl );

    mtsPIDAntiWindup* mtspid = new mtsPIDAntiWindup( "PID", 1.0/900.0, wam, osapid );
    taskManager->AddComponent( mtspid );

// ------------------- Connecting ---------------------------

// ------------ Use key 'C' to enable GC controller ---------

// Control is a provided interface created by mtsController
 if( !taskManager->Connect( kb.GetName(), "GCEnable",
			    GC.GetName(), "Control") ){
    std::cout << "Failed to connect: "
	      << kb.GetName() << "::GCEnable to "
	      << GC.GetName() << "::Control" << std::endl;
    return -1;
  }

  if( !taskManager->Connect( WAM.GetName(), "Input",
			     GC.GetName(),  "Output" ) ){
    std::cout << "Failed to connect: "
	      << WAM.GetName() << "::Input to "
	      << GC.GetName()  << "::Output" << std::endl;
    return -1;
  }

  if( !taskManager->Connect( WAM.GetName(), "Output",
			     GC.GetName(),  "Input" ) ){
    std::cout << "Failed to connect: "
	      << WAM.GetName() << "::Output to "
	      << GC.GetName()  << "::Input" << std::endl;
    return -1;
  }



//----------- connecting keyboard to hybrid controller------------
    // WRONG USE??
 if( !taskManager->Connect( kb.GetName(), "Control",
			    ctrl->GetName(), "Control") ){
    std::cout << "Failed to connect: "
	      << kb.GetName() << "::Control to "
	      << ctrl->GetName() << "::Control" << std::endl;
    return -1;
  }

// connecting pid antiwindup to hybrid controller
// Slave in mtspid is a provided interface
// Slave in ctrl is required interface
 if( !taskManager->Connect( mtspid->GetName(), "Slave",
			    ctrl->GetName(), "Slave") ){
    std::cout << "Failed to connect: "
	      << mtspid->GetName() << ":Slave to "
	      << ctrl->GetName() << ":Slave" << std::endl;
    return -1;
  }
 
  taskManager->CreateAll();
  taskManager->StartAll();

  pause();


    taskManager->KillAll();
    taskManager->WaitForStateAll(mtsComponentState::FINISHED, 2.0 * cmn_s);

    taskManager->Cleanup();


    //osaJR3ForceSensor jr3("/dev/comedi0");
    //jr3.Open();

    //osaJR3ForceSensor::Wrench ft;
    //jr3.Read(ft);
    //std::cout<<ft<<std::endl;

    //jr3.Close();

    return 0;
}
