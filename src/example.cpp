#include <cisstCommon/cmnPath.h>
#include <sawBarrett/osaWAM.h>
#include <sawJR3ForceSensor/osaJR3ForceSensor.h>
#include <sawKeyboard/mtsKeyboard.h>
#include <sawCANBus/osaSocketCAN.h>
#include <sawControllers/osaPIDAntiWindup.h>
#include <cisstOSAbstraction/osaCPUAffinity.h>
#include <cisstParameterTypes/prmPositionJointGet.h>

#include <cisstCommon/cmnGetChar.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaGetTime.h>
#include <sawControllers/mtsGravityCompensation.h>
#include <sawBarrett/mtsWAM.h>


#include "mtsHybridForcePosition.h"

class mtsPIDAntiWindup : public mtsTaskPeriodic {
    
    osaPIDAntiWindup* pid;
    osaWAM* wam;

    mtsInterfaceProvided* slave;
    prmPositionJointGet qs;
    prmPositionJointGet q;
    std::list<double> dt;

public:

    mtsPIDAntiWindup( const std::string& name,
                      double period,
                      osaWAM* wam,
                      osaPIDAntiWindup* pid ) :
        mtsTaskPeriodic( name, period ),
        wam( wam ),
        pid( pid ){
        
        if( wam->GetPositions( q.Position() ) != osaWAM::ESUCCESS ){
            CMN_LOG_RUN_ERROR << "Failed to get positions" << std::endl;
        }
        qs.Position() = q.Position();

        slave = AddInterfaceProvided( "Slave" );
        if( slave ){
            StateTable.AddData( qs, "PositionCMD" );
            StateTable.AddData( q,  "PositionMSR" );
            slave->AddCommandWriteState( StateTable, qs, "SetPositionCMD" );
            slave->AddCommandReadState( StateTable, q,  "GetPositionMSR" );
        }
        else{
            CMN_LOG_RUN_ERROR << "Failed to create interface Output for " << GetName()
                              << std::endl;
        }
        
    }
    ~mtsPIDAntiWindup(){}


    void SendTorques ( const vctDynamicVector<double>& tau )
    { wam->SetTorques( tau ); }

    vctDynamicVector<double> 
    PIDEvaluate
    ( const vctDynamicVector<double>& q,
      const vctDynamicVector<double>& qs ){
        
        // period
        double dt = GetPeriodicity();
        
        vctDynamicVector<double> qtmp( q );
        
        // evaluate the PID
        vctDynamicVector<double> tau( q.size(), 0.0 );
        if( pid->Evaluate( qs, qtmp, tau, dt )  != osaPIDAntiWindup::ESUCCESS ){
            CMN_LOG_RUN_ERROR << "Failed to evaluate controller" << std::endl;
            exit(-1);
        }
        
        return tau;
        
    }
    
    void Configure( const std::string& ){}
    void Startup(){         
        osaCPUSetAffinity( OSA_CPU3 ); 
        Thread.SetPriority( 80 );
    }
    void Cleanup(){}

    void Run(){ 

        // Timing stuff
        static double t1 = osaGetTime();
        double t2 = osaGetTime();
        dt.push_back( t2 - t1 );
        t1 = t2;
        
        if( 1.0/GetPeriodicity() < dt.size() ){
            
            std::list<double>::iterator it=dt.begin();
            double avg=0.0;
            double max=0.0;
            for( ; it!=dt.end(); it++ ){
                avg += *it;
                if( max < *it ) max = *it;
            }
            //std::cout << std::setw(15) << 1.0/(avg/dt.size())
            //          << std::setw(15) << fabs(GetPeriodicity()-avg/dt.size())
            //          << std::setw(15) << fabs(GetPeriodicity()-max)
            //          << std::endl;
            dt.clear();
        }

        /*
        cpu_set_t mask;
        sched_getaffinity( 0, sizeof( cpu_set_t ), &mask );
        std::cout << CPU_ISSET( 0, &mask ) << " "
                  << CPU_ISSET( 1, &mask ) << " "
                  << CPU_ISSET( 2, &mask ) << " "
                  << CPU_ISSET( 3, &mask ) << std::endl;            
        */

        // get all the stuff 
        ProcessQueuedCommands(); 
        ProcessQueuedEvents(); 

        // current joints
        if( wam->GetPositions( q.Position() ) != osaWAM::ESUCCESS ){
            CMN_LOG_RUN_ERROR << "Failed to get positions" << std::endl;
            bool valid=false;
            q.SetValid( valid );
        }
        else{
            bool valid=true;
            q.SetValid( valid );
        }

        bool valid=false;
        qs.GetValid( valid );
        if( valid ){
            //std::cout << GetName() << " " << qs.Position() << std::endl;
            SendTorques( PIDEvaluate( q.Position(), qs.Position() ) ); 
        }
        else{
            SendTorques( vctDynamicVector<double>( 7, 0.0 ) ); 
        }
    }

};

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
    kb.AddKeyWriteFunction( 'C', "GCEnable", "Enable", true );
    taskManager->AddComponent( &kb );

  osaSocketCAN can( "rtcan1", osaCANBus::RATE_1000 );
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


  cmnPath path;
  path.AddRelativeToCisstShare("/models/WAM");

  // Rotate the base
  vctMatrixRotation3<double> Rw0(  0.0,  0.0, -1.0,
                                   0.0,  1.0,  0.0,
                                   1.0,  0.0,  0.0 );
  vctFixedSizeVector<double,3> tw0(0.0);
  vctFrame4x4<double> Rtw0( Rw0, tw0 );

  mtsGravityCompensation GC( "GC", 
			     0.002, 
			    // path.Find( "wam7.rob" ), 
                "/home/lixiao/src/wvu-jhu/models/WAM/wam7cutter.rob",
			     Rtw0,
			     OSA_CPU3 );
  taskManager->AddComponent( &GC );
 
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

  taskManager->CreateAll();
  taskManager->StartAll();

  //pause();

    while(1){

        osaSleep(100.0 * cmn_ms);
    }

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
