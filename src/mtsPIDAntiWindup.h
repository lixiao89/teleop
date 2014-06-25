#ifndef _MTSPIDANTIWINDUP_H
#define _MTSPIDANTIWINDUP_H

#include <sawControllers/osaPIDAntiWindup.h>
#include <sawBarrett/osaWAM.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>

#include <cisstParameterTypes/prmPositionJointGet.h>
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
        
       /* if( wam->GetPositions( q.Position() ) != osaWAM::ESUCCESS ){
            CMN_LOG_RUN_ERROR << "Failed to get positions" << std::endl;
        }*/
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

#endif
