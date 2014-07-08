#ifndef _MTSPIDANTIWINDUP_H
#define _MTSPIDANTIWINDUP_H

#include <sawControllers/osaPIDAntiWindup.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <sawBarrett/osaWAM.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <sawControllers/osaGravityCompensation.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
class mtsPIDAntiWindup : public mtsTaskPeriodic {
    
    osaPIDAntiWindup* pid;
    osaWAM* wam;
    osaGravityCompensation* gc;

    mtsInterfaceProvided* slave;
    mtsInterfaceRequired* GC;

    enum State{ PID, GCOMP };
    State state;

    prmPositionJointGet qs;
    prmPositionJointGet q;
    std::list<double> dt;


  void GravityCompensation(){state = GCOMP;};
  void MovePID()            {state = PID;};
public:

    
    mtsPIDAntiWindup( const std::string& name,
                      double period,
                      osaWAM* wam,
                      osaPIDAntiWindup* pid, 
                      osaGravityCompensation* gc) :
        mtsTaskPeriodic( name, period ),
        wam( wam ),
        pid( pid ),
        gc( gc ),
        state(PID){
        
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
        
        GC = AddInterfaceRequired("GC");
        if( GC ){

          GC->AddEventHandlerVoid( &mtsPIDAntiWindup::GravityCompensation,
                                      this,
                                      "GravityCompensation");
 
          GC->AddEventHandlerVoid( &mtsPIDAntiWindup::MovePID,
                                      this,
                                      "MovePID");
 

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
           dt.clear();
        }

       ProcessQueuedCommands(); 
        ProcessQueuedEvents(); 

        if(state == GCOMP){

            RunGC();
        }
        else{
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
    }


void RunGC(){

      if( wam->GetPositions( q.Position() ) != osaWAM::ESUCCESS ){
            CMN_LOG_RUN_ERROR << "Failed to get positions" << std::endl;
            bool valid=false;
            q.SetValid( valid );
        }
        else{
            bool valid=true;
            q.SetValid( valid );
        }

        vctDynamicVector<double> tau( q.Position().size(), 0.0 );
      if( gc != NULL && state == GCOMP ){

        if( gc->Evaluate( q.Position(), tau ) != 
          osaGravityCompensation::ESUCCESS ){
          CMN_LOG_RUN_ERROR << "Faile to evaluate the controller" << std::endl;
        }

       //vctDynamicVector<double> qc( prmq.Position() );
       
            SendTorques( tau ); 
        }
        else{
            SendTorques( vctDynamicVector<double>( 7, 0.0 ) ); 
        }

        }


};

#endif
