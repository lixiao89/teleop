
#include "mtsNewHybridForcePosition.h"
#include <cisstParameterTypes/prmForceTorqueJointSet.h>
#include <cisstOSAbstraction/osaCPUAffinity.h>

osaJR3ForceSensor::Wrench 
convolve
( const std::list< osaJR3ForceSensor::Wrench >& stdft,
  const vctDynamicVector<double>& sg ){
    
    osaJR3ForceSensor::Wrench ft( 0.0 );
    std::list< osaJR3ForceSensor::Wrench >::const_iterator fti;
    size_t i=0;
    for( fti=stdft.begin(); fti!=stdft.end(); fti++, i++ )
        { ft = ft + sg[i] *(*fti); }

    return ft;
}

mtsNewHybridForcePosition::mtsNewHybridForcePosition
( const std::string& name,
  double period,
  
  const std::string& robotfilename, 
  const vctFrame4x4<double>& Rtw0,
  const vctFrame4x4<double>& Rtnt,
  const vctDynamicVector<double>& qinit,
  const vctDynamicVector<double>& qready,
  
  osaJR3ForceSensor* jr3,
  osaGravityCompensation* gc,
  osaHybridForcePosition* hfp ):
    
    mtsTaskPeriodic( name, period, true ),

    robot( robotfilename, Rtw0 ),
    tool( Rtnt ),
    traj( NULL ),
    timer( 0.0 ),
    Rts( Rtnt.Rotation().Transpose() ),
    
    jr3( jr3 ),
    gc( gc ),
    hfp( hfp ),
    
    mastertel( NULL ),
    mastercmd( NULL ),
    slave( NULL ),
    control( NULL ),

    qready( qready ),
    qsold( qinit ),
    tauold( 7, 0.0 ),
    
    state( IDLE ),
    enable( false ),

    fz( 0.0 ),
    sg( nmrSavitzkyGolay( 1, 0, 100, 0 ) ){

    robot.Attach( &tool );
    
    control = AddInterfaceRequired( "Control" );
    if( control ){
        control->AddEventHandlerVoid( &mtsNewHybridForcePosition::Reset, 
                                      this, 
                                      "Reset" );
        control->AddEventHandlerVoid( &mtsNewHybridForcePosition::Enable, 
                                      this, 
                                      "Enable" );
        control->AddEventHandlerVoid( &mtsNewHybridForcePosition::Test, 
                                      this, 
                                      "Test" );
        control->AddEventHandlerVoid( &mtsNewHybridForcePosition::Force, 
                                      this, 
                                      "Force" );
    }

          
}

void mtsNewHybridForcePosition::Configure( const std::string& ){}
void mtsNewHybridForcePosition::Startup(){
    osaCPUSetAffinity( OSA_CPU2 );
    Thread.SetPriority( 70 );
}
void mtsNewHybridForcePosition::Run(){ 
    
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
        //std::cout << fabs(GetPeriodicity() - avg / dt.size()) << " " 
        //        << fabs( GetPeriodicity() - max ) << std::endl;
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

    if( state == RESET ) { MoveToReady();   }
    else                 { HybridControl(); }

}

    

void mtsNewHybridForcePosition::HybridControl(){
    // current joints
    prmPositionJointGet prmq; 
    GetPosition( prmq );
    vctDynamicVector<double> q = prmq.Position();

    bool valid = true;
    prmTelemetryRn.SetValid( valid );
    prmTelemetryRn.SetPosition( q );
    
    // current Cartesian pose
    vctFrame4x4<double> Rtwt = robot.ForwardKinematics( q );
    vctFrm3 tmp( vctMatrixRotation3<double>( Rtwt.Rotation() ), 
                 Rtwt.Translation() );
    prmTelemetrySE3.SetValid( valid );
    prmTelemetrySE3.SetPosition( tmp );

    switch( state ){

    case TEST:
        {
            double f=0.5;
            static double t = 0.0;

            qsold[0] += f*0.0005*sin( t );
            qsold[1] += f*0.001*sin( t );
            qsold[2] += f*0.0005*sin( t );
            qsold[3] -= f*0.001*sin( t );
            qsold[4] += f*0.0005*sin( t );
            qsold[5] += f*0.001*sin( t );
            qsold[6] += f*0.001*sin( t );

            t += 0.0005;
            bool valid=true;
            prmq.SetValid( valid );
            prmq.Position() = qsold;
            SetPosition( prmq );

        }
        break;
    case HYBRID:

        {

            // desired joints (initialize to old command)
            vctDynamicVector<double> qs( qsold );
            // Cartesian shit
            
            // Read the master
            bool validse3 = false;
            prmCommandSE3.GetValid( validse3 ); // desired position

            // read the master command if it is valid
            if( validse3 && IsEnabled() ){
                
                // desired Cartesian pose
                vctFrm3 Rt = prmCommandSE3.Position();
                vctFrame4x4<double> Rtwts( Rt.Rotation(), Rt.Translation() );
                
                // create a 100ms trajectory 
                if( traj != NULL ) { delete traj; }
                traj = new robLinearSE3( Rtwtsoldcmd, Rtwts, 1.0/100.0 );
                // this is the time that the command was received
                timer = osaGetTime();
                
                // update old cartesian master command
                Rtwtsoldcmd = Rtwts;

            }

            // Evaluate the motion increment from the trajectory
            vctFrame4x4<double> Rttt; // trajectory motion increment
            if( traj != NULL && IsEnabled() ){
                vctFrame4x4<double> Rtwt;
                vctFixedSizeVector<double,6> vw( 0.0 ), vdwd( 0.0 );
                traj->Evaluate( osaGetTime()-timer, Rtwt, vw, vdwd );
                vctFrame4x4<double> Rttw( Rtwtsoldtrj );
                Rttw.InverseSelf();
                Rttt = Rttw * Rtwt;
                Rtwtsoldtrj = Rtwt;
            }

            // this is the desired position of the slave
            vctFrame4x4<double> Rtwts = Rtwtsold * Rttt;

            // extract the rotation of the tool
            vctMatrixRotation3<double> Rwt( Rtwt.Rotation() );
            
            // now get the orientation of the sensor
            vctMatrixRotation3<double> Rws( Rwt * Rts );

            // Current force, compensate for sensor orientation
            osaJR3ForceSensor::Wrench ft;
            jr3->Read( ft, Rws, true, 3 );
            
            // filter the reading
            stdft.push_back( ft );
            if( sg.size() < stdft.size() ) { stdft.pop_front(); }
            ft = convolve( stdft, sg );
            
            // desired force
            vctDynamicVector<double> fts( 6, 0.0 );
            fts[2] = fz;
            
            // if non zero desired force along Z
            if( 0 < fabs( fts[2] ) ){
                osaHybridForcePosition::Mask 
                    mask( osaHybridForcePosition::POSITION );
                mask[2] = osaHybridForcePosition::FORCE; // Force along Z
                hfp->SetMask( mask );
            }
            
            // otherwise all position
            else{
                osaHybridForcePosition::Mask 
                    mask( osaHybridForcePosition::POSITION );
                hfp->SetMask( mask );
            }

            // evaluate the HFP
            // Rtwtsold is now updated to the new position of the slave
            if( hfp != NULL ){ hfp->Evaluate(qs, qsold, ft, fts, Rtwtsold, Rtwts); }
            else{ 
                CMN_LOG_RUN_ERROR << "No controller" << std::endl;
                exit(-1);
            }
            
            bool valid=true;
            prmq.SetValid( valid );
            prmq.Position() = qs;

            SetPosition( prmq );
            qsold = qs;

            validse3 = false;
            prmCommandSE3.SetValid( validse3 ); // desired position
            
        }
        break;

    default:
        {
            if( IsEnabled() && traj == NULL ){
                bool valid=true;
                prmq.SetValid( valid );
                prmq.Position() = qsold;
                SetPosition( prmq );
            }
            else{
                bool valid=false;
                prmq.SetValid( valid );
                prmq.Position() = qsold;
                SetPosition( prmq );
            }
        }
        break;

    }
    
}
