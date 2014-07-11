/*-*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-   */
/*ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:*/

/*
  $Id: mtsHybridForcePosition.cpp 3181 2011-11-15 15:41:28Z sleonard $

  Author(s):  Simon Leonard
  Created on: 2013-07-22

  (C) Copyright 2012 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/
/*
    osaJR3ForceSensor::Wrench zero( 0.0 );
    osaJR3ForceSensor 
    jr3.Open();
*/

#include "mtsHybridForcePosition.h"
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

mtsHybridForcePosition::mtsHybridForcePosition
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
    robot( robotfilename, Rtw0),
    tool(Rtnt),
    
    traj( NULL ),
    timer( 0.0 ),
    Rts( Rtnt.Rotation().Transpose() ),
    Rtnt(Rtnt),
    jr3( jr3 ),
    gc( gc ),
    hfp( hfp ),
    
    slave( NULL ),
    control( NULL ),

    qready( qready ),
    qsold( qinit ),
    tauold( 7, 0.0 ),
    
    state( DONOTHING ),
    enable( false ),

    fz( 0.0 ),
    sg( nmrSavitzkyGolay( 1, 0, 100, 0 ) ){

    robot.Attach( &tool );
    
    control = AddInterfaceRequired( "Control" );
    if( control ){
       
        control->AddEventHandlerVoid( &mtsHybridForcePosition::Move,
                                      this,
                                      "Move");
        control->AddEventHandlerVoid( &mtsHybridForcePosition::ToIdle,
                                      this,
                                      "ToIdle");

       }



    //-------------------------------------------------
    

    slave = AddInterfaceRequired( "Slave" );
    if( slave ){
        slave->AddFunction( "GetPositionMSR", GetPosition );
        slave->AddFunction( "SetPositionCMD", SetPosition );
    }


      ofsForceData.open("/home/lixiao/Desktop/Data1.txt");
      startTime = osaGetTime();

}
 
    void mtsHybridForcePosition::Configure( const std::string& ){}
    void mtsHybridForcePosition::Startup(){
        osaCPUSetAffinity( OSA_CPU2 );
        Thread.SetPriority( 70 );
    }

    void mtsHybridForcePosition::Run(){ 
        
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

        if( state == MOVE )    { MoveTraj();      }
        else if(state == IDLE) { Idle();          }
        else                   { HybridControl(); }

    }

    void mtsHybridForcePosition::Idle(){

        timer = osaGetTime();

              // current joints
        prmPositionJointGet prmq; 
        GetPosition( prmq );
        vctDynamicVector<double> q = prmq.Position();

        // current Cartesian pose
        vctFrame4x4<double> Rtwt = robot.ForwardKinematics( q );
       

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
            
          //  std::cout<<ft[0]<<",   "<<ft[1]<<",   "<<ft[2]<<",   "<<ft[3]<<",   "<<ft[4]<<",   "<<ft[5]<<std::endl;

           // ofsForceData<< timer - startTime <<","<<ft[0]<<", "<<ft[1]<<", "<<ft[2]<<std::endl;

         ofsForceData<< timer - startTime <<","<<q[0]<<", "<<q[1]<<", "<<q[3]<<", "<<q[4]<<", "<<q[5]<<", "<<q[6]<<", "<<Rtwt[0][0]<<", "<<Rtwt[0][1]<<", "<<Rtwt[0][2]<<", "<<Rtwt[0][3]<<", "<<Rtwt[1][0]<<", "<<Rtwt[1][1]<<", "<<Rtwt[1][2]<<", "<<Rtwt[1][3]<<", "<<Rtwt[2][0]<<", "<<Rtwt[2][1]<<", "<<Rtwt[2][2]<<", "<<Rtwt[2][3]<<", "<<Rtwt[3][0]<<", "<<Rtwt[3][1]<<", "<<Rtwt[3][2]<<", "<<Rtwt[3][3]<<std::endl;


    }

   
void mtsHybridForcePosition::MoveTraj(){

            prmPositionJointGet prmq; 
            GetPosition( prmq );
            qready = prmq.Position();

            vctFrame4x4<double> Rtwt = robot.ForwardKinematics( qready );
        

            Rtwtsold = robot.ForwardKinematics( qready );
            Rtwtsoldcmd = robot.ForwardKinematics( qready );
            Rtwtsoldtrj = robot.ForwardKinematics( qready );

            vctFrame4x4<double> Rtwts(Rtwt);

            Rtwtsoldcmd = Rtwt;
            // move along the Y axis for 0.05m
            Rtwts[2-1][4-1] += 0.5; // CHANGE THIS to the correct direction and value

            // create a 10s trajectory from qready to Rtwts
            if( traj != NULL ) { delete traj; }
            traj = new robLinearSE3( Rtwtsoldcmd, Rtwts, 10.0 );
               
           jr3->Zero( Rtwtsold );
          
          // state = HYBRID;

        }




    void mtsHybridForcePosition::HybridControl(){
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

        case HYBRID:
            {

                vctDynamicVector<double> qs( qsold );
               timer = osaGetTime();
                
                vctFrame4x4<double> Rttt; // trajectory motion increment
           //if( traj != NULL && IsEnabled() ){
           if( traj != NULL ){
                vctFrame4x4<double> Rtwt;
                vctFixedSizeVector<double,6> vw( 0.0 ), vdwd( 0.0 );
                // Rtwt is the interpolated new position, vw, vdwd interpolated velocity and acceleration respectively
                traj->Evaluate( osaGetTime()-timer, Rtwt, vw, vdwd );
                vctFrame4x4<double> Rttw( Rtwtsoldtrj );
                Rttw.InverseSelf();
                Rttt = Rttw * Rtwt;
                Rtwtsoldtrj = Rtwt;
            }

    
            // this is the desired position of the slave
           // vctFrame4x4<double> Rtwts = Rtwtsold * Rttt;
            vctFrame4x4<double>  Rtwts = Rtwtsold * Rttt;

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
           // fts[2] = fz;
            fts[2] = -7; // CHANGE THIS to the right value !

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

           // validse3 = false;
           // prmCommandSE3.SetValid( validse3 ); // desired position
            
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
