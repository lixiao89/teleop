#ifndef _WAMPROBE_H
#define _WAMPROBE_H

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmPositionJointGet.h>

#include <cisstVector/vctDynamicVector.h>


class WAMprobe : public mtsTaskPeriodic {

private:
  
  mtsFunctionRead  GetPositions;

//  mtsFunctionWrite SetTorques;

public:

vctDynamicVector<double> qr; // current joint angles (which is also used as qready)

enum State{ IDLE, SET};
State state;


  WAMprobe() : mtsTaskPeriodic( "WAMprobe", 0.002, true ){

    mtsInterfaceRequired* input = AddInterfaceRequired( "Input" );
  //  mtsInterfaceRequired* output = AddInterfaceRequired( "Output" );

    mtsInterfaceRequired* setqready = AddInterfaceRequired("Setqr");

    input->AddFunction( "GetPositionJoint", GetPositions );
  //  output->AddFunction( "SetTorqueJoint", SetTorques );
  
  state = IDLE;  
  
    if(setqready){

       // setqready->AddEventHandlerVoid(&WAMprobe::Idle,this,"Idle");
        setqready->AddEventHandlerVoid(&WAMprobe::Setqready,this,"Setqready");
    } 


  }

  void Idle(){};
  void Setqready(){state = SET;};
  void SetqreadyExecute(){

    prmPositionJointGet q;
    GetPositions( q );

    qr = q.Position();
   // std::cout << q.Position() << std::endl;
    
  };

  void Configure( const std::string& ){}
  void Startup(){}
  void Run(){
    ProcessQueuedCommands();
    
   if( state == IDLE) {Idle();}
   else               {SetqreadyExecute(); state = IDLE;};
   // prmForceTorqueJointSet t;
   // t.SetSize( 7 );
   // t.ForceTorque().SetAll( 0.0 );
   // SetTorques( t );

  }
  
  void Cleanup(){}

};

#endif
