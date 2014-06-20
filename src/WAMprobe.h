class WAMprobe : public mtsTaskPeriodic {

private:
  
  mtsFunctionRead  GetPositions;
  mtsFunctionWrite SetTorques;

public:

  WAMprobe() : mtsTaskPeriodic( "WAMprobe", 0.002, true ){

    mtsInterfaceRequired* input = AddInterfaceRequired( "Input" );
    mtsInterfaceRequired* output = AddInterfaceRequired( "Output" );

    input->AddFunction( "GetPositionJoint", GetPositions );
    output->AddFunction( "SetTorqueJoint", SetTorques );

  }

  void Configure( const std::string& ){}
  void Startup(){}
  void Run(){
    ProcessQueuedCommands();
    
    prmPositionJointGet q;
    GetPositions( q );

    std::cout << q.Position() << std::endl;
    
    prmForceTorqueJointSet t;
    t.SetSize( 7 );
    t.ForceTorque().SetAll( 0.0 );
    SetTorques( t );

  }
  
  void Cleanup(){}

};
