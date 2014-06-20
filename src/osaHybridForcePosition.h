/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-  */
/*ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:*/

/*
  $Id: osaHybridForcePosition.h 4416 2013-08-20 01:45:09Z sleonar7 $
  
  Author(s):  Simon Leonard
  Created on: 2013-12-20
  
  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.
  
  --- begin cisst license - do not edit ---
  
  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.
  
--- end cisst license ---
*/

#ifndef _osaHybridForcePosition_h
#define _osaHybridForcePosition_h

#include <cisstRobot/robManipulator.h>
#include <cisstVector/vctFrame4x4.h>
#include <cisstVector/vctDynamicVector.h>
#include <cisstVector/vctFixedSizeVector.h>

class CISST_EXPORT osaHybridForcePosition{

 public:

  enum Errno{ ESUCCESS, EFAILURE };
  
  enum Type { IDLE, POSITION, FORCE };
  typedef vctFixedSizeVector<int,6> Mask;
  
 private:
  
  Mask mask;
  vctFixedSizeVector<double,6> K;
  robManipulator* robot;
  
 public:

  osaHybridForcePosition( const Mask& mask,
                          robManipulator* robot,
                          const vctFixedSizeVector<double,6>& K );
  
  void SetMask( const Mask& mask );

  osaHybridForcePosition::Errno
      Evaluate
      ( vctDynamicVector<double>& qd,
        const vctDynamicVector<double>& q,
        const vctFixedSizeVector<double,6>& ft,
        const vctFixedSizeVector<double,6>& fts,
        vctFrame4x4<double>& Rt,
        const vctFrame4x4<double>& Rts );
  
};

#endif
