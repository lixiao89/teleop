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

#include <cisstVector/vctRodriguezRotation3.h>
#include <cisstVector/vctMatrixRotation3.h>
#include <cisstNumerical/nmrLSMinNorm.h>

#include "osaHybridForcePosition.h"

osaHybridForcePosition::osaHybridForcePosition
( const Mask& mask, 
  robManipulator* robot,
  const vctFixedSizeVector<double,6>& K ) : 
    mask( mask ),
    K( K ),
    robot( robot ){}

void osaHybridForcePosition::SetMask( const Mask& mask )
{ this->mask = mask; }

osaHybridForcePosition::Errno
osaHybridForcePosition::Evaluate
( vctDynamicVector<double>& qs,
  const vctDynamicVector<double>& q,
  const vctFixedSizeVector<double,6>& ft,
  const vctFixedSizeVector<double,6>& fts,
  vctFrame4x4<double>& Rtwt,
  const vctFrame4x4<double>& Rtwts ){

    // ft error
    vctFixedSizeVector<double,6> eft = fts - ft;

    // compute increment
    vctFixedSizeVector<double,3> v( 0.0 );
    for( size_t i=0; i<3; i++ ){
        switch( mask[i] ){
        case osaHybridForcePosition::IDLE:
        case osaHybridForcePosition::POSITION:
            break;
        case osaHybridForcePosition::FORCE:
            if( eft[i] < -1 )                          { v[i]= 0.00001; }
            if(     -1 <= eft[i] && eft[i] < 0 )       { v[i]= 0.00001*eft[i]; }
            if(      0 <= eft[i] && eft[i] < 1 )       { v[i]=-0.00001*eft[i]; }
            if(                          1 <= eft[i] ) { v[i]=-0.00001; }
            if( fabs( eft[i] ) < 0.5 )                 { v[i]= 0.0; }
            break;
        }
    }

    vctFrame4x4<double> Rtts( vctMatrixRotation3<double>(), v );
    robManipulator::Errno manerrno;
    qs = q;
    Rtwt = Rtwts*Rtts;
    manerrno = robot->InverseKinematics( qs, Rtwt, 1e-6, 300 );
    // if ikin screwed up?
    if( manerrno != robManipulator::ESUCCESS ){ 
        qs = q; 
        return osaHybridForcePosition::EFAILURE;
    }

    return osaHybridForcePosition::ESUCCESS;
    
}
