#
#
# (C) Copyright 2011-2012 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

set(
  REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  cisstRobot
  cisstNumerical )

find_package(cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if( cisst_FOUND  )

  # load cisst configuration
  include( ${CISST_USE_FILE} )
  
  cisst_find_saw_component( sawCANBus REQUIRED )
  cisst_find_saw_component( sawBarrett REQUIRED )
  cisst_find_saw_component( sawControllers REQUIRED )
  cisst_find_saw_component( sawKeyboard REQUIRED )
  cisst_find_saw_component( sawJR3ForceSensor REQUIRED )
 
  include_directories(
    ${sawCANBus_INCLUDE_DIR}
    ${sawBarrett_INCLUDE_DIR}
    ${sawKeyboard_INCLUDE_DIR}
    ${sawControllers_INCLUDE_DIR}
    ${sawJR3ForceSensor_INCLUDE_DIR} )

  
  link_directories( 
    ${sawCANBus_LIBRARY_DIR}
    ${sawBarrett_LIBRARY_DIR}
    ${sawControllers_LIBRARY_DIR} 
    ${sawKeyboard_LIBRARY_DIR}
    ${sawJR3ForceSensor_LIBRARY_DIR} )

  
  set(
    REQUIRED_SAW_COMPONENTS
    sawCANBus
    sawBarrett
    sawControllers
    sawJR3ForceSensor
    sawKeyboard )
  
  add_executable( 
      example 
      src/example.cpp 
      src/osaHybridForcePosition.cpp
      src/mtsHybridForcePosition.cpp )

  target_link_libraries( example ${REQUIRED_SAW_COMPONENTS} )
  cisst_target_link_libraries(example ${REQUIRED_CISST_LIBRARIES})
  
endif( cisst_FOUND )
