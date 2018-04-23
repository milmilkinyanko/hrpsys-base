// -*- C++ -*-
/*!
 * @file  Stabilizer.h
 * @brief stabilizer component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef STABILIZER_COMPONENT_H
#define STABILIZER_COMPONENT_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "TwoDofController.h"
#include "ZMPDistributor.h"
#include "../ImpedanceController/JointPathEx.h"
#include "../ImpedanceController/RatsMatrix.h"
#include "../TorqueFilter/IIRFilter.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

/**
   \brief sample RT component which has one data input port and one data output port
 */

class Stabilizer
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  Stabilizer(RTC::Manager* manager);
};

#endif // STABILIZER_COMPONENT_H
