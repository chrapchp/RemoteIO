/**
 *  @file    DA_LightPositionMgr.h
 *  @author  peter c
 *  @date    10/09/2018
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *  Handle position light height position via setpoints and manual calibration
 *
 *.
 */
#include "DA_LightPositionMgr.h"
#include <Streaming.h>



DA_LightPositionMgr::DA_LightPositionMgr(Encoder &aEncoder,
                                         DA_DiscreteOutput &aIn1,
                                         DA_DiscreteOutput &aIn2,
                                         DA_DiscreteOutput &aEnA)
    : positionEncoder(aEncoder), in1(aIn1), in2(aIn2), enA(aEnA) {
  holdLightPosition();
}

void DA_LightPositionMgr::refresh() {
  computeLightPosition();


  if (calibrationMode) {
    processCalibrationCommands();
  } else
    doAutomatedControls();
  detectAndProcessTransitionToStop();

}
void DA_LightPositionMgr::serialize(Stream *aOutputStream, bool includeCR) {
  /*
      *aOutputStream << "";

      if (includeCR)
        *aOutputStream << endl;
        **/
  *aOutputStream << "Pos:" << currentPositionCount << "  0-100:" << pv
                 << " SP:" << sp << " error:" << processError
                 << " In1:" << in1.isActive() << " In2:" << in2.isActive()
                 << " EnA:" << enA.isActive() << " MaxCount:" << maxPulses
                 << " State " << lightPositionMgrState
                 << " lightMoving:" << lightMoving
                 << " isAtTopPosition:" << (lightPositionMgrState == IdleAtTop)
                 << " isAtBottomPosition:"
                 << (lightPositionMgrState == IdleAtBottom);

  if (includeCR)
    *aOutputStream << endl;
}

void DA_LightPositionMgr::computeLightPosition() {
  lightMoving = false;
  currentPositionCount = positionEncoder.read();
  if (currentPositionCount != previousPositionCount) {
    lightMoving = true;
    previousPositionCount = currentPositionCount;
    pv = 100.0 - (100.0 * currentPositionCount / maxPulses);
    // pv = 100.0 * currentPositionCount / maxPulses;
  }
}

void DA_LightPositionMgr::processCalibrationCommands() {

  if (calibrationMoveBottom && !calibrationMoveTop) {
    if (!isAtBottomPosition())
      lowerLights();
  } else if (calibrationMoveTop && !calibrationMoveBottom) {
    if (!isAtTopPosition())
      raiseLights();

  } else
    holdLightPosition();
}

void DA_LightPositionMgr::doAutomatedControls() {
  processError = pv - sp;
  if (processError > DA_LIGHTPOSITION_DEFAULT_LIGHT_POSITION_DEADBAND) {
    if (!isAtBottomPosition())
      lowerLights();
  } else if (processError < -DA_LIGHTPOSITION_DEFAULT_LIGHT_POSITION_DEADBAND) {
    if (!isAtTopPosition())
      raiseLights();
  } else {
    holdLightPosition();
  }
}
void DA_LightPositionMgr::lowerLights() {
  if (lightPositionMgrState != Lowering) {
    enA.write(false); // stop the motor before changing directions
    in1.write(false);
    in2.write(true);
    enA.write(true);
    lightPositionMgrState = Lowering;
  }
}
void DA_LightPositionMgr::raiseLights() {
  if (lightPositionMgrState != Rising) {
    enA.write(false); // stop the motor before changing directions
    in1.write(true);
    in2.write(false);
    enA.write(true);
    lightPositionMgrState = Rising;
  }
}

void DA_LightPositionMgr::holdLightPosition() {
  if (lightPositionMgrState != IdleAtTop ||
      lightPositionMgrState != IdleAtBottom)
    lightPositionMgrState = Idle;
  enA.write(false);
  in1.write(false);
  in2.write(false);
}

bool DA_LightPositionMgr::isAtTopPosition() {
  bool lRetVal = false;
  if (lightPositionMgrState == IdleAtTop ||
      (lightPositionMgrState == Rising && !lightMoving)) {
    lightPositionMgrState = IdleAtTop;
    positionEncoder.write(0);
    lRetVal = true;
  }

  return (lRetVal);
}

bool DA_LightPositionMgr::isAtBottomPosition() {
  bool lRetVal = false;
  if (lightPositionMgrState == IdleAtBottom ||
      (lightPositionMgrState == Lowering && !lightMoving)) {
    lightPositionMgrState = IdleAtBottom;
    lRetVal = true;
    positionEncoder.write(maxPulses);
  }

  return (lRetVal);
}

void DA_LightPositionMgr::setOnStopCallBack(
    void (*callBack)(DA_LightMgrState aReason)) {
  onStop = callBack;
}

void DA_LightPositionMgr::detectAndProcessTransitionToStop() {

if( !lightMoving && previousLightMoving )
if (onStop != NULL) {
  onStop(lightPositionMgrState);
}

/*
  if (lightPositionMgrState == Idle || lightPositionMgrState == IdleAtTop ||
      lightPositionMgrState == IdleAtBottom)
    if (previousLightPositionMgrState == Rising ||
        previousLightPositionMgrState == Lowering) {
      if (onStop != NULL) {
        onStop(lightPositionMgrState);
      }
    }

    previousLightPositionMgrState = lightPositionMgrState;

*/
previousLightMoving = lightMoving;
}
