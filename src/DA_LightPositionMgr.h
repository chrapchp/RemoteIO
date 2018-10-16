/**
 *  @file    DA_LightPositionMgr.h
 *  @author  peter c
 *  @date    10/09/2018
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *  Handle position light height position via setpoints and manual calibration
 *  Drives an H-Bridge controller board that has three inputs (Ena1,En2 for
 *  directtion, EnA for motor speed PWM /On/off)
 *    Control the light position using via a setpoint
 *  Note: 100% -> lowest  light position e.g. close to plants
 *        0   -> highest light position e.g. away to plants
 *        when arm is completely retracted count = maxcount
 *        when arm is extending counter is decreasing
 *        when arm is retracting counter is increasing
 *        when arm is extending lights are going up
 *        when arm is retracting lights are going down
 *
 */

#ifndef DA_LIGHTPOSITIONMGR_H
#define DA_LIGHTPOSITIONMGR_H

#include <Arduino.h>
#include <DA_DiscreteOutput.h>
#include <Encoder.h>


class DA_LightPositionMgr {
public:

  enum DA_LightMgrState { Idle = 1, Rising, Lowering, IdleAtTop, IdleAtBottom };
  const int32_t DA_LIGHTPOSITION_DEFAULT_MAX_PULSE_COUNT_LIGHT_POSITION = 34115;
  const uint16_t DA_LIGHTPOSITION_DEFAULT_LIGHT_SP                      = 50;
  const float DA_LIGHTPOSITION_DEFAULT_LIGHT_POSITION_DEADBAND          = 0.15;
  const uint16_t DA_LIGHPOSITION_DEFAULT_REFRESH_INTERVAL               = 150;

  /**
   * [DA_LightPositionMgr description]
   * @param aEncoder [Encoder instance to deal with 2x interrupts from position
   * encoder]
   * @param aIn1     [H-Bridge In1 - Direction Forware/backwards]
   * @param aIn2     [H-Bridge In2 Direction Forware/backwards ]
   * @param aEnA     [H-Bridge PWM ]
   */
  DA_LightPositionMgr(Encoder          & aEncoder,
                      DA_DiscreteOutput& aIn1,
                      DA_DiscreteOutput& aIn2,
                      DA_DiscreteOutput& aEnA);

  /**
   * Invoke periodally from main loop
   */
  void refresh();

  void serialize(Stream *aOutputStream,
                 bool    includeCR);

  /**
   * [getPV return curent light position]
   * @return [0-100.0]
   */
  inline float getPV() __attribute__((always_inline)) {
    return pv;
  }

  /**
   * [getRawPV return current light position in counts]
   * @return [0-Maxpulses]
   */
  inline int32_t getRawPV() __attribute__((always_inline)) {
    return currentPositionCount;
  }

  /**
   * [setSetpoint Setpoint representing light disired position]
   * @param aSetPoint [0-100]
   */
  inline void setSetpoint(uint16_t aSetPoint) __attribute__((always_inline)) {
    sp = aSetPoint;
  }

  /**
   * [getSP return current setpoint]
   * @return [0-100]
   */
  inline uint16_t getSP() __attribute__((always_inline)) {
    return sp;
  }

  /**
   * [setCalibrationMode Controller operating mode]
   * @param aCalibrationModeReqest [true for calibration, false for auto]
   */
  inline void setCalibrationMode(bool aCalibrationModeReqest)
  __attribute__((always_inline)) {
    calibrationMode = aCalibrationModeReqest;
  }

  /**
   * [setCalibrationMoveTop move lights to upwards. Controller witl stop when it
   * reaches limit. Controller will remain in idle if both top/down requests
   * exsist]
   * @param aMoveTopRequest [true = move to top]
   */
  inline void setCalibrationMoveTop(bool aMoveTopRequest)
  __attribute__((always_inline)) {
    calibrationMoveTop = aMoveTopRequest;
  }

  /**
   * [setCalibrationMoveBottom move lights to the downwards. Controller witl
   * stop when it reaches limit. Controller will remain in idle if both top/down
   * requests exsist]
   * @param aMoveBottomRequest [=true for move to botton]
   */
  inline void setCalibrationMoveBottom(bool aMoveBottomRequest)
  __attribute__((always_inline)) {
    calibrationMoveBottom = aMoveBottomRequest;
  }

  /**
   * [setOnStopCallBack Optional callback when lights transition from moving to
   * stop]
   * @param callBack [event causing it to stop]
   */
  void        setOnStopCallBack(void (*callBack)(DA_LightMgrState aReason));

  /**
   * [setRawPosition provide controller and encoder with a know start count]
   * @param aRawCount [0-maxPulses]
   */
  inline void setRawPosition(int32_t aRawCount) __attribute__((always_inline)) {
    positionEncoder.write(aRawCount);
    previousPositionCount = aRawCount;
    currentPositionCount  = aRawCount;
    pv                    = 100.0 - (100.0 * currentPositionCount / maxPulses);
    computeLightPosition();
  }

  /**
   * [setMaxPulses set the count value to fully extended the actuator]
   * @param aMaxPulses [non zero value]
   */
  inline void setMaxPulses(int32_t aMaxPulses) __attribute__((always_inline)) {
    if (aMaxPulses != 0) maxPulses = aMaxPulses;
  }

  /**
   * [getMaxPulses return the maximul pulse count representing a fully extended
   * actuator]
   * @return [description]
   */
  inline int32_t getMaxPulses() __attribute__((always_inline)) {
    return maxPulses;
  }

  /**
   * [setRefreshInterval set refresh interval of position manager]
   * @param aRefreshInterval [interval in ms]
   */
  inline void setRefreshInterval(uint16_t aRefreshInterval) __attribute__((
                                                                            always_inline))
  {
    if (aRefreshInterval != 0) refreshInterval = aRefreshInterval;
  }

  /**
   * Manager Enabled
   * @param aEnabled [=true to enable manager]
   */
  inline void setManagerMode(bool aEnabled) __attribute__((always_inline)) {
    managerEnabled = aEnabled;
  }

protected:

  void computeLightPosition();
  void processCalibrationCommands();
  void doAutomatedControls();
  void lowerLights(uint8_t aSpeed);
  void raiseLights(uint8_t aSpeed);
  void holdLightPosition();
  bool isAtTopPosition();
  bool isAtBottomPosition();
  void detectAndProcessTransitionToStop();
  void doTopPositionCheck();
  void doBottomPositionCheck();

private:

  Encoder& positionEncoder;
  DA_LightMgrState lightPositionMgrState         = Idle;
  DA_LightMgrState previousLightPositionMgrState = Idle;
  DA_DiscreteOutput& in1;                  // H-Bridge forward/reverse

  DA_DiscreteOutput& in2;                  // H-Bridge forward/reverse
  DA_DiscreteOutput enA;                   // H-Brdige Motoron/off/PWM

  int32_t currentPositionCount;            // encoder PV in  pulses
  int32_t previousPositionCount = -999999; // some impossible number
  // used for doing oneshot save to EEPROM
  bool previousZIC_015_SV = false;

  // = true when there is movement
  bool lightMoving = false;
  bool previousLightMoving;
  uint16_t sp = DA_LIGHTPOSITION_DEFAULT_LIGHT_SP; // Setpoint from host
  float pv;                                        // present value position
                                                   // 0-100
  // determined emperically
  int32_t maxPulses =
    DA_LIGHTPOSITION_DEFAULT_MAX_PULSE_COUNT_LIGHT_POSITION;
  bool calibrationMode       = true;
  bool calibrationMoveTop    = false;
  bool calibrationMoveBottom = false;
  uint32_t lastUpdateTime    = 0;
  uint16_t refreshInterval   = DA_LIGHPOSITION_DEFAULT_REFRESH_INTERVAL;
  float processError;
  bool managerEnabled = false;
  void (*onStop)(DA_LightMgrState aReason) = NULL;
};

#endif // DA_LIGHTPOSITIONMGR_H
