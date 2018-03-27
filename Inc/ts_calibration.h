#ifndef TS_CALIBRATION_H
#define TS_CALIBRATION_H


bool Touchscreen_Calibration(TSCalibrationPtr calibrationValues);
void configureTouchScreen(TSCalibrationPtr calibrationValues);

uint16_t Calibration_GetX(uint16_t x);
uint16_t Calibration_GetY(uint16_t y);
uint8_t IsCalibrationDone(void);
#endif
