#ifndef CALDATA_H
#define CALDATA_H


typedef struct TSCalibrationType
{
  int16_t    A1;
  int16_t    A2;
  int16_t    B1;
  int16_t    B2;
}TSCalibrationType;
typedef TSCalibrationType  *TSCalibrationPtr;

typedef struct ScaleCalibrationType
{
   float                   frontZero;
   float                   backZero;
   float                   frontAtCal;
   float                   backAtCal;
   float                   frontSlope;
   float                   backSlope;
   float                   calWeight;
   float                   WingPegDist;
   float                   LEstopperDist;
}ScaleCalibrationType;
typedef ScaleCalibrationType *ScaleCalibrationPtr;

typedef struct SystemCalDataType
{
   TSCalibrationType     tsCalibrationData;
   ScaleCalibrationType  scaleCalibrationData;
}SystemCalDataType;
typedef SystemCalDataType  *SystemCalDataPtr;

void writeCalData(uint8_t const *data,uint32_t length);
bool readCalData(uint8_t *data,uint32_t length);

bool loadCalFromFlash(void);
void pushCalToFlash(void);


void updateScaleCalibration(ScaleCalibrationPtr scaleCalibration);
bool loadScaleCalibration(ScaleCalibrationPtr scaleCalibration);

void updateTSCalibration(TSCalibrationPtr tsCalibration);
bool loadTSCalibration(TSCalibrationPtr tsCalibration);


#endif

