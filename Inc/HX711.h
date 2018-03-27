#ifndef HX711_H
#define HX711_H


#define hx711Task_PRIORITY       7
#define hx711Task_STACK_SIZE     512
void startHX711Task(void);


void getLoadCellValues(int32_t *frontValue,int32_t *backValue);


#endif
