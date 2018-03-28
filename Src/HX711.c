#include "type.h"
#include <string.h>
#include "FreeRtosAll.h"

#include "stm32F429xx.h"
#include "stm32F429deviceall.h"

#include "HX711.h"

/**
 * Local stack and tcb for hx711 task 
 * And queue to send interrupt for ready 
 * to thread. 
 * I like static initialization so I know 
 * how much memory is used at link time. 
 */
TaskHandle_t hx711TaskPtr;
StaticTask_t hx711TaskTCB;
StackType_t  hx711TaskStackData[hx711Task_STACK_SIZE];
QueueHandle_t hx711Queue;
StaticQueue_t hx711QueueData;
#define hx711Queue_DEPTH    8
uint32_t hx711QueueArray[hx711Queue_DEPTH];


// Enum for front or back ready.
typedef enum
{
   FRONT_READY = 0xAA550001, // can't be null for queue pointer
   BACK_READY  = 0xAA550002
}HX711Interrupt;


#define HX711_PORT                   GPIOC
                                    
#define HX711_EXTI_PORT              LL_SYSCFG_EXTI_PORTC
#define HX711_FRONT_INTERRUPT_LINE   LL_EXTI_LINE_3
#define HX711_BACK_INTERRUPT_LINE    LL_EXTI_LINE_8
#define HX711_FRONT_SYSCFG_LINE      LL_SYSCFG_EXTI_LINE3
#define HX711_BACK_SYSCFG_LINE       LL_SYSCFG_EXTI_LINE8

#define HX711_FRONT_IRQ_NUMBER       EXTI3_IRQn
#define HX711_BACK_IRQ_NUMBER        EXTI9_5_IRQn

#define HX711_FRONT_HANDLER          EXTI3_IRQHandler
#define HX711_BACK_HANDLER           EXTI9_5_IRQHandler

#define HX711_FRONT_DATA             LL_GPIO_PIN_3
#define HX711_FRONT_CLOCK            LL_GPIO_PIN_11
#define HX711_BACK_DATA              LL_GPIO_PIN_8
#define HX711_BACK_CLOCK             LL_GPIO_PIN_12
                                    
#define READ_FRONT_DATA()            (LL_GPIO_IsInputPinSet(HX711_PORT,HX711_FRONT_DATA))
#define READ_BACK_DATA()             (LL_GPIO_IsInputPinSet(HX711_PORT,HX711_BACK_DATA))
                                    
#define FRONT_CLOCK_HIGH()           LL_GPIO_SetOutputPin(HX711_PORT,HX711_FRONT_CLOCK)
#define FRONT_CLOCK_LOW()            LL_GPIO_ResetOutputPin(HX711_PORT,HX711_FRONT_CLOCK)
                                    
#define BACK_CLOCK_HIGH()            LL_GPIO_SetOutputPin(HX711_PORT,HX711_BACK_CLOCK)
#define BACK_CLOCK_LOW()             LL_GPIO_ResetOutputPin(HX711_PORT,HX711_BACK_CLOCK)

/**
 * PA0 front data pin, input, setup as interrupt falling edge. 
 * When we go to read the data disable the interrupt. 
 * PA1 for back data pin. same setup. 
 * PA2 clock for front. 
 * PA3 clock for back. 
 */

// Setup the data and clock pins.
void setupHX711Pins(void)
{
   LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOG);

   LL_GPIO_SetPinMode(HX711_PORT,HX711_FRONT_DATA,LL_GPIO_MODE_INPUT);
   LL_GPIO_SetPinMode(HX711_PORT,HX711_BACK_DATA,LL_GPIO_MODE_INPUT);
   LL_GPIO_SetPinPull(HX711_PORT,HX711_FRONT_DATA,LL_GPIO_PULL_UP);
   LL_GPIO_SetPinPull(HX711_PORT,HX711_BACK_DATA,LL_GPIO_PULL_UP);
   
   LL_GPIO_SetPinMode(HX711_PORT,HX711_FRONT_CLOCK,LL_GPIO_MODE_OUTPUT);
   LL_GPIO_SetPinMode(HX711_PORT,HX711_BACK_CLOCK,LL_GPIO_MODE_OUTPUT);

   LL_GPIO_SetPinOutputType(HX711_PORT,HX711_FRONT_CLOCK,LL_GPIO_OUTPUT_PUSHPULL);
   LL_GPIO_SetPinOutputType(HX711_PORT,HX711_BACK_CLOCK,LL_GPIO_OUTPUT_PUSHPULL);
   LL_GPIO_SetPinPull(HX711_PORT,HX711_FRONT_CLOCK,LL_GPIO_PULL_NO);
   LL_GPIO_SetPinPull(HX711_PORT,HX711_BACK_CLOCK,LL_GPIO_PULL_NO);
}



/**
 * Interrupt for front sense board.
 * 
 * @author Charles "Mickey" Nowell (3/24/2018)
 */
void HX711_FRONT_HANDLER(void)
{
   uint32_t      value;
   BaseType_t    preemptYes = pdFALSE;

   LL_EXTI_ClearFlag_0_31(HX711_FRONT_INTERRUPT_LINE); // LL_EXTI_LINE_0);
   NVIC_ClearPendingIRQ(HX711_FRONT_IRQ_NUMBER); // EXTI0_IRQn);
   // abuse the queue
   // just send a constant instead of pointer
   value = FRONT_READY;
   xQueueSendFromISR(hx711Queue,(void *)&value,&preemptYes);
   portEND_SWITCHING_ISR(preemptYes);
}

/**
 * 
 * 
 * @author Charles "Mickey" Nowell (3/24/2018)
 */
void HX711_BACK_HANDLER(void)
{
   uint32_t      value;
   BaseType_t    preemptYes = pdFALSE;

   LL_EXTI_ClearFlag_0_31(HX711_BACK_INTERRUPT_LINE); // LL_EXTI_LINE_0);
   NVIC_ClearPendingIRQ(HX711_BACK_IRQ_NUMBER); // EXTI0_IRQn);
   // abuse the queue
   // just send a constant instead of pointer
   value = BACK_READY;
   xQueueSendFromISR(hx711Queue,(void *)&value,&preemptYes);
   portEND_SWITCHING_ISR(preemptYes);
}

/**
 * Sets up the interrupt for front cell. 
 * Does not enable. 
 * Falling edge. 
 * 
 * @author Charles "Mickey" Nowell (3/24/2018)
 */
void setupHX711FrontInterrupt(void)
{
   LL_SYSCFG_SetEXTISource(HX711_EXTI_PORT,HX711_FRONT_SYSCFG_LINE);
   LL_EXTI_EnableIT_0_31(HX711_FRONT_INTERRUPT_LINE);
   LL_EXTI_EnableFallingTrig_0_31(HX711_FRONT_INTERRUPT_LINE);

   uint32_t prioritygroup = NVIC_GetPriorityGrouping();
   // priority has to be greater than FreeRtos min level = 5
   uint32_t priority = NVIC_EncodePriority(prioritygroup,7,1);
   NVIC_SetPriority(HX711_FRONT_IRQ_NUMBER,priority);
}
/**
 * Sets up the interrupt for back cell. 
 * Does not enable. Falling edge.
 * 
 * @author Charles "Mickey" Nowell (3/24/2018)
 */
void setupHX711BackInterrupt(void)
{
   LL_SYSCFG_SetEXTISource(HX711_EXTI_PORT,HX711_BACK_SYSCFG_LINE);
   LL_EXTI_EnableIT_0_31(HX711_BACK_INTERRUPT_LINE);
   LL_EXTI_EnableFallingTrig_0_31(HX711_BACK_INTERRUPT_LINE);

   uint32_t prioritygroup = NVIC_GetPriorityGrouping();
   // priority has to be greater than FreeRtos min level = 5
   uint32_t priority = NVIC_EncodePriority(prioritygroup,7,1);
   NVIC_SetPriority(HX711_BACK_IRQ_NUMBER,priority);
}

/**
 * Enable the front cell interrupt.
 * 
 * @author Charles "Mickey" Nowell (3/24/2018)
 * 
 * @param void 
 */
void enableFrontInterrupt(void)
{
   LL_EXTI_ClearFlag_0_31(HX711_FRONT_INTERRUPT_LINE);
   NVIC_ClearPendingIRQ(HX711_FRONT_IRQ_NUMBER);
   NVIC_EnableIRQ(HX711_FRONT_IRQ_NUMBER);
}
/**
 * Enable the back cell interrupt.
 * 
 * @author Charles "Mickey" Nowell (3/24/2018)
 * 
 * @param void 
 */
void enableBackInterrupt(void)
{
   LL_EXTI_ClearFlag_0_31(HX711_BACK_INTERRUPT_LINE);
   NVIC_ClearPendingIRQ(HX711_BACK_IRQ_NUMBER);
   NVIC_EnableIRQ(HX711_BACK_IRQ_NUMBER);
}
/**
 * Disable front interrupt.
 * 
 * @author Charles "Mickey" Nowell (3/24/2018)
 * 
 * @param void 
 */
void disableFrontInterrupt(void)
{
   NVIC_DisableIRQ(HX711_FRONT_IRQ_NUMBER);
   LL_EXTI_ClearFlag_0_31(HX711_FRONT_INTERRUPT_LINE);
   NVIC_ClearPendingIRQ(HX711_FRONT_IRQ_NUMBER);
}
/**
 * Disable back interrupt.
 * 
 * @author Charles "Mickey" Nowell (3/24/2018)
 * 
 * @param void 
 */
void disableBackInterrupt(void)
{
   NVIC_DisableIRQ(HX711_BACK_IRQ_NUMBER);
   LL_EXTI_ClearFlag_0_31(HX711_BACK_INTERRUPT_LINE);
   NVIC_ClearPendingIRQ(HX711_BACK_IRQ_NUMBER);
}

/**
 * Variables to get the clock frequency 
 * and then mumber of cpu cylces per microsecond. 
 * This is used to count clock cycles for 
 * microsecond delays. 
 * This auto adjusts to cpu clock frequency 
 * as long as it's greater than 1e6. 
 * 
 * @author Charles "Mickey" Nowell (3/26/2018)
 */
static uint32_t cpuClockFrequency;
static uint32_t cyclesPerMicroSecond;

/**
 * At runtime, use HAL to figure out the 
 * clock rate. 
 * Compute how many cycles is 1 Usec. 
 * 
 * @author Charles "Mickey" Nowell (3/26/2018)
 * 
 * @param void 
 */
void setupCpuCycles(void)
{
   cpuClockFrequency = HAL_RCC_GetHCLKFreq();
   cyclesPerMicroSecond = cpuClockFrequency / 1000000ul;
}

/**
 * Delay microseconds (blocking) 
 * using cpu clock cycles. 
 * Note: cycles counter and debug stuff 
 * needs to be turned on. 
 * I could probably do this with a timer. 
 * But this is easier. 
 * Since the scale doesn't have much to do when 
 * it's sampling this little < ~200 usec blocki while reading 
 * the sensor (~25 clocks at 5 usec per clock) is down in 
 * the noise as far as time. 
 * 
 * @author Charles "Mickey" Nowell (3/26/2018)
 * 
 * @param time 
 */
void delayMicroSeconds(int32_t time)
{
   int32_t cycles;
   cycles = (int32_t)(DWT->CYCCNT);
   time *= cyclesPerMicroSecond;
   while(((int32_t)(DWT->CYCCNT) - cycles) < time);
}
/**
 * Must be called at startup to enable the M4 
 * cycles counter. 
 * Why doesn't this run all the time? 
 * Power consumption perhaps. 
 * 
 * @author Charles "Mickey" Nowell (3/26/2018)
 * 
 * @param void 
 */
void enableCyclesCounter(void)
{
      // enable arm cycles counter
   CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
   CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

   DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
   DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
   DWT->CYCCNT = 0;

   __ASM volatile("NOP");
   __ASM volatile("NOP");
   __ASM volatile("NOP");

}

/**
 * Helper functions for HX711
 */

#define SAMPLE_SIZE   32
#define SAMPLE_MASK   31
/**
 * Module variables. 
 * gain, raw values, filtered variables. 
 * valueIndex just keeps up with where 
 * we are writing into these arrays. 
 * 
 * @author Charles "Mickey" Nowell (3/26/2018)
 */
static uint32_t frontGain = 3;
static uint32_t backGain = 3;
static int32_t  frontRawValues[SAMPLE_SIZE];
static int32_t  backRawValues[SAMPLE_MASK];
static int32_t  frontFilteredValues[SAMPLE_SIZE];
static int32_t  backFilteredValues[SAMPLE_MASK];
static uint8_t  valueIndex;

static int32_t  currentFrontAverage;
static int32_t  currentBackAverage;

void setHX711Gain(uint32_t newFrontGain,uint32_t newBackGain)
{
   if(newFrontGain >=1 && newFrontGain <= 3)
   {
      frontGain = newFrontGain;
   }
   if(newBackGain >=1 && newBackGain <= 3)
   {
      backGain = newBackGain;
   }
}

/**
 * Get the current averages. 
 * Critical region makes sure we get a good pair of values. 
 * 
 * @author Charles "Mickey" Nowell (3/24/2018)
 * 
 * @param frontValue 
 * @param backValue 
 */
void getLoadCellValues(int32_t *frontValue,int32_t *backValue)
{
   taskENTER_CRITICAL();
   *frontValue = currentFrontAverage;
   *backValue = currentBackAverage;
   taskEXIT_CRITICAL();
}

#define FILTER_3RD_ORDER
#undef  FILTER_3RD_ORDER

#ifdef FILTER_3RD_ORDER
#define IGNORE_LAST   2
/**
 * Compute 3rd order median value.
 * 
 * @author Charles "Mickey" Nowell (3/26/2018)
 * 
 * @param a 
 * @param b 
 * @param c 
 * 
 * @return uint32_t 
 */
uint32_t thirdOrderMedianValue(uint32_t a,uint32_t b, uint32_t c)
{
   if(a > b)
   {
      if(b > c)
      {
         return(b);
      }
      else
      {  
         // c > b and a > b
         if(c > a)
         {
            return(a);
         }
         else
         {
            return(c);
         }
      }
   }
   else
   {
      // b > a
      if(a > c)
      {
         return(a);
      }
      else
      {  
         // b > a and c > a
         if(b > c)
         {
            return(c);
         }
         else
         {
            return(b);
         }

      }
   }
}
/**
 * 3rd order median filter. 
 * Ignore the last few values. 
 * Will just ignore these samples in the 
 * average. 
 * 
 * @author Charles "Mickey" Nowell (3/26/2018)
 */
void medianFilter(void)
{
   int i;

   for(i=0;i<SAMPLE_SIZE-2;i++)
   {
      frontFilteredValues[i] = thirdOrderMedianValue(frontRawValues[i],frontRawValues[i+1],frontRawValues[i+2]);
      backFilteredValues[i] = thirdOrderMedianValue(backRawValues[i],backRawValues[i+1],backRawValues[i+2]);
   }
}


#else

#define IGNORE_LAST   4

// 5th order
/**
 * Sort the 5 values to find the median.
 * 
 * @author Charles "Mickey" Nowell (3/26/2018)
 * 
 * @param values 
 */
static void sort5(int32_t values[])
{
   int i,j;
   int32 temp;
   for(i=0;i<4;i++)
   {
      for(j=i;j<5;j++)
      {
         if(values[i] > values[j])
         {
            temp = values[i];
            values[i] = values[j];
            values[j] = temp;
         }
      }
   }
}
/**
 * Brute force, non elegant 5th order filter. 
 * Doesn't bother with patching the last couple 
 * of values or any windowing. 
 * We will just ignore the last few samples. 
 * 
 * @author Charles "Mickey" Nowell (3/26/2018)
 * 
 * @param data 
 * 
 * @return uint32_t 
 */
uint32_t fifthOrderMedianValue(int32_t data[])
{
   int32_t temp[5];
   memcpy(temp,data,5*sizeof(uint32_t));
   sort5(temp);
   return(temp[2]);
}

void medianFilter(void)
{
   int i;

   for(i=0;i<SAMPLE_SIZE-IGNORE_LAST;i++)
   {
      frontFilteredValues[i] = fifthOrderMedianValue(frontRawValues+i);
      backFilteredValues[i] = fifthOrderMedianValue(backRawValues+i);
   }
}


#endif


/**
 * Compute the averages, put in the 
 * module variable in critical region 
 * so they are coherent. 
 * 
 * @author Charles "Mickey" Nowell (3/24/2018)
 */
void computeAverages(void)
{
   int32_t sumFront = 0;
   int32_t sumBack = 0;
   int i;
   for(i=0;i<SAMPLE_SIZE-IGNORE_LAST;i++)
   {
      sumFront += frontFilteredValues[i];
      sumBack  += backFilteredValues[i];
   }
   sumFront /= (SAMPLE_SIZE-IGNORE_LAST);
   sumBack  /= (SAMPLE_SIZE-IGNORE_LAST);
   taskENTER_CRITICAL();
   currentFrontAverage = sumFront;
   currentBackAverage = sumBack;
   taskEXIT_CRITICAL();
}

/**
 * Put a front sample in the buffer.
 * 
 * @author Charles "Mickey" Nowell (3/24/2018)
 * 
 * @param value 
 */
void putFront(int32_t value)
{
   frontRawValues[valueIndex] = value;
}
/**
 * Put a sample in the buffer for back data. 
 * If this is the rollover sample, 
 * go up date the average and post it. 
 * 
 * @author Charles "Mickey" Nowell (3/24/2018)
 * 
 * @param value 
 */
void putBack(int32_t value)
{
   backRawValues[valueIndex] = value;
   valueIndex++;
   valueIndex &= SAMPLE_MASK;
   if(valueIndex == 0)
   {
      medianFilter();
      computeAverages();
   }
}



/**
 * Power up the ADC boards.
 * 
 * @author Charles "Mickey" Nowell (3/24/2018)
 * 
 * @param void 
 */
void hx711PowerUp(void)
{
   FRONT_CLOCK_LOW();
   BACK_CLOCK_LOW();
}

/**
 * Power down the ADC boards.
 * 
 * @author Charles "Mickey" Nowell (3/24/2018)
 * 
 * @param void 
 */
void hx711PowerDown(void)
{
   FRONT_CLOCK_LOW();
   BACK_CLOCK_LOW();
   vTaskDelay(2);
   FRONT_CLOCK_HIGH();
   BACK_CLOCK_HIGH();
}
/**
 * Prototype the read bit function pointer.
 */
typedef int32_t (*ReadBitFunction)(void);

/**
 * Matching readbit function for front sensor.
 * 
 * @author Charles "Mickey" Nowell (3/24/2018)
 * 
 * @return int32_t 
 */
int32_t  readFrontBit(void)
{
   int32_t value;
   FRONT_CLOCK_HIGH();
   delayMicroSeconds(2);
   // return a bit in the correct position.
   value = READ_FRONT_DATA() ? BIT0 : 0;
   FRONT_CLOCK_LOW();
   delayMicroSeconds(2);
   return(value);
}
/**
 * Matching function to read a bit on back sensor.
 * 
 * @author Charles "Mickey" Nowell (3/24/2018)
 * 
 * @return int32_t 
 */
int32_t  readBackBit(void)
{
   int32_t value;
   BACK_CLOCK_HIGH();
   delayMicroSeconds(2);
   // return a bit in the correct position.
   value = READ_BACK_DATA() ? BIT0 : 0;
   BACK_CLOCK_LOW();
   delayMicroSeconds(2);
   return(value);
}


/**
 * Read a sensor and set the next gain. 
 * Passed function pointer to get front or back. 
 * 
 * @author Charles "Mickey" Nowell (3/24/2018)
 * 
 * @param readBit 
 * @param gain 
 * 
 * @return int32_t 
 */
int32_t readSensor(ReadBitFunction readBit,uint32_t gain)
{
   int32_t value;
   int i;
   value = 0;
   delayMicroSeconds(10);
   for(i=0;i<24;i++)
   {
      value = (value << 1) | ((*readBit)());
   }
   // sign extend
   if(value & BIT23)
   {
      value |= 0xFF000000; 
   }
   for(i=0;i<gain;i++)
   {
      (*readBit)();
   }
   return(value);
}

/**
 * Debug when not running interrupt mode.
 * 
 * @author Charles "Mickey" Nowell (3/24/2018)
 * 
 * @param void 
 * 
 * @return bool 
 */
bool isFrontReady(void)
{
   return(READ_FRONT_DATA() ? false : true);
}

/**
 * Debug when not running interrupt mode.
 * 
 * @author Charles "Mickey" Nowell (3/24/2018)
 * 
 * @param void 
 * 
 * @return bool 
 */
bool isBackReady(void)
{
   return(READ_BACK_DATA() ? false : true);
}

volatile int32_t delta;
/**
 * Basic hx711 task. 
 * Sets up, waits for interrupts. 
 * Then reads the given sensor. 
 * Runs a filter on the data 
 * and makes the filtered data available. 
 * Allows for different gain settings. 
 * 
 * @author Charles "Mickey" Nowell (3/24/2018)
 * 
 * @param argument 
 */
void hx711TaskFunction(void *argument)
{
   // stops compiler warning
   (void)argument;

   void                    *queueReceiveValue;
   uint8_t                 queueResult;
   uint32_t                whichInterrupt;
   int32_t                 value;
   static int32_t          now,last;


   enableCyclesCounter();
   setupCpuCycles();

   setupHX711Pins();
   hx711PowerDown();
   vTaskDelay(5);

   setupHX711FrontInterrupt();
   setupHX711BackInterrupt();

   enableFrontInterrupt();
   enableBackInterrupt();

   hx711PowerUp();

   while(1)
   {
      queueResult = xQueueReceive(hx711Queue,(void *)&(queueReceiveValue),portMAX_DELAY);
      // avoid compiler warning
      (void)queueResult;
      #if 0
      // Debug to test.
      // manually polls the done bit.
      // not needed after interrupts are working
      if(isFrontReady())
      {
         value = readSensor(readFrontBit,frontGain);
         putFront(value);
      }

      if(isBackReady())
      {
         value = readSensor(readBackBit,backGain);
         putBack(value);
      }
      #endif

      // convert pointer back to the int passed.
      whichInterrupt = (uint32_t)queueReceiveValue;
      if(whichInterrupt == FRONT_READY)
      {
         now = OSTIME();
         delta = now - last;
         last = OSTIME();
         disableFrontInterrupt();
         value = readSensor(readFrontBit,frontGain);
         putFront(value);
         enableFrontInterrupt();
      }
      else if(whichInterrupt == BACK_READY)
      {
         disableBackInterrupt();
         value = readSensor(readBackBit,backGain);
         putBack(value);
         enableBackInterrupt();
      }
      else
      {
         // ?? not likely, but don't do something stupid
      }
   }

}


void startHX711Task(void)
{
   hx711Queue = xQueueCreateStatic(hx711Queue_DEPTH,4,(uint8_t *)(hx711QueueArray),&hx711QueueData);

   vQueueAddToRegistry(hx711Queue,"hx711Queue");

   hx711TaskPtr = xTaskCreateStatic(hx711TaskFunction,"hx711Task",hx711Task_STACK_SIZE,(void *)(0),
                                        hx711Task_PRIORITY,(StackType_t * const)(hx711TaskStackData),
                                        (StaticTask_t * const)&(hx711TaskTCB));
   // stop here if can't create.
    if(hx711TaskPtr == NULL)while(1);
 
}

