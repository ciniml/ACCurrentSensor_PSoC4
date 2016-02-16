/* ========================================
 * main module of AC current sensor.
 * Copyright 2016 Kenta IDA
 * This software 
 * ========================================
*/
#include <project.h>
#include <stdint.h>
#include <stdbool.h>

//#define DEBUG_OUT
#ifdef DEBUG_OUT
static void UartPutString(const char* s) { UART_Debug_UartPutString(s); }
#else
static void UartPutString(const char* s) {}
#endif

/***
 * @brief Format a 8bit value in hexadecimal.
 * @param p Pointer to a buffer to which this function writes a formatted value.
 * @param n A 8bit value to format.
 * @return Pointer to next location in the buffer.
 */
static char* writeHex8(char* p, uint8_t n)
{
    uint8_t nibble;
    nibble = n >> 4;
    *(p++) = nibble < 0x0a ? nibble + '0' : nibble + 'A' - 0x0a;
    nibble = n & 0x0f;
    *(p++) = nibble < 0x0a ? nibble + '0' : nibble + 'A' - 0x0a;
    return p;
}
/***
 * @brief Format a 16 bit value in hexadecimal.
 * @param p Pointer to a buffer to which this function writes a formatted value.
 * @param n A 16 bit value to format.
 * @return Pointer to next location in the buffer.
*/
static char* writeHex16(char* p, uint16_t n)
{
    p = writeHex8(p, n >> 8);
    p = writeHex8(p, n & 0xff);
    return p;
}
/***
 * @brief Format a 32 bit value in hexadecimal.
 * @param p Pointer to a buffer to which this function writes a formatted value.
 * @param n A 32 bit value to format.
 * @return Pointer to next location in the buffer.
 */
static char* writeHex32(char* p, uint32_t n)
{
    p = writeHex8(p, (n >> 24) & 0xff);
    p = writeHex8(p, (n >> 16) & 0xff);
    p = writeHex8(p, (n >>  8) & 0xff);
    p = writeHex8(p, (n >>  0) & 0xff);
    return p;
}

static uint32_t sensorAccumulator = 0;      ///< Raw sensor value accumulator.
static uint32_t sensorValue = 0;            ///< Processed sensor value.
static uint8_t sensorCount = 0;             ///< A counter to count number of samples acquired.
static bool isNotificationEnabled = false;  ///< Notification to the central is enabled or not.

/**
 * @brief ADC state.
 */
typedef enum 
{
    ADC_STATE_IDLE,
    ADC_STATE_ACTIVE,
    ADC_STATE_COMPLETED,
} ADC_STATE;
static ADC_STATE adcState = ADC_STATE_IDLE; ///< Current ADC state.

/**
 * @brief ADC conversion complete interrupt handler
 */
CY_ISR(ADC_Interrupt_Handler)
{
    uint32_t intr_status = ADC_SAR_SEQ_SAR_INTR_REG;
    int16_t value = ADC_SAR_SEQ_GetResult16(0);
    
    switch(adcState)
    {
    case ADC_STATE_ACTIVE:  // If the ADC processing is active.
        sensorAccumulator += value < 0 ? -value : value;    // Accumulate absolute value of raw sensor data.
        sensorCount = (sensorCount + 1) & 63;               // Count up number of samples acquried (mod 63)
        if( sensorCount == 0 )  // Acquired 64 samples.
        {
            sensorValue = sensorAccumulator >> (6 + 2); // Calculate the average of acquired samples. ( divide by (64 * 4) )
            sensorAccumulator = 0;                      // Clear the accumulator.
            adcState = ADC_STATE_COMPLETED;             // Transit to COMPLETED state.
            ADC_SAR_SEQ_Stop();                         // And turn off the ADC.
        }
        break;
    }
    ADC_SAR_SEQ_SAR_INTR_REG = intr_status;
}

/**
 * @brief Watchdog timeout interrupt handler
 */
CY_ISR(WatchDog_Interrupt_Handler)
{
    if( adcState == ADC_STATE_IDLE )    // If the watchdog timer has expired, initiate to measure AC current.
    {
        // Activate the ADC process.
        adcState = ADC_STATE_ACTIVE;
        ADC_SAR_SEQ_Start();
        ADC_SAR_SEQ_StartConvert();
    }
    //Pin_LED_R_Write(~Pin_LED_R_ReadDataReg());
    
    CySysWdtClearInterrupt(CY_SYS_WDT_COUNTER0_INT);
}

static void BleCallback(uint32 eventCode, void *eventParam)
{
    switch(eventCode)
    {
    case CYBLE_EVT_STACK_ON:
    case CYBLE_EVT_GAP_DEVICE_DISCONNECTED:
        // Start Advertisement
        UartPutString("GAP_DEVICE_DISCONNECTED\n");
        CyBle_GappStartAdvertisement(CYBLE_ADVERTISING_FAST);
        break;
    case CYBLE_EVT_GAP_DEVICE_CONNECTED:
        UartPutString("GAP_DEVICE_CONNECTED\n");
        // Update connection parameters
        {
            CYBLE_GAP_CONN_UPDATE_PARAM_T connParam = {
                800,
                800,
                0,
                500,
            };
            
            CyBle_L2capLeConnectionParamUpdateRequest(cyBle_connHandle.bdHandle, &connParam);
        }
        break;
    case CYBLE_EVT_TIMEOUT:
        UartPutString("TIMEOUT\n");
        break;
    case CYBLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
        UartPutString("ADVERTISEMENT_START_STOP\n");
        if( CyBle_GetState() != CYBLE_STATE_ADVERTISING )
        {
            CyBle_GappStartAdvertisement(CYBLE_ADVERTISING_SLOW);
        }
        break;
    case CYBLE_EVT_GATT_CONNECT_IND:
        UartPutString("GATT_CONNECT_IND\n");
        /* Register service specific callback functions and init CCCD values */
        CYBLE_GATT_DB_ATTR_HANDLE_T handle = cyBle_customs[0].customServiceInfo[0].customServiceCharHandle;
        CyBle_GattsEnableAttribute(handle);
        {
            uint16_t cccdValue;
            
            // Battery Service
            CyBle_BasRegisterAttrCallback(BleCallback);
            CyBle_BassGetCharacteristicDescriptor(CYBLE_BATTERY_SERVICE_INDEX, CYBLE_BAS_BATTERY_LEVEL,
                    CYBLE_BAS_BATTERY_LEVEL_CCCD, CYBLE_CCCD_LEN, (uint8 *)&cccdValue);
            
            // Device Information Service
            CyBle_DisRegisterAttrCallback(BleCallback);
        }
        break;
    case CYBLE_EVT_GAP_ENCRYPT_CHANGE:
        {
            char line[10];
            char* p = line;
            
            UartPutString("EVT_GAP_ENCRYPT_CHANGE: ");
            p = writeHex8(p, *(uint8_t*)eventParam);
            
            *(p++) = '\n';
            *(p++) = 0;
            UartPutString(line);
        }
        break;
    case CYBLE_EVT_GAP_KEYINFO_EXCHNGE_CMPLT:
        {
            char line[80];
            char* p = line;
            
            CYBLE_GAP_SMP_KEY_DIST_T * key = (CYBLE_GAP_SMP_KEY_DIST_T *)eventParam;
            UartPutString("CYBLE_EVT_GAP_KEYINFO_EXCHNGE_CMPLT: ");
            for(int i = 0; i < 16; i++)
            {
                p = writeHex8(p, key->ltkInfo[i]);
            }
            *(p++) = ',';
            for(int i = 0; i < 10; i++)
            {
                p = writeHex8(p, key->midInfo[i]);
            }
            *(p++) = '\n';
            *(p++) = 0;
            UartPutString(line);
        }
        break;
    case CYBLE_EVT_GATT_DISCONNECT_IND:
        UartPutString("GATT_DISCONNECT_IND \r\n");
        break;
    
    case CYBLE_EVT_GAP_PASSKEY_DISPLAY_REQUEST:
        {
            char line[10];
            char* p = line;
            
            UartPutString("EVT_GAP_PASSKEY_DISPLAY_REQUEST: ");
            p = writeHex32(p, *(uint32_t*)eventParam);
            
            *(p++) = '\n';
            *(p++) = 0;
            UartPutString(line);
        }
        break;
        
    case CYBLE_EVT_GAP_AUTH_FAILED:
        {
            char line[10];
            char* p = line;
            
            UartPutString("EVT_GAP_AUTH_FAILED: ");
            p = writeHex8(p, *(uint8_t*)eventParam);
            
            *(p++) = '\n';
            *(p++) = 0;
            UartPutString(line);
        }
        break;
    case CYBLE_EVT_GATTS_WRITE_REQ:
        // Write request
        {
            CYBLE_GATTS_WRITE_REQ_PARAM_T* param = (CYBLE_GATTS_WRITE_REQ_PARAM_T*)eventParam;
            switch(param->handleValPair.attrHandle)
            {
            case CYBLE_CURRENT_SENSING_SERVICE_CURRENT_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE:
                {
                    if( param->handleValPair.value.len > 0 )
                    {
                        uint8_t value = *param->handleValPair.value.val;
                        isNotificationEnabled = value != 0;
                    }
                    CyBle_GattsWriteAttributeValue(&param->handleValPair, 0, &param->connHandle, CYBLE_GATT_DB_LOCALLY_INITIATED);
                    CyBle_GattsWriteRsp(param->connHandle);
                }
                break;
            case CYBLE_CURRENT_SENSING_SERVICE_COEFFICIENT_CHAR_HANDLE:
                {
                    if( param->handleValPair.value.len >= 8 )
                    {
                        CyBle_GattsWriteAttributeValue(&param->handleValPair, 0, &param->connHandle, CYBLE_GATT_DB_PEER_INITIATED);
                    }
                    CyBle_GattsWriteRsp(param->connHandle);
                }
                break;
            }
        }
        break;
    case CYBLE_EVT_HCI_STATUS:
        {
            char line[4];
            char* p = line;
            
            UartPutString("EVT_HCI_STATUS: ");
            p = writeHex8(p, *(uint8_t*)eventParam);
            
            *(p++) = '\n';
            *(p++) = 0;
            UartPutString(line);
        }
        break;
//    case CYBLE_EVT_PENDING_FLASH_WRITE:
//        {
//            CyBle_StoreBondingData(false);
//        }
//        break;
    default:
        {
            char line[10];
            char* p = line;
            UartPutString("Unknown Event: ");
            p = writeHex32(p, eventCode);
            *p++ = '\n'; *p = '0';
            UartPutString(line);
        }
        break;
    }
}

int main()
{

#ifdef DEBUG_OUT
    UART_Debug_Start();
#endif 
    ADC_SAR_SEQ_Start();
    OpUpper_Start();
    OpLower_Start();
    
    sensorAccumulator = 0;
    
    ISR_WatchDog_StartEx(WatchDog_Interrupt_Handler);
    CySysWdtEnable(CY_SYS_WDT_COUNTER0_MASK);
    
    ADC_SAR_SEQ_IRQ_SetVector(ADC_Interrupt_Handler);
    ADC_SAR_SEQ_IRQ_Enable();
    
    CyGlobalIntEnable;
    CySysClkIloStop();
//    CySysClkWriteEcoDiv(CY_SYS_CLK_ECO_DIV8);
    
    // Initialize BLE module
    CyBle_Start(BleCallback);
    
    UartPutString("ACCurrentSensor\n");    
    
    
    bool canEnterToDeepSleep = false;
    
    for(;;)
    {
        CyBle_ProcessEvents();
        CyBle_EnterLPM(CYBLE_BLESS_DEEPSLEEP); 
        
        if( CyBle_GetState() == CYBLE_STATE_CONNECTED )
        {
            switch(adcState)
            {
            case ADC_STATE_COMPLETED:
                {
                    CYBLE_GATT_DB_ATTR_HANDLE_T handle = cyBle_customs[0].customServiceInfo[0].customServiceCharHandle;
                    {
                        CYBLE_GATTS_HANDLE_VALUE_NTF_T pair = {
                            {(uint8_t*)&sensorValue, 4, 0},
                            handle,
                        };
                        CyBle_GattsWriteAttributeValue(&pair, 0, &cyBle_connHandle, CYBLE_GATT_DB_LOCALLY_INITIATED);
                        if( isNotificationEnabled )
                        {
                            CyBle_GattsNotification(cyBle_connHandle, &pair);
                        }
                    }
                    //Pin_LED_G_Write(~Pin_LED_G_Read());
                    adcState = ADC_STATE_IDLE;
                    
                    // The sensor value has been updated. Enter to deepsleep.
                    canEnterToDeepSleep = true;
                }
                break;
            case ADC_STATE_IDLE:
                canEnterToDeepSleep = true;
                break;
            case ADC_STATE_ACTIVE:
                canEnterToDeepSleep = false;
                break;
            }
        }
        else
        {
            // Not connected. The system should enter to deepsleep.
            canEnterToDeepSleep = true;
        }

        
        /* Prevent interrupts while entering system low power modes */
        uint8 intrStatus = CyEnterCriticalSection();
        
        /* Get the current state of BLESS block */
        CYBLE_BLESS_STATE_T blessState = CyBle_GetBleSsState();
        
        /* If BLESS is in Deep-Sleep mode or the XTAL oscillator is turning on, 
         * then PSoC 4 BLE can enter Deep-Sleep mode (1.3uA current consumption) */
        if((blessState == CYBLE_BLESS_STATE_ECO_ON || blessState == CYBLE_BLESS_STATE_DEEPSLEEP) && canEnterToDeepSleep )
        {
            CySysPmDeepSleep();
        }
        else
        {
            /* If BLESS is active, then configure PSoC 4 BLE system in 
             * Sleep mode (~1.6mA current consumption) */
            CySysPmSleep();
        }
        
        CyExitCriticalSection(intrStatus);
        
        /* BLE link layer timing interrupt will wake up the system from Sleep 
         * and Deep-Sleep modes */
        /*
        if( cyBle_pendingFlashWrite != 0 )
        {
            if( CyBle_StoreBondingData(0) == CYBLE_ERROR_OK )
            {
                UartPutString("Bonding data was stored.\n");
            }
        }
        */
    }
}

/* [] END OF FILE */
