/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include <project.h>
#include <stdint.h>
#include <stdbool.h>

static char* writeHex8(char* p, uint8_t n)
{
    uint8_t nibble;
    nibble = n >> 4;
    *(p++) = nibble < 0x0a ? nibble + '0' : nibble + 'A' - 0x0a;
    nibble = n & 0x0f;
    *(p++) = nibble < 0x0a ? nibble + '0' : nibble + 'A' - 0x0a;
    return p;
}
static char* writeHex16(char* p, uint16_t n)
{
    p = writeHex8(p, n >> 8);
    p = writeHex8(p, n & 0xff);
    return p;
}
static char* writeHex32(char* p, uint32_t n)
{
    p = writeHex8(p, (n >> 24) & 0xff);
    p = writeHex8(p, (n >> 16) & 0xff);
    p = writeHex8(p, (n >>  8) & 0xff);
    p = writeHex8(p, (n >>  0) & 0xff);
    return p;
}

static uint32_t sensorAccumulator = 0;
static uint32_t sensorValue = 0;
static uint8_t sensorCount = 0;
static bool isSensorValueUpdated = false;
static bool isNotificationEnabled = false;
typedef enum 
{
    ADC_STATE_SENSOR,
    ADC_STATE_BATTERY,
} ADC_STATE;
static ADC_STATE adcState = ADC_STATE_SENSOR;

CY_ISR(ADC_Interrupt_Handler)
{
    uint32_t intr_status = ADC_SAR_SEQ_SAR_INTR_REG;
    int16_t value = ADC_SAR_SEQ_GetResult16(0);
    
    switch(adcState)
    {
    case ADC_STATE_SENSOR:
        sensorAccumulator += value < 0 ? -value : value;
        sensorCount = (sensorCount + 1) & 127;
        if( sensorCount == 0 )
        {
            sensorValue = sensorAccumulator >> (7 + 2);
            sensorAccumulator = 0;
            // TODO: start to measure battery level at next conversion.
            ADC_SAR_SEQ_StartConvert();
        }
        else
        {
            ADC_SAR_SEQ_StartConvert();
        }
        break;
    case ADC_STATE_BATTERY:
        adcState = ADC_STATE_SENSOR;
        break;
    }
    ADC_SAR_SEQ_SAR_INTR_REG = intr_status;
}

CY_ISR(ReportTimerInterrupt_Handler)
{
    uint32_t intr_status = Timer_Report_INTERRUPT_REQ_REG;
    char line[10];
    char* p = line;
    
    p = writeHex32(p, sensorValue);
    
    *(p++) = '\n';
    *(p++) = 0;
    UART_Debug_UartPutString(line);
    
    isSensorValueUpdated = true;
    
    Timer_Report_INTERRUPT_REQ_REG = intr_status;
}

static void BleCallback(uint32 eventCode, void *eventParam)
{
    switch(eventCode)
    {
    case CYBLE_EVT_STACK_ON:
    case CYBLE_EVT_GAP_DEVICE_DISCONNECTED:
        // Start Advertisement
        UART_Debug_UartPutString("GAP_DEVICE_DISCONNECTED\n");
        CyBle_GappStartAdvertisement(CYBLE_ADVERTISING_FAST);
        break;
    case CYBLE_EVT_GAP_DEVICE_CONNECTED:
        UART_Debug_UartPutString("GAP_DEVICE_CONNECTED\n");
        break;
    case CYBLE_EVT_TIMEOUT:
        UART_Debug_UartPutString("TIMEOUT\n");
        break;
    case CYBLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
        UART_Debug_UartPutString("ADVERTISEMENT_START_STOP\n");
        break;
    case CYBLE_EVT_GATT_CONNECT_IND:
        UART_Debug_UartPutString("GATT_CONNECT_IND\n");
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
    case CYBLE_EVT_GATT_DISCONNECT_IND:
        UART_Debug_UartPutString("GATT_DISCONNECT_IND \r\n");
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
            
            UART_Debug_UartPutString("EVT_HCI_STATUS: ");
            p = writeHex8(p, *(uint8_t*)eventParam);
            
            *(p++) = '\n';
            *(p++) = 0;
            UART_Debug_UartPutString(line);
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
            UART_Debug_UartPutString("Unknown Event: ");
            p = writeHex32(p, eventCode);
            *p++ = '\n'; *p = '0';
            UART_Debug_UartPutString(line);
        }
        break;
    }
}

int main()
{
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    Clock_1k_Start();
    UART_Debug_Start();
    ADC_SAR_SEQ_Start();
    Timer_Report_Start();
    
    sensorAccumulator = 0;
    
    ADC_SAR_SEQ_IRQ_SetVector(ADC_Interrupt_Handler);
    ADC_SAR_SEQ_IRQ_Enable();
    
    ReportTimerInterrupt_SetVector(ReportTimerInterrupt_Handler);
    ReportTimerInterrupt_Enable();
    
    // Initialize BLE module
    CyBle_Start(BleCallback);
    
    CyGlobalIntEnable;
    
    ADC_SAR_SEQ_StartConvert();
    
    UART_Debug_UartPutString("ACCurrentSensor\n");    
    
    for(;;)
    {
        CyBle_ProcessEvents();
        
        if( CyBle_GetState() == CYBLE_STATE_CONNECTED )
        {
            if( isSensorValueUpdated )
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
                
                isSensorValueUpdated = false;
            }
        }
        
        /* Prevent interrupts while entering system low power modes */
        uint8 intrStatus = CyEnterCriticalSection();
        
        /* Get the current state of BLESS block */
        CYBLE_BLESS_STATE_T blessState = CyBle_GetBleSsState();
        
        /* If BLESS is in Deep-Sleep mode or the XTAL oscillator is turning on, 
         * then PSoC 4 BLE can enter Deep-Sleep mode (1.3uA current consumption) */
        if(blessState == CYBLE_BLESS_STATE_ECO_ON || 
            blessState == CYBLE_BLESS_STATE_DEEPSLEEP)
        {
            CySysPmDeepSleep();
        }
        else if(blessState != CYBLE_BLESS_STATE_EVENT_CLOSE)
        {
            /* If BLESS is active, then configure PSoC 4 BLE system in 
             * Sleep mode (~1.6mA current consumption) */
            CySysPmSleep();
        }
        else
        {
            /* Keep trying to enter either Sleep or Deep-Sleep mode */    
        }
        CyExitCriticalSection(intrStatus);
        
        /* BLE link layer timing interrupt will wake up the system from Sleep 
         * and Deep-Sleep modes */
    }
}

/* [] END OF FILE */
