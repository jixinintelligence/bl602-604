#include "bl602_common.h"
#include "ring_buffer.h"


typedef enum
{
  LHAL_SUCCESS  = 0,
  LHAL_ERROR   = -1,
  LHAL_TIMEOUT = -2,
}LHAL_Err_Type;

typedef struct {
    uint8_t id;
    uint8_t dataBits;
    uint8_t stopBits;
    uint8_t parity;
    uint32_t baudRate;
    uint8_t enableCts;
    uint8_t enableRts;
    uint8_t enableTx;
    uint8_t enableRx;
}LHAL_UART_CFG_Type;

typedef struct __LHAL_UART_Handle_Type{
    LHAL_UART_CFG_Type cfg;
    /* tx ring buffer*/
    uint8_t *txBuf;
    uint8_t txBufLen;
    Ring_Buffer_Type txRb;
    ringBuffer_Lock_Callback* txRbLock;
    ringBuffer_Lock_Callback* txRbUnlock;
    /* rx ring buffer*/
    uint8_t *rxBuf;
    uint8_t rxBufLen;
    Ring_Buffer_Type rxRb;
    ringBuffer_Lock_Callback* rxRbLock;
    ringBuffer_Lock_Callback* rxRbUnlock;

    /* call back function when use interrupt */
    void (* txDoneCallBack)(struct __LHAL_UART_Handle_Type *uartHandler);
    void (* txFullCallBack)(struct __LHAL_UART_Handle_Type *uartHandler);
    void (* rxDoneCallBack)(struct __LHAL_UART_Handle_Type *uartHandler);
    void (* rxFullCallBack)(struct __LHAL_UART_Handle_Type *uartHandler);

}LHAL_UART_Handle_Type;

LHAL_Err_Type LHal_UART_Init(LHAL_UART_Handle_Type *uartHandler);
LHAL_Err_Type LHal_UART_DeInit(uint8_t uartId);
LHAL_Err_Type LHal_UART_Send_Polling(LHAL_UART_Handle_Type *uartHandler,uint8_t *data,uint32_t len,uint32_t timout);
int32_t LHal_UART_Recv_Polling(LHAL_UART_Handle_Type *uartHandler,uint8_t *data,uint32_t maxLen,uint32_t timout);
LHAL_Err_Type LHal_UART_Send_IT(LHAL_UART_Handle_Type *uartHandler,uint8_t *data,uint32_t len,uint32_t timout);
int32_t LHal_UART_Recv_IT(LHAL_UART_Handle_Type *uartHandler,uint8_t *data,uint32_t maxLen,uint32_t timout);
BL_Err_Type LHal_UART_Notify_Register(LHAL_UART_Handle_Type *uartHandler);
LHAL_Err_Type LHal_Glue_UART_Init(LHAL_UART_Handle_Type *uartHandler);
LHAL_Err_Type LHal_Glue_UART_Enable_TX_Int(LHAL_UART_Handle_Type *uartHandler);
LHAL_Err_Type LHal_Glue_UART_Enable_RX_Int(LHAL_UART_Handle_Type *uartHandler);
LHAL_Err_Type LHal_Glue_UART_Disable_TX_Int(LHAL_UART_Handle_Type *uartHandler);
LHAL_Err_Type LHal_Glue_UART_Disable_RX_Int(LHAL_UART_Handle_Type *uartHandler);
LHAL_Err_Type LHal_Glue_UART_SendData(LHAL_UART_Handle_Type *uartHandler,uint8_t* data,uint32_t len);
int32_t LHal_Glue_UART_RecvData(LHAL_UART_Handle_Type *uartHandler,uint8_t* data,uint32_t maxLen);
