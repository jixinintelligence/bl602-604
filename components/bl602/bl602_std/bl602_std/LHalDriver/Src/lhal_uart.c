#include "bl602_uart.h"
#include "bl602_glb.h"
#include "lhal_uart.h"

static void LHAL_UART_TX_Int_CallBack0(void);
static void LHAL_UART_TX_Int_CallBack1(void);
static void LHAL_UART_RX_Int_CallBack0(void);
static void LHAL_UART_RX_Int_CallBack1(void);

static LHAL_UART_Handle_Type* uartHandlerList[UART_ID_MAX] = {NULL};
static intCallback_Type* uartTxIntCallBackList[UART_ID_MAX] = {&LHAL_UART_TX_Int_CallBack0,&LHAL_UART_TX_Int_CallBack1};
static intCallback_Type* uartRxIntCallBackList[UART_ID_MAX] = {&LHAL_UART_RX_Int_CallBack0,&LHAL_UART_RX_Int_CallBack1};

LHAL_Err_Type LHal_UART_Init(LHAL_UART_Handle_Type *uartHandler)
{
    LHal_Glue_UART_Init(uartHandler);

    if(NULL!=uartHandler->txBuf){
        Ring_Buffer_Init(&uartHandler->txRb,uartHandler->txBuf,uartHandler->txBufLen,uartHandler->txRbLock,uartHandler->txRbUnlock);
    }
    if(NULL!=uartHandler->rxBuf){
        Ring_Buffer_Init(&uartHandler->rxRb,uartHandler->rxBuf,uartHandler->rxBufLen,uartHandler->rxRbLock,uartHandler->rxRbUnlock);
    }
    return LHAL_SUCCESS;
}

LHAL_Err_Type LHal_UART_DeInit(uint8_t uartId)
{
    UART_DeInit((UART_ID_Type)uartId);

    return LHAL_SUCCESS;
}

LHAL_Err_Type LHal_UART_Send_Polling(LHAL_UART_Handle_Type *uartHandler,uint8_t *data,uint32_t len,uint32_t timout)
{
    return LHal_Glue_UART_SendData(uartHandler,data,len);
}

int32_t LHal_UART_Recv_Polling(LHAL_UART_Handle_Type *uartHandler,uint8_t *data,uint32_t maxLen,uint32_t timout)
{
    int32_t recvLen=0;
    //uint64_t curMs=0;

    //curMs=LHAL_Get_Time_MS();
    do{
        //if(LHAL_Get_Time_MS()>curMs+timout){
        //    return 0;
        //}
        recvLen+=LHal_Glue_UART_RecvData(uartHandler,data,maxLen);
    }while(recvLen==0);

    return recvLen;
}

LHAL_Err_Type LHal_UART_Send_IT(LHAL_UART_Handle_Type *uartHandler,uint8_t *data,uint32_t len,uint32_t timout)
{
    uint32_t sentLen=0;
    uint32_t curLen=0;
    //uint64_t curMs=0;
    LHAL_Err_Type ret=LHAL_SUCCESS;

    if(NULL!=uartHandler->txBuf){
        //curMs=LHAL_Get_Time_MS();
        do{
            /* do send data */
            curLen=Ring_Buffer_Get_Empty_Length(&uartHandler->txRb);
            if(curLen>len-sentLen){
                curLen=len-sentLen;
            }
            if(curLen==0){
                LHal_Glue_UART_Enable_TX_Int(uartHandler);
            }else{
                Ring_Buffer_Write(&uartHandler->txRb,data+sentLen,curLen);
                sentLen+=curLen;
            }

            if(timout==0&&sentLen<len){
                ret=LHAL_ERROR;
                break;
            }else{
                //if(LHAL_Get_Time_MS()>curMs+timout&&sentLen<len){
                //    return LHAL_TIMEOUT;
                //}
            }
        }while(sentLen<len);

        /* Enable TX interrupt anyway*/
        LHal_Glue_UART_Enable_TX_Int(uartHandler);
    }
    return ret;
}

int32_t LHal_UART_Recv_IT(LHAL_UART_Handle_Type *uartHandler,uint8_t *data,uint32_t maxLen,uint32_t timout)
{
    //uint64_t curMs=0;
    uint32_t recvLen=0;
    if(NULL!=uartHandler->rxBuf){
        //curMs=LHAL_Get_Time_MS();
        do{
            recvLen=Ring_Buffer_Get_Length(&uartHandler->rxRb);
            if(recvLen>maxLen){
                recvLen=maxLen;
            }
            if(timout==0&&recvLen==0){
                LHal_Glue_UART_Enable_RX_Int(uartHandler);
                return 0;
            }else{
                LHal_Glue_UART_Enable_RX_Int(uartHandler);
                //if(LHAL_Get_Time_MS()>curMs+timout&&recvLen==0){
                //    return LHAL_TIMEOUT;
                //}
            }
        }while(recvLen==0);

        maxLen=Ring_Buffer_Read(&uartHandler->rxRb,data,recvLen);
        LHal_Glue_UART_Enable_RX_Int(uartHandler);
        return maxLen;
    }
    return 0;
}


static void LHAL_UART_TX_RB_CallBack(void *parameter,uint8_t *data,uint16_t len)
{
    LHAL_UART_Handle_Type *uartHandler=parameter;

    LHal_Glue_UART_SendData(uartHandler,data,len);
}

static void LHAL_UART_TX_Int_CallBack(uint8_t uartId)
{
    uint32_t curLen=0;
    uint32_t emptyLen=0;
    LHAL_UART_Handle_Type *uartHandler=uartHandlerList[uartId];
    if(NULL!=uartHandler->txBuf){
        curLen=Ring_Buffer_Get_Length(&uartHandler->txRb);
        emptyLen=UART_GetTxFifoCount(uartHandler->cfg.id);
        if(curLen>emptyLen){
            curLen=emptyLen;
        }
        if(curLen!=0){
            /*Ring buffer has data, just send it out and return */
            Ring_Buffer_Read_Callback(&uartHandler->txRb,curLen,LHAL_UART_TX_RB_CallBack,uartHandler);
            if(uartHandler->txDoneCallBack){
                uartHandler->txDoneCallBack(uartHandler);
            }
            return;
        }
    }
    if(uartHandler->txFullCallBack){
        uartHandler->txFullCallBack(uartHandler);
    }
    /* Mask interrupt and wait for data */
    LHal_Glue_UART_Disable_TX_Int(uartHandler);
}

static void LHAL_UART_TX_Int_CallBack0(void)
{
    LHAL_UART_TX_Int_CallBack(0);
}

static void LHAL_UART_TX_Int_CallBack1(void)
{
    LHAL_UART_TX_Int_CallBack(1);
}

static void LHAL_UART_RX_RB_CallBack(void *parameter,uint8_t *data,uint16_t len)
{
    LHAL_UART_Handle_Type *uartHandler=parameter;

    LHal_Glue_UART_RecvData(uartHandler,data,len);
}

static void LHAL_UART_RX_Int_CallBack(uint8_t uartId)
{
    LHAL_UART_Handle_Type *uartHandler=uartHandlerList[uartId];
    uint32_t rxLen=0;
    uint32_t emptyLen=0;
    if(NULL!=uartHandler->rxBuf){
        emptyLen=Ring_Buffer_Get_Empty_Length(&uartHandler->rxRb);
        rxLen=UART_GetRxFifoCount(uartHandler->cfg.id);
        if(rxLen>emptyLen){
            rxLen=emptyLen;
        }
        if(rxLen!=0){
            Ring_Buffer_Write_Callback(&uartHandler->rxRb,rxLen,LHAL_UART_RX_RB_CallBack,uartHandler);
            if(uartHandler->rxDoneCallBack){
                uartHandler->rxDoneCallBack(uartHandler);
            }
            return;
        }else{
            LHal_Glue_UART_Disable_RX_Int(uartHandler);
            /*Ring buffer is full, what to do*/
            if(uartHandler->rxFullCallBack){
                uartHandler->rxFullCallBack(uartHandler);
            }
        }
    }
}

static void LHAL_UART_RX_Int_CallBack0(void)
{
    LHAL_UART_RX_Int_CallBack(0);
}

static void LHAL_UART_RX_Int_CallBack1(void)
{
    LHAL_UART_RX_Int_CallBack(1);
}

BL_Err_Type LHal_UART_Notify_Register(LHAL_UART_Handle_Type *uartHandler)
{
    Ring_Buffer_Init(&uartHandler->txRb,uartHandler->txBuf,sizeof(uartHandler->txBuf),NULL,NULL);
    Ring_Buffer_Init(&uartHandler->rxRb,uartHandler->rxBuf,sizeof(uartHandler->rxBuf),NULL,NULL);
    
    return LHAL_SUCCESS;
}

LHAL_Err_Type LHal_Glue_UART_Init(LHAL_UART_Handle_Type *uartHandler)
{
    UART_CFG_Type uartCfg = {
        96*1000*1000,                                        /* UART clock */
        115200,                                              /* UART Baudrate */
        UART_DATABITS_8,                                     /* UART data bits length */
        UART_STOPBITS_1,                                     /* UART data stop bits length */
        UART_PARITY_NONE,                                    /* UART no parity */
        DISABLE,                                             /* Disable auto flow control */
        DISABLE,                                             /* Disable rx input de-glitch function */
        DISABLE,                                             /* Disable RTS output SW control mode */
        UART_LSB_FIRST                                       /* UART each data byte is send out LSB-first */
    };

    UART_FifoCfg_Type fifoCfg = {
       16,                                                  /* TX FIFO threshold */
       16,                                                  /* RX FIFO threshold */
       DISABLE,                                             /* Disable tx dma req/ack interface */
       DISABLE                                              /* Disable rx dma req/ack interface */
   };

    uartHandlerList[uartHandler->cfg.id]=uartHandler;

    /* Apply config */
    uartCfg.baudRate=uartHandler->cfg.baudRate;
    uartCfg.dataBits=uartHandler->cfg.dataBits-5+UART_DATABITS_5;
    uartCfg.stopBits=uartHandler->cfg.stopBits-1+UART_STOPBITS_1;
    uartCfg.parity=uartHandler->cfg.parity;
    if(uartHandler->cfg.enableRts){
        uartCfg.rtsSoftwareControl=DISABLE;
    }else{
        uartCfg.rtsSoftwareControl=ENABLE;
    }
    if(uartHandler->cfg.enableCts){
        uartCfg.ctsFlowControl=ENABLE;
    }else{
        uartCfg.ctsFlowControl=DISABLE;
    }


    /* Disable all interrupt */
    UART_IntMask(uartHandler->cfg.id,UART_INT_ALL,MASK);

    /* Disable UART before config */
    UART_Disable(uartHandler->cfg.id,UART_TXRX);

    /* UART init */
    UART_Init(uartHandler->cfg.id,&uartCfg);

    /* UART fifo configuration */
    UART_FifoConfig(uartHandler->cfg.id,&fifoCfg);

    /* Enable tx free run mode */
    UART_TxFreeRun(uartHandler->cfg.id,ENABLE);

    /* Set rx time-out value */
    UART_SetRxTimeoutValue(uartHandler->cfg.id,UART_DEFAULT_RECV_TIMEOUT);

    /* UART interrupt configuration */
    if(NULL!=uartHandler->txBuf){
        /* Install the interrupt callback function */
        UART_Int_Callback_Install(uartHandler->cfg.id,UART_INT_TX_FIFO_REQ,uartTxIntCallBackList[uartHandler->cfg.id]);
        UART_IntMask(uartHandler->cfg.id,UART_INT_TX_FIFO_REQ,UNMASK);
    }
    if(NULL!=uartHandler->rxBuf){
        UART_Int_Callback_Install(uartHandler->cfg.id,UART_INT_RX_FIFO_REQ,uartRxIntCallBackList[uartHandler->cfg.id]);
        UART_Int_Callback_Install(uartHandler->cfg.id,UART_INT_RTO,uartRxIntCallBackList[uartHandler->cfg.id]);
        UART_IntMask(uartHandler->cfg.id,UART_INT_RX_FIFO_REQ,UNMASK);
        UART_IntMask(uartHandler->cfg.id,UART_INT_RTO,UNMASK);
    }

    /* Enable UART interrupt*/
    NVIC_EnableIRQ(uartHandler->cfg.id+UART0_IRQn);
    System_NVIC_SetPriority(uartHandler->cfg.id+UART0_IRQn,4,1);

    /* Enable UART */
    if(uartHandler->cfg.enableTx&&uartHandler->cfg.enableRx){
        UART_Enable(uartHandler->cfg.id,UART_TXRX);
    }else if(uartHandler->cfg.enableTx){
        UART_Enable(uartHandler->cfg.id,UART_TX);
    }else if(uartHandler->cfg.enableRx){
        UART_Enable(uartHandler->cfg.id,UART_RX);
    }
    
    return LHAL_SUCCESS;
}

LHAL_Err_Type LHal_Glue_UART_Enable_TX_Int(LHAL_UART_Handle_Type *uartHandler)
{
    /* Mask interupt and wait for data */
    UART_IntMask(uartHandler->cfg.id,UART_INT_TX_FIFO_REQ,UNMASK);

    return LHAL_SUCCESS;
}

LHAL_Err_Type LHal_Glue_UART_Enable_RX_Int(LHAL_UART_Handle_Type *uartHandler)
{
    UART_IntMask(uartHandler->cfg.id,UART_INT_RX_FIFO_REQ,UNMASK);
    UART_IntMask(uartHandler->cfg.id,UART_INT_RTO,UNMASK);

    return LHAL_SUCCESS;
}

LHAL_Err_Type LHal_Glue_UART_Disable_TX_Int(LHAL_UART_Handle_Type *uartHandler)
{
    /* Mask interupt and wait for data */
    UART_IntMask(uartHandler->cfg.id,UART_INT_TX_FIFO_REQ,MASK);

    return LHAL_SUCCESS;
}

LHAL_Err_Type LHal_Glue_UART_Disable_RX_Int(LHAL_UART_Handle_Type *uartHandler)
{
    UART_IntMask(uartHandler->cfg.id,UART_INT_RX_FIFO_REQ,MASK);
    UART_IntMask(uartHandler->cfg.id,UART_INT_RTO,MASK);

    return LHAL_SUCCESS;
}

LHAL_Err_Type LHal_Glue_UART_SendData(LHAL_UART_Handle_Type *uartHandler,uint8_t* data,uint32_t len)
{

    BL_Err_Type ret=UART_SendData(uartHandler->cfg.id,data,len);

    if(ret==SUCCESS){
        return LHAL_SUCCESS;
    }else if(ret==TIMEOUT){
        return LHAL_TIMEOUT;
    }
    return LHAL_SUCCESS;
}

int32_t LHal_Glue_UART_RecvData(LHAL_UART_Handle_Type *uartHandler,uint8_t* data,uint32_t maxLen)
{
    return UART_ReceiveData(uartHandler->cfg.id,data,maxLen);
}
