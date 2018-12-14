#include "rc523.h"
#include "port.h"
#include <string.h>

static uint8_t ReadReg(uint8_t addr);
static void WriteReg(uint8_t addr,uint8_t v);
static void ClearBitMask(uint8_t addr,uint8_t v);
static void SetBitMask(uint8_t addr,uint8_t v);

uint8_t RCRequestTypeB(void);
uint8_t RCATTRIBTypeB(void);
uint8_t RCGetUIDTypeB(u8 *pUID);

uint16_t test;
uint8_t idtest[5];
uint8_t regbuff[0x40];
void RC523Init(void)
{
  SetCsHigh();
  /// GPIO_Init(GPIOB, GPIO_Pin_2, GPIO_Mode_Out_PP_Low_Slow);
  SetCsLow();
  
  /// SpiInit();
  
  RC_PcdReset();
  RC_PcdISOType(RC_ISO14443_B);
  /*RCRequestTypeB();
  RCATTRIBTypeB();
  RCGetUIDTypeB(regbuff);*/
  //WriteReg(ModeReg, 0x3D);
  
  //WriteReg(CommandReg,0x10);
  while(1)
  {
    /// RCPcdRequest(PICC_REQALL,idtest);
    /// //test = ADC_GetConversionValue(ADC1);
    /// for(int i = 0 ; i < 1000 ; i ++);
    /// ReadCardId(idtest);
    
    for(int i = 0 ; i < 1000 ; i ++);
    RCRequestTypeB();
    RCATTRIBTypeB();
    RCGetUIDTypeB(regbuff);
    for(int i = 0 ; i < 16000 ; i ++);
  }
}

static void RC_DelayMs(uint16_t t)
{
    uint16_t i;

    while (t--)
    {
        for (i=0; i<16000; i++)
        {
            ;
        }
    }
}
static uint8_t ReadReg(uint8_t addr)
{
  uint8_t ret;
  uint8_t raddr = ((addr << 1) & 0x7e) | 0x80;
  SetCsLow();
  /*while(!(SPI1->SR & SPI_FLAG_TXE));
  ret = SPI1->DR;
  SPI1->DR = raddr;
  while(!(SPI1->SR & SPI_FLAG_TXE));
  while(!(SPI1->SR & SPI_FLAG_RXNE));
  ret = SPI1->DR;
  SPI1->DR = 0xff;
  while(!(SPI1->SR & SPI_FLAG_RXNE));
  ret = SPI1->DR;*/
  //ret = SpiReadWriteByte(0xff);
  SpiReadWriteByte(raddr);
  ret = SpiReadWriteByte(0xff);
  SetCsHigh();
  return ret;
}

static void WriteReg(uint8_t addr,uint8_t v)
{
  uint8_t waddr = (addr << 1) & 0x7e;
  SetCsLow();
 /* while(!(SPI1->SR & SPI_FLAG_TXE));
  SPI1->DR = waddr;
  while(!(SPI1->SR & SPI_FLAG_TXE));
  SPI1->DR = v;
  while(!(SPI1->SR & SPI_FLAG_TXE));*/
  SpiReadWriteByte(waddr);
  SpiReadWriteByte(v);
  SetCsHigh();
}

static void ClearBitMask(uint8_t addr,uint8_t v)
{
  WriteReg(addr,ReadReg(addr) & (~v));
}

static void SetBitMask(uint8_t addr,uint8_t v)
{
  WriteReg(addr,ReadReg(addr) | v);
}

static uint8_t RC_PcdCmd(uint8_t cmd, uint8_t *pIn, uint8_t inLen, uint8_t *pOut, uint8_t *pOutLen)
{     
    uint8_t status  = 0;
    uint8_t irqEn   = 0x00;
    uint8_t waitFor = 0x00;
    uint8_t lastBits;
    uint8_t n;
    uint16_t i;

    switch (cmd)
    {
        case PCD_AUTHENT   : irqEn = 0x12; waitFor = 0x10; break;
        case PCD_TRANSCEIVE: irqEn = 0x77; waitFor = 0x30; break;
        default: break;
    }
   
    WriteReg(ComIEnReg, irqEn | 0x80);
    ClearBitMask(ComIrqReg, 0x80);
    WriteReg(CommandReg, PCD_IDLE);
    SetBitMask(FIFOLevelReg, 0x80);
    
    for (i=0; i<inLen; i++)
    {   
        WriteReg(FIFODataReg, pIn[i]);    
    }
    
    WriteReg(CommandReg, cmd);
   
    if (cmd == PCD_TRANSCEIVE)
    {    
        SetBitMask(BitFramingReg,0x80);  
    }
    
    i = 2000;                                                                   // 根据时钟频率调整，操作M1卡最大等待时间25ms
    do 
    {
        n = ReadReg(ComIrqReg);
        i--;
        
    } while ((i!=0) && !(n&0x01) && !(n&waitFor));
    
    ClearBitMask(BitFramingReg, 0x80);
	
  /*  for(int i = 0 ; i < 0x40 ; i ++)
    {
       regbuff[i] = ReadReg(i);
    }*/
    
    if (i != 0)
    {    
        if (!(ReadReg(ErrorReg) & 0x1B))
        {
            status = 0;
            if (n & irqEn & 0x01)
            {   
                status = 1;  
            }
            
            if (cmd == PCD_TRANSCEIVE)
            {
                n = ReadReg(FIFOLevelReg);
              	lastBits = ReadReg(ControlReg) & 0x07;
                *pOutLen = (lastBits) ? ((n-1)*8 + lastBits) : (n*8);
                
                if (n > 18)
                {   
                    n = 18;   
                }
                
                for (i=0; i<n; i++)
                {   
                    pOut[i] = ReadReg(FIFODataReg);    
                }
            }
        }
        else
        {   
            status = 1;   
        }
    }
  
    SetBitMask(ControlReg, 0x80);                                            // stop timer now
    WriteReg(CommandReg, PCD_IDLE); 
    return status;
}

static void RC_Antenna(uint8_t mode)
{
    if (mode == 1)
    {
        SetBitMask(TxControlReg, 0x03);                                  // 天线开
    }
    else
    {
        ClearBitMask(TxControlReg, 0x03);                                    // 天线关
    }
}

void RC_PcdISOType(uint8_t type)
{
    switch (type)
    {
        case  RC_ISO14443_A:
        {
            ClearBitMask(Status2Reg, 0x08);                                      // 
            WriteReg(ModeReg, 0x3D);
            WriteReg(TReloadRegL, 30);
            WriteReg(TReloadRegH, 0);
            WriteReg(TModeReg, 0x8D);
            WriteReg(TPrescalerReg, 0x3E);
            WriteReg(TxASKReg, 0x40);
            
            // ------------------------- 发送部分设置 ------------------------------
            
            // ------------------------- 接收部分设置 ------------------------------
            WriteReg(RFCfgReg, 0x7F);
            WriteReg(RxSelReg, 0x86);
            
            // ------------------------- 天线开关设置 ------------------------------
            //RC_Antenna(0);
            //RC_DelayMs(1);
            RC_Antenna(1); 
            break;
        }
        
        case  RC_ISO14443_B:
        {
            ClearBitMask(Status2Reg, 0x08);
            WriteReg(ModeReg, 0x3F);                                       // For 0xFFFF crc
            WriteReg(TReloadRegL, 30);
            WriteReg(TReloadRegH, 0);
            WriteReg(TModeReg, 0x8D);
            WriteReg(TPrescalerReg, 0x3E); 
            WriteReg(TxASKReg, 0);                                         // Force 100ASK = 0
    
            // ------------------------- 发送部分设置 ------------------------------
            WriteReg(GsNReg, 0xFa);                                        // TX输出电导设置
            WriteReg(CWGsPReg, 0x3F);
            WriteReg(ModGsPReg, 0x2a);  
            //WriteReg(ModGsPReg, 0x1A); // 调制指数设置RegModGsp,, TYPEB ModConductance 0x1A
            WriteReg(TxModeReg, 0x83);                                     // 编码器设置,106kbps,14443B
            WriteReg(BitFramingReg, 0x00);                                 // 调制脉宽,0x13->2.95us RegTypeBFraming ,,TYPEB
            WriteReg(AutoTestReg, 0x00);
            WriteReg(TypeBReg, 0xc0);
            WriteReg(ModWidthReg,0x68);
            WriteReg(DemodReg, 0x5D);
    
            // ------------------------- 接收部分设置 ------------------------------
            // 低二位为接收增益，
            // 00,10,20,30,40,50,60,70
            // 18,23,18,23,33,38,43,48dB
            WriteReg(RFCfgReg, 0x70);   
            //WriteReg(RFCfgReg, 0x70);// 0x59 RegRxControl1//73,
            WriteReg(RxModeReg, 0x83);                                     // 解码器设置,,106kbps,14443B
            WriteReg(RxThresholdReg, 0x65);//0x75);                                // 高四位->最小信号强度，低三位->冲突最小信号强度,最大0xF7
            
            // ------------------------- TYPEB特征参数设定 -------------------------
            ClearBitMask(RxSelReg,0x3F);                                     // TR0
            SetBitMask(RxSelReg, 0x08);
            ClearBitMask(TxModeReg, 0x80);                                   // 无CRC,无奇偶校验
            ClearBitMask(RxModeReg, 0x80);
            ClearBitMask(Status2Reg, 0x08);                                  // MFCrypto1On =0
    
            // ------------------------- 天线开关设置 ------------------------------
            //RC_Antenna(0);
            //RC_DelayMs(1);
            RC_Antenna(1);   
            /*ClearBitMask(Status2Reg,0x08);
            WriteReg(ModeReg,0x2F); 
            WriteReg(TReloadRegL,30);
            WriteReg(TReloadRegH,0);
            WriteReg(TModeReg,0x8D);//
            WriteReg(TPrescalerReg,0x3E);
            WriteReg(TxASKReg,0);
            WriteReg(GsNReg,0xF8);
            WriteReg(CWGsPReg,0x3F);
            WriteReg(ModGsPReg,0x0D);
            WriteReg(TxModeReg,0x03);
            WriteReg(BitFramingReg,0);//
            WriteReg(AutoTestReg,0);
            WriteReg(RFCfgReg,0x73);
            WriteReg(RxModeReg,0x03);
            WriteReg(RxThresholdReg,0x75);
            ClearBitMask(RxSelReg,0x3F);
            SetBitMask(RxSelReg,0x08);
            ClearBitMask(TxModeReg,0x80);
            ClearBitMask(RxModeReg,0x80);
            ClearBitMask(Status2Reg,0x08);//
            RC_Antenna(1);
             WriteReg(TxASKReg, 0x00);
             WriteReg(ControlReg, 0x10);
             WriteReg(TxModeReg, 0x03);
             WriteReg(RxModeReg, 0x0B);
             WriteReg(TypeBReg, 0x03);
             WriteReg(DemodReg, 0x4D);
             WriteReg(GsNReg, 0xFF);
             WriteReg(CWGsPReg, 0x3F);
             WriteReg(ModGsPReg, 0x18);
             WriteReg(RxThresholdReg, 0x4D);
             WriteReg(ModWidthReg,0x68);
             RC_Antenna(1);*/
            break;
        }
       
    }
    
}

uint8_t ReadCardId(uint8_t id[])
{
    uint8_t status;
    uint8_t i, snrCheck = 0;
    uint8_t len;
    uint8_t buf[18]; 

    ClearBitMask(Status2Reg, 0x08);
    WriteReg(BitFramingReg, 0x00);
    ClearBitMask(CollReg, 0x80);
 
    buf[0] = PICC_ANTICOLL1;
    buf[1] = 0x20;

    status = RC_PcdCmd(PCD_TRANSCEIVE, buf, 2, buf, &len);

    if (status == 0)
    {
    	 for (i=0; i<4; i++)
         {   
             *(id+i)  = buf[i];
             snrCheck  ^= buf[i];
         }
         
         if (snrCheck != buf[i])
         {   
             status = 1;    
         }
    }
    
    SetBitMask(CollReg, 0x80);
    
    return status;
}

uint8_t RCPcdRequest(uint8_t reqCode, uint8_t *pType)
{
    uint8_t status;  
    uint8_t len;
    uint8_t buf[18]; 

    ClearBitMask(Status2Reg, 0x08);
    WriteReg(BitFramingReg, 0x07);
    SetBitMask(TxControlReg, 0x03);
 
    buf[0] = reqCode;
    status = RC_PcdCmd(PCD_TRANSCEIVE, buf, 1, buf, &len);
    if ((status == 0) && (len == 0x10))
    {    
        *pType     = buf[0];
        *(pType+1) = buf[1];
    }
    else
    {   
        status = 1;   
    } 

    return status;
}

void RC_PcdReset(void)
{
    GPIO_WriteBit(1);
    RC_DelayMs(1);                                   
    GPIO_WriteBit(0);
    RC_DelayMs(1);                               
    GPIO_WriteBit(1);
    RC_DelayMs(1);

    WriteReg(CommandReg, PCD_RESETPHASE);              
    RC_DelayMs(100);
    /*WriteReg(ModeReg, 0x3D);                                               // 和Mifare卡通讯，CRC初始值0x6363
    WriteReg(TReloadRegL, 30);           
    WriteReg(TReloadRegH, 0);
    WriteReg(TModeReg, 0x8D);
    WriteReg(TPrescalerReg, 0x3E);
    WriteReg(TxASKReg, 0x40); */
}

uint8_t RCRequestTypeB(void)
{
    uint8_t status;
    uint8_t len;
    uint8_t buf[18] = {0};
    
    buf[0] = 0x05;
    buf[1] = 0x00;
    buf[2] = 0x00;
    
    buf[3] = 0x71;                                                              // crc
    buf[4] = 0xFF;
    //RC_CalulateCRC(buf, 3, &buf[3]);
    status = RC_PcdCmd(PCD_TRANSCEIVE, buf, 5, buf, &len);                      // 判断回应的数据buf是否为"50,00,...."           
    
    return status;
}

/**************************************************************************************
* FunctionName   : RCATTRIBTypeB()
* Description    : 向TypeB卡发送ATTRIB指令
* EntryParameter : None
* ReturnValue    : 成功返回MI_OK
**************************************************************************************/
uint8_t RCATTRIBTypeB(void)
{
    uint8_t status;
    uint8_t len;
    uint8_t buf[18] = {0};
    
    buf[ 0] = 0x1D;                                                             // 1d 00 00 00 00 00 08 01 08
    
    buf[ 1] = 0x00;                                                             // PUPI
    buf[ 2] = 0x00;
    buf[ 3] = 0x00;
    buf[ 4] = 0x00;
    
    buf[ 5] = 0x00;
    buf[ 6] = 0x08;
    buf[ 7] = 0x01;
    buf[ 8] = 0x08;
    
    buf[ 9] = 0xF3;                                                             // crc
    buf[10] = 0x10;
    //RC_CalulateCRC(buf, 9, &buf[9]);
    status = RC_PcdCmd(PCD_TRANSCEIVE, buf, 11, buf, &len);                     // 判断回应的数据buf是否为"08"
    
    return status;
}

/**************************************************************************************
* FunctionName   : RCGetUIDTypeB()
* Description    : 获取UID
* EntryParameter : pUID - 返回UID
* ReturnValue    : 成功返回MI_OK
**************************************************************************************/
uint8_t RCGetUIDTypeB(u8 *pUID)
{
    uint8_t status;
    uint8_t len;
    uint8_t buf[18] = {0};
    
    buf[0] = 0x00;
    buf[1] = 0x36;
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4] = 0x08;
    
    buf[5] = 0x57;                                                              // crc
    buf[6] = 0x44;
    //RC_CalulateCRC(buf, 5, &buf[5]);
    status = RC_PcdCmd(PCD_TRANSCEIVE, buf, 7, buf, &len);                      // 判断回应的数据是否为UID "....9000"
    if (status == 0)
    {
        memcpy(pUID, buf, 10);
    }
    
    return status;
}

