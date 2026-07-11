#include <stdio.h>
#include <string.h>
//#include "osal.h"
//#include "oshw.h"
#include "ethercattype.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
//#include "ethercatfoe.h"
#include "ethercatcoe.h"
#include "LCT Eca.h"
#define bool unsigned char
#define PDO_WRITE_PARA_MAX_COUNT 44

typedef struct sdo_para_t
{
    uint16 index; 
    uint8 subindex;
    bool CA;
    int timeout;
}sdo_para;

typedef struct pdo_write_para_t
{
    sdo_para para;
    int32 size;
    char buffer[4];
    bool enable;
}pdo_write_para;

pdo_write_para dm3e_556_pdo_write_para_table[PDO_WRITE_PARA_MAX_COUNT]={
  {{0x1c12,0,1,10},1,{0x00,0x00,0x00,0x00},1},
	{{0x1c12,1,1,10},2,{0x03,0x16,0x00,0x00},1},
	{{0x1c12,2,1,10},2,{0x13,0x16,0x00,0x00},1},
	{{0x1c12,0,1,10},1,{0x02,0x00,0x00,0x00},1},
	
	{{0x1603,0,1,10},1,{0x00,0x00,0x00,0x00},1},
	{{0x1603,1,1,10},4,{0x10,0x00,0x40,0x60},1},
	{{0x1603,2,1,10},4,{0x20,0x00,0x7A,0x60},1},
	{{0x1603,3,1,10},4,{0x10,0x00,0x98,0x60},1},
	{{0x1603,4,1,10},4,{0x20,0x00,0x99,0x60},1},
	{{0x1603,5,1,10},4,{0x20,0x00,0x99,0x60},1},
	{{0x1603,6,1,10},4,{0x20,0x00,0xFE,0x60},1},
	{{0x1603,7,1,10},4,{0x10,0x00,0x60,0x60},1},	
	{{0x1603,0,1,10},1,{0x07,0x00,0x00,0x00},1},
	
	{{0x1613,0,1,10},1,{0x00,0x00,0x00,0x00},1},
	{{0x1613,1,1,10},4,{0x10,0x00,0x40,0x68},1},
	{{0x1613,2,1,10},4,{0x20,0x00,0x7A,0x68},1},
	{{0x1613,3,1,10},4,{0x10,0x00,0x98,0x68},1},
	{{0x1613,4,1,10},4,{0x20,0x00,0x99,0x68},1},
	{{0x1613,5,1,10},4,{0x20,0x00,0x99,0x68},1},
	{{0x1613,6,1,10},4,{0x20,0x00,0xFE,0x68},1},
	{{0x1613,7,1,10},4,{0x10,0x00,0x60,0x68},1},	
	{{0x1613,0,1,10},1,{0x07,0x00,0x00,0x00},1},
	
	
	{{0x1c13,0,1,10},1,{0x00,0x00,0x00,0x00},1},
	{{0x1c13,1,1,10},2,{0x03,0x1A,0x00,0x00},1},
	{{0x1c13,2,1,10},2,{0x13,0x1A,0x00,0x00},1},
	{{0x1c13,0,1,10},1,{0x02,0x00,0x00,0x00},1},
	
	{{0x1A03,0,1,10},1,{0x00,0x00,0x00,0x00},1},
	{{0x1A03,1,1,10},4,{0x10,0x00,0x3F,0x60},1},
	{{0x1A03,2,1,10},4,{0x10,0x00,0x41,0x60},1},
	{{0x1A03,3,1,10},4,{0x20,0x00,0x64,0x60},1},
	{{0x1A03,4,1,10},4,{0x20,0x00,0x6C,0x60},1},
	{{0x1A03,5,1,10},4,{0x20,0x00,0xFD,0x60},1},
	{{0x1A03,6,1,10},4,{0x10,0x00,0x61,0x60},1},
	{{0x1A03,0,1,10},1,{0x06,0x00,0x00,0x00},1},
	
  {{0x1A13,0,1,10},1,{0x00,0x00,0x00,0x00},1},
	{{0x1A13,1,1,10},4,{0x10,0x00,0x3F,0x68},1},
	{{0x1A13,2,1,10},4,{0x10,0x00,0x41,0x68},1},
	{{0x1A13,3,1,10},4,{0x20,0x00,0x64,0x68},1},
	{{0x1A13,4,1,10},4,{0x20,0x00,0x6C,0x68},1},
	{{0x1A13,5,1,10},4,{0x20,0x00,0xFD,0x68},1},
	{{0x1A13,6,1,10},4,{0x10,0x00,0x61,0x68},1},
	{{0x1A13,0,1,10},1,{0x06,0x00,0x00,0x00},1},
	
	{{0x6060,0,1,10},1,{0x08,0x00,0x00,0x00},1},
	{{0x6860,0,1,10},1,{0x08,0x00,0x00,0x00},1},

};
uint32 current_pdo_write_para_count = 0;
uint32 dm3e_556_pdo_count = 0;
typedef __packed struct 
{
	 uint16 ErrorCode;
   uint16 StatusWord;
   int32 CurrentPosition;
   int32 CurrentVelocity;
   uint32 DigitalInputs;
   uint16 CurrentMode;
	 
	 uint16 ErrorCode2;
   uint16 StatusWord2;
   int32 CurrentPosition2;
   int32 CurrentVelocity2;
   uint32 DigitalInputs2;
   uint16 CurrentMode2;
}ecat_dm3e_556_in;


typedef __packed struct 
{
   uint16 ControlWord;
   int32 TargetPos;
	 int16 HomingMethod;
	 uint32 HomingLowSpeed;
	 uint32 HomingHightSpeed;
	 uint32 PhysicalOutputs;
   uint16 ModeOfOperation;
	
	 uint16 ControlWord2;
   int32 TargetPos2;
	 int16 HomingMethod2;
	 uint32 HomingLowSpeed2;
	 uint32 HomingHightSpeed2;
	 uint32 PhysicalOutputs2;
   uint16 ModeOfOperation2;
}ecat_dm3e_556_out;





void ecat_dm3e_556_config_get_pdo_write_para(pdo_write_para** para_table,uint32* table_count)
{
	current_pdo_write_para_count = PDO_WRITE_PARA_MAX_COUNT;
    dm3e_556_pdo_count = sizeof(ecat_dm3e_556_in) + sizeof(ecat_dm3e_556_out);
    *para_table = dm3e_556_pdo_write_para_table;
    *table_count = current_pdo_write_para_count;
}

int ecat_dm3e_556_init(ecx_contextt *context, uint16 slave) 
{
	int wkc;
	int i = 0;
	
    pdo_write_para* pdo_para = NULL;
    uint32 pdo_para_count;

	
	ecat_dm3e_556_config_get_pdo_write_para(&pdo_para, &pdo_para_count);
	for(i = 0;i < pdo_para_count;i++){
	    wkc = ecx_SDOwrite(context, slave, pdo_para[i].para.index, pdo_para[i].para.subindex, FALSE, pdo_para[i].size, pdo_para[i].buffer, EC_TIMEOUTRXM);
       if (wkc > 0){}else{
	       #if printf_cmd
	       printf("++++++++++++++++++++++++++++++++++\n \r");
	       #endif
       }
	   #if printf_cmd
	   printf("0x%x,%d,size:%d,buffer:%02x%02x%02x%02x \t insize: %d ; outsize: %d \n \r", pdo_para[i].para.index, pdo_para[i].para.subindex, pdo_para[i].size,
			(uint8)pdo_para[i].buffer[0], pdo_para[i].buffer[1], pdo_para[i].buffer[2], pdo_para[i].buffer[3],sizeof(ecat_dm3e_556_in),sizeof(ecat_dm3e_556_out));
	   #endif
	}
	return 0;
}
