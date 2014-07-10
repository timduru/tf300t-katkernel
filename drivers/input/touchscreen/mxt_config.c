#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/i2c/atmel_maxtouch.h>

#include "atmel_firmware.h"

int init_touch_config_v2(struct mxt_data *mxt, u8 touch_vendor_id){
       int i, object_size;
       u8 version = mxt->device_info.major << 4 | mxt->device_info.minor;
       u16 address;
       struct mxt_object *object;
       printk("Touch: init register in function:%s, vendor=%2.2X version=0x%2.2X\n", __func__, touch_vendor_id, version);
	  
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt), 0x0F);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt)+1, 0xFF);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt)+2, 0x0A);

      switch(touch_vendor_id){
      case HANNSTOUCH_SINTEK_PANEL:
      case TPK_50_OHM_PANEL:
          mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt), 0x41);
          break;
      case TPK_75_OHM_PANEL:
          mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt), 0x4B);
      }	  	
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+1, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+2, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+3, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+4, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+5, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+6, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+7, 0x01);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+8, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+9, 0);

	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt), 0x8F);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+1, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+2, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+3, 0x18);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+4, 0x20);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+5, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+6, 0x90);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+7, 0x3C);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+8, 0x02);
	if(touch_vendor_id == HANNSTOUCH_SINTEK_PANEL)
	    mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+9, 0x03);
	else
	    mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+9, 0x07);    	
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+10, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+11, 0x14);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+12, 0x03);//3
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+13, 0x1F);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+14, 0x0A);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+15, 0x0A);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+16, 0x14);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+17, 0x05);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+18, 0xE7);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+19, 0x03);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+20, 0x3F);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+21, 0x06);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+22, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+23, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+24, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+25, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+26, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+27, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+28, 0x40);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+29, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+30, 0x64);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+31, 0x0F);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+32, 0x39);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+33, 0x45);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+34, 0);
	for(i = 0; i < 35; i++) // set TOUCH_MULTITOUCHSCREEN_T9 INSTANCE 1 all zero
	    mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+35+i, 0);
		
      object = get_object(MXT_TOUCH_KEYARRAY_T15, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);

	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18, mxt),0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18, mxt)+1,0);

      object = get_object(MXT_SPT_GPIOPWM_T19, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);
	  
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt), 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+1, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+2, 0xC8);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+3, 0x32);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+4, 0x88);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+5, 0x03);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+6, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+7, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+8, 0);

      object = get_object(MXT_USER_INFO_T38, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);
	  
      object = get_object(MXT_PROCI_GRIPSUPPRESSION_T40, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);

      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt), 0x31);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+1, 0x3C);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+2, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+3, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+4, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+5, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+6, 0x0A);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+7, 0);
	  mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+8, 0x03);
	  mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+9, 0);
	for(i = 0; i < 10; i++) // set instance 1 
	    mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+10+i, 0);
	  
      object = get_object(MXT_SPT_DIGITIZER_T43, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);

      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt), 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+1, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+2, 0x10);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+3, 0x10);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+4, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+5, 0);
      switch(touch_vendor_id){
      case HANNSTOUCH_SINTEK_PANEL:
          mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+6, 0x01);
          break;
      case TPK_75_OHM_PANEL:
      case TPK_50_OHM_PANEL:
          mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+6, 0);
      }	  		    	
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+7, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+8, 0);

      object = get_object(MXT_PROCI_STYLUS_T47, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);

      object = get_object(MXT_PROCG_NOISESUPPRESSION_T48, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);
	  
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt), 0x07);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 1, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 2, 0x42);
      switch(touch_vendor_id){
      case HANNSTOUCH_SINTEK_PANEL:
          mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 3, 0);
          break;
      case TPK_50_OHM_PANEL:
          mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 3, 0x0C);
          break;
      case TPK_75_OHM_PANEL:
          mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 3, 0x19);
      }	  	
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 8, 0x0A);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 9, 0x14);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 10, 0x90);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 11, 0x28);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 13, 0x06);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 14, 0x06);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 17, 0x64);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 18, 0x14);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 19, 0x3F);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 20, 0x0A);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 21, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 22, 0x14);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 23, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 24, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 25, 0x1E);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 26, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 27, 0x19);

	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 34, 0x60);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 35, 0x3C);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 36, 0x02);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 37, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 38, 0x03);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 39, 0x1F);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 40, 0x0A);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 41, 0x0A);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 42, 0x14);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 43, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 44, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 46, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 47, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 48, 0);	
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 49, 0x40);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 50, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 51, 0x64);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 52, 0x0F);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 53, 0);
	
      object = get_object(MXT_TOUCH_PROXKEY_T52, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);
	
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_DATASOURCE_T53, mxt), 0x01);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_DATASOURCE_T53, mxt)+1, 0x20);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_DATASOURCE_T53, mxt)+2, 0x01);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_DATASOURCE_T53, mxt)+3, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_DATASOURCE_T53, mxt)+4, 0x02);


      object = get_object(MXT_PROCI_ADAPTIVETHRESHOLD_T55, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);

      object = get_object(MXT_PROCI_SHIELDLESS_T56, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);
	  
      object = get_object(MXT_PROCI_EXTRATOUCHAREADATA_T57, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);
	  
	mxt_write_byte(mxt->client,MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, mxt) + MXT_ADR_T6_CALIBRATE, 1);
	return 0;
}

int init_touch_config(struct mxt_data *mxt, u8 touch_vendor_id)
{
       int i, object_size;
       u8 version = mxt->device_info.major << 4 | mxt->device_info.minor;
       u16 address;
       struct mxt_object *object;   
	   
	printk("Touch: init register in function:%s, vendor=%2.2X version=0x%2.2X\n", __func__, touch_vendor_id, version);
      if(version & 0x20)
          return init_touch_config_v2(mxt, touch_vendor_id);
	  
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt), 0x0F);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt)+1, 0xFF);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt)+2, 0x0A);

      switch(touch_vendor_id){
      case HANNSTOUCH_SINTEK_PANEL:
          mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt), 0x41);
          break;
      case TPK_75_OHM_PANEL:
      case TPK_50_OHM_PANEL:
          mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt), 0x4B);
      }	  	
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+1, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+2, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+3, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+4, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+5, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+6, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+7, 0x01);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+8, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+9, 0);

	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt), 0x8F);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+1, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+2, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+3, 0x18);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+4, 0x20);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+5, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+6, 0x90);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+7, 0x3C);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+8, 0x02);
	if(touch_vendor_id == HANNSTOUCH_SINTEK_PANEL)
	    mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+9, 0x03);
	else
	    mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+9, 0x07);    	
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+10, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+11, 0x00);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+12, 0x03);//3
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+13, 0x1F);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+14, 0x0A);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+15, 0x0A);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+16, 0x14);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+17, 0x05);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+18, 0xE7);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+19, 0x03);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+20, 0x3F);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+21, 0x06);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+22, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+23, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+24, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+25, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+26, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+27, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+28, 0x40);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+29, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+30, 0x64);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+31, 0x0F);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+32, 0x39);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+33, 0x45);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+34, 0);
	for(i = 0; i < 35; i++) // set TOUCH_MULTITOUCHSCREEN_T9 INSTANCE 1 all zero
	    mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+35+i, 0);
		
      object = get_object(MXT_TOUCH_KEYARRAY_T15, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);

	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18, mxt),0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18, mxt)+1,0);

      object = get_object(MXT_SPT_GPIOPWM_T19, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);
	  
      object = get_object(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);

	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt), 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+1, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+2, 0xC8);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+3, 0x32);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+4, 0x88);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+5, 0x03);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+6, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+7, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+8, 0);

      object = get_object(MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);

      object = get_object(MXT_USER_INFO_T38, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);
	  
      object = get_object(MXT_PROCI_GRIPSUPPRESSION_T40, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);

      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt), 0x31);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+1, 0x3C);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+2, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+3, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+4, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+5, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+6, 0x0A);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+7, 0);
	  mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+8, 0x03);
	  mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+9, 0);
	for(i = 0; i < 10; i++) // set instance 1 
	    mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+10+i, 0);
	  
      object = get_object(MXT_SPT_DIGITIZER_T43, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);

      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt), 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+1, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+2, 0x10);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+3, 0x10);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+4, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+5, 0);
      switch(touch_vendor_id){
      case HANNSTOUCH_SINTEK_PANEL:
          mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+6, 0x01);
          break;
      case TPK_75_OHM_PANEL:
      case TPK_50_OHM_PANEL:
          mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+6, 0);
      }	  		    	
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+7, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+8, 0);

      object = get_object(MXT_PROCI_STYLUS_T47, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);

      object = get_object(MXT_PROCG_NOISESUPPRESSION_T48, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);
	  
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt), 0x03);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 1, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 2, 0x40);
      switch(touch_vendor_id){
      case HANNSTOUCH_SINTEK_PANEL:
          mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 3, 0);
          break;
      case TPK_75_OHM_PANEL:
      case TPK_50_OHM_PANEL:
          mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 3, 0x19);
      }	  	
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 8, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 9, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 13, 0x06);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 14, 0x06);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 17, 0x64);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 18, 0x14);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 19, 0x3F);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 20, 0x0A);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 21, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 22, 0x14);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 23, 0x06);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 24, 0x00);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 25, 0x2E);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 26, 0x00);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 27, 0);
	
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 34, 0x60);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 35, 0x3C);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 36, 0x02);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 37, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 38, 0x03);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 39, 0x1F);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 40, 0x0A);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 41, 0x0A);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 42, 0x14);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 43, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 44, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 46, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 47, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 48, 0);	
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 49, 0x40);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 50, 0);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 51, 0x64);
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 52, 0x0F);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, mxt) + 53, 0);
	  
      object = get_object(MXT_TOUCH_PROXKEY_T52, mxt);
      address = object->chip_addr;
      object_size = object->size * object->instances;
      for(i = 0; i < object_size; i++)
          mxt_write_byte(mxt->client, address + i, 0);
	
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_DATASOURCE_T53, mxt), 0x01);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_DATASOURCE_T53, mxt)+1, 0x20);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_DATASOURCE_T53, mxt)+2, 0x01);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_DATASOURCE_T53, mxt)+3, 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_DATASOURCE_T53, mxt)+4, 0x02);

	mxt_write_byte(mxt->client,MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, mxt) + MXT_ADR_T6_CALIBRATE, 1);
	return 0;
}


static int mxt_unlock_bootloader(struct i2c_client *client)
{
	u8 buf[2];

	buf[0] = MXT_UNLOCK_CMD_LSB;
	buf[1] = MXT_UNLOCK_CMD_MSB;

	if (i2c_master_send(client, buf, 2) != 2) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt_check_boot_status(struct i2c_client *client, u8 state){
      u8 val;
	  
      msleep(20);
recheck:
	if (i2c_master_recv(client, &val, 1) != 1) {
		dev_err(&client->dev, "%s: i2c recv failed\n", __func__);
		return -EIO;
	}

	switch (state) {
	case MXT_WAITING_BOOTLOAD_COMMAND:
	case MXT_WAITING_FRAME_DATA:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_FRAME_CRC_PASS:
		if (val == MXT_FRAME_CRC_CHECK)
			goto recheck;
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		dev_err(&client->dev, "Unvalid bootloader mode state\n");
		return -EINVAL;
	}

	return 0;
}

int mxt_update_firmware(struct mxt_data *mxt, bool inBoot)
{
    u8 firmware_id[MXT_ID_BLOCK_SIZE];
    int ret, retry = 0;
    unsigned char *firmware_data;
    unsigned int pos = 0;
    unsigned int frame_size = 0, firmware_size;
    struct i2c_client *client;

    client = mxt->client;
    firmware_data = mXT768E_2_0_firmware;
    firmware_size = sizeof(mXT768E_2_0_firmware);
    if(inBoot) goto mxt_boot_start; // if already in boot loader mode
    	
    /*checkout firmware version and don't update if the version is equal to firmware ID in header file */
    mxt_identify(mxt->client, mxt, firmware_id);
    if(firmware_id[2] == MXT768_FIRMWARE_ID){
        dev_info(&client->dev, "Firware is the latest version %d.%d. No need to update!\n", 
			mxt->device_info.major, mxt->device_info.major);
	  return FW_VERSION_HAS_UPDATED; 
    }
    /* start get into boot loader mode*/
    mxt_write_byte(client, MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, mxt) + MXT_ADR_T6_RESET,
                            0xA5);
    msleep(250);
mxt_boot_start:
    if (client->addr == MXT_I2C_ADDRESS) {
		dev_info(&client->dev,"Change I2C address from 0x%02X to 0x%02X\n",
			MXT_I2C_ADDRESS, MXT_BL_ADDRESS);
		client->addr = MXT_BL_ADDRESS;  
    }
    
    ret = mxt_check_boot_status(client,  MXT_WAITING_BOOTLOAD_COMMAND);
    if(ret < 0){
        dev_info(&client->dev, "Error to get into state MXT_WAITING_BOOTLOAD_COMMAND\n");
        goto mxt_boot_exit;
    }
    /*unlock the bootloader*/
    ret = mxt_unlock_bootloader(client);
    while(pos < firmware_size){
        retry = 10;
        ret = mxt_check_boot_status(client, MXT_WAITING_FRAME_DATA);
        if (ret < 0){
            dev_err(&client->dev, "Error to get into state: MXT_WAITING_FRAME_DATA\n");
            goto mxt_boot_exit;
        }
			
        frame_size = ((*(firmware_data + pos) << 8) | *(firmware_data + pos + 1));
        /* We should add 2 at frame size as the the firmware data is not
        * included the CRC bytes.*/
        frame_size += 2;
mxt_send_boot_frame:
        /* Write one frame to device */
        i2c_master_send(client, (unsigned char *)(firmware_data + pos), frame_size);
        ret = mxt_check_boot_status(client, MXT_FRAME_CRC_PASS);
        if (ret < 0){
            if(retry-- > 0){
                dev_info(&client->dev, "#########Try send frame again!\n ");
                goto mxt_send_boot_frame;
            }
            goto mxt_boot_exit;
	  }
			
         pos += frame_size;
         dev_info(&client->dev, "Updated %d bytes / %zd bytes\n", pos, firmware_size);		
    }
    msleep(1000);
mxt_boot_exit: 
    if (client->addr == MXT_BL_ADDRESS) {
        dev_info(&client->dev,"Change I2C address from 0x%02X to 0x%02X\n", MXT_BL_ADDRESS, MXT_I2C_ADDRESS);
        client->addr = MXT_I2C_ADDRESS;  
    }  
    dev_info(&client->dev, "QT_Boot end\n");
    return 0;  
}


