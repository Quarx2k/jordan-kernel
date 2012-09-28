
#ifndef __TDA19989_H__
#define __TDA19989_H__

#define TDA19989_CEC_REGULATOR_NAME_SIZE   (32)

struct tda19989_platform_data {
	int  pwr_en_gpio;
	int  int_gpio;
	int  cec_i2c_dev;
	char cec_reg_name[TDA19989_CEC_REGULATOR_NAME_SIZE];
};

#endif /* __TDA19989_H__ */

