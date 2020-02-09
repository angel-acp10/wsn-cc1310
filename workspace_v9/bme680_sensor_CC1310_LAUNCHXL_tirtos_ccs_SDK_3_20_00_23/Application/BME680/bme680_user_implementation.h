#ifndef BME680_USER_IMPLMENTATION_H_
#define BME680_USER_IMPLMENTATION_H_

#include "BME680/bme680.h"

void BME680_init();
void BME680_measure(struct bme680_field_data *data);

#endif
