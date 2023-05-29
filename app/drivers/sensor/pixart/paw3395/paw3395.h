#ifndef ZEPHYR_INCLUDE_PAW3395_H_
#define ZEPHYR_INCLUDE_PAW3395_H_

#include "../pixart.h"

/**
 * @file paw3395.h
 *
 * @brief Header file for the paw3395 driver.
 */

#include <drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

  enum paw3395_run_mode {
    HP_MODE, // high performance mode (the default mode using standard power-up register sets)
    LP_MODE, // low power mode
    OFFICE_MODE, // office mode (reduced to 10 ips, most power-efficient)
    GAME_MODE, // corded game mode (with best performance and highest power consumption)

    RUN_MODE_COUNT // end flag
  };

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_PAW3395_H_ */
