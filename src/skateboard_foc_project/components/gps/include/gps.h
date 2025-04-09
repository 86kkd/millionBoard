#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GPS position and status data
 */
typedef struct {
  float latitude;  // Latitude in decimal degrees
  float longitude; // Longitude in decimal degrees
  float speed_kmh; // Speed in km/h
  float course;    // Course in degrees (0-359)
  bool valid;      // Data validity flag

  // Date and time
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} gps_data_t;

/**
 * @brief GPS date and time structure
 */
typedef struct {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} gps_datetime_t;

/**
 * @brief Initialize GPS module
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t gps_init(void);

/**
 * @brief Get current GPS location and status
 *
 * @param data Pointer to GPS data structure to fill
 * @return esp_err_t ESP_OK on success
 */
esp_err_t gps_get_location(gps_data_t *data);

/**
 * @brief Get current GPS speed
 *
 * @param speed_kmh Pointer to store speed in km/h
 * @return esp_err_t ESP_OK on success
 */
esp_err_t gps_get_speed(float *speed_kmh);

/**
 * @brief Check if GPS data is valid
 *
 * @return bool true if GPS has a valid fix
 */
bool gps_is_valid(void);

/**
 * @brief Get current GPS date and time
 *
 * @param datetime Pointer to datetime structure to fill
 * @return esp_err_t ESP_OK on success
 */
esp_err_t gps_get_datetime(gps_datetime_t *datetime);

#ifdef __cplusplus
}
#endif
