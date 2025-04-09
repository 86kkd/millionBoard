#include "gps.h"
#include "esp_log.h"
#include "math.h"
#include "string.h"
#include "uart_comm.h"

static const char *TAG = "GPS";

// NMEA parser state
static gps_data_t gps_data = {0};
static bool is_initialized = false;

// Helper function to parse decimal degrees from NMEA format
static float parse_degrees(const char *value, char direction) {
  if (value == NULL || strlen(value) < 3) {
    return 0.0f;
  }

  // Extract degrees part (first 2 or 3 digits)
  int deg_len =
      (value[0] == '0' || direction == 'N' || direction == 'S') ? 2 : 3;

  char deg_str[4] = {0};
  strncpy(deg_str, value, deg_len);

  // Extract minutes part
  float minutes = atof(value + deg_len);

  // Calculate decimal degrees
  float decimal_degrees = atof(deg_str) + minutes / 60.0f;

  // Handle negatives for South and West
  if (direction == 'S' || direction == 'W') {
    decimal_degrees = -decimal_degrees;
  }

  return decimal_degrees;
}

// Parse NMEA sentence
static void parse_nmea_sentence(const char *sentence) {
  if (sentence == NULL || strlen(sentence) < 10) {
    return;
  }

  // Verify checksum (ignored for simplicity here)

  // Parse different sentence types
  if (strncmp(sentence, "$GPRMC", 6) == 0 ||
      strncmp(sentence, "$GNRMC", 6) == 0) {
    // RMC sentence (Recommended Minimum data)
    char temp[15] = {0};
    char field[20][15] = {0};
    int field_idx = 0;

    // Split the sentence into fields
    size_t len = strlen(sentence);
    int temp_idx = 0;

    for (size_t i = 7; i < len && field_idx < 20; i++) {
      if (sentence[i] == ',' || sentence[i] == '*') {
        temp[temp_idx] = '\0';
        strcpy(field[field_idx++], temp);
        temp_idx = 0;
        memset(temp, 0, sizeof(temp));
      } else {
        temp[temp_idx++] = sentence[i];
      }
    }

    // Process fields
    if (field_idx >= 10) {
      // Status (field[1]): A=active, V=void
      gps_data.valid = (field[1][0] == 'A');

      if (gps_data.valid) {
        // Time (field[0])
        if (strlen(field[0]) >= 6) {
          int hour = (field[0][0] - '0') * 10 + (field[0][1] - '0');
          int minute = (field[0][2] - '0') * 10 + (field[0][3] - '0');
          int second = (field[0][4] - '0') * 10 + (field[0][5] - '0');

          gps_data.hour = hour;
          gps_data.minute = minute;
          gps_data.second = second;
        }

        // Latitude (field[2], field[3])
        if (strlen(field[2]) > 0 && strlen(field[3]) > 0) {
          gps_data.latitude = parse_degrees(field[2], field[3][0]);
        }

        // Longitude (field[4], field[5])
        if (strlen(field[4]) > 0 && strlen(field[5]) > 0) {
          gps_data.longitude = parse_degrees(field[4], field[5][0]);
        }

        // Speed (field[6])
        if (strlen(field[6]) > 0) {
          // Convert knots to km/h
          gps_data.speed_kmh = atof(field[6]) * 1.852f;
        }

        // Course (field[7])
        if (strlen(field[7]) > 0) {
          gps_data.course = atof(field[7]);
        }

        // Date (field[8])
        if (strlen(field[8]) >= 6) {
          int day = (field[8][0] - '0') * 10 + (field[8][1] - '0');
          int month = (field[8][2] - '0') * 10 + (field[8][3] - '0');
          int year = 2000 + (field[8][4] - '0') * 10 + (field[8][5] - '0');

          gps_data.day = day;
          gps_data.month = month;
          gps_data.year = year;
        }

        ESP_LOGI(TAG, "GPS: %.6f,%.6f %.1fkm/h Course:%.1f°", gps_data.latitude,
                 gps_data.longitude, gps_data.speed_kmh, gps_data.course);
      } else {
        ESP_LOGD(TAG, "GPS data invalid");
      }
    }
  }
  // Additional sentence types could be parsed here (GGA, GSA, etc.)
}

esp_err_t gps_init(void) {
  if (is_initialized) {
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Initializing GPS module");

  // Initialize UART communication
  uart_config_t uart_config = {
      .baud_rate = CONFIG_GPS_UART_BAUD,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 0,
      .source_clk = UART_SCLK_APB,
  };

  if (uart_comm_init(CONFIG_GPS_UART_PORT, &uart_config,
                     CONFIG_GPS_UART_TX_GPIO,
                     CONFIG_GPS_UART_RX_GPIO) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize UART for GPS");
    return ESP_FAIL;
  }

  // Set default values
  gps_data.valid = false;
  gps_data.latitude = 0.0f;
  gps_data.longitude = 0.0f;
  gps_data.speed_kmh = 0.0f;
  gps_data.course = 0.0f;

  is_initialized = true;
  ESP_LOGI(TAG, "GPS initialized successfully");

  return ESP_OK;
}

esp_err_t gps_get_location(gps_data_t *data) {
  if (!is_initialized || data == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  // Read NMEA sentences
  char nmea_buffer[128] = {0};
  if (uart_comm_read_line(CONFIG_GPS_UART_PORT, nmea_buffer,
                          sizeof(nmea_buffer), 500) == ESP_OK) {
    parse_nmea_sentence(nmea_buffer);
  }

  // Copy the current GPS data
  memcpy(data, &gps_data, sizeof(gps_data_t));

  return ESP_OK;
}

esp_err_t gps_get_speed(float *speed_kmh) {
  if (!is_initialized || speed_kmh == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  *speed_kmh = gps_data.speed_kmh;
  return ESP_OK;
}

bool gps_is_valid(void) { return is_initialized && gps_data.valid; }

esp_err_t gps_get_datetime(gps_datetime_t *datetime) {
  if (!is_initialized || !gps_data.valid || datetime == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  datetime->year = gps_data.year;
  datetime->month = gps_data.month;
  datetime->day = gps_data.day;
  datetime->hour = gps_data.hour;
  datetime->minute = gps_data.minute;
  datetime->second = gps_data.second;

  return ESP_OK;
}