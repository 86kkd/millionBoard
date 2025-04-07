#include "motor_control_foc.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "math.h"

static const char *TAG = "MOTOR_CONTROL";

// Motor configuration
#define MOTOR_PWM_FREQ 25000                   // PWM frequency
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_10_BIT // 10-bit resolution (0-1023)
#define MOTOR_PWM_TIMER LEDC_TIMER_0
#define MOTOR_PWM_MODE LEDC_LOW_SPEED_MODE

// Motor GPIO pins (defined in sdkconfig)
#define MOTOR_PWM_U_PIN CONFIG_MOTOR_PWM_U_PIN
#define MOTOR_PWM_V_PIN CONFIG_MOTOR_PWM_V_PIN
#define MOTOR_PWM_W_PIN CONFIG_MOTOR_PWM_W_PIN
#define MOTOR_ENABLE_PIN CONFIG_MOTOR_ENABLE_PIN

// Motor parameters
#define MOTOR_POLE_PAIRS 10        // Number of pole pairs
#define MOTOR_MAX_CURRENT 8.0f     // Max current in Amps
#define MOTOR_SUPPLY_VOLTAGE 36.0f // Supply voltage in Volts

// FOC control state
typedef struct {
  float angle;         // Electrical angle
  float voltage_alpha; // Alpha component of voltage
  float voltage_beta;  // Beta component of voltage
  float duty_a;        // Duty cycle for phase A
  float duty_b;        // Duty cycle for phase B
  float duty_c;        // Duty cycle for phase C
} foc_state_t;

// Motor control state
static struct {
  bool initialized;
  bool enabled;
  float target_speed;  // Target speed (-1.0 to 1.0)
  float current_speed; // Current speed
  motor_direction_t direction;
  foc_state_t foc;
} motor_state = {0};

// Initialize FOC motor control
esp_err_t motor_control_init(void) {
  if (motor_state.initialized) {
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Initializing FOC motor control");

  // Configure LEDC timer
  ledc_timer_config_t ledc_timer = {
      .duty_resolution = MOTOR_PWM_RESOLUTION,
      .freq_hz = MOTOR_PWM_FREQ,
      .speed_mode = MOTOR_PWM_MODE,
      .timer_num = MOTOR_PWM_TIMER,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  // Configure LEDC channels for each motor phase
  ledc_channel_config_t ledc_channel_u = {
      .channel = LEDC_CHANNEL_0,
      .duty = 0,
      .gpio_num = MOTOR_PWM_U_PIN,
      .speed_mode = MOTOR_PWM_MODE,
      .hpoint = 0,
      .timer_sel = MOTOR_PWM_TIMER,
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_u));

  ledc_channel_config_t ledc_channel_v = {
      .channel = LEDC_CHANNEL_1,
      .duty = 0,
      .gpio_num = MOTOR_PWM_V_PIN,
      .speed_mode = MOTOR_PWM_MODE,
      .hpoint = 0,
      .timer_sel = MOTOR_PWM_TIMER,
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_v));

  ledc_channel_config_t ledc_channel_w = {
      .channel = LEDC_CHANNEL_2,
      .duty = 0,
      .gpio_num = MOTOR_PWM_W_PIN,
      .speed_mode = MOTOR_PWM_MODE,
      .hpoint = 0,
      .timer_sel = MOTOR_PWM_TIMER,
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_w));

  // Configure enable pin
  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = (1ULL << MOTOR_ENABLE_PIN),
      .pull_down_en = 0,
      .pull_up_en = 0,
  };
  gpio_config(&io_conf);

  // Initially disable motor
  gpio_set_level(MOTOR_ENABLE_PIN, 0);
  motor_state.enabled = false;

  // Initialize FOC state
  motor_state.foc.angle = 0.0f;
  motor_state.foc.voltage_alpha = 0.0f;
  motor_state.foc.voltage_beta = 0.0f;
  motor_state.foc.duty_a = 0.0f;
  motor_state.foc.duty_b = 0.0f;
  motor_state.foc.duty_c = 0.0f;

  // Initialize motor state
  motor_state.target_speed = 0.0f;
  motor_state.current_speed = 0.0f;
  motor_state.direction = MOTOR_DIR_STOP;

  motor_state.initialized = true;
  ESP_LOGI(TAG, "FOC motor control initialized successfully");

  return ESP_OK;
}

// Set motor speed
esp_err_t motor_control_set_speed(float speed) {
  if (!motor_state.initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  // Clamp speed to valid range
  if (speed > 1.0f)
    speed = 1.0f;
  if (speed < -1.0f)
    speed = -1.0f;

  motor_state.target_speed = speed;

  // Set direction based on speed
  if (speed > 0.05f) {
    motor_state.direction = MOTOR_DIR_FORWARD;
  } else if (speed < -0.05f) {
    motor_state.direction = MOTOR_DIR_BACKWARD;
  } else {
    motor_state.direction = MOTOR_DIR_STOP;
  }

  // Update PWM signals using FOC algorithm
  float angle = motor_state.foc.angle;
  float amplitude = fabsf(speed);

  // Space Vector Modulation (SVM)
  float ua, ub, uc;

  // Generate sine waves for each phase (120 degrees apart)
  ua = amplitude * sinf(angle);
  ub = amplitude * sinf(angle - 2.0f * M_PI / 3.0f);
  uc = amplitude * sinf(angle - 4.0f * M_PI / 3.0f);

  // Convert to PWM duty cycles (0-1)
  float duty_a = (ua + 1.0f) * 0.5f;
  float duty_b = (ub + 1.0f) * 0.5f;
  float duty_c = (uc + 1.0f) * 0.5f;

  // Apply duty cycles
  uint32_t duty_max = (1 << MOTOR_PWM_RESOLUTION) - 1;
  ledc_set_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_0, (uint32_t)(duty_a * duty_max));
  ledc_set_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_1, (uint32_t)(duty_b * duty_max));
  ledc_set_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_2, (uint32_t)(duty_c * duty_max));

  ledc_update_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_0);
  ledc_update_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_1);
  ledc_update_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_2);

  // Store current FOC state
  motor_state.foc.duty_a = duty_a;
  motor_state.foc.duty_b = duty_b;
  motor_state.foc.duty_c = duty_c;

  // Increment angle for next update (simulating motor rotation)
  // In a real implementation, this would be determined by sensor feedback
  motor_state.foc.angle += 0.1f * motor_state.target_speed;
  if (motor_state.foc.angle > 2.0f * M_PI) {
    motor_state.foc.angle -= 2.0f * M_PI;
  } else if (motor_state.foc.angle < 0.0f) {
    motor_state.foc.angle += 2.0f * M_PI;
  }

  ESP_LOGD(TAG, "Motor speed set to %.2f, direction %d", speed,
           motor_state.direction);

  return ESP_OK;
}

// Get motor status
esp_err_t motor_control_get_status(motor_status_t *status) {
  if (!motor_state.initialized || status == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  status->current_speed = motor_state.current_speed;
  status->target_speed = motor_state.target_speed;
  status->motor_current =
      0.0f; // Would be read from current sensor in real implementation
  status->motor_temp =
      25.0f; // Would be read from temperature sensor in real implementation
  status->direction = motor_state.direction;

  return ESP_OK;
}

// Enable motor
esp_err_t motor_control_enable(void) {
  if (!motor_state.initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI(TAG, "Enabling motor");
  gpio_set_level(MOTOR_ENABLE_PIN, 1);
  motor_state.enabled = true;

  return ESP_OK;
}

// Disable motor
esp_err_t motor_control_disable(void) {
  if (!motor_state.initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI(TAG, "Disabling motor");
  gpio_set_level(MOTOR_ENABLE_PIN, 0);
  motor_state.enabled = false;

  // Set all PWM channels to zero
  ledc_set_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_0, 0);
  ledc_set_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_1, 0);
  ledc_set_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_2, 0);
  ledc_update_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_0);
  ledc_update_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_1);
  ledc_update_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_2);

  return ESP_OK;
}

// Calculate incline compensation
float calculate_incline_compensation(float angle) {
  // Simple linear compensation based on angle
  // Positive angle = uphill, negative angle = downhill

  // Limit compensation to reasonable values
  const float max_compensation = 0.5f; // Max 50% compensation

  // Apply compensation based on angle
  // 10 degrees of incline results in ~0.17 compensation value
  float compensation = sinf(angle * M_PI / 180.0f) * max_compensation;

  ESP_LOGD(TAG, "Incline compensation: %.2f for angle %.1f°", compensation,
           angle);

  return compensation;
}
