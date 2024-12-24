#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "example_foc";

// 引脚定义
#define MOTOR_PIN_UH 8  // U相 INH
#define MOTOR_PIN_UL 18 // U相 IN
#define MOTOR_PIN_VH 3  // V相 INH
#define MOTOR_PIN_VL 13 // V相 IN
#define MOTOR_PIN_WH 11 // W相 INH
#define MOTOR_PIN_WL 12 // W相 IN

#define EXAMPLE_FOC_MCPWM_TIMER_RESOLUTION_HZ 100000 // 100kHz
#define EXAMPLE_FOC_MCPWM_PERIOD 2000                // 50kHz PWM频率
#define EXAMPLE_FOC_WAVE_FREQ 0.5                    // 0.5Hz旋转频率
#define EXAMPLE_FOC_WAVE_AMPL 0.8                    // 调制深度

void app_main(void) {
  ESP_LOGI(TAG, "BTN7960B FOC Init");

  mcpwm_timer_handle_t timers[3] = {NULL};
  mcpwm_oper_handle_t opers[3] = {NULL};
  mcpwm_cmpr_handle_t comparators[3] = {NULL};
  mcpwm_gen_handle_t generators_h[3] = {NULL};
  mcpwm_gen_handle_t generators_l[3] = {NULL};

  // 定义组和定时器的映射关系
  const int group_timer_map[3][2] = {
      {0, 0}, // 第一相使用组0的定时器0
      {0, 1}, // 第二相使用组0的定时器1
      {1, 0}  // 第三相使用组1的定时器0
  };

  for (int i = 0; i < 3; i++) {
    // 配置定时器
    mcpwm_timer_config_t timer_config = {
        .group_id = group_timer_map[i][0], // 使用映射的组ID
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = EXAMPLE_FOC_MCPWM_TIMER_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
        .period_ticks = EXAMPLE_FOC_MCPWM_PERIOD,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timers[i]));

    // 配置运算单元
    mcpwm_operator_config_t operator_config = {
        .group_id = group_timer_map[i][0], // 使用映射的组ID
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &opers[i]));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(opers[i], timers[i]));

    // 配置比较器
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(
        mcpwm_new_comparator(opers[i], &comparator_config, &comparators[i]));

    // 配置生成器
    int h_pins[3] = {MOTOR_PIN_UH, MOTOR_PIN_VH, MOTOR_PIN_WH};
    int l_pins[3] = {MOTOR_PIN_UL, MOTOR_PIN_VL, MOTOR_PIN_WL};

    // 配置INH信号生成器
    mcpwm_generator_config_t gen_config = {
        .gen_gpio_num = h_pins[i],
    };
    ESP_ERROR_CHECK(
        mcpwm_new_generator(opers[i], &gen_config, &generators_h[i]));

    // 设置INH为高电平有效的PWM输出
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(
        generators_h[i],
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i],
                                       MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN,
                                       comparators[i], MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

    // 配置IN信号生成器
    gen_config.gen_gpio_num = l_pins[i];
    ESP_ERROR_CHECK(
        mcpwm_new_generator(opers[i], &gen_config, &generators_l[i]));

    // 设置IN为反相PWM输出
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(
        generators_l[i],
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i],
                                       MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN,
                                       comparators[i], MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

    // 启动定时器
    ESP_ERROR_CHECK(mcpwm_timer_enable(timers[i]));
    ESP_ERROR_CHECK(
        mcpwm_timer_start_stop(timers[i], MCPWM_TIMER_START_NO_STOP));
  }

  float angle = 0.0f;
  const float angle_step =
      360.0f * EXAMPLE_FOC_WAVE_FREQ /
      (EXAMPLE_FOC_MCPWM_TIMER_RESOLUTION_HZ / EXAMPLE_FOC_MCPWM_PERIOD);

  float ramp_factor = 0.0f;
  const float ramp_step = 0.0005f;

  while (true) {
    angle += angle_step;
    if (angle >= 360.0f) {
      angle -= 360.0f;
    }

    if (ramp_factor < 1.0f) {
      ramp_factor += ramp_step;
      if (ramp_factor > 1.0f)
        ramp_factor = 1.0f;
    }

    float angle_rad = angle * M_PI / 180.0f;
    float current_ampl = EXAMPLE_FOC_WAVE_AMPL * ramp_factor;

    float u = current_ampl * sinf(angle_rad);
    float v = current_ampl * sinf(angle_rad - (2.0f * M_PI / 3.0f));
    float w = current_ampl * sinf(angle_rad + (2.0f * M_PI / 3.0f));

    float duties[3] = {u, v, w};
    for (int i = 0; i < 3; i++) {
      int compare_value = EXAMPLE_FOC_MCPWM_PERIOD / 2 * (1.0f + duties[i]);
      if (compare_value < 0)
        compare_value = 0;
      if (compare_value > EXAMPLE_FOC_MCPWM_PERIOD)
        compare_value = EXAMPLE_FOC_MCPWM_PERIOD;

      ESP_ERROR_CHECK(
          mcpwm_comparator_set_compare_value(comparators[i], compare_value));
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
