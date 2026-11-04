#include "command.h"

#include "pid.h"
#include "usart.h"

#include <stdlib.h>
#include <string.h>

#define VOFA_RX_BUFFER_SIZE       128U
#define VOFA_COMMAND_SIZE          80U
#define VOFA_TELEMETRY_PERIOD_MS   20U

#define VOFA_KP_MAX               100.0f
#define VOFA_KI_MAX               100.0f
#define VOFA_KD_MAX               100.0f
#define VOFA_TARGET_MAX          3000.0f

static uint8_t rx_buffer[VOFA_RX_BUFFER_SIZE];
static volatile uint16_t rx_read_index;
static volatile uint16_t rx_write_index;

static char command_line[VOFA_COMMAND_SIZE];
static uint16_t command_length;
static uint32_t telemetry_tick;

static uint8_t VOFA_IsSpace(char value)
{
  return (uint8_t)((value == ' ') || (value == '\t'));
}

static char VOFA_ToLower(char value)
{
  if ((value >= 'A') && (value <= 'Z')) {
    return (char)(value + ('a' - 'A'));
  }
  return value;
}

static uint8_t VOFA_KeyEquals(const char *left, const char *right)
{
  while ((*left != '\0') && (*right != '\0')) {
    if (VOFA_ToLower(*left) != VOFA_ToLower(*right)) {
      return 0U;
    }
    left++;
    right++;
  }
  return (uint8_t)((*left == '\0') && (*right == '\0'));
}

static char *VOFA_Trim(char *text)
{
  char *end;

  while (VOFA_IsSpace(*text) != 0U) {
    text++;
  }

  end = text + strlen(text);
  while ((end > text) && (VOFA_IsSpace(*(end - 1)) != 0U)) {
    end--;
  }
  *end = '\0';
  return text;
}

static uint8_t VOFA_ParseFloat(char *text, float min_value,
                               float max_value, float *result)
{
  char *end;
  float value;

  text = VOFA_Trim(text);
  value = strtof(text, &end);
  while (VOFA_IsSpace(*end) != 0U) {
    end++;
  }

  if ((end == text) || (*end != '\0') || (value != value) ||
      (value < min_value) ||
      (value > max_value)) {
    return 0U;
  }

  *result = value;
  return 1U;
}

static void VOFA_ResetControllerState(void)
{
  motor_pid.integral = 0.0f;
  motor_pid.error = motor_pid.target - motor_pid.actual;
  motor_pid.last_error = motor_pid.error;
}

static void VOFA_ApplySingleValue(const char *key, float value)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (VOFA_KeyEquals(key, "kp") != 0U) {
    motor_pid.kp = value;
  } else if (VOFA_KeyEquals(key, "ki") != 0U) {
    motor_pid.ki = value;
  } else if (VOFA_KeyEquals(key, "kd") != 0U) {
    motor_pid.kd = value;
  } else {
    motor_pid.target = value;
  }
  VOFA_ResetControllerState();
  if (primask == 0U) {
    __enable_irq();
  }
}

static uint8_t VOFA_ParsePidPacket(char *text)
{
  float values[4];
  float limits[4] = {VOFA_KP_MAX, VOFA_KI_MAX, VOFA_KD_MAX,
                     VOFA_TARGET_MAX};
  char *item = text;
  char *separator;
  uint8_t index;
  uint32_t primask;

  for (index = 0U; index < 4U; index++) {
    separator = strchr(item, ',');
    if (index < 3U) {
      if (separator == NULL) {
        return 0U;
      }
      *separator = '\0';
    } else if (separator != NULL) {
      return 0U;
    }

    if (VOFA_ParseFloat(item, 0.0f, limits[index], &values[index]) == 0U) {
      return 0U;
    }
    if (separator != NULL) {
      item = separator + 1;
    }
  }

  primask = __get_PRIMASK();
  __disable_irq();
  motor_pid.kp = values[0];
  motor_pid.ki = values[1];
  motor_pid.kd = values[2];
  motor_pid.target = values[3];
  VOFA_ResetControllerState();
  if (primask == 0U) {
    __enable_irq();
  }
  return 1U;
}

static void VOFA_ExecuteCommand(char *line)
{
  char *key;
  char *value_text;
  char *separator;
  float value;

  line = VOFA_Trim(line);
  if (*line == '\0') {
    return;
  }

  if (VOFA_KeyEquals(line, "reset") != 0U) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    VOFA_ResetControllerState();
    if (primask == 0U) {
      __enable_irq();
    }
    return;
  }

  separator = strchr(line, '=');
  if (separator == NULL) {
    separator = strchr(line, ':');
  }
  if (separator == NULL) {
    return;
  }

  *separator = '\0';
  key = VOFA_Trim(line);
  value_text = separator + 1;

  if (VOFA_KeyEquals(key, "pid") != 0U) {
    (void)VOFA_ParsePidPacket(value_text);
    return;
  }

  if (VOFA_KeyEquals(key, "kp") != 0U) {
    if (VOFA_ParseFloat(value_text, 0.0f, VOFA_KP_MAX, &value) != 0U) {
      VOFA_ApplySingleValue(key, value);
    }
  } else if (VOFA_KeyEquals(key, "ki") != 0U) {
    if (VOFA_ParseFloat(value_text, 0.0f, VOFA_KI_MAX, &value) != 0U) {
      VOFA_ApplySingleValue(key, value);
    }
  } else if (VOFA_KeyEquals(key, "kd") != 0U) {
    if (VOFA_ParseFloat(value_text, 0.0f, VOFA_KD_MAX, &value) != 0U) {
      VOFA_ApplySingleValue(key, value);
    }
  } else if ((VOFA_KeyEquals(key, "target") != 0U) ||
             (VOFA_KeyEquals(key, "set") != 0U)) {
    if (VOFA_ParseFloat(value_text, 0.0f, VOFA_TARGET_MAX, &value) != 0U) {
      VOFA_ApplySingleValue("target", value);
    }
  }
}

void VOFA_RxWrite(const uint8_t *data, uint16_t length)
{
  uint16_t index;

  for (index = 0U; index < length; index++) {
    uint16_t next = (uint16_t)((rx_write_index + 1U) % VOFA_RX_BUFFER_SIZE);
    if (next == rx_read_index) {
      break;
    }
    rx_buffer[rx_write_index] = data[index];
    rx_write_index = next;
  }

  /* Receive-to-idle marks one command frame even when VOFA omits CR/LF. */
  if ((length > 0U) && (data[length - 1U] != '\r') &&
      (data[length - 1U] != '\n')) {
    uint16_t next = (uint16_t)((rx_write_index + 1U) % VOFA_RX_BUFFER_SIZE);
    if (next != rx_read_index) {
      rx_buffer[rx_write_index] = '\n';
      rx_write_index = next;
    }
  }
}

void VOFA_Process(void)
{
  while (rx_read_index != rx_write_index) {
    char value = (char)rx_buffer[rx_read_index];
    rx_read_index = (uint16_t)((rx_read_index + 1U) % VOFA_RX_BUFFER_SIZE);

    if ((value == '\r') || (value == '\n')) {
      if (command_length > 0U) {
        command_line[command_length] = '\0';
        VOFA_ExecuteCommand(command_line);
        command_length = 0U;
      }
    } else if (command_length < (VOFA_COMMAND_SIZE - 1U)) {
      command_line[command_length++] = value;
    } else {
      command_length = 0U;
    }
  }
}

static uint16_t VOFA_AppendText(char *buffer, uint16_t position,
                                const char *text)
{
  while (*text != '\0') {
    buffer[position++] = *text++;
  }
  return position;
}

static uint16_t VOFA_AppendFloat(char *buffer, uint16_t position, float value)
{
  uint32_t scaled;
  uint32_t whole;
  uint32_t divisor = 1U;

  if (value < 0.0f) {
    buffer[position++] = '-';
    value = -value;
  }
  if (value > 99999.0f) {
    value = 99999.0f;
  }

  scaled = (uint32_t)(value * 100.0f + 0.5f);
  whole = scaled / 100U;
  while ((whole / divisor) >= 10U) {
    divisor *= 10U;
  }
  do {
    buffer[position++] = (char)('0' + ((whole / divisor) % 10U));
    divisor /= 10U;
  } while (divisor != 0U);

  buffer[position++] = '.';
  buffer[position++] = (char)('0' + ((scaled / 10U) % 10U));
  buffer[position++] = (char)('0' + (scaled % 10U));
  return position;
}

void VOFA_SendTelemetry(void)
{
  static char tx_buffer[128];
  uint32_t now = HAL_GetTick();
  uint16_t length = 0U;
  float target;
  float actual;
  float output;
  float error;
  float kp;
  float ki;
  float kd;
  uint32_t primask;

  if ((uint32_t)(now - telemetry_tick) < VOFA_TELEMETRY_PERIOD_MS) {
    return;
  }
  telemetry_tick = now;

  primask = __get_PRIMASK();
  __disable_irq();
  target = motor_pid.target;
  actual = motor_pid.actual;
  output = motor_pid.out;
  error = motor_pid.error;
  kp = motor_pid.kp;
  ki = motor_pid.ki;
  kd = motor_pid.kd;
  if (primask == 0U) {
    __enable_irq();
  }

  length = VOFA_AppendText(tx_buffer, length, "target:");
  length = VOFA_AppendFloat(tx_buffer, length, target);
  length = VOFA_AppendText(tx_buffer, length, ",actual:");
  length = VOFA_AppendFloat(tx_buffer, length, actual);
  length = VOFA_AppendText(tx_buffer, length, ",out:");
  length = VOFA_AppendFloat(tx_buffer, length, output);
  length = VOFA_AppendText(tx_buffer, length, ",error:");
  length = VOFA_AppendFloat(tx_buffer, length, error);
  length = VOFA_AppendText(tx_buffer, length, ",kp:");
  length = VOFA_AppendFloat(tx_buffer, length, kp);
  length = VOFA_AppendText(tx_buffer, length, ",ki:");
  length = VOFA_AppendFloat(tx_buffer, length, ki);
  length = VOFA_AppendText(tx_buffer, length, ",kd:");
  length = VOFA_AppendFloat(tx_buffer, length, kd);
  length = VOFA_AppendText(tx_buffer, length, "\r\n");

  (void)HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, length, 10U);
}
