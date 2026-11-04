#include "vofa.h"

#include "pid.h"

#define VOFA_BAUD_RATE          115200U
#define VOFA_COMMAND_LENGTH     48U
#define VOFA_TELEMETRY_PERIOD   50U

extern volatile int16_t v_actuall;
extern volatile float v_target;

static volatile char rx_build[VOFA_COMMAND_LENGTH];
static volatile char rx_command[VOFA_COMMAND_LENGTH];
static volatile uint8_t rx_length;
static volatile uint8_t command_ready;

static void VOFA_WriteChar(char value)
{
	while ((USART2->SR & USART_SR_TXE) == 0U) {
	}
	USART2->DR = (uint16_t)(uint8_t)value;
}

static void VOFA_WriteString(const char *text)
{
	while (*text != '\0') {
		VOFA_WriteChar(*text++);
	}
}

static char *VOFA_AppendString(char *destination, const char *text)
{
	while (*text != '\0') {
		*destination++ = *text++;
	}
	return destination;
}

static char *VOFA_AppendUnsigned(char *destination, uint32_t value)
{
	char digits[10];
	uint8_t count = 0U;

	do {
		digits[count++] = (char)('0' + (value % 10U));
		value /= 10U;
	} while (value != 0U);

	while (count != 0U) {
		*destination++ = digits[--count];
	}
	return destination;
}

static char *VOFA_AppendFixed3(char *destination, float value)
{
	int32_t scaled;
	uint32_t magnitude;

	if (value >= 0.0f) {
		scaled = (int32_t)(value * 1000.0f + 0.5f);
	} else {
		scaled = (int32_t)(value * 1000.0f - 0.5f);
	}

	if (scaled < 0) {
		*destination++ = '-';
		magnitude = (uint32_t)(-scaled);
	} else {
		magnitude = (uint32_t)scaled;
	}

	destination = VOFA_AppendUnsigned(destination, magnitude / 1000U);
	*destination++ = '.';
	*destination++ = (char)('0' + ((magnitude / 100U) % 10U));
	*destination++ = (char)('0' + ((magnitude / 10U) % 10U));
	*destination++ = (char)('0' + (magnitude % 10U));
	return destination;
}

static uint8_t VOFA_StringEqual(const char *left, const char *right)
{
	while ((*left != '\0') && (*right != '\0')) {
		char a = *left++;
		char b = *right++;
		if ((a >= 'a') && (a <= 'z')) {
			a = (char)(a - ('a' - 'A'));
		}
		if ((b >= 'a') && (b <= 'z')) {
			b = (char)(b - ('a' - 'A'));
		}
		if (a != b) {
			return 0U;
		}
	}
	return (uint8_t)((*left == '\0') && (*right == '\0'));
}

static uint8_t VOFA_ParseFloat(const char *text, float *result)
{
	float value = 0.0f;
	float fraction = 0.1f;
	uint8_t negative = 0U;
	uint8_t has_digit = 0U;

	while ((*text == ' ') || (*text == '\t')) {
		text++;
	}
	if ((*text == '+') || (*text == '-')) {
		negative = (uint8_t)(*text == '-');
		text++;
	}
	while ((*text >= '0') && (*text <= '9')) {
		value = value * 10.0f + (float)(*text - '0');
		text++;
		has_digit = 1U;
	}
	if (*text == '.') {
		text++;
		while ((*text >= '0') && (*text <= '9')) {
			value += (float)(*text - '0') * fraction;
			fraction *= 0.1f;
			text++;
			has_digit = 1U;
		}
	}
	while ((*text == ' ') || (*text == '\t')) {
		text++;
	}
	if ((has_digit == 0U) || (*text != '\0')) {
		return 0U;
	}
	*result = negative ? -value : value;
	return 1U;
}

static void VOFA_ResetControl(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	PID_Reset(&post_pid);
	PID_Reset(&motor_pid);
	if (primask == 0U) {
		__enable_irq();
	}
}

static void VOFA_SendStatus(const char *prefix)
{
	char message[160];
	char *cursor = message;

	cursor = VOFA_AppendString(cursor, "# ");
	cursor = VOFA_AppendString(cursor, prefix);
	cursor = VOFA_AppendString(cursor, " T=");
	cursor = VOFA_AppendFixed3(cursor, v_target);
	cursor = VOFA_AppendString(cursor, " VP=");
	cursor = VOFA_AppendFixed3(cursor, post_pid.kp);
	cursor = VOFA_AppendString(cursor, " VI=");
	cursor = VOFA_AppendFixed3(cursor, post_pid.ki);
	cursor = VOFA_AppendString(cursor, " VD=");
	cursor = VOFA_AppendFixed3(cursor, post_pid.kd);
	cursor = VOFA_AppendString(cursor, " IP=");
	cursor = VOFA_AppendFixed3(cursor, motor_pid.kp);
	cursor = VOFA_AppendString(cursor, " II=");
	cursor = VOFA_AppendFixed3(cursor, motor_pid.ki);
	cursor = VOFA_AppendString(cursor, " ID=");
	cursor = VOFA_AppendFixed3(cursor, motor_pid.kd);
	*cursor++ = '\r';
	*cursor++ = '\n';
	*cursor = '\0';
	VOFA_WriteString(message);
}

static void VOFA_SendTelemetry(void)
{
	char message[112];
	char *cursor = message;
	float target = v_target;
	float actual = (float)v_actuall;
	float output = (float)motor_pwm;

	cursor = VOFA_AppendString(cursor, "target:");
	cursor = VOFA_AppendFixed3(cursor, target);
	cursor = VOFA_AppendString(cursor, ",actual:");
	cursor = VOFA_AppendFixed3(cursor, actual);
	cursor = VOFA_AppendString(cursor, ",output:");
	cursor = VOFA_AppendFixed3(cursor, output);
	cursor = VOFA_AppendString(cursor, ",error:");
	cursor = VOFA_AppendFixed3(cursor, target - actual);
	*cursor++ = '\r';
	*cursor++ = '\n';
	*cursor = '\0';
	VOFA_WriteString(message);
}

static void VOFA_ApplyCommand(char *command)
{
	char *separator = command;
	float value;
	float *parameter = 0;
	float minimum = -1000.0f;
	float maximum = 1000.0f;

	while ((*separator != '\0') && (*separator != '=')) {
		separator++;
	}
	if (*separator == '\0') {
		if (VOFA_StringEqual(command, "GET") != 0U) {
			VOFA_SendStatus("PID");
			return;
		}
		if (VOFA_StringEqual(command, "RESET") != 0U) {
			VOFA_ResetControl();
			VOFA_SendStatus("OK RESET");
			return;
		}
		if (VOFA_StringEqual(command, "STOP") != 0U) {
			v_target = 0.0f;
			VOFA_ResetControl();
			VOFA_SendStatus("OK STOP");
			return;
		}
		VOFA_WriteString("# ERR use GET, RESET, STOP or KEY=value\r\n");
		return;
	}

	*separator++ = '\0';
	if (VOFA_ParseFloat(separator, &value) == 0U) {
		VOFA_WriteString("# ERR invalid number\r\n");
		return;
	}

	if ((VOFA_StringEqual(command, "T") != 0U) ||
	    (VOFA_StringEqual(command, "TARGET") != 0U)) {
		parameter = (float *)&v_target;
	} else if ((VOFA_StringEqual(command, "P") != 0U) ||
	           (VOFA_StringEqual(command, "VP") != 0U)) {
		parameter = &post_pid.kp;
	} else if ((VOFA_StringEqual(command, "I") != 0U) ||
	           (VOFA_StringEqual(command, "VI") != 0U)) {
		parameter = &post_pid.ki;
	} else if ((VOFA_StringEqual(command, "D") != 0U) ||
	           (VOFA_StringEqual(command, "VD") != 0U)) {
		parameter = &post_pid.kd;
	} else if (VOFA_StringEqual(command, "IP") != 0U) {
		parameter = &motor_pid.kp;
	} else if (VOFA_StringEqual(command, "II") != 0U) {
		parameter = &motor_pid.ki;
	} else if (VOFA_StringEqual(command, "ID") != 0U) {
		parameter = &motor_pid.kd;
	} else {
		VOFA_WriteString("# ERR unknown key\r\n");
		return;
	}

	if ((value < minimum) || (value > maximum)) {
		VOFA_WriteString("# ERR range is -1000..1000\r\n");
		return;
	}
	*parameter = value;
	VOFA_SendStatus("OK");
}

void VOFA_Init(void)
{
	GPIO_InitTypeDef gpio = {0};
	uint32_t peripheral_clock;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();

	gpio.Pin = GPIO_PIN_2;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &gpio);

	gpio.Pin = GPIO_PIN_3;
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &gpio);

	peripheral_clock = HAL_RCC_GetPCLK1Freq();
	USART2->BRR = (peripheral_clock + (VOFA_BAUD_RATE / 2U)) / VOFA_BAUD_RATE;
	USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

	HAL_NVIC_SetPriority(USART2_IRQn, 1U, 0U);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	VOFA_WriteString("# VOFA PID ready, send GET for settings\r\n");
}

void VOFA_Task(void)
{
	static uint32_t last_telemetry;
	char command[VOFA_COMMAND_LENGTH];
	uint8_t index;
	uint32_t now;
	uint32_t primask;

	if (command_ready != 0U) {
		primask = __get_PRIMASK();
		__disable_irq();
		for (index = 0U; index < VOFA_COMMAND_LENGTH; index++) {
			command[index] = rx_command[index];
			if (command[index] == '\0') {
				break;
			}
		}
		command[VOFA_COMMAND_LENGTH - 1U] = '\0';
		command_ready = 0U;
		if (primask == 0U) {
			__enable_irq();
		}
		VOFA_ApplyCommand(command);
	}

	now = HAL_GetTick();
	if ((uint32_t)(now - last_telemetry) >= VOFA_TELEMETRY_PERIOD) {
		last_telemetry = now;
		VOFA_SendTelemetry();
	}
}

void VOFA_USART2_IRQHandler(void)
{
	uint32_t status = USART2->SR;
	char value;
	uint8_t index;

	if ((status & USART_SR_RXNE) != 0U) {
		value = (char)(USART2->DR & 0xFFU);
		if ((value == '\r') || (value == '\n')) {
			if ((rx_length != 0U) && (command_ready == 0U)) {
				for (index = 0U; index < rx_length; index++) {
					rx_command[index] = rx_build[index];
				}
				rx_command[rx_length] = '\0';
				command_ready = 1U;
			}
			rx_length = 0U;
		} else if ((value >= ' ') && (value <= '~')) {
			if (rx_length < (VOFA_COMMAND_LENGTH - 1U)) {
				rx_build[rx_length++] = value;
			} else {
				rx_length = 0U;
			}
		}
	} else if ((status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U) {
		(void)USART2->DR;
	}
}
