#include "Serial.h"
#include <string.h>
#include <stdarg.h>

char Serial_RxPacket[SERIAL_BUF_SIZE];
uint8_t Serial_RxFlag = 0;

static QueueHandle_t uart_queue;

void Serial_Init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = SERIAL_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(SERIAL_UART_NUM, SERIAL_BUF_SIZE * 2, SERIAL_BUF_SIZE * 2, 10, &uart_queue, 0);
    uart_param_config(SERIAL_UART_NUM, &uart_config);
    uart_set_pin(SERIAL_UART_NUM, SERIAL_TXD_PIN, SERIAL_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // 启动串口接收任务
    xTaskCreate(Serial_Task, "serial_task", 2048, NULL, 10, NULL);
}

void Serial_SendByte(uint8_t byte)
{
    uart_write_bytes(SERIAL_UART_NUM, (const char *)&byte, 1);
}

void Serial_SendArray(uint8_t *array, uint16_t length)
{
    uart_write_bytes(SERIAL_UART_NUM, (const char *)array, length);
}

void Serial_SendString(const char *str)
{
    uart_write_bytes(SERIAL_UART_NUM, str, strlen(str));
}

static uint32_t Serial_Pow(uint32_t x, uint32_t y)
{
    uint32_t result = 1;
    while (y--) result *= x;
    return result;
}

void Serial_SendNumber(uint32_t num, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++)
    {
        uint8_t digit = (num / Serial_Pow(10, length - i - 1)) % 10;
        Serial_SendByte(digit + '0');
    }
}

void Serial_Printf(const char *format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    Serial_SendString(buffer);
}

/**
 * 串口接收任务（类似 STM32 的 HAL_UART_RxCpltCallback）
 * 接收格式：@开头，\r\n结尾
 */
void Serial_Task(void *arg)
{
    uint8_t data;
    int idx = 0;
    uint8_t state = 0; // 0: 等待@，1: 记录数据，2: 等待\r\n

    while (1)
    {
        int len = uart_read_bytes(SERIAL_UART_NUM, &data, 1, pdMS_TO_TICKS(20));
        if (len > 0)
        {
            if (state == 0)
            {
                if (data == '@' && Serial_RxFlag == 0)
                {
                    state = 1;
                    idx = 0;
                }
            }
            else if (state == 1)
            {
                if (data == '\r')
                {
                    state = 2;
                }
                else
                {
                    if (idx < SERIAL_BUF_SIZE - 1)
                        Serial_RxPacket[idx++] = data;
                }
            }
            else if (state == 2)
            {
                if (data == '\n')
                {
                    Serial_RxPacket[idx] = '\0';
                    Serial_RxFlag = 1;
                    state = 0;
                }
            }
        }
    }
}
