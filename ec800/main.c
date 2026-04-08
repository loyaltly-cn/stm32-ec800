#include "RTE_Components.h"
#include CMSIS_device_header
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

// ========== 接收缓冲区配置 ==========
#define RX_BUF_SIZE 512
#define RX_TIMEOUT_MS 2000

volatile uint8_t usart2_rx_buf[RX_BUF_SIZE];
volatile uint16_t usart2_rx_head = 0;
volatile uint16_t usart2_rx_tail = 0;
volatile bool usart2_rx_complete = false;

// 延时
void Delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile int j = 0; j < 7200; j++);
    }
}

// ========== USART2 - EC800模块 ==========
void USART2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    
    // PA2 - TX (复用推挽)
    GPIOA->CRL &= ~(0xF << 8);
    GPIOA->CRL |= (0xB << 8);
    // PA3 - RX (浮空输入)
    GPIOA->CRL &= ~(0xF << 12);
    GPIOA->CRL |= (0x4 << 12);
    
    USART2->BRR = 313;  // 115200 @ 36MHz
    
    // 使能接收中断
    USART2->CR1 |= USART_CR1_RXNEIE;
    NVIC->ISER[1] |= (1 << (37 - 32));  // USART2_IRQn = 37
    
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) {
        uint8_t data = USART2->DR;
        uint16_t next_head = (usart2_rx_head + 1) % RX_BUF_SIZE;
        if (next_head != usart2_rx_tail) {
            usart2_rx_buf[usart2_rx_head] = data;
            usart2_rx_head = next_head;
        }
        if (data == '\n') {
            usart2_rx_complete = true;
        }
    }
}

bool USART2_GetByte(uint8_t *data) {
    if (usart2_rx_head == usart2_rx_tail) {
        return false;
    }
    *data = usart2_rx_buf[usart2_rx_tail];
    usart2_rx_tail = (usart2_rx_tail + 1) % RX_BUF_SIZE;
    return true;
}

void USART2_FlushRx(void) {
    usart2_rx_head = 0;
    usart2_rx_tail = 0;
    usart2_rx_complete = false;
}

void USART2_SendByte(uint8_t data) {
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = data;
}

void USART2_SendBytes(const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        USART2_SendByte(data[i]);
    }
}

void USART2_SendString(const char *str) {
    while (*str) USART2_SendByte(*str++);
}

// ========== USART1 - USB-TTL调试打印 (PA9/PA10) ==========
void USART1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // USART1在APB2上
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    
    // PA9 - TX (复用推挽输出)
    GPIOA->CRH &= ~(0xF << 4);   // 清除PA9配置
    GPIOA->CRH |= (0xB << 4);    // 复用推挽，50MHz
    
    // PA10 - RX (浮空输入) - 可选，调试用不到接收
    GPIOA->CRH &= ~(0xF << 8);   // 清除PA10配置
    GPIOA->CRH |= (0x4 << 8);    // 浮空输入
    
    // 115200波特率 @ 72MHz APB2
    // 72MHz / 115200 = 625 = 0x271
    USART1->BRR = 0x271;
    
    // 只使能发送即可
    USART1->CR1 |= USART_CR1_TE | USART_CR1_UE;
}

void Debug_Print(const char *msg) {
    while (*msg) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = *msg++;
    }
}

void Debug_PrintChar(char c) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = c;
}

void Debug_PrintHex(const uint8_t *data, uint16_t len) {
    char buf[8];
    for (uint16_t i = 0; i < len; i++) {
        sprintf(buf, "%02X ", data[i]);
        Debug_Print(buf);
    }
    Debug_Print("\r\n");
}

// ========== EC800 AT指令操作 ==========
void SendAT(const char *cmd) {
    USART2_FlushRx();
    
    Debug_Print("-> ");
    Debug_Print(cmd);
    Debug_Print("\r\n");
    
    USART2_SendString(cmd);
    USART2_SendString("\r\n");
    
    Debug_Print("<- ");
    
    uint32_t timeout = RX_TIMEOUT_MS;
    uint16_t received = 0;
    
    while (timeout > 0) {
        uint8_t ch;
        while (USART2_GetByte(&ch)) {
            Debug_PrintChar(ch);
            received++;
        }
        
        if (usart2_rx_complete) {
            Delay_ms(50);
            if (usart2_rx_head == usart2_rx_tail) {
                break;
            }
            usart2_rx_complete = false;
        }
        
        Delay_ms(1);
        timeout--;
    }
    
    if (received == 0) {
        Debug_Print("(No response - timeout)");
    }
    Debug_Print("\r\n");
}

// ========== EC800操作 ==========
void EC800_Init(void) {
    Debug_Print("=== Testing AT ===\r\n");
    SendAT("AT");
    
    Debug_Print("=== Checking SIM ===\r\n");
    SendAT("AT+CPIN?");
    
    Debug_Print("=== Checking signal ===\r\n");
    SendAT("AT+CSQ");
    
    Debug_Print("=== Checking network ===\r\n");
    SendAT("AT+CREG?");
    
    Debug_Print("=== Check PDP status ===\r\n");
    SendAT("AT+QIACT?");
}

void EC800_ConnectUDP(const char *ip, uint16_t port) {
    char cmd[128];
    
    Debug_Print("=== Activating PDP ===\r\n");
    SendAT("AT+QIACT=1");
    Delay_ms(3000);
    
    SendAT("AT+QIACT?");
    
    sprintf(cmd, "AT+QIOPEN=1,0,\"UDP\",\"%s\",%d,0,0", ip, port);
    Debug_Print("=== Opening UDP ===\r\n");
    SendAT(cmd);
    Delay_ms(3000);
    
    SendAT("AT+QISTATE=0,0");
}

void EC800_SendBinary(const uint8_t *data, uint16_t len) {
    char cmd[32];
    
    sprintf(cmd, "AT+QISEND=0,%d", len);
    Debug_Print("=== Sending binary ===\r\n");
    SendAT(cmd);
    
    Delay_ms(100);
    
    Debug_Print("[Sending raw bytes]\r\n");
    USART2_SendBytes(data, len);
    
    Debug_Print("=== Waiting for SEND OK ===\r\n");
    
    USART2_FlushRx();
    uint32_t timeout = 2000;
    while (timeout > 0) {
        uint8_t ch;
        while (USART2_GetByte(&ch)) {
            Debug_PrintChar(ch);
        }
        Delay_ms(1);
        timeout--;
    }
    Debug_Print("\r\n");
}

void EC800_SendHex(const uint8_t *data, uint16_t len) {
    char cmd[128];
    char hex[4];
    
    strcpy(cmd, "AT+QISENDEX=0,\"");
    for (uint16_t i = 0; i < len; i++) {
        sprintf(hex, "%02X", data[i]);
        strcat(cmd, hex);
    }
    strcat(cmd, "\"");
    
    Debug_Print("=== Sending HEX ===\r\n");
    SendAT(cmd);
}

int main() {
    USART2_Init();   // EC800 - PA2/PA3
    USART1_Init();   // 调试输出 - PA9/PA10 (USB-TTL)
    
    Debug_Print("\r\n========== System Start ==========\r\n");
    
    Delay_ms(3000);
    
    Debug_Print("Initializing EC800...\r\n");
    EC800_Init();
    
    EC800_ConnectUDP("218.93.177.50", 8081);
    
    uint8_t binary_data[] = {0xDD, 0x53, 0x0B, 0xBC, 0xC0, 0xD6, 0xDD, 0xEE, 0xC2, 0xD6};
    Debug_Print("Binary data to send: ");
    Debug_PrintHex(binary_data, sizeof(binary_data));
    
    EC800_SendBinary(binary_data, sizeof(binary_data));
    
    Debug_Print("========== Main Loop ==========\r\n");
    
    for (;;) {
        uint8_t ch;
        while (USART2_GetByte(&ch)) {
            Debug_PrintChar(ch);
        }
        
        Delay_ms(500);
        EC800_SendBinary(binary_data, sizeof(binary_data));
    }
}
