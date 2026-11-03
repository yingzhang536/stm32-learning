// 20_示波器-采集 —— 综合项目 简易示波器 步骤 4 定稿：按键量程 + 串口导出
// 硬件：光敏分压电路（同 18）：3V3 ──光敏──┬── PA0  10K ── GND
//       OLED（同 08）：VCC→3V3  GND→GND  SCL→PB8  SDA→PB9
//       按键（同 09）：PB14 上拉输入，按下拉低
//       串口 CH340 照旧（PA9=TX PA10=RX 交叉接 + 共地）
// 现象：OLED 行 1 电压读数 + 右侧档位（R1/R2/R3），48 行整屏波形（最右=最新）；串口每 200ms 一行 t/V
//
// 步骤 4 新增：
//   - 短按按键（<500ms）：切换量程，3 档循环 4096/2048/1024（对应 0-3.3V/0-1.65V/0-0.8V）
//     y 映射 row = 47 - v*47/span，v 超量程截断到顶部（示波器 V/div 概念）
//   - 长按（≥500ms）：串口导出当前一屏 128 点 CSV（PC 端可存分析）
//   - 按键 10ms 节拍轮询状态机：按下记时间，松开按时长判定短/长按
//
// 步骤 3 架构保留：TIM3 更新事件 TRGO(MMS=010=0x20) → ADC1 EXTSEL=100+EXTTRIG+DMA
//   → DMA1_CH1 循环搬 adc_buf[128]，CPU 只取帧；环形窗口旋转 (newest+1+k)%128，最右=最新
// 易错点：DMA 先使能再配 ADC 触发；读数 mV = v*3300/4096 与量程无关（量程只改显示增益）

#include "stm32f10x.h"
#include "OLED.h"
#include <stdio.h>

extern void OLED_SetCursor(uint8_t Y, uint8_t X);
extern void OLED_WriteData(uint8_t Data);

volatile uint32_t g_ms = 0;

// ---------- SysTick 毫秒时钟 ----------
void SysTick_Init(void) {
    SysTick->LOAD = SystemCoreClock / 1000 - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = 0x07;
}

// ---------- USART1 ----------
void USART1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    GPIOA->CRH &= ~(0xF << 4 * 1);
    GPIOA->CRH |=  (0xB << 4 * 1);
    GPIOA->CRH &= ~(0xF << 4 * 2);
    GPIOA->CRH |=  (0x4 << 4 * 2);
    USART1->BRR = 0x271;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

int fputc(int ch, FILE *f) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = ch;
    return ch;
}

// ---------- 按键：PB14 上拉输入（按下=0） ----------
#define KEY_PIN (1u << 14)
void Key_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    GPIOB->CRH &= ~(0xF << 24);        // PB14 在 CRH bit24~27
    GPIOB->CRH |=  (0x8 << 24);        // 1000 = 上拉/下拉输入
    GPIOB->ODR |= KEY_PIN;             // 1 = 上拉（平时读 1，按下拉 0）
}

// ---------- 采样链路（步骤 3） ----------
#define FRAME_LEN 128
static uint16_t adc_buf[FRAME_LEN];
static uint16_t snap[FRAME_LEN];

void DMA1_Init(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)adc_buf;
    DMA1_Channel1->CNDTR = FRAME_LEN;
    DMA1_Channel1->CCR = 0x15A1;             // CIRC+MINC+PSIZE16+MSIZE16+PL高+EN
}

void ADC1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->CFGR   |= RCC_CFGR_ADCPRE_DIV6;
    GPIOA->CRL &= ~(0xF << 0);
    ADC1->SQR1 = 0;
    ADC1->SQR3 = 0;
    ADC1->CR2 = ADC_CR2_ADON;
    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL);
    ADC1->CR2 &= ~ADC_CR2_EXTSEL;
    ADC1->CR2 |= 0x80000;                    // EXTSEL=100 → TIM3_TRGO
    ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_EXTTRIG;
}

void TIM3_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 7199;
    TIM3->ARR = 399;
    TIM3->CNT = 0;
    TIM3->EGR |= 1;
    TIM3->CR2 |= 0x20;                       // MMS=010：更新事件 → TRGO
    TIM3->CR1 |= 1;
}

// ---------- 波形显示（步骤 2/3） ----------
static uint8_t wbuf[6][128];
static uint16_t span = 4096;                 // 当前量程满量程 ADC 值（4096/2048/1024）

void Wave_Clear(void) {
    uint8_t p, c;
    for (p = 0; p < 6; p++)
        for (c = 0; c < 128; c++)
            wbuf[p][c] = 0;
}

void Wave_DrawPoint(uint8_t x, uint8_t row) {
    wbuf[row / 8][x] |= (uint8_t)(1u << (row % 8));
}

void Wave_Refresh(void) {
    uint8_t p, c;
    for (p = 0; p < 6; p++) {
        OLED_SetCursor(p + 2, 0);
        for (c = 0; c < 128; c++)
            OLED_WriteData(wbuf[p][c]);
    }
}

// 量程切换：3 档循环（示波器 V/div）
void Range_Next(void) {
    if (span == 4096)      span = 2048;
    else if (span == 2048) span = 1024;
    else                   span = 4096;
}

// 档位显示（行 1 右侧列 10~11——行 2 会撞波形区：OLED 行 2 = 页 2~3，波形区从页 2 开始，刷新即擦掉）
void Range_Show(void) {
    if (span == 4096)      OLED_ShowString(1, 10, "R1");
    else if (span == 2048) OLED_ShowString(1, 10, "R2");
    else                   OLED_ShowString(1, 10, "R3");
}

// 串口导出当前一屏（CSV，PC 端可保存分析）
void ExportFrame(void) {
    uint16_t k;
    for (k = 0; k < FRAME_LEN; k++)
        snap[k] = adc_buf[k];                // 快照（DMA 在写）
    printf("EXPORT t=%lu len=%u span=%u\r\n",
           (unsigned long)g_ms, (unsigned)FRAME_LEN, (unsigned)span);
    for (k = 0; k < FRAME_LEN; k++)
        printf("%u%s", (unsigned)snap[k], (k == FRAME_LEN - 1) ? "\r\n" : ",");
}

// ---------- 主循环：按键 10ms 轮询；显示 200ms ----------
int main(void) {
    uint32_t last_disp = 0;
    uint32_t last_key = 0;
    uint32_t key_press_ms = 0;               // 按下时刻（计时长按）
    uint8_t key_down = 0;                    // 当前是否按住
    uint16_t ndtr, newest, latest, mv;
    uint16_t idx;
    uint8_t k, row;

    USART1_Init();
    SysTick_Init();
    OLED_Init();
    Key_Init();
    DMA1_Init();
    ADC1_Init();
    TIM3_Init();

    OLED_Clear();
    OLED_ShowString(1, 1, "V=");
    OLED_ShowString(1, 7, "mV");
    Range_Show();
    printf("scope step4: short=range long=export, sampling 25Hz\r\n");

    while (1) {
        // 按键轮询 10ms：按下记时刻，松开按时长分短按/长按
        if (g_ms - last_key >= 10) {
            last_key = g_ms;
            if (!(GPIOB->IDR & KEY_PIN)) {          // 按下（低电平）
                if (!key_down) {
                    key_down = 1;
                    key_press_ms = g_ms;            // 记录按下时刻
                }
            } else {                                // 松开
                if (key_down) {
                    key_down = 0;
                    if (g_ms - key_press_ms >= 500) {   // 长按：导出数据
                        ExportFrame();
                    } else {                            // 短按：切换量程
                        Range_Next();
                        Range_Show();
                    }
                }
            }
        }
        // 显示 200ms：环形窗口旋转，最右=最新
        if (g_ms - last_disp >= 200) {
            last_disp = g_ms;
            for (k = 0; k < FRAME_LEN; k++)
                snap[k] = adc_buf[k];
            ndtr = DMA1_Channel1->CNDTR;
            newest = (uint16_t)(255 - ndtr) % FRAME_LEN;
            Wave_Clear();
            for (k = 0; k < FRAME_LEN; k++) {
                idx = (uint16_t)((newest + 1 + k) % FRAME_LEN);
                uint32_t yv = (uint32_t)snap[idx] * 47 / span;   // 按当前量程映射
                if (yv > 47) yv = 47;                            // 超量程截顶
                row = (uint8_t)(47 - yv);
                Wave_DrawPoint(k, row);
            }
            Wave_Refresh();
            latest = snap[newest];
            mv = (uint32_t)latest * 3300 / 4096;                 // 真实电压，与量程无关
            OLED_ShowNum(1, 3, mv, 4);
            printf("t=%lu V=%lumV\r\n", (unsigned long)g_ms, mv);
        }
    }
}
