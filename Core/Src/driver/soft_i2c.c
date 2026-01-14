#include "soft_i2c.h"

#define SCL_PIN     SOFT_SCL_Pin
#define SCL_PORT    SOFT_SCL_GPIO_Port
#define SDA_PIN     SOFT_SDA_Pin
#define SDA_PORT    SOFT_SDA_GPIO_Port

//I2C is Open-Drain
// Output 1 (HIGH) = release IO (pull up by resistance) -> SetBits
// Output 0 (LOW) = activate drive IO to Low -> ResetBits

#define SCL_H()     HAL_GPIO_WritePin(SCL_PORT,SCL_PIN,GPIO_PIN_SET)
#define SCL_L()     HAL_GPIO_WritePin(SCL_PORT,SCL_PIN,GPIO_PIN_RESET)
#define SDA_H()     HAL_GPIO_WritePin(SDA_PORT,SDA_PIN,GPIO_PIN_SET)
#define SDA_L()     HAL_GPIO_WritePin(SDA_PORT,SDA_PIN,GPIO_PIN_RESET)
#define SDA_READ() (HAL_GPIO_ReadPin(SDA_PORT, SDA_PIN) == GPIO_PIN_SET)

static void Delay_us(uint32_t us) {
    uint32_t count = us * (SystemCoreClock / 1000000 / 4);
    while (count--) __NOP();
}

void SoftI2C_Init(void) {
    SDA_H();
    SCL_H();

}

// 2. Start Condition (起始訊號)
// 當 SCL 為 High 時，SDA 從 High 拉到 Low
void SoftI2C_Start(void) {
    SDA_H();
    SCL_H();
    Delay_us(I2C_DELAY_US);
    
    SDA_L(); // START: SDA 下降
    Delay_us(I2C_DELAY_US);
    
    SCL_L(); // 把 SCL 拉低，準備開始傳輸數據 (鉗住總線)
}

// 3. Stop Condition (停止訊號)
// 當 SCL 為 High 時，SDA 從 Low 拉到 High
void SoftI2C_Stop(void) {
    SDA_L(); // 先確保 SDA 是 Low
    Delay_us(I2C_DELAY_US);
    
    SCL_H(); // SCL 拉高
    Delay_us(I2C_DELAY_US);
    
    SDA_H(); // STOP: SDA 上升
    Delay_us(I2C_DELAY_US);
}

// 4. 寫入一個 Byte (8 bits) 並等待 ACK
// 回傳值：1 (ACK 成功), 0 (NACK 失敗)
uint8_t SoftI2C_WriteByte(uint8_t data) {
    uint8_t i;
    
    // MSB First (最高位先傳)
    for(i = 0; i < 8; i++) {
        // 準備數據
        if(data & 0x80) {
            SDA_H();
        } else {
            SDA_L();
        }
        Delay_us(I2C_DELAY_US);
        
        // SCL Pulse (High 讀取/鎖存)
        SCL_H(); 
        Delay_us(I2C_DELAY_US);
        SCL_L(); // 拉低準備下一位
        
        data <<= 1; // 移位
    }
    
    // --- 接收 ACK ---
    // 釋放 SDA (切換為輸入模式的概念，但在 OD 模式下只要輸出 High 即可)
    SDA_H(); 
    Delay_us(I2C_DELAY_US);
    
    SCL_H(); // 產生 Clock 讓 Slave 回傳 ACK
    Delay_us(I2C_DELAY_US);
    
    uint8_t ack = !SDA_READ(); // 如果 SDA 被拉低(0)，代表 ACK(1)
    
    SCL_L(); // 結束 ACK cycle
    return ack;
}