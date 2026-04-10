#include "bsp_buzzer.h"
#include "main.h"

#define TIM_CLK 84000000
#define TIM4_ARR 21000

extern TIM_HandleTypeDef htim4;

typedef struct
{
    uint16_t freq;
    uint16_t duration; // 持续时间（ms）
    uint16_t pwm;

} Melody_TypeDef;

static Melody_TypeDef melody_eva[] = {
    {220, 500, 20000}, {261, 500,20000}, {293, 400,20000}, {261, 300,20000},   
     {293, 300,20000},{10, 10,0}, {293, 250,20000},{10, 10,0},{293, 250,20000}, {392,300,20000},
      {349, 300,20000},{330,200,20000},{294, 200,20000},{330,200,20000},{0,0,0}
};

static Melody_TypeDef melody_mao[] = {
{215,11,20000},
{147,11,20000},
{225,11,20000},
{480,69,20000},
{505,58,20000},
{572,11,20000},
{506,11,20000},
{592,81,20000},
{285,92,20000},
{600,23,20000},
{568,34,20000},
{595,174,20000},
{558,11,20000},
{587,185,20000},
{179,46,20000},
{668,23,20000},
{617,58,20000},
{671,104,20000},
{700,46,20000},
{671,11,20000},
{703,46,20000},
{525,162,20000},
{559,23,20000},
{527,23,20000},
{145,34,20000},
{531,116,20000},
{421,23,20000},
{450,92,20000},
{425,34,20000},
{399,23,20000},
{426,104,20000},
{452,267,20000},
{152,69,20000},
};
void buzzer_on(uint16_t freq, uint16_t pwm)
{
    uint16_t psc = (TIM_CLK/(freq*TIM4_ARR))-1;
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

// 通用的蜂鸣器播放函数
void buzzer_play(Melody_TypeDef *melody)
{
    uint8_t i = 0;

    // 遍历旋律数组，直到遇到duration为0的结构体（结束标志）
    while (melody[i].duration != 0)
    {
        buzzer_on(melody[i].freq, melody[i].pwm);
        HAL_Delay(melody[i].duration);
        i++;
    }
    buzzer_off();
}

// 播放EVA旋律
void buzzer_play_eva(void)
{
    buzzer_play(melody_eva);
}

// 播放毛毛完整形态坐牢の小曲
void buzzer_play_mao(void)
{
    buzzer_play(melody_mao);
}