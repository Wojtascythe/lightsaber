

GPIO_InitTypeDef gpio;
 USART_InitTypeDef uart;
 TIM_TimeBaseInitTypeDef tim;
 TIM_OCInitTypeDef  channel;
 NVIC_InitTypeDef nvic;


 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
 GPIO_StructInit(&gpio);
 gpio.GPIO_Pin = GPIO_Pin_5;
 gpio.GPIO_Speed = GPIO_Speed_2MHz;
 gpio.GPIO_Mode = GPIO_Mode_Out_PP;
 GPIO_Init(GPIOA, &gpio);

 gpio.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
  gpio.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &gpio);

 gpio.GPIO_Pin = GPIO_Pin_2;
 gpio.GPIO_Mode = GPIO_Mode_AF_PP;
 GPIO_Init(GPIOA, &gpio);

 gpio.GPIO_Pin = GPIO_Pin_3;
 gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
 GPIO_Init(GPIOA, &gpio);

  TIM_TimeBaseStructInit(&tim);
  tim.TIM_CounterMode = TIM_CounterMode_Up;
  tim.TIM_Prescaler = 64 - 1;
  tim.TIM_Period = 1000 - 1;
  TIM_TimeBaseInit(TIM4, &tim);

  TIM_OCStructInit(&channel);
  channel.TIM_OCMode = TIM_OCMode_PWM2;
  channel.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OC1Init(TIM4, &channel);
  TIM_OC2Init(TIM4, &channel);
  TIM_OC3Init(TIM4, &channel);

  TIM_Cmd(TIM4, ENABLE);

  nvic.NVIC_IRQChannel = TIM4_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 0;
  nvic.NVIC_IRQChannelSubPriority = 0;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);

  SysTick_Config(SystemCoreClock / 1000);

 USART_StructInit(&uart);
 uart.USART_BaudRate = 57600;
 USART_Init(USART2, &uart);

 USART_Cmd(USART2, ENABLE);
