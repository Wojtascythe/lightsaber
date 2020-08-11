

send_string("Wybierz kolor miecza\r\n");
 while (1) {
 if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE)) {
     char c = USART_ReceiveData(USART2);

     send_string("Czerwony -> r\r\n");
     send_string("Niebieski -> b\r\n");
     send_string("Zielony -> g\r\n");
     switch (c)
     {
         case 'g':
             send_string("Wybrales zielony!\r\n");

             TIM_SetCompare1(TIM4, 0.0f);
             TIM_SetCompare2(TIM4, 0.0f);
        	 TIM_SetCompare3(TIM4, 50.0f);
             break;
         case 'b':
             send_string("Wybrales niebieski!\r\n");

             TIM_SetCompare1(TIM4, 0.0f);
             TIM_SetCompare2(TIM4, 50.0f);
        	 TIM_SetCompare3(TIM4, 0.0f);
             break;
         case 'r':
             send_string("Wybrales czerwony\r\n");

             TIM_SetCompare1(TIM4, 50.0f);
             TIM_SetCompare2(TIM4, 0.0f);
        	 TIM_SetCompare3(TIM4, 0.0f);
             break;
         default:
        	 send_string("Zly wybor, sprobuj jeszcze raz\r\n");
        	 break;
     }
 }
 }
}
