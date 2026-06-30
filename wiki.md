Links and Resources

DRV8225 and NEMA17
https://lastminuteengineers.com/drv8825-stepper-motor-driver-arduino-tutorial/
- gives good info on how controls stepping motor
https://www.ampheo.com/blog/stm32-four-precision-control-methods-for-stepper-motors 

TIM
https://wiki.st.com/stm32mcu/wiki/Getting_started_with_TIM#What_is_a_timer_-28TIM-29--
https://www.st.com/resource/en/application_note/an4013-introduction-to-timers-for-stm32-mcus-stmicroelectronics.pdf

speed of flow = about 200ml/min.
PWM
https://madhawapolkotuwa.medium.com/pwm-your-guide-to-timers-frequencies-and-duty-cycles-with-dma-5cb0aa194c36
- clear info on calculations.
- here they make prescaler = tim clock value so the frequency = 1khz, which allows easily scalability by arr
https://deepbluembedded.com/stm32-pwm-example-timer-pwm-mode-tutorial/
- includes examples and code and config stuff.

HAL library
https://www.st.com/resource/en/user_manual/um1884-description-of-stm32l4l4-hal-and-lowlayer-drivers-stmicroelectronics.pdf

STM32 PIN DIAGRAM
https://os.mbed.com/platforms/ST-Nucleo-L432KC/




console stuff
Adding UART2 console view in terminal for Debugging purposes:

Window > perspective > other > Debug

On right side panel click the console option (blue computer)

Click the yellow icon with blue line at top with yellow plus (open console)

Choose Command Shell Prompt

- Connection Type: Serial Port
- Connection name: -> New: 
---Choose a Connection Name, 
---Serial Port: Go into Device Manager and Find the Port Number of the STM named one, Use this.

Then Create.
