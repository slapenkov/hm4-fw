//////////////////////////////////////////////////////////////////////////////////////////
////                                      modbus.c                                    ////
////				modified for STM32												  ////
////                                                                                  ////
////                 MODBUS protocol driver for serial communications.                ////
////                                                                                  ////
////  Refer to documentation at http://www.modbus.org for more information on MODBUS. ////

#include "modbus.h"
//#include "stm32f1xx_hal_def.h"


/*
 * useful routines
 */
int8_t make8(int16_t var, int8_t offset){
	//
	return (int8_t)((var >> (offset * 8)) & 0xff);
}

int16_t make16(int8_t varhigh, int8_t varlow){
	//
	return (int16_t)((varhigh&0xff)*0x100+(varlow&0xff));
}

//USART send data with wait
void Transmitt(UART_HandleTypeDef *huart, uint16_t Data)
{
  /* Transmit Data */
  huart->Instance->DR = (Data & (uint16_t)0x01FF);

  /*wait for transmission complete*/
  while(__HAL_UART_GET_FLAG(huart, UART_FLAG_TC));

  __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TC);
}


//uart receive data from DR
uint16_t Receive(UART_HandleTypeDef *huart){

	return (uint16_t)(huart->Instance->DR & (uint16_t)0x01FF);
}

//delay_us

/*
 * TODO Check this function for using
 * */
void delay_us(uint32_t us)
{
		register uint32_t nCount;
        //RCC_ClocksTypeDef RCC_Clocks;
	    //RCC_GetClocksFreq (&RCC_Clocks);

        nCount=us*3747/1000;
        while(nCount){
        	//
        	nCount--;
        };
}

/*
 * off receive interrupts
 */
void RCV_OFF(void){
	//disable USART interrupt
	__HAL_UART_DISABLE_IT(&mbuart, UART_IT_RXNE);
}

// Purpose:    Enable data reception
// Inputs:     None
// Outputs:    None
void RCV_ON(void)
{
	//clear buffer if not empty

	while(__HAL_UART_GET_FLAG(&mbuart, UART_FLAG_RXNE) == SET)
	{
		__HAL_UART_FLUSH_DRREGISTER(&mbuart);
	};

	//clear USART interrupt
	__HAL_UART_CLEAR_FLAG(&mbuart, UART_FLAG_RXNE);

	//enable USART interrupt
	__HAL_UART_ENABLE_IT(&mbuart, UART_IT_RXNE);
}

// Purpose:    Initialize RS485 communication. Call this before
//             using any other RS485 functions.
// Inputs:     None
// Outputs:    None
void modbus_init(void){
	//uart hardware was previously init by main function

	//setup USART interrupt

#if (MODBUS_SWITCH==USART_1)
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);

#elif (MODBUS_SWITCH==USART_2)
	HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);

#elif (MODBUS_SWITCH==USART_3)
	HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);

#endif

   //clear port and enable it's interrupt
	RCV_ON();

   //if rtu mode - setup timer and it's interrupts
#if (MODBUS_SERIAL_TYPE == MODBUS_RTU)
	//timer module set active in main

	//setup TIM7 interrupt
	HAL_NVIC_SetPriority(TIM7_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(TIM7_IRQn);


#endif
}

// Purpose:    Start our timeout timer
// Inputs:     Enable, used to turn timer on/off
// Outputs:    None
// Not used for ASCII mode
#if (MODBUS_SERIAL_TYPE == MODBUS_RTU)
void modbus_enable_timeout(uint8_t enable)
{
   //disable timer interrupts
	__HAL_TIM_DISABLE_IT(&mbtim, TIM_IT_UPDATE);
   if (enable) {
      //reset counter
	   __HAL_TIM_SET_COUNTER(&mbtim, 0);
      //clear timer interrupt;
	   __HAL_TIM_CLEAR_IT(&mbtim, TIM_IT_UPDATE);
      //enable timer interrupts
	   __HAL_TIM_ENABLE_IT(&mbtim, TIM_IT_UPDATE);
   }
}
#endif

// Purpose:    Check if we have timed out waiting for a response
// Inputs:     None
// Outputs:    None
// Not used for ASCII mode
#if (MODBUS_SERIAL_TYPE == MODBUS_RTU)
   //timer IRQ Handler - modbus timeout now
   void TIM7_IRQHandler(void)
   {
	   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);

      if((modbus_serial_state == MODBUS_GETDATA) && (modbus_serial_crc.d == 0x0000) && (!modbus_serial_new))
      {
         modbus_rx.len-=2;
         modbus_serial_new=TRUE;
      }
      else
         modbus_serial_new=FALSE;
   
      modbus_serial_crc.d=0xFFFF;
      modbus_serial_state=MODBUS_GETADDY;
      modbus_enable_timeout(FALSE);
   }
#endif

// Purpose:    Calculate crc of data and updates global crc
// Inputs:     Character
// Outputs:    None
void modbus_calc_crc(char data)
{
   #if (MODBUS_SERIAL_TYPE == MODBUS_ASCII)
      modbus_serial_lrc+=data;
   #else
      uint8_t uIndex ; // will index into CRC lookup table

      uIndex = (modbus_serial_crc.b[1]) ^ data; // calculate the CRC
      modbus_serial_crc.b[1] = (modbus_serial_crc.b[0]) ^ modbus_auchCRCHi[uIndex];
      modbus_serial_crc.b[0] = modbus_auchCRCLo[uIndex];
   #endif
}

// Purpose:    Puts a character onto the serial line
// Inputs:     Character
// Outputs:    None
void modbus_serial_putc(uint8_t c)
{
   #if (MODBUS_SERIAL_TYPE==MODBUS_ASCII)
      uint8_t asciih,asciil;
      
      asciih=c>>4;
      if(asciih>9)
         asciih+=0x37;
      else
         asciih+=0x30;
      asciil=c&0xF;
      if(asciil>9)
         asciil+=0x37;
      else
         asciil+=0x30;
      Transmitt(&mbuart, (uint8_t)asciih);//fputc(asciih,MODBUS_SERIAL);
      Transmitt(&mbuart, (uint8_t)asciil);//fputc(asciil,MODBUS_SERIAL);
      modbus_calc_crc(c);
   #else
      //SendDataWait(MODBUS_PORT,(uint8_t)c);//fputc(c, MODBUS_SERIAL);
      Transmitt(&mbuart, (uint8_t)c);
      modbus_calc_crc(c);
      delay_us(1000000/MODBUS_SERIAL_BAUD); //one stop bit
   #endif
}

// Purpose:   Interrupt service routine for handling incoming serial data
// Inputs:    None
// Outputs:   None
#if (MODBUS_SWITCH==USART_1)
void USART1_IRQHandler(void)
#elif (MODBUS_SWITCH==USART_2)
void USART2_IRQHandler(void)
#elif (MODBUS_SWITCH==USART_3)
void USART3_IRQHandler(void)
#endif
{
   char c;
   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
   #if (MODBUS_SERIAL_TYPE==MODBUS_ASCII)
      static uint8_t two_characters=0;
      static uint8_t datah,datal,data;
   #endif

      c = (char)Receive(&mbuart);//c=fgetc(MODBUS_SERIAL);
   
   if (!modbus_serial_new)
   {
      #if (MODBUS_SERIAL_TYPE==MODBUS_ASCII)
         if(modbus_serial_state == MODBUS_START)
         {
            if(c==':')
               modbus_serial_state++;
         }
         else if(modbus_serial_state == MODBUS_GETADDY)
         {
            if(!two_characters)
            {
               if(c>=0x41)
                  datah=((c-0x37)<<4);
               else
                  datah=((c-0x30)<<4);
               modbus_serial_lrc=0;
            }
            else
            {
               if(c>=0x41)
                  datal=c-0x37;
               else
                  datal=c-0x30;
               data=(datah | datal);
               modbus_rx.address=data;
               modbus_calc_crc(data);
               modbus_serial_state++;
            }
            two_characters++;
         }
         else if(modbus_serial_state == MODBUS_GETFUNC)
         {
            if(!two_characters)
            {
               if(c>=0x41)
                  datah=((c-0x37)<<4);
               else
                  datah=((c-0x30)<<4);
            }
            else
            {
               if(c>=0x41)
                  datal=c-0x37;
               else
                  datal=c-0x30;
               data=(datah | datal);
               modbus_rx.func=data;
               modbus_calc_crc(data);
               modbus_serial_state++;
               modbus_rx.len=0;
               modbus_rx.error=0;
            }
            two_characters++;
         }
         else if(modbus_serial_state == MODBUS_GETDATA)
         {
            if(c=='\r')
            {
               modbus_serial_state++;
               modbus_rx.len--;
               modbus_serial_lrc-=data;
            }
            else if(!two_characters)
            {
               if(c>=0x41)
                  datah=((c-0x37)<<4);
               else
                  datah=((c-0x30)<<4);
               two_characters++;
            }
            else
            {
               if(c>=0x41)
                  datal=c-0x37;
               else
                  datal=c-0x30;
               data=(datah | datal);
               if (modbus_rx.len>=MODBUS_SERIAL_RX_BUFFER_SIZE)
                  modbus_rx.len=MODBUS_SERIAL_RX_BUFFER_SIZE-1;
               modbus_rx.data[modbus_rx.len]=data;
               modbus_rx.len++;
               modbus_calc_crc(data);
               two_characters++;
            }
         }
         else if(modbus_serial_state==MODBUS_STOP)
         {
            if(c=='\n')
            {
               modbus_serial_lrc=((0xFF-modbus_serial_lrc)+1);
               if(modbus_serial_lrc==data)
                  modbus_serial_new=TRUE;
            }
            modbus_serial_state=MODBUS_START;
            two_characters=0;
         }
      #else
            
         if(modbus_serial_state == MODBUS_GETADDY)
         {
            modbus_serial_crc.d = 0xFFFF;
            modbus_rx.address = c;
            modbus_serial_state++;
            modbus_rx.len = 0;
            modbus_rx.error=0;
         }
         else if(modbus_serial_state == MODBUS_GETFUNC)
         {
            modbus_rx.func = c;
            modbus_serial_state++;
         }
         else if(modbus_serial_state == MODBUS_GETDATA)
         {
            if (modbus_rx.len>=MODBUS_SERIAL_RX_BUFFER_SIZE) {modbus_rx.len=MODBUS_SERIAL_RX_BUFFER_SIZE-1;}
            modbus_rx.data[modbus_rx.len]=c;
            modbus_rx.len++;
         }
   
         modbus_calc_crc(c);
         modbus_enable_timeout(TRUE);
      #endif
   }
   #if (MODBUS_TYPE == MODBUS_TYPE_MASTER)
      modbus_serial_wait=MODBUS_SERIAL_TIMEOUT;
   #endif
}

// Purpose:    Send a message over the RS485 bus
// Inputs:     1) The destination address
//             2) The number of bytes of data to send
//             3) A pointer to the data to send
//             4) The length of the data
// Outputs:    TRUE if successful
//             FALSE if failed
// Note:       Format:  source | destination | data-length | data | checksum
void modbus_serial_send_start(uint8_t to, uint8_t func)
{
   #if (MODBUS_SERIAL_TYPE==MODBUS_ASCII)
      modbus_serial_lrc=0;
   #else
      modbus_serial_crc.d=0xFFFF;
   #endif
   modbus_serial_new=FALSE;

   RCV_OFF();
   
#if (MODBUS_SERIAL_ENABLE_PIN!=0) 
   output_high(MODBUS_SERIAL_ENABLE_PIN);
#endif

   #if (MODBUS_SERIAL_TYPE==MODBUS_RTU)
      delay_us(3500000/MODBUS_SERIAL_BAUD); //3.5 character delay
   #else
      fputc(':',MODBUS_SERIAL);
   #endif

   modbus_serial_putc(to);
   modbus_serial_putc(func);
}

void modbus_serial_send_stop()
{
   #if (MODBUS_SERIAL_TYPE == MODBUS_ASCII)
      int8 i;
      
      for(i=0;i<8;i++)
      {
         if(bit_test(modbus_serial_lrc,i))
            bit_clear(modbus_serial_lrc,i);
         else
            bit_set(modbus_serial_lrc,i);
      }
      modbus_serial_lrc++;
      
      modbus_serial_putc(modbus_serial_lrc);
      fputc('\r',MODBUS_SERIAL);
      fputc('\n',MODBUS_SERIAL);
   #else
      uint8_t crc_low, crc_high;
   
      crc_high=modbus_serial_crc.b[1];
      crc_low=modbus_serial_crc.b[0];
   
      modbus_serial_putc(crc_high);
      modbus_serial_putc(crc_low);
   #endif
   
/*#if (MODBUS_SERIAL_INT_SOURCE!=MODBUS_INT_EXT)
   WAIT_FOR_HW_BUFFER();
#endif*/
   
   #if (MODBUS_SERIAL_TYPE == MODBUS_RTU)
      delay_us(3500000/MODBUS_SERIAL_BAUD); //3.5 character delay
   #endif

   RCV_ON();

#if (MODBUS_SERIAL_ENABLE_PIN!=0) 
   output_low(MODBUS_SERIAL_ENABLE_PIN);
#endif

   #if (MODBUS_SERIAL_TYPE == MODBUS_ASCII)
      modbus_serial_lrc=0;
   #else
      modbus_serial_crc.d=0xFFFF;
   #endif
}

// Purpose:    Get a message from the RS485 bus and store it in a buffer
// Inputs:     None
// Outputs:    TRUE if a message was received
//             FALSE if no message is available
// Note:       Data will be filled in at the modbus_rx struct:
uint8_t modbus_kbhit()
{
   if(!modbus_serial_new)
      return FALSE;
   else if(modbus_rx.func & 0x80)           //did we receive an error?
   {
      modbus_rx.error = modbus_rx.data[0];  //if so grab the error and return true
      modbus_rx.len = 1;
   }
   modbus_serial_new=FALSE;
   return TRUE;
}

#if (MODBUS_TYPE==MODBUS_TYPE_MASTER)
/*MODBUS Master Functions*/

/********************************************************************
The following functions are defined in the MODBUS protocol.  Please
refer to http://www.modbus.org for the purpose of each of these.
All functions take the slaves address as their first parameter.
Each function returns the exception code received from the response.
The function will return 0 if there were no errors in transmission.
********************************************************************/

/*
read_coils
Input:     int8       address            Slave Address
           int16      start_address      Address to start reading from
           int16      quantity           Amount of addresses to read
Output:    exception                     0 if no error, else the exception
*/
exception modbus_read_coils(uint8_t address, uint16_t start_address, uint16_t quantity)
{
   modbus_serial_send_start(address, FUNC_READ_COILS);

   modbus_serial_putc(make8(start_address,1));
   modbus_serial_putc(make8(start_address,0));

   modbus_serial_putc(make8(quantity,1));
   modbus_serial_putc(make8(quantity,0));

   modbus_serial_send_stop();
   
   MODBUS_SERIAL_WAIT_FOR_RESPONSE();

   return modbus_rx.error;
}

/*
read_discrete_input
Input:     int8       address            Slave Address
           int16      start_address      Address to start reading from
           int16      quantity           Amount of addresses to read
Output:    exception                     0 if no error, else the exception
*/
exception modbus_read_discrete_input(uint8_t address, uint16_t start_address, uint16_t quantity)
{
   modbus_serial_send_start(address, FUNC_READ_DISCRETE_INPUT);

   modbus_serial_putc(make8(start_address,1));
   modbus_serial_putc(make8(start_address,0));

   modbus_serial_putc(make8(quantity,1));
   modbus_serial_putc(make8(quantity,0));

   modbus_serial_send_stop();
      
   MODBUS_SERIAL_WAIT_FOR_RESPONSE();

   return modbus_rx.error;
}

/*
read_holding_registers
Input:     int8       address            Slave Address
           int16      start_address      Address to start reading from
           int16      quantity           Amount of addresses to read
Output:    exception                     0 if no error, else the exception
*/
exception modbus_read_holding_registers(uint8_t address, uint16_t start_address, uint16_t quantity)
{
   modbus_serial_send_start(address, FUNC_READ_HOLDING_REGISTERS);

   modbus_serial_putc(make8(start_address,1));
   modbus_serial_putc(make8(start_address,0));

   modbus_serial_putc(make8(quantity,1));
   modbus_serial_putc(make8(quantity,0));

   modbus_serial_send_stop();
   
   MODBUS_SERIAL_WAIT_FOR_RESPONSE();

   return modbus_rx.error;
}

/*
read_input_registers
Input:     int8       address            Slave Address
           int16      start_address      Address to start reading from
           int16      quantity           Amount of addresses to read
Output:    exception                     0 if no error, else the exception
*/
exception modbus_read_input_registers(uint8_t address, uint16_t start_address, uint16_t quantity)
{
   modbus_serial_send_start(address, FUNC_READ_INPUT_REGISTERS);

   modbus_serial_putc(make8(start_address,1));
   modbus_serial_putc(make8(start_address,0));

   modbus_serial_putc(make8(quantity,1));
   modbus_serial_putc(make8(quantity,0));

   modbus_serial_send_stop();
   
   MODBUS_SERIAL_WAIT_FOR_RESPONSE();

   return modbus_rx.error;
}

/*
write_single_coil
Input:     int8       address            Slave Address
           int16      output_address     Address to write into
           int1       on                 true for on, false for off
Output:    exception                     0 if no error, else the exception
*/
exception modbus_write_single_coil(uint8_t address, uint16_t output_address, uint8_t on)
{
   modbus_serial_send_start(address, FUNC_WRITE_SINGLE_COIL);

   modbus_serial_putc(make8(output_address,1));
   modbus_serial_putc(make8(output_address,0));

   if(on)
       modbus_serial_putc(0xFF);
   else
       modbus_serial_putc(0x00);
   
   modbus_serial_putc(0x00);

   modbus_serial_send_stop();

   MODBUS_SERIAL_WAIT_FOR_RESPONSE();

   return modbus_rx.error;
}

/*
write_single_register
Input:     int8       address            Slave Address
           int16      reg_address        Address to write into
           int16      reg_value          Value to write
Output:    exception                     0 if no error, else the exception
*/
exception modbus_write_single_register(uint8_t address, uint16_t reg_address, uint16_t reg_value)
{
   modbus_serial_send_start(address, FUNC_WRITE_SINGLE_REGISTER);

   modbus_serial_putc(make8(reg_address,1));
   modbus_serial_putc(make8(reg_address,0));

   modbus_serial_putc(make8(reg_value,1));
   modbus_serial_putc(make8(reg_value,0));

   modbus_serial_send_stop();
   
   MODBUS_SERIAL_WAIT_FOR_RESPONSE();

   return modbus_rx.error;
}

/*
read_exception_status
Input:     int8       address            Slave Address
Output:    exception                     0 if no error, else the exception
*/
exception modbus_read_exception_status(uint8_t address)
{
   modbus_serial_send_start(address, FUNC_READ_EXCEPTION_STATUS);
   modbus_serial_send_stop();

   MODBUS_SERIAL_WAIT_FOR_RESPONSE();

   return modbus_rx.error;
}

/*
diagnostics
Input:     int8       address            Slave Address
           int16      sub_func           Subfunction to send
           int16      data               Data to send, changes based on subfunction
Output:    exception                     0 if no error, else the exception
*/
exception modbus_diagnostics(uint8_t address, uint16_t sub_func, uint16_t data)
{
   modbus_serial_send_start(address, FUNC_DIAGNOSTICS);

   modbus_serial_putc(make8(sub_func,1));
   modbus_serial_putc(make8(sub_func,0));

   modbus_serial_putc(make8(data,1));
   modbus_serial_putc(make8(data,0));

   modbus_serial_send_stop();

   MODBUS_SERIAL_WAIT_FOR_RESPONSE();

   return modbus_rx.error;
}

/*
get_comm_event_couter
Input:     int8       address            Slave Address
Output:    exception                     0 if no error, else the exception
*/
exception modbus_get_comm_event_counter(uint8_t address)
{
   modbus_serial_send_start(address, FUNC_GET_COMM_EVENT_COUNTER);
   modbus_serial_send_stop();

   MODBUS_SERIAL_WAIT_FOR_RESPONSE();

   return modbus_rx.error;
}

/*
get_comm_event_log
Input:     int8       address            Slave Address
Output:    exception                     0 if no error, else the exception
*/
exception modbus_get_comm_event_log(uint8_t address)
{
   modbus_serial_send_start(address, FUNC_GET_COMM_EVENT_LOG);
   modbus_serial_send_stop();

   MODBUS_SERIAL_WAIT_FOR_RESPONSE();

   return modbus_rx.error;
}

/*
write_multiple_coils
Input:     int8       address            Slave Address
           int16      start_address      Address to start at
           int16      quantity           Amount of coils to write to
           int1*      values             A pointer to an array holding the values to write
Output:    exception                     0 if no error, else the exception
*/
exception modbus_write_multiple_coils(uint8_t address, uint16_t start_address, uint16_t quantity,
                           uint8_t *values)
{
   uint8_t i,count;
   
   count = (uint8_t)((quantity/8));
   
   if(quantity%8)
      count++;      

   modbus_serial_send_start(address, FUNC_WRITE_MULTIPLE_COILS);

   modbus_serial_putc(make8(start_address,1));
   modbus_serial_putc(make8(start_address,0));

   modbus_serial_putc(make8(quantity,1));
   modbus_serial_putc(make8(quantity,0));

   modbus_serial_putc(count);

   for(i=0; i < count; ++i) 
      modbus_serial_putc(values[i]);

   modbus_serial_send_stop();

   MODBUS_SERIAL_WAIT_FOR_RESPONSE();

   return modbus_rx.error;
}

/*
write_multiple_registers
Input:     int8       address            Slave Address
           int16      start_address      Address to start at
           int16      quantity           Amount of coils to write to
           int16*     values             A pointer to an array holding the data to write
Output:    exception                     0 if no error, else the exception
*/
exception modbus_write_multiple_registers(uint8_t address, uint16_t start_address, uint16_t quantity,
                           uint16_t *values)
{
   uint8_t i,count;
   
   count = quantity*2;

   modbus_serial_send_start(address, FUNC_WRITE_MULTIPLE_REGISTERS);

   modbus_serial_putc(make8(start_address,1));
   modbus_serial_putc(make8(start_address,0));

   modbus_serial_putc(make8(quantity,1));
   modbus_serial_putc(make8(quantity,0));
   
   modbus_serial_putc(count);

   for(i=0; i < quantity; ++i)
   {
      modbus_serial_putc(make8(values[i],1));
      modbus_serial_putc(make8(values[i],0));
   }

   modbus_serial_send_stop();

   MODBUS_SERIAL_WAIT_FOR_RESPONSE();

   return modbus_rx.error;
}

/*
report_slave_id
Input:     int8       address            Slave Address
Output:    exception                     0 if no error, else the exception
*/
exception modbus_report_slave_id(uint8_t address)
{
   modbus_serial_send_start(address, FUNC_REPORT_SLAVE_ID);
   modbus_serial_send_stop();

   MODBUS_SERIAL_WAIT_FOR_RESPONSE();

   return modbus_rx.error;
}

/*
read_file_record
Input:     int8                address            Slave Address
           int8                byte_count         Number of bytes to read
           read_sub_request*   request            Structure holding record information
Output:    exception                              0 if no error, else the exception
*/
exception modbus_read_file_record(uint8_t address, uint8_t byte_count,
                            modbus_read_sub_request *request)
{
   uint8_t i;

   modbus_serial_send_start(address, FUNC_READ_FILE_RECORD);

   modbus_serial_putc(byte_count);

   for(i=0; i < (byte_count/7); i+=7)
   {
      modbus_serial_putc(request->reference_type);
      modbus_serial_putc(make8(request->file_number, 1));
      modbus_serial_putc(make8(request->file_number, 0));
      modbus_serial_putc(make8(request->record_number, 1));
      modbus_serial_putc(make8(request->record_number, 0));
      modbus_serial_putc(make8(request->record_length, 1));
      modbus_serial_putc(make8(request->record_length, 0));
      request++;
   }

   modbus_serial_send_stop();

   MODBUS_SERIAL_WAIT_FOR_RESPONSE();

   return modbus_rx.error;
}

/*
write_file_record
Input:     int8                address            Slave Address
           int8                byte_count         Number of bytes to read
           read_sub_request*   request            Structure holding record/data information
Output:    exception                              0 if no error, else the exception
*/
exception modbus_write_file_record(uint8_t address, uint8_t byte_count,
                            modbus_write_sub_request *request)
{
   uint8_t i, j=0;

   modbus_serial_send_start(address, FUNC_WRITE_FILE_RECORD);

   modbus_serial_putc(byte_count);

   for(i=0; i < byte_count; i+=(7+(j*2)))
   {
      modbus_serial_putc(request->reference_type);
      modbus_serial_putc(make8(request->file_number, 1));
      modbus_serial_putc(make8(request->file_number, 0));
      modbus_serial_putc(make8(request->record_number, 1));
      modbus_serial_putc(make8(request->record_number, 0));
      modbus_serial_putc(make8(request->record_length, 1));
      modbus_serial_putc(make8(request->record_length, 0));

      for(j=0; (j < request->record_length) && 
            (j < MODBUS_SERIAL_RX_BUFFER_SIZE-8); j+=2)
      {
         modbus_serial_putc(make8(request->data[j], 1));
         modbus_serial_putc(make8(request->data[j], 0));
      }
      request++;
   }

   modbus_serial_send_stop();

   MODBUS_SERIAL_WAIT_FOR_RESPONSE();

   return modbus_rx.error;
}

/*
mask_write_register
Input:     int8       address            Slave Address
           int16      reference_address  Address to mask
           int16      AND_mask           A mask to AND with the data at reference_address
           int16      OR_mask            A mask to OR with the data at reference_address
Output:    exception                              0 if no error, else the exception
*/
exception modbus_mask_write_register(uint8_t address, uint16_t reference_address,
                           uint16_t AND_mask, uint16_t OR_mask)
{
   modbus_serial_send_start(address, FUNC_MASK_WRITE_REGISTER);

   modbus_serial_putc(make8(reference_address,1));
   modbus_serial_putc(make8(reference_address,0));

   modbus_serial_putc(make8(AND_mask,1));
   modbus_serial_putc(make8(AND_mask,0));

   modbus_serial_putc(make8(OR_mask, 1));
   modbus_serial_putc(make8(OR_mask, 0));

   modbus_serial_send_stop();

   MODBUS_SERIAL_WAIT_FOR_RESPONSE();

   return modbus_rx.error;
}

/*
read_write_multiple_registers
Input:     int8       address                Slave Address
           int16      read_start             Address to start reading
           int16      read_quantity          Amount of registers to read
           int16      write_start            Address to start writing
           int16      write_quantity         Amount of registers to write
           int16*     write_registers_value  Pointer to an aray us to write
Output:    exception                         0 if no error, else the exception
*/
exception modbus_read_write_multiple_registers(uint8_t address, uint16_t read_start,
                                    uint16_t read_quantity, uint16_t write_start,
                                    uint16_t write_quantity,
                                    uint16_t *write_registers_value)
{
   uint8_t i;

   modbus_serial_send_start(address, FUNC_READ_WRITE_MULTIPLE_REGISTERS);

   modbus_serial_putc(make8(read_start,1));
   modbus_serial_putc(make8(read_start,0));

   modbus_serial_putc(make8(read_quantity,1));
   modbus_serial_putc(make8(read_quantity,0));

   modbus_serial_putc(make8(write_start, 1));
   modbus_serial_putc(make8(write_start, 0));

   modbus_serial_putc(make8(write_quantity, 1));
   modbus_serial_putc(make8(write_quantity, 0));

   modbus_serial_putc((uint8_t)(2*write_quantity));

   for(i=0; i < write_quantity ; i+=2)
   {
      modbus_serial_putc(make8(write_registers_value[i], 1));
      modbus_serial_putc(make8(write_registers_value[i+1], 0));
   }

   modbus_serial_send_stop();

   MODBUS_SERIAL_WAIT_FOR_RESPONSE();

   return modbus_rx.error;
}

/*
read_FIFO_queue
Input:     int8       address           Slave Address
           int16      FIFO_address      FIFO address
Output:    exception                    0 if no error, else the exception
*/
exception modbus_read_FIFO_queue(uint8_t address, uint16_t FIFO_address)
{
   modbus_serial_send_start(address, FUNC_READ_FIFO_QUEUE);

   modbus_serial_putc(make8(FIFO_address, 1));
   modbus_serial_putc(make8(FIFO_address, 0));

   modbus_serial_send_stop();

   MODBUS_SERIAL_WAIT_FOR_RESPONSE();

   return modbus_rx.error;
}

#else
/*MODBUS Slave Functions*/

/********************************************************************
The following slave functions are defined in the MODBUS protocol.
Please refer to http://www.modbus.org for the purpose of each of
these.  All functions take the slaves address as their first
parameter.
********************************************************************/

/*
read_coils_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      coil_data          Pointer to an array of data to send
Output:    void
*/
void modbus_read_coils_rsp(uint8_t address, uint8_t byte_count, uint8_t* coil_data)
{
   uint8_t i;

   modbus_serial_send_start(address, FUNC_READ_COILS);

   modbus_serial_putc(byte_count);

   for(i=0; i < byte_count; ++i)
   {
      modbus_serial_putc(*coil_data);
      coil_data++;
   }

   modbus_serial_send_stop();
}

/*
read_discrete_input_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      input_data         Pointer to an array of data to send
Output:    void
*/
void modbus_read_discrete_input_rsp(uint8_t address, uint8_t byte_count,
                                    uint8_t *input_data)
{
   uint8_t i;

   modbus_serial_send_start(address, FUNC_READ_DISCRETE_INPUT);

   modbus_serial_putc(byte_count);

   for(i=0; i < byte_count; ++i)
   {
      modbus_serial_putc(*input_data);
      input_data++;
   }

   modbus_serial_send_stop();
}

/*
read_holding_registers_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      reg_data           Pointer to an array of data to send
Output:    void
*/
void modbus_read_holding_registers_rsp(uint8_t address, uint8_t byte_count,
                                        uint16_t *reg_data)
{
   uint8_t i;

   modbus_serial_send_start(address, FUNC_READ_HOLDING_REGISTERS);

   modbus_serial_putc(byte_count);

   for(i=0; i < byte_count; i+=2)
   {
      modbus_serial_putc(make8(*reg_data,1));
      modbus_serial_putc(make8(*reg_data,0));
      reg_data++;
   }

   modbus_serial_send_stop();
}

/*
read_input_registers_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      input_data         Pointer to an array of data to send
Output:    void
*/
void modbus_read_input_registers_rsp(uint8_t address, uint8_t byte_count,
                                        uint16_t *input_data)
{
   uint8_t i;

   modbus_serial_send_start(address, FUNC_READ_INPUT_REGISTERS);

   modbus_serial_putc(byte_count);

   for(i=0; i < byte_count; i+=2)
   {
      modbus_serial_putc(make8(*input_data,1));
      modbus_serial_putc(make8(*input_data,0));
      input_data++;
   }

   modbus_serial_send_stop();
}

/*
write_single_coil_rsp
Input:     int8       address            Slave Address
           int16      output_address     Echo of output address received
           int16      output_value       Echo of output value received
Output:    void
*/
void modbus_write_single_coil_rsp(uint8_t address, uint16_t output_address,
                                    uint16_t output_value)
{
   modbus_serial_send_start(address, FUNC_WRITE_SINGLE_COIL);

   modbus_serial_putc(make8(output_address,1));
   modbus_serial_putc(make8(output_address,0));

   modbus_serial_putc(make8(output_value,1));
   modbus_serial_putc(make8(output_value,0));

   modbus_serial_send_stop();
}

/*
write_single_register_rsp
Input:     int8       address            Slave Address
           int16      reg_address        Echo of register address received
           int16      reg_value          Echo of register value received
Output:    void
*/
void modbus_write_single_register_rsp(uint8_t address, uint16_t reg_address,
                                        uint16_t reg_value)
{
   modbus_serial_send_start(address, FUNC_WRITE_SINGLE_REGISTER);

   modbus_serial_putc(make8(reg_address,1));
   modbus_serial_putc(make8(reg_address,0));

   modbus_serial_putc(make8(reg_value,1));
   modbus_serial_putc(make8(reg_value,0));

   modbus_serial_send_stop();
}

/*
read_exception_status_rsp
Input:     int8       address            Slave Address
Output:    void
*/
void modbus_read_exception_status_rsp(uint8_t address, uint8_t data)
{
   modbus_serial_send_start(address, FUNC_READ_EXCEPTION_STATUS);
   modbus_serial_send_stop();
}

/*
diagnostics_rsp
Input:     int8       address            Slave Address
           int16      sub_func           Echo of sub function received
           int16      data               Echo of data received
Output:    void
*/
void modbus_diagnostics_rsp(uint8_t address, uint16_t sub_func, uint16_t data)
{
   modbus_serial_send_start(address, FUNC_DIAGNOSTICS);

   modbus_serial_putc(make8(sub_func,1));
   modbus_serial_putc(make8(sub_func,0));

   modbus_serial_putc(make8(data,1));
   modbus_serial_putc(make8(data,0));

   modbus_serial_send_stop();
}

/*
get_comm_event_counter_rsp
Input:     int8       address            Slave Address
           int16      status             Status, refer to MODBUS documentation
           int16      event_count        Count of events
Output:    void
*/
void modbus_get_comm_event_counter_rsp(uint8_t address, uint16_t status,
                                        uint16_t event_count)
{
   modbus_serial_send_start(address, FUNC_GET_COMM_EVENT_COUNTER);

   modbus_serial_putc(make8(status, 1));
   modbus_serial_putc(make8(status, 0));

   modbus_serial_putc(make8(event_count, 1));
   modbus_serial_putc(make8(event_count, 0));

   modbus_serial_send_stop();
}

/*
get_comm_event_counter_rsp
Input:     int8       address            Slave Address
           int16      status             Status, refer to MODBUS documentation
           int16      event_count        Count of events
           int16      message_count      Count of messages
           int8*      events             Pointer to event data
           int8       events_len         Length of event data in bytes
Output:    void
*/
void modbus_get_comm_event_log_rsp(uint8_t address, uint16_t status,
                                    uint16_t event_count, uint16_t message_count,
                                    uint8_t *events, uint8_t events_len)
{
   uint8_t i;
    
   modbus_serial_send_start(address, FUNC_GET_COMM_EVENT_LOG);

   modbus_serial_putc(events_len+6);

   modbus_serial_putc(make8(status, 1));
   modbus_serial_putc(make8(status, 0));

   modbus_serial_putc(make8(event_count, 1));
   modbus_serial_putc(make8(event_count, 0));

   modbus_serial_putc(make8(message_count, 1));
   modbus_serial_putc(make8(message_count, 0));

   for(i=0; i < events_len; ++i)
   {
      modbus_serial_putc(*events);
      events++;
   }

   modbus_serial_send_stop();
}

/*
write_multiple_coils_rsp
Input:     int8       address            Slave Address
           int16      start_address      Echo of address to start at
           int16      quantity           Echo of amount of coils written to
Output:    void
*/
void modbus_write_multiple_coils_rsp(uint8_t address, uint16_t start_address,
                                        uint16_t quantity)
{
   modbus_serial_send_start(address, FUNC_WRITE_MULTIPLE_COILS);

   modbus_serial_putc(make8(start_address,1));
   modbus_serial_putc(make8(start_address,0));

   modbus_serial_putc(make8(quantity,1));
   modbus_serial_putc(make8(quantity,0));

   modbus_serial_send_stop();
}

/*
write_multiple_registers_rsp
Input:     int8       address            Slave Address
           int16      start_address      Echo of address to start at
           int16      quantity           Echo of amount of registers written to
Output:    void
*/
void modbus_write_multiple_registers_rsp(uint8_t address, uint16_t start_address,
                                            uint16_t quantity)
{
   modbus_serial_send_start(address, FUNC_WRITE_MULTIPLE_REGISTERS);

   modbus_serial_putc(make8(start_address,1));
   modbus_serial_putc(make8(start_address,0));

   modbus_serial_putc(make8(quantity,1));
   modbus_serial_putc(make8(quantity,0));

   modbus_serial_send_stop();
}

/*
report_slave_id_rsp
Input:     int8       address            Slave Address
           int8       slave_id           Slave Address
           int8       run_status         Are we running?
           int8*      data               Pointer to an array holding the data
           int8       data_len           Length of data in bytes
Output:    void
*/
void modbus_report_slave_id_rsp(uint8_t address, uint8_t slave_id, uint8_t run_status,
                              uint8_t *data, uint8_t data_len)
{
   uint8_t i;

   modbus_serial_send_start(address, FUNC_REPORT_SLAVE_ID);

   modbus_serial_putc(data_len+2);
   modbus_serial_putc(slave_id);

   if(run_status)
    modbus_serial_putc(0xFF);
   else
    modbus_serial_putc(0x00);

   for(i=0; i < data_len; ++i)
   {
      modbus_serial_putc(*data);
      data++;
   }

   modbus_serial_send_stop();
}

/*
read_file_record_rsp
Input:     int8                     address            Slave Address
           int8                     byte_count         Number of bytes to send
           read_sub_request_rsp*    request            Structure holding record/data information
Output:    void
*/
void modbus_read_file_record_rsp(uint8_t address, uint8_t byte_count,
                                    modbus_read_sub_request_rsp *request)
{
   uint8_t i=0,j;

   modbus_serial_send_start(address, FUNC_READ_FILE_RECORD);

   modbus_serial_putc(byte_count);

   while(i < byte_count);
   {
      modbus_serial_putc(request->record_length);
      modbus_serial_putc(request->reference_type);

      for(j=0; (j < request->record_length); j+=2)
      {
         modbus_serial_putc(make8(request->data[j], 1));
         modbus_serial_putc(make8(request->data[j], 0));
      }

      i += (request->record_length)+1;
      request++;
   }

   modbus_serial_send_stop();
}

/*
write_file_record_rsp
Input:     int8                     address            Slave Address
           int8                     byte_count         Echo of number of bytes sent
           write_sub_request_rsp*   request            Echo of Structure holding record information
Output:    void
*/
void modbus_write_file_record_rsp(uint8_t address, uint8_t byte_count,
                                    modbus_write_sub_request_rsp *request)
{
   uint8_t i, j=0;

   modbus_serial_send_start(address, FUNC_WRITE_FILE_RECORD);

   modbus_serial_putc(byte_count);

   for(i=0; i < byte_count; i+=(7+(j*2)))
   {
      modbus_serial_putc(request->reference_type);
      modbus_serial_putc(make8(request->file_number, 1));
      modbus_serial_putc(make8(request->file_number, 0));
      modbus_serial_putc(make8(request->record_number, 1));
      modbus_serial_putc(make8(request->record_number, 0));
      modbus_serial_putc(make8(request->record_length, 1));
      modbus_serial_putc(make8(request->record_length, 0));

      for(j=0; (j < request->record_length); j+=2)
      {
         modbus_serial_putc(make8(request->data[j], 1));
         modbus_serial_putc(make8(request->data[j], 0));
      }
   }

   modbus_serial_send_stop();
}

/*
mask_write_register_rsp
Input:     int8        address            Slave Address
           int16       reference_address  Echo of reference address
           int16       AND_mask           Echo of AND mask
           int16       OR_mask            Echo or OR mask
Output:    void
*/
void modbus_mask_write_register_rsp(uint8_t address, uint16_t reference_address,
                           uint16_t AND_mask, uint16_t OR_mask)
{
   modbus_serial_send_start(address, FUNC_MASK_WRITE_REGISTER);

   modbus_serial_putc(make8(reference_address,1));
   modbus_serial_putc(make8(reference_address,0));

   modbus_serial_putc(make8(AND_mask,1));
   modbus_serial_putc(make8(AND_mask,0));

   modbus_serial_putc(make8(OR_mask, 1));
   modbus_serial_putc(make8(OR_mask, 0));

   modbus_serial_send_stop();
}

/*
read_write_multiple_registers_rsp
Input:     int8        address            Slave Address
           int16*      data               Pointer to an array of data
           int8        data_len           Length of data in bytes
Output:    void
*/
void modbus_read_write_multiple_registers_rsp(uint8_t address, uint8_t data_len,
                                                uint16_t *data)
{
   uint8_t i;

   modbus_serial_send_start(address, FUNC_READ_WRITE_MULTIPLE_REGISTERS);

   modbus_serial_putc(data_len*2);

   for(i=0; i < data_len*2; i+=2)
   {
      modbus_serial_putc(make8(data[i], 1));
      modbus_serial_putc(make8(data[i], 0));
   }

   modbus_serial_send_stop();
}

/*
read_FIFO_queue_rsp
Input:     int8        address            Slave Address
           int16       FIFO_len           Length of FIFO in bytes
           int16*      data               Pointer to an array of data
Output:    void
*/
void modbus_read_FIFO_queue_rsp(uint8_t address, uint16_t FIFO_len, uint16_t *data)
{
   uint8_t i;
   uint16_t byte_count;

   byte_count = ((FIFO_len*2)+2);

   modbus_serial_send_start(address, FUNC_READ_FIFO_QUEUE);

   modbus_serial_putc(make8(byte_count, 1));
   modbus_serial_putc(make8(byte_count, 0));

   modbus_serial_putc(make8(FIFO_len, 1));
   modbus_serial_putc(make8(FIFO_len, 0));

   for(i=0; i < FIFO_len; i+=2)
   {
      modbus_serial_putc(make8(data[i], 1));
      modbus_serial_putc(make8(data[i], 0));
   }

   modbus_serial_send_stop();
}

void modbus_exception_rsp(uint8_t address, uint16_t func, exception error)
{
   modbus_serial_send_start(address, func|0x80);
   modbus_serial_putc(error);
   modbus_serial_send_stop();
}

#endif
