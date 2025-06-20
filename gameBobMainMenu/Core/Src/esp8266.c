/*
 * esp8266.c
 *
 *  Created on: Mar 2, 2025
 *      Author: Eryk
 */
#include "esp8266.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "extraTools.h"


CmdType cmd_type = DEFAULT;

volatile uint8_t rx_buffer[MAX_RX_BUFFER] = {0};
volatile uint16_t rx_index;
volatile uint8_t tx_completed_flag;
volatile uint8_t found_flag;

UART_HandleTypeDef *huart_esp = NULL;


uint8_t Esp8266_Init(UART_HandleTypeDef *huart){
	huart_esp = huart;
	cmd_type = DEFAULT;
	//reset_flags();
	Esp8266_Reset();
	char cmd[20];
	sprintf(cmd, "AT\r\n");
	uint8_t result = sendAndReceive(cmd);

	return result;
	/*
	HAL_UART_Transmit_IT(huart_esp, (uint8_t*)cmd, strlen(cmd));
	while(!tx_completed_flag){
	}
	HAL_UART_Receive_IT(huart_esp, (uint8_t*)rx_buffer, 1);
	while(found_flag == ESP8266_BUSY){
	}
	*/
	//return found_flag;
}

uint8_t Esp8266_SetCwMode(uint8_t cw_mode){
	if(huart_esp == NULL){
		return ESP_UART_NOT_DEFINED;
	}
	if(!(cw_mode>=1 && cw_mode<=3)){
		return ESP8266_ERROR;
	}
	reset_flags();
	cmd_type = DEFAULT;
	char cmd[20];
	sprintf(cmd, "AT+CWMODE=%d\r\n",cw_mode);
	uint8_t result = sendAndReceive(cmd);
	if(result == ESP8266_OK){
		//result = Esp8266_Reset();
		//HAL_Delay(200);
		return result;
	}
	else
		return ESP8266_ERROR;

}

uint8_t Esp8266_ListAP(char *output){
	if(huart_esp == NULL){
		return ESP_UART_NOT_DEFINED;
	}
	reset_flags();
	cmd_type = DEFAULT;
	char cmd[30];
	sprintf(cmd, "AT+CWLAP\r\n");
	uint8_t result = sendAndReceive(cmd);
	if(result == ESP8266_OK){
		output = malloc(strlen((char*)rx_buffer)+1);
		strcpy(output,(char*)rx_buffer);
	}
	return result;
}


uint8_t Esp8266_Reset(){
	if(huart_esp == NULL)
		return ESP_UART_NOT_DEFINED;
	char cmd[20];
	sprintf(cmd,"AT+RST\r\n");
	uint8_t result = sendAndReceive(cmd);
	HAL_Delay(500);
	return result;
}

uint8_t Esp8266_ConnectAP(char *ssid, char *psw){
	if(huart_esp == NULL){
		return ESP_UART_NOT_DEFINED;
	}
	reset_flags();
	cmd_type = DEFAULT;
	char cmd[50];
	sprintf(cmd, "AT+CWJAP=%c%s%c,%c%s%c\r\n",'"',ssid,'"','"',psw,'"');
	uint8_t result = sendAndReceive(cmd);
	return result;
}

uint8_t Esp8266_GetIpAddress(char *buffer, size_t size){
	if(huart_esp == NULL){
		return ESP_UART_NOT_DEFINED;
	}
	reset_flags();
	cmd_type = DEFAULT;
	char cmd[50];
	sprintf(cmd, "AT+CIFSR\r\n");
	uint8_t result = sendAndReceive(cmd);
	if(result == ESP8266_OK){
		char *ip = strstr((char*)rx_buffer,"CIFSR:STAIP,\"");
		if(ip!=NULL){
			ip+=13;//set pointer to the beginning of ip address
			char temp_buf[20];
			size_t i = 0;
			while(*ip != '"'){
				temp_buf[i]=*ip;
				ip++;
				i++;
			}
			temp_buf[i] = '\0';
			snprintf(buffer,size,temp_buf);
			return ESP8266_OK;
		}
		return ESP8266_ERROR;
	}
	return result;
}

uint8_t Esp8266_StartTCPIPConnection(char *server, char *port){
	if(huart_esp == NULL){
		return ESP_UART_NOT_DEFINED;
	}
	reset_flags();
	cmd_type = DEFAULT;
	char cmd[50];
	sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n",server,port);
	uint8_t result = sendAndReceive(cmd);
	return result;

}


uint8_t Esp8266_SendIpCommand(char *cmd){
	if(huart_esp == NULL){
		return ESP_UART_NOT_DEFINED;
	}
	uint16_t cmd_length = strlen(cmd);
	reset_flags();
	cmd_type = IP_SEND;
	char cmd_length_str[4];
	char command[50];
	intToString(cmd_length, cmd_length_str, 50);
	sprintf(command,"AT+CIPSEND=%s\r\n",cmd_length_str);
	uint8_t result = sendAndReceive(command);
	if(result == ESP8266_OK){
		reset_flags();
		cmd_type = GET_REQUEST;
		result = sendAndReceive(cmd);
		if(result == ESP8266_OK){

		}
		return result;
	}
	return result;

}

uint8_t Esp8266_ChangeBaudRate(int baudrate, int8_t data_bits, int8_t stop_bits, int8_t parity, int8_t flow_control){
	if(huart_esp == NULL){
		return ESP_UART_NOT_DEFINED;
	}
	reset_flags();
	char command[50];
	sprintf(command, "AT+UART=%d,%d,%d,%d,%d\r\n",baudrate,data_bits, stop_bits, parity, flow_control);
	uint8_t result = sendAndReceive(command);
	//uint8_t result = 0;
	return result;
}


void ESP8266_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart!=huart_esp)
		return;
	tx_completed_flag = 1;
}
void ESP8266_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart!=huart_esp)
		return;

//	if(rx_buffer[rx_index] == '\0'){
//		rx_buffer[rx_index] = 'X';
//		rx_buffer[rx_index+1] = 'D';
//		rx_index+=2;
//	}
//	else{
//		rx_index++;
//	}
	rx_index++;
	HAL_UART_Receive_IT(huart_esp, rx_buffer+rx_index, 1);
//	}
}

static void reset_flags(){
	found_flag = ESP8266_BUSY;
	rx_index = 0;
	tx_completed_flag = 0;
	for(int i=0;i<MAX_RX_BUFFER;i++){
		rx_buffer[i] = 0;
	}
}


static uint8_t sendAndReceive(char *cmd){
	reset_flags();
	HAL_UART_Transmit_IT(huart_esp, (uint8_t*)cmd, strlen(cmd));
	while(!tx_completed_flag){
	}
	HAL_UART_Receive_IT(huart_esp, (uint8_t*)rx_buffer, 1);
	const char *ok = "OK\r\n";
	const char *error = "ERROR\r\n";
	const char *start_ip = ">";
	const char *connected = "ALREADY CONNECT\r\n";
	const char *closed = "CLOSED\r";

	while(found_flag == ESP8266_BUSY){
		if(find_str((char*)rx_buffer,ok,strlen(ok)) > -1 && cmd_type == DEFAULT){
			found_flag = ESP8266_OK;
		}
		else if(find_str((char*)rx_buffer,connected,strlen(connected))> -1){
			found_flag = ESP8266_OK;
		}
		else if(cmd_type == IP_SEND && find_str((char*)rx_buffer,start_ip,strlen(start_ip)) > -1){
			found_flag = ESP8266_OK;
		}
		else if(cmd_type == GET_REQUEST && find_str((char*)rx_buffer,closed,strlen(closed))> -1 ){
			found_flag = ESP8266_OK;
		}
		else if(cmd_type == DEFAULT && find_str((char*)rx_buffer,error,strlen(error)) > -1 ){
			found_flag = ESP8266_ERROR;
		}
		else{
			found_flag = ESP8266_BUSY;
		}
	}
	return found_flag;
}

int find_str(char* str, const char *str_to_find, int str_length){
	int index = -1;
	uint8_t temp_flag;
	char debu;
	for(int i=str-(char*)rx_buffer;i<=rx_index;i++){
		temp_flag = 1;
		if(i+str_length <= rx_index){
			for(int j=0;j<str_length;j++){
				if(str[i-(int)(str-(char*)rx_buffer)+j]!=str_to_find[j]){
					debu = str[i+j];
					temp_flag = 0;
					break;
				}
			}
			if(temp_flag){
				index = i;
				return index;
			}
		}
	}
	return index;
}


