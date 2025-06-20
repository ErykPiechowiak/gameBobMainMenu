/*
 * extraTools.c
 *
 *  Created on: Jun 20, 2025
 *      Author: Eryk
 */


#include "extraTools.h"

void intToString(int N, char *str, int size){
    int i = 0;
    // Save the copy of the number for sign
    int sign = N;
    // If the number is negative, make it positive
    if (N < 0)
        N = -N;
    // Extract digits from the number and add them to the
    // string
    while (N > 0) {
        // Convert integer digit to character and store
      	// it in the str
    	if(i+1<size){
    		str[i++] = N % 10 + '0';
    		N /= 10;
    	}
    	else{
    		str[i] = '\0';
    		return;
    	}

    }
    // If the number was negative, add a minus sign to the
    // string
    if (sign < 0) {
    	if(i+1 < size){
    		str[i++] = '-';
    	}
    	else{
    		str[i] = '\0';
    	}
    }
    // Null-terminate the string
    str[i] = '\0';

    // Reverse the string to get the correct order
    for (int j = 0, k = i - 1; j < k; j++, k--) {
        char temp = str[j];
        str[j] = str[k];
        str[k] = temp;
    }
    return;
}
