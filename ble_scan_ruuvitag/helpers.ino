
#include "Arduino.h"
#include "String.h"

// Converts hexadecimal values to decimal values
uint16_t hexadecimalToDecimal(String hexVal)
{
    uint8_t len = hexVal.length();
    uint16_t base = 1;
    int dec_val = 0;

    //Serial.print("hexadecimalToDecimal(");  Serial.print(hexVal);
    for (int i = len - 1; i >= 0; i--)
    {
        if (hexVal[i] >= '0' && hexVal[i] <= '9')
        {
            dec_val += (hexVal[i] - 48) * base;
            base = base * 16;
        }
        else if (hexVal[i] >= 'A' && hexVal[i] <= 'F')
        {
            dec_val += (hexVal[i] - 55) * base;
            base = base * 16;
        }
    }
    return dec_val;
};
