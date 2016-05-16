/*
 * unit.cpp
 *
 *  Created on: 2014-7-26
 *      Author: root
 */
#include "unit.h"

void print_hex(const char *array, int count , const char * Tags )
{
	int i;
	if (Tags !=NULL)
		printf("<%s>", Tags);
	printf("[%02X", array[0] & 0xFF);
	for (i = 1; i < count; i++)
	{
		printf(" %02X", array[i] & 0xFF);
	}
	printf("]\n");
}

void Hex2Str(const char * Hex, char * Str, int Len)
{ //Len为hex的位数
	int i;
	for (i = 0; i < Len; i++)
		sprintf(Str + i * 2, "%02X ", Hex[i] & 0xFF);
	*(Str + i * 2) = 0;
}

bool Str2Hex(const char * Str, char * Hex, int Len)
{ //Len为Str的位数

	if (Len%2 != 0) return false;
	int i;
	int tmp = 0;
	char StrTmp[3];
	for (i = 0; i < Len/2; i++)
	{
		StrTmp[0] = Str[2 * i];
		StrTmp[1] = Str[2 * i + 1];
		StrTmp[2] = 0;
		sscanf(StrTmp, "%02X", &tmp);
		*(Hex + i) = tmp;
	}
	return true;
}

unsigned int Hex2Int(const char * Hex , int count )
{
	unsigned int res = 0 ;
	int j = 1 ;
	if ( count > 4 ) return res ;
	for (int i = 0 ; i < count ; i++)
	{
		j = 1 ;
		j <<= ( 8 * (count - i - 1) ) ;
		res += (Hex[i] & 0xFF) * j ;
	}
	return res;
}

bool Int2Hex(const unsigned int src , char * Hexstr , int strlen )
{
	int len = 4 ;
	if (strlen < 4) len = strlen ;
	int j = 1 ;
	bool res = true;
	char tmp[4];
	unsigned int n = src ;
	for (int i = 0 ; i < 4 ; i++)
	{
		j = 1 ;
		j <<= ( 8 * (4 - i - 1) ) ;
		tmp[i] = n / j ;
		n %= j ;
		if ( (4 - i) <= len )
		{
			Hexstr[len - (4- i)] = tmp[i] ;
		}
	}
	//print_hex (tmp , 4 , "Int2Hex") ;
	for (int k = 0 ; k < 4-len ; k++ )
	{
		if (tmp[k] != 0x00 )
		{
			res = false ;
			break;
		}
	}
	return res;
}

//解析二进制字符串
bool Str2Bin(const char szStr[8], char* cpRst)
{
	char cRtn = '\0';
	if (szStr == NULL)
		return false;
	for (int i = 0; i < 8; i++)
	{
		if (szStr[i] != '0' && szStr[i] != '1')
			return false;
		cRtn = szStr[i] - 48; // '0' = 48
		cRtn = cRtn << (8 - i - 1);
		*cpRst |= cRtn;
	}
	return true;
}

//二进制转二进制字符串
void Bin2Str(const char ch , char * BinStr)
{
	int j = 1;
	for (int i = 7; i >= 0; i--)
	{
		BinStr[i] = (char) (bool) (ch & j) + '0';
		j <<= 1;
	}
	BinStr[8] = 0;
}

char StrBCC (const char * str , int len)
{
	char BCC = 0x00;
	BCC = str[0];
	for (int i = 1 ; i < len ; i++)
	{
		BCC = BCC^str[i] ;
	}
	return BCC ;
}

void Hexstrncpy (char * dst , const char * src , int len)
{
     for (int i = 0 ; i < len ; i++)
     {
    	 dst[i] = src[i];
     }
}

bool Hexstrcmp (const char * src , const char * dst , int srclen)
{
     for (int i = 0 ; i < srclen ; i++)
     {
    	 if (src[i] != dst[i])
    		 return false ;
     }
     return true;
}
