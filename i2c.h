#define PM_INIT()     P0M0 = 0x00;P0M1 = 0x00;P1M0 = 0x00;P1M1 = 0x00;P2M0 = 0x00;P2M1 = 0x00;P3M0 = 0x00;P3M1 = 0x00;P4M0 = 0x00;P4M1 = 0x00;P5M0 = 0x00;P5M1 = 0x00;P6M0 = 0x00;P6M1 = 0x00;P7M0 = 0x00;P7M1 = 0x00

#define SDA P32
#define SCL P33

void Delay()		//@11.0592MHz  3usec
{
	unsigned char i;

	_nop_();
	_nop_();
	_nop_();
	i = 5;
	while (--i);
}

void Delay1000ms()		//@11.0592MHz
{
	unsigned char i, j, k;

	_nop_();
	_nop_();
	i = 43;
	j = 6;
	k = 203;
	do
	{
		do
		{
			while (--k);
		} while (--j);
	} while (--i);
}



sfr P0M1 = 0x93;
sfr P0M0 = 0x94;
sfr P1M1 = 0x91;
sfr P1M0 = 0x92;
sfr P2M1 = 0x95;
sfr P2M0 = 0x96;
sfr P3M1 = 0xb1;
sfr P3M0 = 0xb2;
sfr P4M1 = 0xb3;
sfr P4M0 = 0xb4;
sfr P5M1 = 0xC9;
sfr P5M0 = 0xCA;
sfr P6M1 = 0xCB;
sfr P6M0 = 0xCC;
sfr P7M1 = 0xE1;
sfr P7M0 = 0xE2;

sbit P00 = P0^0;
sbit P01 = P0^1;
sbit P02 = P0^2;
sbit P03 = P0^3;
sbit P04 = P0^4;
sbit P05 = P0^5;
sbit P06 = P0^6;
sbit P07 = P0^7;
sbit P10 = P1^0;
sbit P11 = P1^1;
sbit P12 = P1^2;
sbit P13 = P1^3;
sbit P14 = P1^4;
sbit P15 = P1^5;
sbit P16 = P1^6;
sbit P17 = P1^7;
sbit P20 = P2^0;
sbit P21 = P2^1;
sbit P22 = P2^2;
sbit P23 = P2^3;
sbit P24 = P2^4;
sbit P25 = P2^5;
sbit P26 = P2^6;
sbit P27 = P2^7;
sbit P30 = P3^0;
sbit P31 = P3^1;
sbit P32 = P3^2;
sbit P33 = P3^3;
sbit P34 = P3^4;
sbit P35 = P3^5;
sbit P36 = P3^6;
sbit P37 = P3^7;

sfr P4   = 0xC0;
sbit P40 = P4^0;
sbit P41 = P4^1;
sbit P42 = P4^2;
sbit P43 = P4^3;
sbit P44 = P4^4;
sbit P45 = P4^5;
sbit P46 = P4^6;
sbit P47 = P4^7;

sfr P5   = 0xC8;
sbit P50 = P5^0;
sbit P51 = P5^1;
sbit P52 = P5^2;
sbit P53 = P5^3;
sbit P54 = P5^4;
sbit P55 = P5^5;
sbit P56 = P5^6;
sbit P57 = P5^7;

sfr P6   = 0xE8;
sbit P60 = P6^0;
sbit P61 = P6^1;
sbit P62 = P6^2;
sbit P63 = P6^3;
sbit P64 = P6^4;
sbit P65 = P6^5;
sbit P66 = P6^6;
sbit P67 = P6^7;

sfr P7   = 0xF8;
sbit P70 = P7^0;
sbit P71 = P7^1;
sbit P72 = P7^2;
sbit P73 = P7^3;
sbit P74 = P7^4;
sbit P75 = P7^5;
sbit P76 = P7^6;
sbit P77 = P7^7;

	

void i2c_send_byte(unsigned char _data){
		unsigned char i;
		for(i = 0; i < 8; i++){
			SDA = (_data & 0x80);
			Delay();
			SCL = 1;
			Delay();
			SCL = 0;
			_data = (_data << 1);
		}
	  // stop bit
		SDA = 0;
		Delay();
		SCL = 1;
		Delay();
		SCL = 0;
		SDA = 1;
		Delay();Delay();
}