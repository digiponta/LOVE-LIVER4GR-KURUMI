// GR-KURUMI PicalicoFree Template V1.02 (Based on V1.06 Template)
#include <RLduino78.h>
#define TWI_FREQ 50000L
#include <Wire.h>
#include <math.h>

void set_LED_RGB(void);
void MyInt1( void );
#define DI()      asm("di")
#define EI()      asm("ei")

// Pin 22,23,24 has an LED connected on most Arduino boards.
#define LED_R   22
#define LED_G   23
#define LED_B   24

// 端子         マイコンリソース
// D2           P3 テープLEDの制御  0xFFF03 アドレス
// #define TAPELED 2
#define TAPELED 3
#define VOLUME  A0

//              テープLEDの信号を制御
//#define TAPELED_ON      (*(volatile unsigned char *)0xFFF03) = 1
//#define TAPELED_OFF     (*(volatile unsigned char *)0xFFF03) = 0
#define TAPELED_ON      (*(volatile unsigned char *)0xFFF01) = 0x40
#define TAPELED_OFF     (*(volatile unsigned char *)0xFFF01) = 0

//              テープLEDの時間待ち処理
#define TAPELED_ON_WAIT1   dummy2++; dummy++
#define TAPELED_ON_WAIT0   for ( dummy = 0; dummy<0; dummy++ ) { ; }; dummy1++
#define TAPELED_OFF_WAIT1  for ( dummy = 0; dummy<0; dummy++ ) { ; }; dummy1++
#define TAPELED_OFF_WAIT0  for ( dummy = 0; dummy<1; dummy++ ) { ; }; dummy1++
#define TAPELED_WAIT       for ( dummy2 = 0; dummy2<100; dummy2++ ) { ; }

#define LEDDISTANCE 40          // テープLEDを時計モードにするA/D値
#define LEDMAX      60          // テープLEDに接続されているLEDの数
//#define LEDMAX      16          // テープLEDに接続されているLEDの数
#define LEDDATAMAX  160         // テープLEDのカラーデータ数

unsigned char sdat[14+7];
#define AXH	0x00
#define AXL	0x01
#define AYH	0x02
#define AYL	0x03
#define AZH	0x04
#define AZL	0x05
#define TMH	0x06
#define TML	0x07
#define WXH	0x08
#define WXL	0x09
#define WYH	0x0a
#define WYL	0x0b
#define WZH	0x0c
#define WZL	0x0d
#define MXH	0x0f
#define MXL	0x10
#define MYH	0x11
#define MYL	0x12
#define MZH	0x13
#define MZL	0x14
#define MST	0x15

void SetLed( void );            //  テープLEDにデータを送る関数

// #define MAG_ADDR  0x0E //7-bit address for the MAG3110, doesn't change
#define MAG_ADDR  0x0C //7-bit address for the MPU-9250, doesn't change

void config(void);            // turn the MAG3110 on
void print_values(void);
void readAll(void);
int readmx(void);
int readmy(void);
int readmz(void);

long xx, yy, zz;
long xx_min = 0x7fffffff;
long xx_max = 0x80000000;
long yy_min = 0x7fffffff;
long yy_max = 0x80000000;
long zz_min = 0x7fffffff;
long zz_max = 0x80000000;

int gAd0value;

//  テープLED LEDMAX分のバッファ
uint8_t  gtLedData[ LEDMAX ][ 3 ] =
{
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,

     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,

     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,

     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,

     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,

     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x00,   0x00,   0x00 } 

};


//  テープLED 色を変化させるためのデータテーブル 160個分
uint8_t  gtLedColorData[ LEDDATAMAX ][ 3 ] =
{   //  G       R       B
     { 0x00,   0x10,   0x00 } ,     //  1
     { 0x00,   0x20,   0x00 } ,
     { 0x00,   0x30,   0x00 } ,
     { 0x00,   0x40,   0x00 } ,
     { 0x00,   0x50,   0x00 } ,
     { 0x00,   0x60,   0x00 } ,
     { 0x00,   0x70,   0x00 } ,
     { 0x08,   0x80,   0x00 } ,
     { 0x00,   0x90,   0x00 } ,
     { 0x08,   0xa0,   0x00 } ,
     { 0x00,   0xb0,   0x00 } ,
     { 0x08,   0xc0,   0x00 } ,
     { 0x00,   0xd0,   0x00 } ,
     { 0x08,   0xe0,   0x00 } ,
     { 0x00,   0x00,   0x10 } ,
     { 0x00,   0x00,   0x20 } ,

     { 0x00,   0x00,   0x30 } ,     // 2
     { 0x00,   0x00,   0x40 } ,
     { 0x00,   0x00,   0x50 } ,
     { 0x00,   0x00,   0x60 } ,
     { 0x00,   0x00,   0x70 } ,
     { 0x00,   0x00,   0x80 } ,
     { 0x00,   0x00,   0x90 } ,
     { 0x00,   0x00,   0xa0 } ,
     { 0x00,   0x00,   0xb0 } ,
     { 0x00,   0x00,   0xc0 } ,
     { 0x00,   0x00,   0xd0 } ,
     { 0x00,   0x00,   0xe0 } ,
     { 0x10,   0x00,   0x10 } ,
     { 0x20,   0x00,   0x20 } ,
     { 0x30,   0x00,   0x30 } ,
     { 0x40,   0x00,   0x40 } ,

     { 0x50,   0x00,   0x50 } ,     // 3
     { 0x60,   0x00,   0x60 } ,
     { 0x70,   0x00,   0x70 } ,
     { 0x80,   0x00,   0x80 } ,
     { 0x90,   0x00,   0x90 } ,
     { 0xa0,   0x00,   0xa0 } ,
     { 0xb0,   0x00,   0xb0 } ,
     { 0xc0,   0x00,   0xc0 } ,
     { 0xd0,   0x00,   0xd0 } ,
     { 0xe0,   0x00,   0xe0 } ,
     { 0x00,   0x10,   0x10 } ,
     { 0x00,   0x20,   0x20 } ,
     { 0x00,   0x30,   0x30 } ,
     { 0x00,   0x40,   0x40 } ,
     { 0x00,   0x50,   0x50 } ,
     { 0x00,   0x60,   0x60 } ,

     { 0x00,   0x70,   0x70 } ,     // 4
     { 0x00,   0x80,   0x80 } ,
     { 0x00,   0x90,   0x90 } ,
     { 0x00,   0xa0,   0xa0 } ,
     { 0x00,   0xb0,   0xb0 } ,
     { 0x00,   0xc0,   0xc0 } ,
     { 0x00,   0xd0,   0xd0 } ,
     { 0x00,   0xe0,   0xe0 } ,
     { 0x10,   0x10,   0x00 } ,
     { 0x20,   0x20,   0x00 } ,
     { 0x30,   0x30,   0x00 } ,
     { 0x40,   0x40,   0x00 } ,
     { 0x50,   0x50,   0x00 } ,
     { 0x60,   0x60,   0x00 } ,
     { 0x70,   0x70,   0x00 } ,
     { 0x80,   0x80,   0x00 } ,

     { 0x90,   0x90,   0x00 } ,     // 5
     { 0xa0,   0xa0,   0x00 } ,
     { 0xb0,   0xb0,   0x00 } ,
     { 0xc0,   0xc0,   0x00 } ,
     { 0xe0,   0xe0,   0x00 } ,
     { 0xf0,   0xf0,   0x00 } ,
     { 0xe0,   0xb0,   0x08 } ,
     { 0xd0,   0x90,   0x10 } ,
     { 0xc0,   0x80,   0x18 } ,
     { 0xb8,   0x70,   0x20 } ,
     { 0xb0,   0x60,   0x30 } ,
     { 0xa8,   0x58,   0x40 } ,
     { 0xa0,   0x50,   0x38 } ,
     { 0x98,   0x48,   0x30 } ,
     { 0x90,   0x40,   0x20 } ,
     { 0x90,   0x38,   0x10 } ,

     { 0x88,   0x30,   0x20 } ,     // 6
     { 0x80,   0x28,   0x28 } ,
     { 0x78,   0x20,   0x30 } ,
     { 0x70,   0x18,   0x38 } ,
     { 0x68,   0x10,   0x40 } ,
     { 0x60,   0x20,   0x48 } ,
     { 0x58,   0x30,   0x50 } ,
     { 0x50,   0x40,   0x60 } ,
     { 0x48,   0x50,   0x68 } ,
     { 0x40,   0x60,   0x70 } ,
     { 0x50,   0x70,   0x78 } ,
     { 0x48,   0x80,   0x80 } ,
     { 0x40,   0x90,   0x88 } ,
     { 0x30,   0xa0,   0x90 } ,
     { 0x20,   0xb0,   0xa0 } ,
     { 0x10,   0xc0,   0xb0 } ,

     { 0x20,   0xd0,   0xc0 } ,     // 7
     { 0x30,   0xe0,   0xd0 } ,
     { 0x40,   0xff,   0xe0 } ,
     { 0x50,   0x90,   0xf0 } ,
     { 0x60,   0x80,   0x80 } ,
     { 0x70,   0x78,   0x78 } ,
     { 0x80,   0x70,   0x70 } ,
     { 0x90,   0x68,   0x68 } ,
     { 0xa0,   0x60,   0x60 } ,
     { 0xb0,   0x58,   0x58 } ,
     { 0xc0,   0x50,   0x50 } ,
     { 0xa0,   0x48,   0x48 } ,
     { 0x80,   0x40,   0x40 } ,
     { 0x60,   0x30,   0x30 } ,

     { 0x4f,   0x20,   0x20 } ,     // 8
     { 0x20,   0x10,   0x10 } ,
     { 0x00,   0x00,   0x00 } ,
     { 0x08,   0x08,   0x08 } ,
     { 0x10,   0x10,   0x10 } ,
     { 0x18,   0x18,   0x18 } ,
     { 0x20,   0x20,   0x20 } ,
     { 0x28,   0x28,   0x28 } ,
     { 0x30,   0x30,   0x30 } ,
     { 0x38,   0x38,   0x38 } ,
     { 0x40,   0x40,   0x40 } ,
     { 0x50,   0x50,   0x50 } ,
     { 0x60,   0x60,   0x60 } ,
     { 0x70,   0x70,   0x70 } ,
     { 0x80,   0x80,   0x80 } ,
     { 0x90,   0x90,   0x90 } ,

     { 0xb0,   0xb0,   0xb0 } ,     // 9
     { 0xe0,   0xe0,   0xe0 } ,
     { 0xff,   0xff,   0xff } ,
     { 0xff,   0xff,   0xff } ,
     { 0xe0,   0xe0,   0xe0 } ,
     { 0xd0,   0xd0,   0xd0 } ,
     { 0xc0,   0xc0,   0xc0 } ,
     { 0xb0,   0xb0,   0xb0 } ,
     { 0xa8,   0xa8,   0xa8 } ,
     { 0xa0,   0xa0,   0xa0 } ,
     { 0x98,   0x98,   0x98 } ,
     { 0x90,   0x90,   0x90 } ,
     { 0x88,   0x88,   0x88 } ,
     { 0x80,   0x80,   0x80 } ,
     { 0x78,   0x78,   0x78 } ,
                    
     { 0x70,   0x70,   0x70 } ,     // 10
     { 0x68,   0x68,   0x68 } ,
     { 0x60,   0x60,   0x60 } ,
     { 0x58,   0x58,   0x58 } ,
     { 0x50,   0x50,   0x50 } ,
     { 0x48,   0x48,   0x48 } ,
     { 0x40,   0x40,   0x40 } ,
     { 0x38,   0x38,   0x38 } ,
     { 0x30,   0x30,   0x30 } ,
     { 0x28,   0x28,   0x28 } ,
     { 0x20,   0x20,   0x20 } ,
     { 0x18,   0x18,   0x18 } ,
     { 0x10,   0x10,   0x10 } ,
     { 0x08,   0x08,   0x08 } ,
     { 0x04,   0x04,   0x04 } ,
     { 0x00,   0x00,   0x00 } 
};

int led_rgb = 8;
int led_rgb_prev = 8;
int led_r = 0;
int led_g = 0;
int led_b = 0;

void set_LED_RGB(void)
{

	if (led_rgb != led_rgb_prev) 
	switch(led_rgb) {
	case 0:
		analogWrite( LED_R, led_r = 255 ); 
		analogWrite( LED_G, led_g = 255 ); 
		analogWrite( LED_B, led_b = 255 ); 
		break;
	case 1:
		analogWrite( LED_R, led_r = 255 ); 
		analogWrite( LED_G, led_g = 255 ); 
		analogWrite( LED_B, led_b = 0 ); 
		break;
	case 2:
		analogWrite( LED_R, led_r = 255 ); 
		analogWrite( LED_G, led_g = 0 ); 
		analogWrite( LED_B, led_b = 255 ); 
		break;
	case 3:
		analogWrite( LED_R, led_r = 255 ); 
		analogWrite( LED_G, led_g = 0 ); 
		analogWrite( LED_B, led_b = 0 ); 
		break;
	case 4:
		analogWrite( LED_R, led_r = 255 ); 
		analogWrite( LED_G, led_g = 94 ); 
		analogWrite( LED_B, led_b = 0 ); 
		break;
	case 5:
		analogWrite( LED_R, led_r = 0 ); 
		analogWrite( LED_G, led_g = 255 ); 
		analogWrite( LED_B, led_b = 0 ); 
		break;
	case 6:
		analogWrite( LED_R, led_r = 0); 
		analogWrite( LED_G, led_g = 0 ); 
		analogWrite( LED_B, led_b = 255 ); 
		break;
	case 7:
	case 8:
		analogWrite( LED_R, led_r = 255 ); 
		analogWrite( LED_G, led_g = 94 ); 
		analogWrite( LED_B, led_b = 94 ); 
	default:
		;
	}
	led_rgb_prev = led_rgb;
}

void MyInt1( void )
{
	led_rgb = led_rgb + 1;
	if (led_rgb > 9) led_rgb = 0;
}

// the setup routine runs once when you press reset:
void setup() 
{                
  pinMode( LED_R, OUTPUT );
  pinMode( LED_G, OUTPUT );
  pinMode( LED_B, OUTPUT );

  set_LED_RGB();

  pinMode( TAPELED, OUTPUT );

//  pinMode( 7, INPUT_PULLUP );
//  pinMode( 8, INPUT_PULLUP );

  Serial.begin(115200);

  delay( 3000 );

  Wire.begin();        // join i2c bus (address optional for master)
  delay( 1000 );
  config();            // turn the MAG3110 on
  delay( 1000 );
  Serial.print("GO!¥n¥r");
  
  attachInterrupt( 0, MyInt1, FALLING );

}


unsigned int cntLoop = 0;

// the loop routine runs over and over again forever:
void loop() 
{
    uint8_t datg, datr, datb;
    uint16_t datadrs;

    // ポテンションメータの直値をgAdvalへ代入
//    gAd0value = analogRead( VOLUME ) / 8;
//    gAd0value = ( cntLoop++ ) % LEDDATAMAX; 
    print_values();

	if ( led_rgb == 8 ) {
	    datadrs = gAd0value;
	    if ( datadrs  > ( LEDDATAMAX - 1 ) )
	    {
	        datadrs = LEDDATAMAX - 1;
	    }
	    datg = gtLedColorData[ datadrs ][ 0 ];
	    gtLedData[ 0 ][ 0 ] = ((led_g = xx) * (cntLoop % 9)) / 4 ; // = datg;
	
	    datr = gtLedColorData[ datadrs ][ 1 ];
	    gtLedData[ 0 ][ 1 ] = ((led_r = yy) * (cntLoop % 9)) / 4; // datr;
	
	    datb = gtLedColorData[ datadrs ][ 2 ];
	    gtLedData[ 0 ][ 2 ] = ((led_b = zz)* (cntLoop++ % 9)) / 4;  // datb;
	} else {
		gtLedData[ 0 ][ 0 ] = (led_g * (cntLoop % 9)) / 8 ;
		gtLedData[ 0 ][ 1 ] = (led_r * (cntLoop % 9)) / 8 ;
		gtLedData[ 0 ][ 2 ] = (led_b * (cntLoop++ % 9)) / 8 ;
	}
	
    SetLed();

  	set_LED_RGB();
  	
    delay( 30 );
}


//  テープLEDにデータを送る
void SetLed( void )
{
    uint8_t datg, datr, datb, ledloop, phase;
    volatile uint8_t dummy;
    volatile uint16_t dummy1;
    volatile uint32_t dummy2;

    // 割り込みを禁止して、データ送信がずれないようにする
    DI();
//         for ( ledloop = 0; ledloop < (LEDMAX); ledloop++ )
#define LEDLENGTH	(16)
    for (phase=0; phase<2; phase++ ) {
//        for ( ledloop = phase; ledloop < (LEDMAX); ledloop+=2 )
        for ( ledloop = phase; ledloop < (LEDLENGTH); ledloop+=2 )
        {
            if ( phase ) {
                datg = gtLedData[ LEDLENGTH - ledloop ][ 0 ];
                datr = gtLedData[ LEDLENGTH - ledloop ][ 1 ];
                datb = gtLedData[ LEDLENGTH - ledloop ][ 2 ];
            } else {
                datg = gtLedData[ ledloop ][ 0 ];
                datr = gtLedData[ ledloop ][ 1 ];
                datb = gtLedData[ ledloop ][ 2 ];
            }
            // 緑LED
            if ( datg & 0x80 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datg & 0x40 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datg & 0x20 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datg & 0x10 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datg & 0x08 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datg & 0x04 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datg & 0x02 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datg & 0x01 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
        
            // 赤LED
            if ( datr & 0x80 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datr & 0x40 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datr & 0x20 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datr & 0x10 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datr & 0x08 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datr & 0x04 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datr & 0x02 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datr & 0x01 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
     
       
            // 青LED
            if ( datb & 0x80 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datb & 0x40 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datb & 0x20 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datb & 0x10 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datb & 0x08 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datb & 0x04 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datb & 0x02 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        
            if ( datb & 0x01 )
            {   // on data send
                TAPELED_ON;
                TAPELED_ON_WAIT1;
                TAPELED_OFF;
                TAPELED_OFF_WAIT1;
            }
            else
            {   // off data send
                TAPELED_ON;
                TAPELED_ON_WAIT0;
                TAPELED_OFF;
                TAPELED_OFF_WAIT0;
            }
        }
    }

    // テープLEDのバッファをずらして、流れる表示にする
    for ( ledloop = LEDLENGTH - 1; ledloop > 0; ledloop-- )
//    for ( ledloop = 60 - 1; ledloop > 0; ledloop-- )
    {
        datg = gtLedData[ ledloop - 1 ][ 0 ];
        gtLedData[ ledloop ][ 0 ] = datg;

        datr = gtLedData[ ledloop - 1 ][ 1 ];
        gtLedData[ ledloop ][ 1 ] = datr;

        datb = gtLedData[ ledloop - 1 ][ 2 ];
        gtLedData[ ledloop ][ 2 ] = datb;
    }

    EI();
}


void config(void)
{

  int st;
  Wire.beginTransmission(0x69); // transmit to device 0x69 (AD0=H)
  Wire.write(0x75);              // z MSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom(0x69, 1); // request 1 byte
  while(Wire.available())    // slave may send less than requested
  { 
    st = Wire.read(); // receive the byte
  }
  Wire.endTransmission();        // stop transmitting
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop

  Serial.print( st );

  Wire.beginTransmission(0x69); // transmit to device 0x0E
  Wire.write(0x6b);              // cntrl register2
  Wire.write(0x00);              // send 0x80, enable auto resets
  Wire.endTransmission();        // stop transmitting

  delay(15);
 
  Wire.beginTransmission(0x69); // transmit to device 0x0E
  Wire.write(0x37);              // cntrl register2
  Wire.write(0x02);              // send 0x80, enable auto resets
  Wire.endTransmission();        // stop transmitting
  
  delay(15);

  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0C
  Wire.write(0x0a);              // cntrl register1
  Wire.write(0x12);                 // send 0x01, active mode
  Wire.endTransmission();        // stop transmitting
}

void print_values(void)
{
  int cx, cy, cz;
  double len;
  
  readAll();
  
  xx = (long)(cx = readmx())-(xx_max + xx_min)/2;
  yy = (long)(cy = readmy())-(yy_max + yy_min)/2;
  zz = (long)(cz = readmz())-(zz_max + zz_min)/2;


  if (xx < 0) xx = 0;
  if (yy < 0) yy = 0;
  zz = -zz;
  if (zz < 0) zz = 0;

  len = sqrt( xx*xx + yy*yy + zz*zz );
  xx = ((xx * 128) / len); // + 128;
  yy = ((yy * 128) / len); // + 128;
  zz = ((zz * 128) / len); // + 128;

#if 0
  xx = xx/4;
  yy = yy/4;
  zz = zz/4;
#else
  xx = xx/2;
  yy = yy/2;
  zz = zz/2;
#endif

#if 1
  Serial.print("x=");
  Serial.print((int)(256 * sdat[MXH] + sdat[MXL]) ); 
  Serial.print(",");       
  Serial.print("y=");    
  Serial.print((int)(256 * sdat[MYH] + sdat[MYL]) ); 
  Serial.print(",");       
  Serial.print("z=");
  Serial.print((int)(256 * sdat[MZH] + sdat[MZL]) ); 
  Serial.print("\n\r");
  Serial.flush();
#endif
}

void readAll(void)
{
  int i;
	
  Wire.beginTransmission( 0x69 ); // transmit to device 0x0E
  Wire.write(0x3B);              // x MSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom( 0x69, 14); // request 14 byte
  while(Wire.available() < 14);    // slave may send less than requested

  for ( i=0; i<14; i++ )     // slave may send less than requested
  { 
    sdat[i] = Wire.read(); // receive the byte
  }
  
  Wire.beginTransmission( MAG_ADDR ); // transmit to device 0x0E
  Wire.write(0x03);              // x MSB reg
  Wire.endTransmission();       // stop transmitting

  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
 
  Wire.requestFrom(MAG_ADDR, 7); // request 1 byte
  while(Wire.available() < 7);    // slave may send less than requested

  for ( ; i<(14+7); i++ )     // slave may send less than requested
  { 
    sdat[i] = Wire.read(); // receive the byte
  }
}

int readmx(void)
{
  int xl, xh;  //define the MSB and LSB

  xh = sdat[MXH];
  xl = sdat[MXL];

//  int xout = (xl|(xh << 8)); //concatenate the MSB and LSB
  int xout = (xl + (xh * 256)); //concatenate the MSB and LSB
  if (xout > xx_max) xx_max = xout;
  if (xout < xx_min) xx_min = xout;
  return xout;
}

int readmy(void)
{
  int yl, yh;  //define the MSB and LSB

  yh = sdat[MYH];
  yl = sdat[MYL];

//  int yout = (yl|(yh << 8)); //concatenate the MSB and LSB
  int yout = (yl + (yh * 256)); //concatenate the MSB and LSB
  if (yout > yy_max) yy_max = yout;
  if (yout < yy_min) yy_min = yout;
  return yout;
}

int readmz(void)
{
  int zl, zh;  //define the MSB and LSB

  zh = sdat[MZH];
  zl = sdat[MZL];

//  int zout = (zl|(zh << 8)); //concatenate the MSB and LSB
  int zout = (zl + (zh * 256)); //concatenate the MSB and LSB
  if (zout > zz_max) zz_max = zout;
  if (zout < zz_min) zz_min = zout;
  return zout;
}
