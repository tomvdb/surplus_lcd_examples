#define LCD_STB   4
#define LCD_RESET 2

void SPI_Send( char Data )
{   
    uint8_t i;
    digitalWrite( LCD_STB, LOW );   
    
  __asm__ __volatile__ ("cbi 0x18,4" "\n\t" :: );

  i = SPSR;
  SPDR = Data;

  // Wait for transmission to complete
  for( ; ( SPSR & 0x80 ) != 0x80  ; );

  __asm__ __volatile__ ("nop" "\n\t" :: );
  __asm__ __volatile__ ("nop" "\n\t" :: );
  __asm__ __volatile__ ("sbi 0x18,4" "\n\t" :: );    
  
    digitalWrite( LCD_STB, HIGH );
}

void SPI_Send_37uS_Delay(char Data)
{
    SPI_Send(Data);
    delayMicroseconds(37);
}


void LCD_Init (void)
{
    delay(50);

    // This does the basic setup of the controller:
    SPI_Send_37uS_Delay(0x38);  //function set
    SPI_Send_37uS_Delay(0x3C);  //function set
    SPI_Send_37uS_Delay(0x40); //set cgram 0

    // R bias
    SPI_Send_37uS_Delay(0x06);

    SPI_Send_37uS_Delay(0x38);  
    SPI_Send_37uS_Delay(0x06);
    SPI_Send_37uS_Delay(0x0C);
    SPI_Send_37uS_Delay(0x40); 
    SPI_Send_37uS_Delay(0x3C);
    SPI_Send_37uS_Delay(0x80);
    SPI_Send_37uS_Delay(0x00);

    // This sets the icon characters all off:
    SPI_Send_37uS_Delay(0x38); 
    SPI_Send_37uS_Delay(0x40);
    SPI_Send_37uS_Delay(0x3C); 
    SPI_Send_37uS_Delay(0x85); 
    SPI_Send(0x00);
    SPI_Send(0x00);
    SPI_Send(0x00);
    SPI_Send(0x00);
    SPI_Send(0x00);
    SPI_Send(0x00);
}

void LCD_Write( char Line , char *Data )
{
    uint8_t i;

    SPI_Send_37uS_Delay(0x38);

    if ( Line == 1 ) 
        SPI_Send_37uS_Delay(0x80);
    else
        SPI_Send_37uS_Delay(0x80 + 0x40);

    SPI_Send_37uS_Delay(0x3C);
    SPI_Send_37uS_Delay(0x8F);
    SPI_Send(0x00);

    for ( i = 0 ; i < 15 ; i++ )
        {
            if ( Data[i] == 0 )
            break;
            SPI_Send(Data[i]);
        }

    for ( ; i < 15 ; i++ )
        SPI_Send(' ');
}

const uint8_t Sig_Tab[6][6] = {  {0x04,0x00,0x00,0x00,0x00,0x00},
              {0x04,0x04,0x00,0x00,0x00,0x00},
              {0x04,0x04,0x04,0x00,0x00,0x00},
              {0x04,0x04,0x04,0x04,0x00,0x00},
              {0x04,0x04,0x04,0x04,0x04,0x00},
              {0x04,0x04,0x04,0x04,0x04,0x04} };

const uint8_t Batt_Tab[6][6] = { {0x10,0x00,0x00,0x00,0x00,0x00},
              {0x10,0x00,0x00,0x10,0x00,0x00},
              {0x10,0x00,0x10,0x10,0x00,0x00},
              {0x10,0x10,0x10,0x10,0x00,0x00},
              {0x10,0x10,0x10,0x10,0x10,0x00},
              {0x10,0x10,0x10,0x10,0x10,0x10} }; 

void LCD_Icons( uint8_t Batt , uint8_t Sig, uint8_t plug, uint8_t down, uint8_t up, uint8_t phone, uint8_t env )
{
SPI_Send_37uS_Delay(0x38);
SPI_Send_37uS_Delay(0x40);
SPI_Send_37uS_Delay(0x3C);
SPI_Send_37uS_Delay(0x85);

SPI_Send_37uS_Delay( Sig_Tab[Sig][0] + Batt_Tab[Batt][0]  ); 
SPI_Send_37uS_Delay( Sig_Tab[Sig][1] + Batt_Tab[Batt][1] + plug );//plug
SPI_Send_37uS_Delay( Sig_Tab[Sig][2] + Batt_Tab[Batt][2] + down ); //call down
SPI_Send_37uS_Delay( Sig_Tab[Sig][3] + Batt_Tab[Batt][3] + up ); //call up
SPI_Send_37uS_Delay( Sig_Tab[Sig][4] + Batt_Tab[Batt][4] + phone  ); //phone
SPI_Send_37uS_Delay( Sig_Tab[Sig][5] + Batt_Tab[Batt][5] + env ); //envelope

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  Serial.println( "Starting" );
  pinMode(LCD_STB, OUTPUT);
  pinMode(LCD_RESET, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, INPUT);
  pinMode(13, OUTPUT);

  digitalWrite( LCD_STB, LOW );
  digitalWrite( LCD_RESET, HIGH);
  
  SPCR  = 0x5E;

  int enablePlug = 1;
  
  LCD_Init(); 
  LCD_Write(1,"where is my" );
  LCD_Write(2,"beer" );
  LCD_Icons(5, 5, 0x08, 0x08, 0x08, 0x08, 0x08);


}


void loop() {
}
