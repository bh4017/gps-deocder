// Project:			GPS Decoder
// Program:			GPS_Firmware
// Version:			V0.3D
// Author:			Brian J Hoskins
// Date:			September 2010

#include <system.h>
#include <string.h>

// SETUP CONFIG WORD
#pragma DATA _CONFIG1, _FCMEN_OFF & _IESO_OFF & _CLKOUTEN_OFF & _BOREN_OFF & _CPD_OFF & _CP_OFF & _MCLRE_ON & _PWRTE_OFF & _WDTE_OFF & _FOSC_INTOSC
#pragma DATA _CONFIG2, _LVP_OFF & _DEBUG_OFF & _STVREN_OFF & _VCAPEN_OFF & _WRT_OFF & _PLLEN_OFF

// SETUP CLOCK SPEED
#pragma CLOCK_FREQ 16000000						// Set for 16MHz clock.  This is used for delay routines!!!

// BEGIN Definitions
#define LCD_E							portc.5	// LCD Enable Pin
#define LCD_RS							portc.4	// LCD Command / Data pin
#define LCD_K							portc.3	// LCD Backlight cathode
#define LCD_RW							portc.2 // LCD Read/Write Pin
#define LCD_DATA						porta	// LCD Data Pins
#define LCD_DATA_DIR					trisa	// LCD Data Bus Direction				
#define Line1							0		// LCD Cursor value for Line1
#define Line2							40		// LCD Cursor value for Line2
#define Line3							20		// LCD Cursor value for Line3
#define Line4							60		// LCD Cursor value for Line4
#define SetFunction_8bit_1Line_5x7		0x30	// LCD Function Set 8Bit interface, 1 line, 5x7 Font
#define SetFunction_8bit_2Line_5x7		0x38	// LCD Function Set 8Bit interface, 2 line, 5x7 Font
#define LCD_Cursor_Inc_Mode				0x06	// LCD Set the cursor to increment mode
#define LCD_Cursor_Dec_Mode				0x04	// LCD Set the cursor to decrement mode
#define LCD_DispON_CursorON_BlinkON		0x0F	// LCD Display ON, Cursor ON, Blinking Cursor ON
#define LCD_DispON_CursorON_BlinkOFF	0x0E	// LCD Display ON, Cursor ON, Blinking Cursor OFF
#define LCD_DispON_CursorOFF			0x0C	// LCD Display ON, Cursor OFF
#define LCD_DispOFF						0x08	// LCD Display OFF
#define LCD_Home						0x02	// LCD Home command
#define LCD_Cursor_Shift_Right			0x14	// LCD Shift Cursor Right command
#define LCD_Cursor_Shift_Left			0x10	// LCD Shift Cursor Left command		
#define LCD_Clear						0x01	// LCD Clear Screen command
#define No_Fix							portb.0 // Output pin for No Fix LED
#define Fix2D							portb.1	// Output pin for 2D Fix LED
#define Fix3D							portb.2	// Output pin for 3D Fix LED
// END Definitions

// BEGIN Custom Characters	******************************************************************************************************

// ATTENTION - COMMENT OUT THIS SECTION IF CUSTOM CHARACTERS ARE NOT USED.  OTHERWISE YOU WILL WASTE SOME ROM.
/*rom char *Custom_Char =	
						{
							0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// Put your character data for CGRAM 0 here
							0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// Put your character data for CGRAM 1 here
							0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// Put your character data for CGRAM 2 here
							0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// Put your character data for CGRAM 3 here
							0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Put your character data for CGRAM 4 here
							0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	// Put your character data for CGRAM 5 here
						} ;										
// END Custom Characters	******************************************************************************************************
*/

// BEGIN Declarations
rom char *TitleLine1 = "ACW Test Engineering" ;
rom char *TitleLine2 = "    GPS Analyser" ;
rom char *TitleLine3 = "" ;
rom char *TitleLine4 = "  V1.0 August 2010" ;
rom char *FrameError = "Framing Error!" ;
rom char *TitleLat = "00 deg 00.0000' *" ;
rom char *TitleLon = "000 deg 00.0000' *" ;
rom char *TitleTime = "00:00:00    00/00/00" ;
rom char *TitleSat = "SatV: ** SatU: **" ;
bit NewNMEA = 0 ;
bit FrameErr = 0 ;
bit process_it = 0 ;
char RS232 = 0 ;
char GPS_Data_Type[6] = 0 ;
char GPRMC[65] ;
char GPGGA[75] ;
char GPGSA[65] ;
bit RMC_Ready = 0;
bit GGA_Ready = 0;
bit GSA_Ready = 0;

// END Declarations

// BEGIN Program

void lcd_busy ( void )
{
	bit mode =  LCD_RS ;							// Save the current RS Mode
	trisa = 1 ;										// Set LCD Data Bus as input so we can read bit 7 (Busy Flag) later
	LCD_RS = 0 ;									// Enter Command Mode	
	LCD_RW = 1 ;									// Now we are reading
	LCD_E = 1 ;
	delay_ms ( 1 ) ;
	while ( porta.7 )
	{
		LCD_E = 0 ;
		LCD_E = 1 ;
		delay_ms ( 2 ) ;
	}
	LCD_E = 0 ;
	trisa = 0 ;										// Restore LCD Data Bus to output
	LCD_RS = mode ;
	LCD_RW = 0 ;
	return ;
}

void lcd_command ( char command )
{
	LCD_RS = 0 ;									// Set RS pin for command mode.
	LCD_RW = 0 ;									// Set Read/Write pin for write mode.
	LCD_DATA = command ;							// Put command on LCD Data pins.
	LCD_E = 1 ;										// Send Data.
	LCD_E = 0 ;										// Return E pin low	delay
	asm nop ;										// Make sure E stays low for at least 1 instruction cycle.  Just to be sure.
	lcd_busy () ;									// Wait for Busy flag
}

void lcd_print ( char cursor, rom char *message )
{
	char i ;
	lcd_command ( LCD_Home ) ;						// Set cursor to home position
	for ( i = cursor; i>0; i=i-1 )					// If cursor > 0 perform a cursor shift
	{
		lcd_command ( LCD_Cursor_Shift_Right ) ;	// Shift cursor right.  Do this "cursor" amount of times!
	}
	i = 0 ;
	LCD_RS = 1 ;									// Set RS pin for character mode.
	while ( message[i] != 0x0 )						// Print characters until string terminator is found
	{
		LCD_DATA = message[i] ;						// Set character data on Port B
		LCD_E = 1 ;									// Send Data
		LCD_E = 0 ;									// Return E pin low
		asm nop ;									// Waste an instruction cycle so that E stays low for >500nS
		lcd_busy () ;								// Wait for Busy flag
		i++ ;
	}
}

void lcd_print2 ( char cursor, char *message )
{
	char i ;
	lcd_command ( LCD_Home ) ;						// Set cursor to home position
	for ( i = cursor; i>0; i=i-1 )					// If cursor > 0 perform a cursor shift
	{
		lcd_command ( LCD_Cursor_Shift_Right ) ;	// Shift cursor right.  Do this "cursor" amount of times!
	}
	i = 0 ;
	LCD_RS = 1 ;									// Set RS pin for character mode.
	while ( message[i] != 0x0 )						// Print characters until string terminator is found
	{
		LCD_DATA = message[i] ;						// Set character data on Port B
		LCD_E = 1 ;									// Send Data
		LCD_E = 0 ;									// Return E pin low
		asm nop ;									// Waste an instruction cycle so that E stays low for >500nS
		lcd_busy () ;								// Wait for Busy flag
		i++ ;
	}
}

void lcd_print_char ( char cursor, char result )
{
	char i ;
	lcd_command ( LCD_Home ) ;						// Set cursor to home position
	for ( i = cursor; i>0; i-- )					// If cursor > 0 perform a cursor shift
	{
		lcd_command ( LCD_Cursor_Shift_Right ) ;	// Shift cursor right.  Do this "cursor" amount of times!
	}
	LCD_RS = 1 ;									// Set RS pin for character mode.
	LCD_DATA = result ;								// Set character data on Port B
	LCD_E = 1 ;										// Send Data
	LCD_E = 0 ;										// Return E pin low
	asm nop ;										// Waste an instruction cycle so that E stays low for >500nS
	lcd_busy () ;									// Wait for Busy flag
}

/*void setup_custom_characters (void)
{
	unsigned char i = 0 ;
	LCD_RS = 0 ;									// Set LCD to Command Mode
	LCD_E = 0 ;										// Set LCD E pin to low - data is transferred on rising edge
	LCD_DATA = 0x40	;								// Address start of CGRAM
	LCD_E = 1 ;
	asm nop ;
	LCD_E = 0 ;										// Waste an instruction cycle to give LCD chance to digest
	lcd_busy () ;									// Wait for Busy flag	
	LCD_RS = 1 ;									// Set LCD to Data Mode
	for ( i = 0 ; i<48 ; i++ )
	{
		LCD_DATA = Custom_Char[i] ;
		LCD_E = 1 ;									// Send the data!
		asm nop ;									// Waste an instruction cycle to make sure E stays high long enough
		LCD_E = 0 ;									// Return E pin low
		lcd_busy () ;								// Wait for Busy flag
	}		
}*/

void lcd_reset (void)
{
	lcd_command ( SetFunction_8bit_1Line_5x7 ) ;	// Send command for 8-line mode.  Do this 3 times (datasheet says to do this)
	delay_ms (20) ;									// Datasheet says allow 4.1mS but we'll allow 20 to be safe
	lcd_command ( SetFunction_8bit_1Line_5x7 ) ;
	delay_ms (20) ;
	lcd_command ( SetFunction_8bit_1Line_5x7 ) ;
	delay_ms (20) ;
	lcd_command ( SetFunction_8bit_2Line_5x7 ) ;	// Send SET FUNCTION command.  Data = 8 bits. Lines = 2. Font = 5x7.
	delay_ms (20) ;
}

void lcd_init (void)
{
	lcd_reset () ;									// Complete reset cycle (defined in datasheet)
	lcd_command ( LCD_DispOFF ) ;					// Turn off LCD
	lcd_command ( LCD_Clear ) ;						// Clear Display Memory
	lcd_command ( LCD_Cursor_Inc_Mode ) ;			// Set Entry Mode to cursor increment, no display shift
	lcd_command ( LCD_Home ) ;						// Return cursor to home position
	lcd_command ( LCD_DispON_CursorOFF ) ;			// Turn on LCD, no cursor displayed
}	

void process_gps (void)
{
	unsigned char i = 0 ;
	process_it = 0 ;								// We don't want to process GPS until the interrupt handler sets it true again.
	NewNMEA = 0 ;									// NewNMEA should be 0 until we've finished with the current sentance.
	while (i<6)										// Collect characters until 8 characters have been collected or the current character = ","
	{
		while (!(process_it)) ;						// Wait here until a new character comes along.  Interrupt handler will set it true when that happens.
		GPS_Data_Type[i] = RS232 ;					// Set character to that seen by the RS232 port and then go to the next character.
		i++ ;
		process_it = 0 ;							// Now we need to wait until interrupt handler sets process_it true again.
	}
	i = 0 ;
	if (!(strcmp( GPS_Data_Type, "GPRMC," )))		// Compare the 5 characters entered into GPS_Data_Type with "GPRMC".  If they match, we've recieved a GPRMC sentance.
	{
		while (RS232 != 0x0D)						// Do until we see a carriage return character
		{
			while (!(process_it)) ;
			GPRMC[i] = RS232 ;
			i++ ;
			process_it = 0 ;
		}
		RMC_Ready = 1 ;

	}
	if (!(strcmp( GPS_Data_Type, "GPGGA," )))
	{
		while (RS232 != 0x0D)						// Do until we see a carriage return character
		{
			while (!(process_it)) ;
			GPGGA[i] = RS232 ;
			i++ ;
			process_it = 0 ;
		}
		GGA_Ready = 1 ;

	}
	if (!(strcmp( GPS_Data_Type, "GPGSA," )))
	{
		while (RS232 != 0x0D)						// Do until we see a carriage return character
		{
			while (!(process_it)) ;
			GPGSA[i] = RS232 ;
			i++ ;
			process_it = 0 ;
		}
		GSA_Ready = 1 ;								// AUTHOR NOTE: THIS REPEATED CODE IS WASTEFUL.  DO SOMETHING ABOUT THIS LATER!
	}
}

void print_gps (void)
{
	RMC_Ready = 0;
	GGA_Ready = 0;
	GSA_Ready = 0;
	unsigned char j = Line1 ;						// Start at the beginning of Line 1 on the LCD.
	if (GPRMC[11] == 0x56)							// If we don't have valid GPRMC data (position 11 is V)
	{
		for (unsigned char i = 0; i<25; i++)
		{
			if ((i==2) || (i==4) || (i==21) || (i==23)) j++;	// at certain points in the string we need to skip a cursor position on the LCD to account for time and date separators already printed.
			if ( i==6)
			{	i=19 ; 
				j=12 ;
			} 										// once i gets to 5 we've finished the time, jump to the date position in the string.
			lcd_print_char (j, GPRMC[i]);
			j++ ;
		}
		lcd_print ( Line2, TitleLat ) ;
		lcd_print ( Line3, TitleLon ) ;
		lcd_print ( Line4, TitleSat ) ;
	}

		if (GPRMC[11] == 0x41) 							// if position 11 is an A then we have valid GPS data.
	{
		// PRINT DATE AND TIME
		for (unsigned char i = 0; i<50; i++)
		{
			if ((i==2) || (i==4) || (i==46) || (i==48))
				{
					j++;							// at certain points in the string we need to skip a cursor position on the LCD to account for time and date separators already printed.
				}
			if ( i==6)
			{	i=44 ; 
				j=12 ;
			} 										// once i gets to 5 we've finished the time, jump to the date position in the string.
			lcd_print_char (j, GPRMC[i]);
			j++ ;
		}
	
		// PRINT LATITUDE DATA
		j = Line2 ;
		for (i = 13; i<24; i++)
		{
			if (i == 15)
				{
					j = 47;							// at certain points in the string we need to skip a cursor position on the LCD to account for time and date separators already printed.
				}
			if (i==22)
				{	
					i++ ; 
					j=56 ;
				} 										// once i gets to 5 we've finished the time, jump to the date position in the string.
			lcd_print_char (j, GPRMC[i]);
			j++ ;	
		}
		
		// PRINT LONGITUDE DATA
		j = Line3 ;
		for (i = 25; i<37; i++)
		{
			if (i == 28)
				{
					j = 28;							// at certain points in the string we need to skip a cursor position on the LCD to account for time and date separators already printed.
				}
			if (i==35)
				{	
					i++ ; 
					j=37 ;
				} 										// once i gets to 5 we've finished the time, jump to the date position in the string.
			lcd_print_char (j, GPRMC[i]);
			j++ ;	
		}
		
		// OUTPUT FIX LEDs
		if (GPGSA[2] = 0x31)							// AUTHOR NOTE: MIGHT BE BEST TO CONVERT THIS TO A NUMBER FIRST.
		{
			No_Fix = 1;
			Fix2D, Fix3D = 0;
		}
		if (GPGSA[2] = 0x32)							// AUTHOR NOTE: MIGHT BE BEST TO CONVERT THIS TO A NUMBER FIRST.
		{
			Fix2D = 1;
			No_Fix, Fix3D = 0;
		}
		if (GPGSA[2] = 0x33)							// AUTHOR NOTE: MIGHT BE BEST TO CONVERT THIS TO A NUMBER FIRST.
		{
			Fix3D = 1;
			No_Fix, Fix2D = 0;
		}
			
	}
}

void interrupt (void)
{
	if ( rcsta.FERR == 1 )
	{
		FrameErr = 1 ;
	}
	RS232 = rcreg ;
	if ( RS232 == 0x24 )
	{
		NewNMEA = 1 ;
	}
	
	process_it = 1 ;
	rcreg = 0 ;
}
	
void setup_hardware (void)
{
	osccon = 0b01111000 ;
	adcon0.ADON = 0 ;								// Disable ADC
	ansela = 0x00 ;									// Set Port A for digital I/O (turn analog input off)
	porta = 0x00 ;
	anselb = 0x00 ;									// Set Port B for digital I/O (turn analog input off)
	trisc.TRISC7 = 1 ;								// Set RXDT pin of UART for input
	trisc.TRISC6 = 0 ;								// Set UART TX pin to output (TX not used but set properly to ensure proper operation of UART module)
	trisc.TRISC5 = 0 ;								// Set LCD_E pin as an output
	trisc.TRISC4 = 0 ;								// Set LCD_RS pin as an output
	trisc.TRISC3 = 0 ;								// Set LCD Backlight pin as an output
	trisc.TRISC2 = 0 ;								// Set LCD_RW pin as an output
	trisb.TRISB0 = 0 ;
	trisb.TRISB1 = 0 ;
	trisb.TRISB2 = 0 ;
	trisa = 0x00 ;									// Set PortA for output (LCD data pins)
}

void setup_uart (void)
{
	rcsta.CREN = 1 ;
	txsta.SYNC = 0 ;
	rcsta.SPEN = 1 ;
	spbrgh = 0 ;									// Disable 16-bit BAUD Generator - we want normal speed
	spbrg = 25 ;									// Set for 9600 BAUD (see datasheet for formula!)
	intcon.GIE = 1 ;								// Enable Global interrupts
	intcon.PEIE = 1 ;								// Enable Peripheral interrupts
	pie1.RCIE = 1 ;									// Enable UART Recieve interrupts
}

void main (void)
{
	setup_hardware() ;								// Setup the device peripherals
	delay_ms ( 15 ) ;								// Wait >15mS to ensure that LCD is ready.
	lcd_init () ;									// Initialise the LCD
	lcd_command ( LCD_Clear ) ;						// Clear the screen!
	lcd_print ( Line1, TitleLine1 ) ;				// Send Title Screen
	lcd_print ( Line2, TitleLine2 ) ;
	lcd_print ( Line3, TitleLine3 ) ;
	lcd_print ( Line4, TitleLine4 ) ;
	delay_s (2) ;
	lcd_command ( LCD_Clear ) ;
	lcd_print ( Line1, TitleTime ) ;
	lcd_print ( Line2, TitleLat ) ;
	lcd_print ( Line3, TitleLon ) ;
	lcd_print ( Line4, TitleSat ) ;
	setup_uart () ;

	while (1) 
	{
		while (!(NewNMEA)) ;						// Stay here until there is new data coming in.
		{
			process_gps() ;
			if (RMC_Ready && GSA_Ready && GGA_Ready)// When all the strings are ready it's time to display the data
			{
				print_gps() ;
			}
		}
	}
}