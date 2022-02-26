// sensordef.h

#ifndef  _SENSORDEF_32x32_H
#define  _SENSORDEF_32x32_H



// DEVICE ADRESS
#define  SENSOR_ADDRESS          0x1A
#define  EEPROM_ADDRESS          0x50

// I2C CLOCK
#define CLOCK_SENSOR            1000000
#define CLOCK_EEPROM            400000

// TIMER (for sample rate)
#define NORM_BW                  68.598
#define SAFETY_FAC               0.90
#define E_BW1                    0x0065
#define E_BW2                    0x0066

// ---EEPROM---
// ID
#define  E_ID1                   0x0074
#define  E_ID2                   0x0075
#define  E_ID3                   0x0076
#define  E_ID4                   0x0077
// calibration settings
#define  E_MBIT_CALIB            0x001A
#define  E_BIAS_CALIB            0x001B
#define  E_CLK_CALIB             0x001C
#define  E_BPA_CALIB             0x001D
#define  E_PU_CALIB              0x001E
// user settings
#define  E_MBIT_USER             0x0060
#define  E_BIAS_USER             0x0061
#define  E_CLK_USER              0x0062
#define  E_BPA_USER              0x0063
#define  E_PU_USER               0x0064
// VDD
#define  E_VDDTH1_1              0x0026
#define  E_VDDTH1_2              0x0027
#define  E_VDDTH2_1              0x0028
#define  E_VDDTH2_2              0x0029
#define E_VDDSCGRAD             0x004E
#define E_VDDSCOFF              0x004F
// PTAT TH
#define E_PTATTH1_1             0x003C
#define E_PTATTH1_2             0x003D
#define E_PTATTH2_1             0x003E
#define E_PTATTH2_2             0x003F
// PTAT Gradient
#define E_PTATGR_1              0x0034
#define E_PTATGR_2              0x0035
#define E_PTATGR_3              0x0036
#define E_PTATGR_4              0x0037
// PTAT OFFSET
#define E_PTATOFF_1             0x0038
#define E_PTATOFF_2             0x0039
#define E_PTATOFF_3             0x003A
#define E_PTATOFF_4             0x003B
// PixCmin/PixCmax
#define E_PIXCMIN_1             0x0000
#define E_PIXCMIN_2             0x0001
#define E_PIXCMIN_3             0x0002
#define E_PIXCMIN_4             0x0003
#define E_PIXCMAX_1             0x0004
#define E_PIXCMAX_2             0x0005
#define E_PIXCMAX_3             0x0006
#define E_PIXCMAX_4             0x0007
// GlobalOff/GlobalGain
#define E_GLOBALOFF             0x0054
#define E_GLOBALGAIN_1          0x0055
#define E_GLOBALGAIN_2          0x0056
// start address for arrays
#define E_VDDCOMPGRAD             0x0340
#define E_VDDCOMPOFF              0x0540
#define E_THGRAD                  0x0740
#define E_THOFFSET                0x0F40
#define E_PIJ                     0x1740
#define E_DEADPIXADR              0x0080
#define E_DEADPIXMASK             0x00B0

// other
#define E_NROFDEFPIX            0x007F
#define E_GRADSCALE             0x0008
#define E_TABLENUMBER1          0x000B
#define E_TABLENUMBER2          0x000C
#define E_EPSILON               0x000D
#define E_ARRAYTYPE             0x0022
#define E_IP                    0x021C
#define E_SUBNET                0x0212
#define E_MAC                   0x0216



// ---Sensor---
// write only
#define CONFIGURATION_REGISTER  0x01
#define TRIM_REGISTER1          0x03
#define TRIM_REGISTER2          0x04
#define TRIM_REGISTER3          0x05
#define TRIM_REGISTER4          0x06
#define TRIM_REGISTER5          0x07
#define TRIM_REGISTER6          0x08
#define TRIM_REGISTER7          0x09
// read only
#define STATUS_REGISTER         0x02
#define TOP_HALF                0x0A
#define BOTTOM_HALF             0x0B


// --- LOOKUP TABLE---


#define ReadToFromTable
#ifdef ReadToFromTable
//#define HTPA32x32dR2L1_6HiGe_Gain3k3
		// #define HTPA32x32dR2L2_1SiF5_0_N2
		#define HTPA32x32dR2L2_1HiSiF5_0_Gain3k3			//is used for SensorRev. 1 onwards	
		//#define HTPA32x32dR2L1k8_0k7HiGe		
		//#define HTPA32x32dR2L1k8_0k7HiGe_TaExtended	//same like the above but with a larger working ambient temperature range
		// #define HTPA32x32dR2L2_1HiSiF5_0_Precise	
		// #define HTPA32x32dR2L2_1HiSiF5_0_Gain3k3_Extended
		//#define HTPA32x32dR2L2_85Hi_Gain3k3		
		//#define HTPA32x32dR2L3_6HiSi_Rev1_Gain3k3
		//#define HTPA32x32dR2L3_6HiSi_Rev1_Gain3k3_TaExtended	//same like the above but with a larger working ambient temperature range
		//#define HTPA32x32dR2L5_0HiGeF7_7_Gain3k3	
		//#define HTPA32x32dR2L5_0HiGeF7_7_Gain3k3_TaExtended	//same like the above but with a larger working ambient temperature range
		// #define HTPA32x32dR2L5_0HiGeF7_7_Gain3k3_Fever			//higher resolution (not higher accuracy) but limited to Ta between +5 and +50 �C
		//#define HTPA32x32dR2L7_0HiSi_Gain3k3
 
 
   #ifdef HTPA32x32dR2L5_0HiGeF7_7_Gain3k3_Fever
	 	#define TABLENUMBER		113
		#define PCSCALEVAL		100000000 //327000000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	19
		#define NROFADELEMENTS 	1595	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	25		//dK
		#define ADEQUIDISTANCE	8		//dig
		#define ADEXPBITS		3		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1024
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
    #endif		
	
    #ifdef HTPA32x32dR2L5_0HiGeF7_7_Gain3k3
	 	#define TABLENUMBER		113
		#define PCSCALEVAL		100000000 //327000000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	1595	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1024
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
    #endif	
	
    #ifdef HTPA32x32dR2L5_0HiGeF7_7_Gain3k3_TaExtended
	 	#define TABLENUMBER		113
		#define PCSCALEVAL		100000000 //327000000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	12
		#define NROFADELEMENTS 	1595	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1024
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
    #endif		

    #ifdef HTPA32x32dR2L1_6HiGe_Gain3k3
	 	#define TABLENUMBER		119
		#define PCSCALEVAL		100000000 //327000000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	1595	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1024
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
    #endif	

    #ifdef HTPA32x32dR2L2_1SiF5_0_N2
	 	#define TABLENUMBER		130
		#define PCSCALEVAL		100000000 //327000000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	1595	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		192
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
    #endif	
	
    #ifdef HTPA32x32dR2L2_1HiSiF5_0_Gain3k3
	 	#define TABLENUMBER		114
		#define PCSCALEVAL		100000000 //327000000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	1595	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1024
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
		#define MBITTRIMDefault 0x2C	//use REF_CAL=2 here. Table does not match, so GlobalGain ist set to 50 % to compensate this.
		#define SensRv 2				//Sensor Revision is set to 1 (Redesign)
    #endif	
	
    #ifdef HTPA32x32dR2L2_1HiSiF5_0_Gain3k3_Extended
	 	#define TABLENUMBER		114
		#define PCSCALEVAL		100000000 //327000000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	12
		#define NROFADELEMENTS 	1595	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1792
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
    #endif	
 	
    #ifdef HTPA32x32dR2L2_1HiSiF5_0_Precise
	 	#define TABLENUMBER		116
		#define PCSCALEVAL		100000000 //327000000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	22
		#define NROFADELEMENTS 	1000	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	50		//dK
		#define ADEQUIDISTANCE	32		//dig
		#define ADEXPBITS		5		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1024
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
    #endif	
	
    #ifdef HTPA32x32dR2L2_85Hi_Gain3k3
	 	#define TABLENUMBER		127
		#define PCSCALEVAL		100000000 //327000000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	1595	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1024
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
    #endif	

    #ifdef HTPA32x32dR2L3_6HiSi_Rev1_Gain3k3
	 	#define TABLENUMBER		117
		#define PCSCALEVAL		100000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	1595	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1024
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
	#endif	
	
  #ifdef HTPA32x32dR2L3_6HiSi_Rev1_Gain3k3_TaExtended
	 	#define TABLENUMBER		117
		#define PCSCALEVAL		100000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	12
		#define NROFADELEMENTS 	1595	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1024
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
	#endif		
	
    #ifdef HTPA32x32dR2L7_0HiSi_Gain3k3
	 	#define TABLENUMBER		118
		#define PCSCALEVAL		100000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	7
		#define NROFADELEMENTS 	1595	//130 possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		640
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
	#endif	
	
    #ifdef HTPA32x32dR2L1k8_0k7HiGe
	 	#define TABLENUMBER		115	
		#define PCSCALEVAL		100000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	10
		#define NROFADELEMENTS 	471		//possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1024
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
    #endif	
	
	#ifdef HTPA32x32dR2L1k8_0k7HiGe_TaExtended
	 	#define TABLENUMBER		115	
		#define PCSCALEVAL		100000000		//PixelConst scale value for table... lower 'L' for (long)
		#define NROFTAELEMENTS 	12
		#define NROFADELEMENTS 	471		//possible due to Program memory, higher values possible if NROFTAELEMENTS is decreased
		#define TAEQUIDISTANCE	100		//dK
		#define ADEQUIDISTANCE	64		//dig
		#define ADEXPBITS		6		//2^ADEXPBITS=ADEQUIDISTANCE
		#define TABLEOFFSET		1024
		#define EQUIADTABLE		//if defined, ADELEMENTS have to be 2^N quantizied! else more CPU Power is needed
		#ifdef EQUIADTABLE
			#undef FLOATTABLE
		#endif   
		#define MBITTRIMDefault 0x2C
		#define SensRv 1
    #endif	
  
#endif





#endif  // _SENSORDEF_32x32_H
