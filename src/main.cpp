//
//	pedometer for M5StickC Plus
//		(c) 2022,2023	1YEN Toru
//
//
//	2023/03/04	ver.1.00
//		M5StickC Plus edition
//
// ================================
//	2023/01/14	ver.1.04
//		bug fixed: ped_acl calculation
//
//	2022/08/13	ver.1.02
//		corresponding to OLED display 128x32 (ssd1306)
//
//	2022/07/16	ver.1.00
//		Arduino DUE edition
//


#include	<M5StickCPlus.h>
#include	<m5cp_ps.h>


//#define		UI_SEL_ENG				// u/i: select English
//#define		DEBUG_MODE				// dbg: enable debug mode function


#ifdef		ARDUINO_M5Stick_C_PLUS
#else	// ERROR:
#error	"This MCU board is not supported."
#endif


const	char	prog_name[]="m5cp_pedometer";
const	char	prog_vers[]="1.00";
#ifdef		UI_SEL_ENG
const	char	ttl_NAME[]=" Pedometer";
#else	//	UI_SEL_ENG
const	char	ttl_NAME[]=" \x96\x9c\x95\xe0\x8c\x76" /*  万歩計 */;
#endif	//	UI_SEL_ENG
const	int		max_lbuf=64;			// line buffer size
const	int		ser_BAUD=9600;			// serial monitor [baud]
const	int		ErrNo=0;				// no error
const	int		ErrI2cBusy=-1;			// I2C (Wire1) busy
const	int		ped_msec_ITVL=10;		// ped: sampling rate [ms]
const	int		ped_msec_WK_MAX=2000;	// ped: maximum time for 1 count [ms]
const	int		ped_MAX_BUF=3*1024;		// ped: data buffer (axis*count)
const	int		ped_RNGE=2329 - 562;	// ped: acel range peak to peak
const	int		ped_MAX_MVA=16;			// ped: moving average size
const	int		ped_WK_DECR=0;			// ped: ped_stat_walk
const	int		ped_WK_INCL=1;
const	int		ped_WEIGHT=50;			// ped: weight [kg]
const	int		ped_TALL=160;			// ped: tall [cm]
const	int		ped_STEP=ped_TALL*45/100;	// ped: step width [cm]
const	int		ped_MINU_LOG=1;			// ped: logging interval [minu]
const	int		ped_ELM_LOG=3;			// ped: elements of ped_log[]
const	int		ped_MAX_LOG=ped_ELM_LOG*24*60;	// ped: size of ped_log[]
const	int		sem_HOLD=0;				// thrd: check holder command
const	int		sem_TAKE=1;				// thrd: take semaphore command
const	int		sem_GIVE=2;				// thrd: give semaphore command

const	int		ped_mets[]=				// ped: walking speed vs mets table
{
	// m/h, mets*10, ...
	1,		20,
	3200,	28,
	4000,	30,
	4500,	35,
	5100,	35,
	5600,	43,
	6400,	50,
	7200,	70,
	8000,	83,
	0,		0
};


static	volatile	bool	thrd_upd_screen;	// thrd: update screen command
static	bool	ped_log_mode;			// ped: log mode
static	int		ped_stat_walk;			// ped: walking detection phase
static	int		ped_cnt_walk;			// ped: pedometer counter
static	int		ped_cal_walk;			// ped: walking calories burned [cal]
static	int		ped_spd_walk;			// ped: walking speed [m/h]
static	int		ped_dst_walk;			// ped: walking distance [cm]
static	int		ped_mva_ptr;			// ped: write pointer for mbuf[]
static	int		ped_mva_sum;			// ped: sum for moving average
static	int		ped_acl_max;			// ped: maximum acel in increasing
static	int		ped_acl_min;			// ped: minimum acel in decreasing
static	int		ped_log_prev;			// ped: previously logging minute
static	int		ped_log_ptr;			// ped: pointer to ped_log[]
static	uint32	ped_msec_walk;			// ped: walking time [ms]
static	uint32	ped_msec_pre;			// ped: previous step
static	TaskHandle_t	thrd_htsk;		// thrd: task handle
static	SemaphoreHandle_t	sem_wire1_hdl;	// sem: Wire1 semaphore handle

static	char	lbuf[max_lbuf];			// line buffer
static	int		mbuf[ped_MAX_MVA];		// ped: moving average buffer
static	uint8	ped_rtc[rtc_SIZ_CNT];	// ped: date & time: logging start
static	int		abuf_ptr;				// ped: write pointer for abuf[]
static	int16	abuf[ped_MAX_BUF];		// ped: acel data buffer
static	uint32	ped_log[ped_MAX_LOG];	// ped: logging buffer


bool	sem_wire1 (
int		cmnd,
int		blk_tick=1)
{
	// semaphore for Wire1
	// return: true=holding semaphore / false=not
	int		rtncod;

	// semaphore
	rtncod=false;
	if (xTaskGetCurrentTaskHandle ()==xSemaphoreGetMutexHolder (sem_wire1_hdl))
	{
		// holding semaphore
		if (cmnd==sem_GIVE)
		{
			// release semaphore
			xSemaphoreGive (sem_wire1_hdl);
		}
		else
			rtncod=true;
	}
	else
	{
		// not holding semaphore
		if (cmnd==sem_TAKE)
		{
			// get semaphore
			if (xSemaphoreTake (sem_wire1_hdl,blk_tick))
				rtncod=true;
		}
	}

	return (rtncod);
}


void	draw_screen (void)
{
	// draw screen
	char	nbuf[16];
	char	sbuf[32];

	// walking steps
	tft_locate (8,28);
	tft_color (WHITE,BLACK);
	tft_font_size (0);
#ifdef		UI_SEL_ENG
	tft_kprint ("Steps");
#else	//	UI_SEL_ENG
	tft_kprint ("\x95\xe0\x90\x94" /* 歩数 */);
#endif	//	UI_SEL_ENG
	sprintf (sbuf,"%s",strnum3c (ped_cnt_walk,nbuf));
	tft_locate (24,44);
	tft_color (RED + DARKCYAN);
	tft_font_size (22);
	tft_kprint (sbuf);

	// walking speed
	tft_locate (8,64);
	tft_color (WHITE,BLACK);
	tft_font_size (0);
#ifdef		UI_SEL_ENG
	tft_kprint ("Speed [km/h]");
#else	//	UI_SEL_ENG
	tft_kprint ("\x91\xac\x82\xb3 [km/h]" /* 速さ [km/h] */);
#endif	//	UI_SEL_ENG
	sprintf (sbuf,"%d.%02d  ",
		ped_spd_walk/1000,(ped_spd_walk/10)%100);
	tft_locate (24,80);
	tft_color (RED + DARKCYAN);
	tft_font_size (22);
	tft_kprint (sbuf);

	// walking time
	tft_locate (8,100);
	tft_color (WHITE,BLACK);
	tft_font_size (0);
#ifdef		UI_SEL_ENG
	tft_kprint ("Time [h:minu]");
#else	//	UI_SEL_ENG
	tft_kprint ("\x8e\x9e\x8a\xd4 [\x8e\x9e:\x95\xaa]" /* 時間 [時:分] */);
#endif	//	UI_SEL_ENG
	sprintf (sbuf,"%u:%02u",
		ped_msec_walk/(1000*60)/60,(ped_msec_walk/(1000*60))%60);
	tft_locate (24,116);
	tft_color (RED + DARKCYAN);
	tft_font_size (22);
	tft_kprint (sbuf);

	// walking distance
	tft_locate (8,136);
	tft_color (WHITE,BLACK);
	tft_font_size (0);
#ifdef		UI_SEL_ENG
	tft_kprint ("Distance [m]");
#else	//	UI_SEL_ENG
	tft_kprint ("\x8b\x97\x97\xa3 [m]" /* 距離 [m] */);
#endif	//	UI_SEL_ENG
	sprintf (sbuf,"%s",strnum3c (ped_dst_walk/100,nbuf));
	tft_locate (24,152);
	tft_color (RED + DARKCYAN);
	tft_font_size (22);
	tft_kprint (sbuf);

	// calories burned
	tft_locate (8,172);
	tft_color (WHITE,BLACK);
	tft_font_size (0);
#ifdef		UI_SEL_ENG
	tft_kprint ("Calories [kcal]");
#else	//	UI_SEL_ENG
	tft_kprint ("\x83\x4a\x83\x8d\x83\x8a [kcal]" /* カロリ [kcal] */);
#endif	//	UI_SEL_ENG
	sprintf (sbuf,"%s",strnum3c (ped_cal_walk/1000,nbuf));
	tft_locate (24,188);
	tft_color (RED + DARKCYAN);
	tft_font_size (22);
	tft_kprint (sbuf);
}

void	thrd_draw_screen (
void	*argv)
{
	// draw screen thread function
	//	argv: not used

	// forever
	for (;;)
	{
		// every 500ms
		vTaskDelay (500/portTICK_PERIOD_MS);
		if (!thrd_upd_screen)
			continue;

		// get semaphore
#ifdef		DEBUG_MODE
		uint32	msec=millis ();
#endif	//	DEBUG_MODE
		while (!sem_wire1 (sem_TAKE))
#ifdef		DEBUG_MODE
			if (millis() - msec>10000)
				break;
		if (!sem_wire1 (sem_TAKE))
		{
			// ERROR:
			tft_locate (8,200);
			tft_color (RED,BLACK);
			tft_font_size (0);
			tft_kprint ("ERR: sem blk 10s");
			continue;
		}
#endif	//	DEBUG_MODE
			;
		// draw title
		tft_draw_title (ttl_NAME);
		// release semaphore
		sem_wire1 (sem_GIVE);

		// draw screen
		draw_screen ();
		thrd_upd_screen=false;
	}
}


int		meas_mpu6886_acl (
int16	&ax,
int16	&ay,
int16	&az)
{
	// measure accelerometer vector
	// (ax,ay,az)[LSB], signed 16 bits data for mpu6886-acel
	// return: ErrNo / ErrI2cBusy

	// parameter
	ax=0;
	ay=0;
	az=0;

	// get semaphore
	if (!sem_wire1 (sem_TAKE))
		return (ErrI2cBusy);

	// measure
	M5.Imu.getAccelAdc (&ax,&ay,&az);

	// release semaphore
	sem_wire1 (sem_GIVE);

	return (ErrNo);
}


int		xtoi (
const	char	*shex)
{
	// convert hexadecimal string to integer
	const	char	*str_c;
	const	char	*hex_c;
	int		ihex;
	const	char	*xdigit="0123456789abcdef";

	ihex=0;
	for (str_c=shex; (*str_c)!='\0'; str_c++)
	{
		hex_c=strchr (xdigit,tolower (*str_c));
		if (hex_c!=NULL)
			ihex=ihex*16 + (hex_c - xdigit);
		else
			break;
	}

	return (ihex);
}

void	get_line (void)
{
	// get string from serial communication
	// string: delimited by LF / CRLF
	int		idx;
	int		chr;

	// get until LF
	for (idx=0; idx<max_lbuf - 1; idx++)
	{
		// block until data arrival
		do
		{
			chr=Serial.read ();
		} while (chr==-1);
		// stop by LF
		if (chr=='\n')
			break;

		lbuf[idx]=chr;
	}
	// remove last CR of string(lbuf)
	if (idx>0 && lbuf[idx - 1]=='\r')
		idx--;
	lbuf[idx]='\0';
}


void	setup (void)
{
	// serial
	Serial.begin (ser_BAUD);

	// m5 library
	M5.begin (true,true,false);			// tft, power, serial
	M5.Lcd.setRotation (0);
	M5.Beep.setBeep (2800,50);

	// init
	digitalWrite (LED_BUILTIN, LED_BUILTIN_OFF);
	pinMode (LED_BUILTIN, OUTPUT);
	gpio_set_drive_capability (gpio_num_t (LED_BUILTIN), GPIO_DRIVE_CAP_0);
	tft_clear ();
	tft_font_size (0);
	tft_color (WHITE,BLACK);
	rtc_get_date_time (ped_rtc);
	thrd_upd_screen=true;

	// MPU6886 setting
	// PWR_MGMT_1.DEVICE_RESET=1
	i2c1_write (i2c_MPU6886,0x6b,0x80);
	while (i2c1_read (i2c_MPU6886,0x6b)&0x80)
		;
	// PWR_MGMT_1.GYRO_STANDBY=0, CLKSEL=0b001
	i2c1_write (i2c_MPU6886,0x6b,0x01);
	// ACCEL_CONFIG2=bypass LPF
	i2c1_write (i2c_MPU6886,0x1d,0x08);

	// subthread
	sem_wire1_hdl=xSemaphoreCreateMutex();
	xTaskCreate (
		thrd_draw_screen,				// thread function
		"draw_screen",					// thread name
		2*1024,							// stack size
		NULL,							// parameter
		uxTaskPriorityGet (NULL),		// priority
		&thrd_htsk);					// task handle
	if (thrd_htsk==NULL || sem_wire1_hdl==NULL)
	{
		// ERROR:
		Serial.println ("ERR: could not create task or semaphore");
		tft_locate (8,8);
		tft_kprint ("ERR: could not create task or semaphore");
		M5.Beep.beep ();
		delay (500);
		M5.Beep.mute ();
		for (;;)
			;
	}

	// init moving average
	int		idx;
	for (idx=0; idx<ped_MAX_MVA; idx++)
		mbuf[idx]=0;
	ped_mva_sum=0;
	ped_mva_ptr=0;

	// ready
	Serial.println ("RDY");
}

void	loop (void)
{
	// check serial state
	while (Serial.available ()>0)
	{
		const	char	*str_c;

		// receive
		get_line ();

		// command interpretation
		if (strncmp (lbuf,"log",3)==0)
		{
			// log
			int		idx;

			sprintf (lbuf,"PEDO,%04x,%d",
				(ped_rtc[rtc_YEAR]&rtc_MSK_YEAR) +
					((ped_rtc[rtc_C_MON]&rtc_MSK_C)? 0x1900: 0x2000),
				F_CPU/1000000);
			Serial.println (lbuf);
			Serial.println ("#idx,step,msec");
			for (idx=0; idx<ped_MAX_LOG && ped_log[idx]>0;
				idx += ped_ELM_LOG)
			{
				if (Serial.available ()>0)
				{
					Serial.println ("WRN: log: user canceled");
					break;
				}

				sprintf (lbuf,"%u,%u,%u",
					ped_log[idx],ped_log[idx + 1],ped_log[idx + 2]);
				Serial.println (lbuf);
			}
			Serial.println ("END");
			// response
			Serial.println ("OK: log");
		}
		else if (strncmp (lbuf,"set,",4)==0)
		{
			// set,<walk_step>[,<walk_time_s>]
			str_c=strchr (lbuf,',');
			ped_cnt_walk=atoi (str_c + 1);
			str_c=strchr (str_c + 1,',');
			if (str_c!=NULL)
				ped_msec_walk=atoi (str_c + 1)*1000;
			else
				ped_msec_walk=ped_cnt_walk*(1000*10/15);
			// response
			sprintf (lbuf,"OK: set %d[step] %lu[s]",
				ped_cnt_walk,ped_msec_walk/1000);
			Serial.println (lbuf);
		}
		else if (strncmp (lbuf,"time",4)==0)
		{
			// time
			while (!sem_wire1 (sem_TAKE))
				;
			rtc_get_date_time ();
			sem_wire1 (sem_GIVE);
			// response
			if (rtc_reg[rtc_VL_SEC]&rtc_MSK_VL)
			{
				sprintf (lbuf,"ERR: rtc VL=1 (counter not valid)");
			}
			else
			{
				sprintf (lbuf,"OK: time %04x/%02x/%02x(%s) %02x:%02x:%02x",
					(rtc_reg[rtc_YEAR]&rtc_MSK_YEAR) +
						((rtc_reg[rtc_C_MON]&rtc_MSK_C)? 0x1900: 0x2000),
					rtc_reg[rtc_C_MON]&rtc_MSK_MON,
					rtc_reg[rtc_DAY]&rtc_MSK_DAY,
					rtc_week[rtc_reg[rtc_WEEK]&rtc_MSK_WEEK],
					rtc_reg[rtc_HOUR]&rtc_MSK_HOUR,
					rtc_reg[rtc_MINU]&rtc_MSK_MINU,
					rtc_reg[rtc_VL_SEC]&rtc_MSK_SEC);
			}
			Serial.println (lbuf);
		}
		else if (strncmp (lbuf,"mset,",4)==0)
		{
			// mset,<yyyy>,<mm>,<dd>,<HH>,<MM>,<SS>
			int		yr;
			int		mo;
			int		dy;
			int		hr;
			int		mi;
			int		sc;
			int		wk;

			str_c=strchr (lbuf,',');
			if (str_c!=NULL)
			{
				// <yyyy>
				yr=xtoi (str_c + 1);
				str_c=strchr (str_c + 1,',');
			}
			if (str_c!=NULL)
			{
				// <mm>
				mo=xtoi (str_c + 1);
				mo=mo&rtc_MSK_MON;
				str_c=strchr (str_c + 1,',');
			}
			if (str_c!=NULL)
			{
				// <dy>
				dy=xtoi (str_c + 1);
				dy=dy&rtc_MSK_DAY;
				str_c=strchr (str_c + 1,',');
			}
			if (str_c!=NULL)
			{
				// <hr>
				hr=xtoi (str_c + 1);
				hr=hr&rtc_MSK_HOUR;
				str_c=strchr (str_c + 1,',');
			}
			if (str_c!=NULL)
			{
				// <mi>
				mi=xtoi (str_c + 1);
				mi=mi&rtc_MSK_MINU;
				str_c=strchr (str_c + 1,',');
			}
			if (str_c!=NULL)
			{
				// <sc>
				sc=xtoi (str_c + 1);
				sc=sc&rtc_MSK_SEC;
			}
			if (str_c==NULL)
			{
				// ERROR:
				Serial.println ("ERR: mset syntax");
				break;
			}
			// set date & time
			wk=rtc_day_of_week (bcd2dec (yr),bcd2dec (mo),bcd2dec (dy));
			if (yr<0x2000)
				mo=mo | rtc_MSK_C;
			rtc_reg[rtc_VL_SEC]=sc;
			rtc_reg[rtc_MINU]=mi;
			rtc_reg[rtc_HOUR]=hr;
			rtc_reg[rtc_DAY]=dy;
			rtc_reg[rtc_WEEK]=wk;
			rtc_reg[rtc_C_MON]=mo;
			rtc_reg[rtc_YEAR]=yr;
			// set RTC
			while (!sem_wire1 (sem_TAKE))
				;
			rtc_set_date_time ();
			sem_wire1 (sem_GIVE);
			// response
			Serial.println ("OK: mset");
		}
		else if (strncmp (lbuf,"who",3)==0)
		{
			// who
			// response
			sprintf (lbuf,"OK: %s ver.%s",prog_name,prog_vers);
			Serial.println (lbuf);
		}
		else if (strncmp (lbuf,"help",4)==0)
		{
			// help
			// response
			Serial.println ("OK+ log");
			Serial.println ("OK+ set,<walk_step>[,<walk_time_s>]");
			Serial.println ("OK+ time");
			Serial.println ("OK+ mset,<yyyy>,<mm>,<dd>,<HH>,<MM>,<SS>");
			Serial.println ("OK+ who");
			Serial.println ("OK: help");
		}
		else if (lbuf[0]=='\0')
		{
			// empty command, ignore
		}
		else
		{
			// ERROR: unsupported command
			Serial.print ("ERR: unsupported command: ");
			Serial.println (lbuf);
		}
		break;
	}


	// U/I handling
	static	bool	disp_on=true;

	M5.update ();

	// SW-A
	if (M5.BtnA.wasPressed ())
	{
		// set update screen command
		M5.Lcd.fillScreen (BLACK);
		thrd_upd_screen=true;

		// backlight on
		disp_on=true;
		M5.Axp.SetLDO2 (disp_on);
	}

	// SW-B
	if (M5.BtnB.pressedFor (4000) && disp_on)
	{
		// beep
		M5.Beep.beep ();
		delay (50);
		M5.Beep.mute ();
		delay (50);
		M5.Beep.beep ();
		delay (50);
		M5.Beep.mute ();

		// clear screen
		M5.Lcd.fillScreen (BLACK);

		// wait release SW-B
		while (!M5.BtnB.isReleased ())
			M5.BtnB.read ();

		// software reset for data clear
		esp_restart ();
	}
	else if (M5.BtnB.wasReleased ())
	{
		// backlight off
		disp_on=false;
		M5.Axp.SetLDO2 (disp_on);
	}

	// time
	uint32	msec;
	static	uint32	msec_prev;
	static	uint32	msec_log_prev;

	msec=millis ();

	// every 1 second
	while (msec - msec_log_prev>1000)
	{
		int		minu;

		// get semaphore
		if (!sem_wire1 (sem_TAKE))
			break;
		msec_log_prev=msec;

		// read RTC
		rtc_get_date_time ();
		// get battery level
		int		blev;
		blev=axp_get_bat_level ();
		blev=(blev<=0)? 0: (blev>=100)? 99: blev;
		// release semaphore
		sem_wire1 (sem_GIVE);

		// check logging interval
		minu=bcd2dec (rtc_reg[rtc_MINU]&rtc_MSK_MINU);
		if (minu!=ped_log_prev && minu%ped_MINU_LOG==0)
		{
			// logging
			ped_log_prev=minu;
			ped_log[ped_log_ptr + 0]=(((
				bcd2dec (rtc_reg[rtc_C_MON]&rtc_MSK_MON)*100 +
				bcd2dec (rtc_reg[rtc_DAY]&rtc_MSK_DAY))*100 +
				bcd2dec (rtc_reg[rtc_HOUR]&rtc_MSK_HOUR))*100 +
				minu)*100 + blev;
			ped_log[ped_log_ptr + 1]=ped_cnt_walk;
			ped_log[ped_log_ptr + 2]=ped_msec_walk;
			// update pointer
			ped_log_ptr += ped_ELM_LOG;
			if (ped_log_ptr>=ped_MAX_LOG)
				ped_log_ptr=0;
		}
	}

	// check sampling rate
	if (msec - msec_prev<ped_msec_ITVL)
		return;
	msec_prev=msec;

	// sampling
	int16	acl_x;
	int16	acl_y;
	int16	acl_z;

	if (meas_mpu6886_acl (acl_x,acl_y,acl_z)==ErrNo)
	{
		if (ped_log_mode)
		{
			// logging mode
			abuf[abuf_ptr + 0]=acl_x;
			abuf[abuf_ptr + 1]=acl_y;
			abuf[abuf_ptr + 2]=acl_z;
			abuf_ptr += 3;
			if (abuf_ptr>=ped_MAX_BUF)
			{
				abuf_ptr=0;
				ped_log_mode=false;
				M5.Beep.beep ();
				digitalWrite (LED_BUILTIN,LOW);
				Serial.println ("DBG: exit log mode");
			}
		}
		else
		{
			// pedometer mode
			int		ped_acl;

			// acel: 1G corresponding to 1024
			ped_acl=(sq (acl_x)>>14) + (sq (acl_y)>>14) + (sq (acl_z)>>14);
			ped_acl=ped_acl>>(14 - 10);

			// moving average
			ped_mva_sum=ped_mva_sum - mbuf[ped_mva_ptr] + ped_acl;
			mbuf[ped_mva_ptr]=ped_acl;
			ped_mva_ptr++;
			if (ped_mva_ptr>=ped_MAX_MVA)
				ped_mva_ptr=0;
			ped_acl=ped_mva_sum/ped_MAX_MVA;

			// walking detection
			if (ped_stat_walk==ped_WK_INCL)
			{
				// increasing
				ped_acl_max=(ped_acl>ped_acl_max)? ped_acl: ped_acl_max;
				if (ped_acl<ped_acl_max - ped_RNGE/2)
				{
					int		idx;
					int		mets;
					int		sec_walk;

					// start decreasing
					ped_stat_walk=ped_WK_DECR;
					ped_acl_min=ped_acl;

					// detect walking
					if (disp_on)
						M5.Beep.beep ();
					// count up
					ped_cnt_walk++;
					// walking time [ms]
					if (ped_msec_pre>0)
					{
						uint32	msec_one_step;

						msec_one_step=msec - ped_msec_pre;
						ped_msec_walk += (msec_one_step>ped_msec_WK_MAX)?
							ped_msec_WK_MAX: msec_one_step;
					}
					ped_msec_pre=msec;
					// walking distance [cm]
					ped_dst_walk=ped_STEP*ped_cnt_walk;
					// walking speed [m/h]
					sec_walk=ped_msec_walk/1000;
					if (sec_walk<=0)
						ped_spd_walk=0;
					else
						ped_spd_walk=ped_dst_walk*36/sec_walk;
					// METs [METs] *10 fixed point
					for (idx=0; true; idx += 2)
						if (ped_mets[idx]==0 || ped_spd_walk<=ped_mets[idx])
							break;
					if (ped_mets[idx]==1)
						mets=ped_mets[idx + 1];
					else if (ped_mets[idx]==0)
						mets=ped_mets[idx - 1];
					else
					{
						int		spd0;
						int		spd1;
						int		met0;
						int		met1;

						// interpolate
						spd0=ped_mets[idx - 2];
						spd1=ped_mets[idx];
						met0=ped_mets[idx - 1];
						met1=ped_mets[idx + 1];
						mets=met0 + (met1 - met0)*
							(ped_spd_walk - spd0)/(spd1 - spd0);
					}
					// calories burned [cal]
					ped_cal_walk=(105*mets*ped_WEIGHT/1000)*
						int (ped_msec_walk/3600);

					// set update screen command
					if (disp_on)
						thrd_upd_screen=true;
				}
			}
			else
			{
				// decreasing
				ped_acl_min=(ped_acl<ped_acl_min)? ped_acl: ped_acl_min;
				if (ped_acl>ped_acl_min + ped_RNGE/2)
				{
					// start increasing
					ped_stat_walk=ped_WK_INCL;
					ped_acl_max=ped_acl;
				}
			}
		}
	}

#ifdef		DEBUG_MODE
	// measure loop() processing time
	static	int		th_cnt;
	if (th_cnt<100)
		th_cnt++;
	else
	{
		uint32	msec_loop;
		static	uint32	msec_max_loop=0;

		msec_loop=millis () - msec;
		if (msec_loop>msec_max_loop)
		{
			msec_max_loop=msec_loop;
			sprintf (lbuf,"DBG: max loop=%lu [ms]",msec_max_loop);
			Serial.println (lbuf);
		}
	}

	// power switch short press
	if (false && M5.Axp.GetBtnPress ()&axp_PWR_SHORT)
	{
		// software reset for debug
		M5.Beep.beep ();
		delay (500);
		M5.Beep.mute ();

		esp_restart ();
	}
#endif	//	DEBUG_MODE
}