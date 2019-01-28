/*
 *  regulate the speed of a 4pin PWM fan with PID logic
 *  a analog value (thermostat, potmeter) is used to set the PID setpoint reference
 *  this module uses digital (onewire) temp sensors
 * 	output to 128x64 OLED display via U8glib 
 *	developed and tested on Ardiono Uno
 
 * by W. Horsman, based on countless code snippets and examples :-)
*/

// TODO: make the display dimensions variable (e.g. 128x128 )

/*
 * List of I/O ports ::
 *
 * dig  2 : DS18B20 1-wire temp sensor, DQ port (with 5Kohm pullup to +5Vdc)
 * pwm  3 : pwm port (p4) of 4-pin 12" computer fan (on auxilary 12V power)
 * dig 12 : tag sensor (p3)  of 4-pin 12" computer fan
 *
 * ana  5 : SCL (clock) port of monochrome OLED display SSD1306_128X64
 * ana  4 : SDA (data) port of monochrome OLED display SSD1306_128X64
 * ana  1 : center (moving) connector of 5Kohm potmeter
 */
 
// libs to include
#include "U8glib.h"		// lib for OLED display
#include <OneWire.h>	// lib for digital sensors
#include <DallasTemperature.h>	//lib for easy access to digital temp sensors
#include <PID_v1.h>		// lib for PID control

// use DEBUGLOG switch to produce more serial (status) output
// use U8GDEBUG switch to verbose all U8G funtion calls
#define DEBUGLOG 1   //produces variable values in main loop()
#define U8GDEBUG 0	 //echos every U8G function that is being called
#define LOOPTIMING 0 //echos breakpoints in main loop() for performance analyses

/*
 * Init the OneWire and DallasTemperature lib 
*/
// Data wire is plugged into dig I/O pin *2* on the Arduino
#define ONE_WIRE_BUS 2

// use following resolutions for IC temp readout
#define ICRESOLUTION9BIT   9  // resolution is 0.5 degrees
#define ICRESOLUTION10BIT 10  // resolution is 0.25 degrees

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

/*
 * Init the PID library and necassary variables
*/
//Define PID global variables 
double g_pidSetpoint, g_pidInput, g_pidOutput;  //for the output to be PWM-compatible, use 0..255 range for all variables
//Specify the links and initial tuning parameters
PID myPID(&g_pidInput, &g_pidOutput, &g_pidSetpoint,1,3,2, REVERSE);  //Reverse the output because more fanspeed means lower temperature. std values are 2,5,1

/*
 * global variables and constants to pass through to functions
 * by using global variables (compared to function variables) stack usage is reduced
*/
// global timer vars and constant values
// the timer values set the waiting time between executing certain updates
const unsigned long c_calcCycle = 5;      // process the temperature measurements every 5 seconds
const unsigned long c_cycle5m = 300;      // 300 seconds = 5 minutes cycle,  for debug purposes this value may be set to 30 to speed up the chart updates
const unsigned long c_cycle15m = 900;     // 900 seconds = 15 minutes cycle, for debug purposes this value may be set to 90 to speed up the chart updates
const unsigned long c_cycle1h = 3600;     // 3600 seconds = 1 hour cycle,    for debug purposes this value may be set to 360 to speed up the chart updates
unsigned long g_timerCalcCycle = 0;       // timer for processing temp measurements and average calculations
unsigned long g_timer5mCycle = 0;         // timer for handling display and average updates every 5 minutes
unsigned long g_timer15mCycle = 0;        // timer for handling display and average updates every 15 minutes
unsigned long g_timer1hCycle = 0;         // timer for handling display and average updates every 1 hour
unsigned long g_tempValCounter = 1;       // counts the number of temperatur values
float g_sumT=0, g_avgT=0, g_maxT=0, g_minT=0, g_currT=0;   // temperature (float) values for sum(temp), avg(temp), max(temp), min(temp)

//temperature bounderies
const unsigned long c_lowT = 10;		// low-bound value for room temperature
const unsigned long c_highT = 50;		// high-bound value for room temperature

// I/O constants
// this module uses digital and analog I/O
// the following constants help with the boundery scope of digital and analog I/O
const int c_anaMin = 0;    //minimal analog value (potmeter e.g.) to read from analog port
const int c_anaMax = 1023; //maximum analog value (potmeter e.g.) to read from analog port
const int c_digMin = 1;    //minimal value of the digital output (yes, could have been 0)
const int c_digMax = 255;  //maximum value of the digital output

/*
 * additional I/O ports to use
 * please note that not all PWM ports have the same frequency:
 * standard: 3,9,10,11 is 490Hz, 5,6 is 980 Hz
 * Arduino has 3 timers: timer 0 for 5,6,  timer 1 for 9,10,  timer 2 for 3, 11
*/
// Fan sens (tach) and PWM wires connected to digital ports
const int c_fanSensePort = 12; //digital port to read the fan rpm speed
const int c_fanPWMPort = 3;    //digital (PWM) port to set/write the PWM value
const int c_setPointPin = 1;	 // potmeter to analog_A1 port will set the PID SetPoint value while running (thermostat)
const int c_minFanRPM = 200;   // lowest RPM for fan
const int c_maxFanRPM = 2500;  // highest RPM for fan

//set correct OLED display specs, full list: https://github.com/olikraus/u8glib/wiki/device
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);	// Display which does not send ACK


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////


void u8g_textprep_small(void) {
// prepare the display for writing small font
#if U8GDEBUG
  Serial.println("u8g_textprep_small()");
#endif
  u8g.setFont(u8g_font_6x10);
  u8g.setFontRefHeightExtendedText();
  u8g.setDefaultForegroundColor();
  u8g.setFontPosTop();
  
} // end of u8g_textprep_small()


void u8g_textprep_normal(void) {
// prepare the display for normal font size
#if U8GDEBUG
  Serial.println("u8g_textprep_normal()");
#endif
  u8g.setFont(u8g_font_6x13B);
  u8g.setFontRefHeightExtendedText();
  u8g.setDefaultForegroundColor();
  u8g.setFontPosTop();
  
} // end of u8g_textprep_normal()


void u8g_write_minmax (float minTemp, float maxTemp) {
// write the Tmin and Tmax values on the display
#if U8GDEBUG
  Serial.println("u8g_write_minmax()");
#endif
  int yPos=15;
  char str [48];  //the string buffer to format the text for display
  int dectemp;    //truncate temp because sprintf cannot print floats directly

  sprintf(str, "min %d.%d%c", int(minTemp), int((minTemp*10)-(int(minTemp)*10)), char(176));  //char(176) is the Celcius degree symbol
  u8g.drawStr(0,yPos, str );
  sprintf(str, "max %d.%d%c", int(maxTemp), int((maxTemp*10)-(int(maxTemp)*10)), char(176));
  u8g.drawStr(76,yPos, str );
  
} // end of u8g_write_minmax()


void u8g_write_temp_RPM(float temp, float spTemp, int fanSpeed) {
// write the temperature, setpoint and fan speed at the top of the display
#if U8GDEBUG
  Serial.println("u8g_write_temp_RPM()");
#endif
  char str [16];            //the string buffer to format the text for display
  int dectemp = int(temp);  //truncate temp because sprintf cannot print floats directly
  
  sprintf(str, "%d.%d%c", dectemp, int((temp*10)-(dectemp*10)), char(176));  //char(176) is the Celcius degree symbol
  u8g.drawStr(2,0, str );
  sprintf(str, "f: %d%%", map(fanSpeed,0,100,0,99));  // the double %% reverts to a single "%" sign
  u8g.drawStr(92,0, str );

  dectemp = int(spTemp);
  u8g_textprep_small();
  sprintf(str, ">%d.%d<", dectemp, int((spTemp*10)-(dectemp*10)));  //char(176) is the Celcius degree symbol
  u8g.drawStr(45,3, str );
  
} // end of u8g_write_temp_RPM()


void u8g_chart_frame(void) {
// draw the 3 outline frames of the chart areas including header text
#if U8GDEBUG
  Serial.println("u8g_chart_frame()");
#endif
  int yPos=25;
  u8g.drawStr (0, yPos, "hrs:");
  u8g.drawFrame(0, 34,  42, 30);
  u8g.drawStr (43, yPos, "15m:");
  u8g.drawFrame(43,34,  42, 30);
  u8g.drawStr (86, yPos, "5m:");
  u8g.drawFrame(86,34,  42, 30);
  
} // end of u8g_chart_frame()


void u8g_fill_charts(int vh1, int vh2, int vh3, int vq1, int vq2, int vq3, int vm1, int vm2, int vm3) {
// for every column chart, draw the outline frame and fill the chart with a box
// arguments are constraint temperatures between 10 - 50 gr
// columns are numbered (and drawn) left to right: vh1, vh2, vh3, .. etc
#if U8GDEBUG
  Serial.println("u8g_fill_charts()");
#endif
      int startX = 2;       //the first column chart
const int startY = 36;      //the top of the column charts
const int chartWidth = 12;  //a column chart is 12 pixels in width
const int chartHeight = 27; //max height of a column chart
      int tBound;           //constraint temperature value
      int hCalc;            //the calculated (top) y-position of the start of the chart box
      int normH;            //normalise the temp value to the chart height

// hours
  startX = 2; 
  tBound = constrain(vh1, c_lowT, c_highT);
  normH = map(tBound, c_lowT, c_highT, 1, chartHeight); 
  hCalc = startY + chartHeight - normH;  //calculate top-y of the chart
  u8g.drawFrame ( startX, startY, chartWidth, chartHeight);
  u8g.drawBox (startX, hCalc, chartWidth, normH);

  tBound = constrain(vh2, c_lowT, c_highT);
  normH = map(tBound, c_lowT, c_highT, 1, chartHeight);
  hCalc = startY + chartHeight - normH;  //calculate top-y of the chart
  u8g.drawFrame ( startX+1+chartWidth, startY, chartWidth, chartHeight);
  u8g.drawBox   ( startX+1+chartWidth, hCalc, chartWidth, normH);
  
  tBound = constrain(vh3, c_lowT, c_highT);
  normH = map(tBound, c_lowT, c_highT, 1, chartHeight);
  hCalc = startY + chartHeight - normH;  //calculate top-y of the chart
  u8g.drawFrame ( startX+2*(chartWidth+1), startY, chartWidth, chartHeight);
  u8g.drawBox   ( startX+2*(chartWidth+1), hCalc, chartWidth, normH);

// 15min
  startX = 45;
  tBound = constrain(vq1, c_lowT, c_highT);
  normH = map(tBound, c_lowT, c_highT, 1, chartHeight);
  hCalc = startY + chartHeight - normH;  //calculate top-y of the chart
  u8g.drawFrame ( startX, startY, chartWidth, chartHeight);
  u8g.drawBox   ( startX, hCalc, chartWidth, normH);

  tBound = constrain(vq2, c_lowT, c_highT);
  normH = map(tBound, c_lowT, c_highT, 1, chartHeight);
  hCalc = startY + chartHeight - normH;  //calculate top-y of the chart
  u8g.drawFrame ( startX+1+chartWidth, startY, chartWidth, chartHeight);
  u8g.drawBox   ( startX+1+chartWidth, hCalc, chartWidth, normH);
  
  tBound = constrain(vq3, c_lowT, c_highT);
  normH = map(tBound, c_lowT, c_highT, 1, chartHeight);
  hCalc = startY + chartHeight - normH;  //calculate top-y of the chart
  u8g.drawFrame ( startX+2*(chartWidth+1), startY, chartWidth, chartHeight);
  u8g.drawBox   ( startX+2*(chartWidth+1), hCalc, chartWidth, normH);

// 5 min
  startX = 88;
  tBound = constrain(vm1, c_lowT, c_highT);
  normH = map(tBound, c_lowT, c_highT, 1, chartHeight);
  hCalc = startY + chartHeight - normH;  //calculate top-y of the chart
  u8g.drawFrame ( startX, startY, chartWidth, chartHeight);
  u8g.drawBox   ( startX, hCalc, chartWidth, normH);

  tBound = constrain(vm2, c_lowT, c_highT);
  normH = map(tBound, c_lowT, c_highT, 1, chartHeight);
  hCalc = startY + chartHeight - normH;  //calculate top-y of the chart
  u8g.drawFrame ( startX+1+chartWidth, startY, chartWidth, chartHeight);
  u8g.drawBox   ( startX+1+chartWidth, hCalc, chartWidth, normH);
  
  tBound = constrain(vm3, c_lowT, c_highT);
  normH = map(tBound, c_lowT, c_highT, 1, chartHeight);
  hCalc = startY + chartHeight - normH;  //calculate top-y of the chart
  u8g.drawFrame ( startX+2*(chartWidth+1), startY, chartWidth, chartHeight);
  u8g.drawBox   ( startX+2*(chartWidth+1), hCalc, chartWidth, normH);
  
} // end of u8g_fill_charts()

int readRPM() {
// read the fan speed from the tag sensor of the fan)
  unsigned long pulseDuration = pulseIn(c_fanSensePort, HIGH);
  double frequency = 0;
  int rpm = -1;
  if (pulseDuration != 0){
      frequency = 1000000/pulseDuration;
      rpm = int(frequency/4*60);
  }
 Serial.print("RPM = ");
 Serial.println(rpm);
 return rpm;
} // end of readRPM()


int temp2dig(double temp){
//map the actual temperature to the digital port scale
  return map(int(temp),c_lowT, c_highT, c_digMin, c_digMax);
}//end of temp2dig()

float ana2temp(int anaVal){ 
//convert an analog port value to temperature scale value
  float temp = float(anaVal)/c_anaMax;
  temp = c_lowT + (temp * (c_highT-c_lowT));
 
  return temp;
}//end of ana2temp()

int ana2dig(int anaVal){ 
//map a value in the analog scale to digital scale
  return map(anaVal,c_anaMin,c_anaMax,0,c_digMax);
} //end of ana2dig()

int dig2perc(int digVal) {
//convert a digital port value to a PWM percentage (0..100)
  int perc;
  perc=map(digVal,0,c_digMax,0,100);
	return perc;
} //end of dig2perc()
	

void setup(void) {
// one-time init before starting loop()
  Serial.begin(115200);  //set serial output baud rate to fast
#ifdef ARDUINO
  pinMode(13, OUTPUT);           
  digitalWrite(13, HIGH);  
#endif

  pinMode(c_fanPWMPort, OUTPUT); 	//set arduino pin I/O
  pinMode(c_fanSensePort, INPUT);
  digitalWrite(c_fanSensePort,HIGH);

  sensors.begin(); // Start up the OneWire library
  sensors.setResolution(ICRESOLUTION9BIT); // 9-bit resolution (0.5gr) is fine for room temp and fast to read
  sensors.requestTemperatures();           // Send the Dallas OneWire command to get temperatures
  g_currT = sensors.getTempCByIndex(0);    // index 0 refers to the first IC on the wire
  g_minT = g_currT;                        // initialise (global) minimal temp
  g_maxT = g_currT;                        // initialise (global) maximum temp
  
  g_pidInput = temp2dig(g_currT); //init the initial measured input value for PID
  g_pidSetpoint = temp2dig(25); //initial setpoit set to arbitrary 25 degrees
  myPID.SetMode(AUTOMATIC);

  randomSeed(analogRead(0)); //initialise new randow set
  g_timerCalcCycle = millis()/1000;  //initial start time
  g_timer5mCycle = g_timerCalcCycle;
  g_timer15mCycle = g_timerCalcCycle;
  g_timer1hCycle = g_timerCalcCycle;
} // end of setup()

void loop(void) {
/* read OneWire temperature en update the display with charts and data
 * the 3 rightmost charts show the average every 5 min period; 5 min, 10min and 15min ago
 * the middle 3 charts show the average of every 15 min period, 30min, 45min and 60 min ago
 * the leftmost 3 charts show the average of every hour, 2 hours, 3 hours and 4 hours ago 
 * fanSpeed is the result of managing the temperature to an initial setPoint of 25 degrees, using PID logic
 */
#if LOOPTIMING
  Serial.println("loop()");
#endif
  unsigned long currentTime, secondsPast;
  bool doRefresh = false;             	 // update the display only if something has changed
  int  f_perc=0, fanspeed = readRPM();	 // get the fanspeed from digital port
  int  analogValue = analogRead(c_setPointPin); // analog (potmeter) port value between 0 and 1023
  float setPointTemp;
static float prevT=0;                    // used to determine if the temp has changed  
static char serBuff [96];                // string buffer for printing to Serial
static unsigned long loopCntr;           // count the loops... this may become very large
static float t5m=0, t10m=0, t15m=0, t20m=0, t25m=0, t30m=0;  //calculated arguments for charts
static float tq1=0, tq2=0, tq3=0, tq4=0, tq5=0, tq6=0, tq7=0;
static float th1=0, th2=0, th3=0, tmax=0, tmin=0;

#if LOOPTIMING
  Serial.println("* end of declarations");
#endif

  loopCntr++;
  currentTime = millis() / 1000; //set time in seconds since start
  if (int(prevT*10) != int(g_currT*10)) doRefresh = true; //do a display update, even if no cycletimer has gone off
  
#if LOOPTIMING
  Serial.println("* end of millis()");
#endif
  
  prevT = g_currT;				 // save the temperature of the previous cycle
  sensors.requestTemperatures(); // Send the Dallas OneWire command to get temperatures
  g_currT = sensors.getTempCByIndex(0); //read OneWire sensor, 0 refers to the first IC on the wire

  g_pidSetpoint = ana2dig(analogValue);  // set PID reference, never mind it's not a real temperatur, pidSetpoint works in 0..255 anyway)
  setPointTemp = ana2temp(analogValue);
   Serial.print("SPT = "); Serial.println(setPointTemp);
  g_pidInput = temp2dig(g_currT);		   // let PID lib know what the actual temperature is
   Serial.print("TMP = "); Serial.println(g_currT);
  myPID.Compute(); 					   // calculate and set new g_pidOutput between 0..255
  analogWrite(c_fanPWMPort, g_pidOutput);  //convert the value between 0..255 into PWM signal on the specified port)
   Serial.print("PWM = "); Serial.print(dig2perc(g_pidOutput)); Serial.println("%"); Serial.println(""); 

#if LOOPTIMING
  Serial.println("* end of getTemp() and PID.Compute()");
#endif

  secondsPast = currentTime - g_timerCalcCycle; //calculate time in seconds since g_timerCalcCycle reset
  
  if (secondsPast >= c_calcCycle)  //do every #c_calcCycle seconds because temp changes slowly
  {
#if DEBUGLOG
    Serial.println ("do CalcCycle");
#endif
    g_sumT += g_currT; //summerize all temperature measurements for calculating rolling average
    g_avgT = g_sumT / float(g_tempValCounter); //calculate the rolling average temperature
    if (g_currT > g_maxT) g_maxT = g_currT;      // remember the lowest and highest value, for display only
    if (g_currT < g_minT) g_minT = g_currT;
    g_timerCalcCycle = currentTime;  //reset timer
     Serial.print ("loopcount:"); Serial.print (loopCntr);
     Serial.print (" valcount:"); Serial.print (g_tempValCounter);
     Serial.print (" sumT:"); Serial.print (g_sumT, 2);
     Serial.print (" avgT:"); Serial.print (g_avgT, 2);
     Serial.print (" minT:"); Serial.print (g_minT, 2);
     Serial.print (" maxT:"); Serial.println (g_maxT, 2);
    g_tempValCounter++; // increase the number of temperatures that have been processed
    doRefresh = true; //update display at least every calCycle[s]
}

// Every 5 minutes the average temperature is calculated based on 
// the number of measurements and the total sum of temperatures.
// the 15m average and 1h average is calulated based on the 5m average
   secondsPast = currentTime - g_timer5mCycle; //calculate time in seconds since g_timer5mCycle reset
   if (secondsPast >= c_cycle5m) // do every 5m
   { //shift all 5m values to the next index
     t30m = t25m; t25m = t20m; t20m = t15m; t15m = t10m; t10m = t5m; t5m = g_avgT;  //shift all avg calcs of 5 min periods

#if DEBUGLOG
     Serial.print ("*** do 5mCycle ***");
     Serial.print ("   cycle speed [loops/s]: "); Serial.println (loopCntr/currentTime);
     Serial.print ("t5m:");  Serial.print (t5m);
     Serial.print (" t10m:"); Serial.print (t10m);
     Serial.print (" t15m:"); Serial.println (t15m);
     Serial.print ("t20m:");  Serial.print (t20m);
     Serial.print (" t25m:"); Serial.print (t25m);
     Serial.print (" t30m:"); Serial.println (t30m);
#endif
     g_timer5mCycle = currentTime; //reset timer
     g_tempValCounter = 1;  g_sumT = 0; //reset value counter and sum, they are only used for the 5m cycles
     doRefresh = true; // update the display
   }

// Process every 15 minutes
   secondsPast = currentTime - g_timer15mCycle; //request time in seconds since g_timer15mCycle reset
   if (secondsPast >= c_cycle15m) // do every 15m
   {
     tq7 = tq6; tq6 = tq5; tq5 = tq4; tq4 = tq3; tq3 = tq2; tq2 = tq1; 
	 tq1 = (t20m + t25m + t30m)/3 ; //the first quarter chart is the average of the 5m intervals more than 30 min ago
#if DEBUGLOG
     Serial.println ("*** do 15mCycle ***");
     Serial.print ("tq1:");  Serial.print (tq1);
     Serial.print (" tq2:"); Serial.print (tq2);
     Serial.print (" tq3:"); Serial.print (tq3);
     Serial.print (" tq4:"); Serial.print (tq4);
     Serial.print (" tq5:"); Serial.print (tq5);
     Serial.print (" tq6:"); Serial.print (tq6);
     Serial.print (" tq7:"); Serial.println (tq7);
#endif
     g_timer15mCycle = currentTime; //reset timer
     doRefresh = true; // update the display
   }

//process every hour
   secondsPast = currentTime - g_timer1hCycle; //request time in seconds since g_timer1hCycle reset
   if (secondsPast >= c_cycle1h) // do every hour
   {
     th3 = th2; th2 = th1; 
	 th1 = (tq7 + tq6 + tq5 + tq4)/4 ; // the first hour chart is the average of 4 quarter intervals 2 hours ago
#if DEBUGLOG
     Serial.println ("*** do 1hCycle ***");
     Serial.print ("th1:");  Serial.print (th1);
     Serial.print (" th2:"); Serial.print (th2);
     Serial.print (" th3:"); Serial.println (th3);
#endif
     g_timer1hCycle = currentTime; //reset timer
     doRefresh = true; // update the display
   }

  if ( doRefresh )  //some values have changed, so update the display
  {
//  f_perc = map (fanspeed,c_minFanRPM, c_maxFanRPM, 0,100);  //this should work if the RPM readout is accurate. But is isn't,...at all...
    f_perc = dig2perc(g_pidOutput); // so the next best thing is to display the PWM value that is sent to the fan

#if DEBUGLOG
    Serial.println ("*** do Display Update ***");
#endif
#if LOOPTIMING
  Serial.println("* start of pictureLoop()");
#endif
// picture loop  
    u8g.firstPage();  // this picture loop is repeated about 8 times and every time it displays a new part of the total display
    do {			  // ignore the do..while construction. When this loop is finished the display is refreshed 1 time.
        u8g_textprep_normal();
        u8g_write_temp_RPM(g_currT, setPointTemp, f_perc);
        u8g_textprep_small();
        u8g_write_minmax (g_minT, g_maxT);
        u8g_chart_frame();
        u8g_fill_charts(int(th3), int(th2), int(th1), int(tq3), int(tq2), int(tq1), int(t15m), int(t10m), int(t5m));
    } while( u8g.nextPage() );
    // end of picture loop
#if LOOPTIMING
  Serial.println("* end of pictureLoop()");
#endif
  }
 
  // do not loop statement for debug purposes
  //  while(true) {};

} //end loop()
