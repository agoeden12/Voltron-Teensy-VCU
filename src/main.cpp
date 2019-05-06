#include <Arduino.h>
#include <Chrono.h>
#include <FastFloatPID.h>
#include <FlexCAN.h>  
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include "QuadDecode_def.h"

#define USE_TEENSY_HW_SERIAL
#define PID FastFloatPID
#define throttle A22
#define regen A22
#define relay_pin 24

void state_set(const std_msgs::Bool &emstate);
void target_set(const std_msgs::Float32 &v_msg);

ros::NodeHandle nh;

std_msgs::Float32 cart_speed;



// -------------------------------------------------------------

ros::Publisher
pub_speed("Cart_Speed",
			&cart_speed); // TODO Create custome msg that sends back all data
ros::Subscriber<std_msgs::Float32> sub("/tvelocity", &target_set);
ros::Subscriber<std_msgs::Bool> sub_em("/rc_in_node/cmd_stop", &state_set);

// -------------------------------------------------------------
// used for reciving messages from the kelly
static CAN_message_t rmsg;
static uint8_t hex[17] = "0123456789abcdef";

class Kellylisten : public CANListener {
public:
	void kellyRpm(CAN_message_t &frame, int mailbox);

	bool frameHandler(CAN_message_t &frame, int mailbox,
					uint8_t controller); // overrides the parent version so
										// we can actually do something
};

void Kellylisten::kellyRpm(CAN_message_t &frame, int mailbox) {
	//m_rpm = uint16_t((frame.buf[0] << 8) + frame.buf[1]); // rpm
//cart_speed.data = m_rpm;
}

bool Kellylisten::frameHandler(CAN_message_t &frame, int mailbox,
							uint8_t controller) {
	if (frame.id == 0x73)
		kellyRpm(frame, mailbox);
	return true;
}

Kellylisten kellylisten;
// -------------------------------------------------------------

// -------------------------------------------------------------

// -------------------------------------------------------------
elapsedMicros freq;//used to track totalloop time
elapsedMillis enct;
Chrono rosmet;// used to regulate ros update rate
Chrono controlmet;// used to regulate cotroller loop update rate
Chrono acommet;// used to regulat serail update rate
Chrono rosheartbeat;// stops cart if no controller input
Chrono serialheartbeat; //stops cart if no controller input
int shbtimeout = 5000;

// -------------------------------------------------------------
int stime=10; // sample time of controll loop and enconder vel
double Kp = 0.3, Ki = 0, Kd = 0, Kf = 0; // pid vals
float t_rpm = 0; // variable the PID controller operates on
float m_rpm = 0;
float o_set = 0;
PID myPID(&m_rpm, &o_set, &t_rpm, Kp, Ki, Kd, DIRECT);// create pid controller

QuadDecode<1> menc;// creates enconder pins 3,4

float countsrev=1042;//encoder ticks
float filtrpm=0; // creates low pass filter for encoder
float filt=0.90; // filter param
float torpm=100*60/countsrev;

float dband=200; // encoder deadband setting


// -------------------------------------------------------------
void setup() {
	delay(1000);
	// -------------------------------------------------------------

	pinMode(relay_pin, OUTPUT);
	digitalWrite(relay_pin, LOW);

	analogWriteResolution(12);
	myPID.SetMode(AUTOMATIC);
	myPID.SetSampleTime(stime/1000.0);
	// myPID.SetOutputLimits(-993, 993);
	myPID.SetOutputLimits(-2048, 2048);

	menc.setup();
    menc.start();

	// -------------------------------------------------------------
	// Can0.begin(1000000);
	Serial2.begin(115200);
	Serial2.println("init");
	// -------------------------------------------------------------
// 	Can0.begin(1000000);
// 	Can0.attachObj(&kellylisten);

// 	CAN_filter_t allPassFilter;
// 	allPassFilter.id = 0;
// 	allPassFilter.ext = 0;
// 	allPassFilter.rtr = 0;

// 	for (uint8_t filterNum = 0; filterNum < 16; filterNum++) {
// 		Can0.setFilter(allPassFilter, filterNum);

// 	}
// 	for (uint8_t filterNum = 0; filterNum < 16; filterNum++) {
// 		kellylisten.attachMBHandler(filterNum);

// 	}
// 	rmsg.ext = 0;
// 	rmsg.id = 0x6b;
// 	rmsg.len = 1;
// 	rmsg.buf[0] = 0x37;
// 	rmsg.buf[1] = 0;
// 	rmsg.buf[2] = 0;
// 	rmsg.buf[3] = 0;
// 	rmsg.buf[4] = 0;
// 	rmsg.buf[5] = 0;
// 	rmsg.buf[6] = 0;
// 	rmsg.buf[7] = 0;

// 	Can0.write(rmsg);

	// -------------------------------------------------------------

	// ------------------------------------------------------------
	nh.getHardware()->setBaud(57600);
	nh.initNode();
	nh.advertise(pub_speed);
	nh.subscribe(sub);
	nh.subscribe(sub_em);
}

void loop() {

	// -------------------------------------------------------------
	// Serial.print("test");
	

	// -------------------------------------------------------------

	// -------------------------------------------------------------


	// if (kellymet.hasPassed(stime, true)) {

	// 	if (issim) {
	// 		m_rpm = sim_rpm;
	// 		cart_speed.data = m_rpm;
	// 	} else {

	// 		Can0.write(rmsg);
	// 	}
	// 	cart_speed.data = t_rpm;
	// 	myPID.Compute();
	// 	kellysim();
	// }
	if(controlmet.hasPassed(stime, true)){
		int curpos =menc.calcPosn();
		 if(abs(curpos)<(dband/(torpm))){//dead band
           filtrpm=filt*filtrpm;
        }
        else{//if the control loop is time to run its run
        filtrpm = filt*filtrpm+(1-filt)*(curpos);//low pass filter
		}
		//counts per 10ms
	   //filtrpm = (menc.calcPosn()/(countsrev*enct));
       
        m_rpm=(-filtrpm);// pid input update
		menc.zeroFTM();
        
    	cart_speed.data = -filtrpm*torpm;// ros speed update
    	myPID.Compute();// pid update
	
	}


	if (!rosheartbeat.hasPassed(1000) ||
		!serialheartbeat.hasPassed(shbtimeout)) {
		//digitalWrite(LED_BUILTIN,HIGH);
		

		int output = (int) Kf*t_rpm+o_set;
		analogWrite(throttle, output + 2048);
	} else {

		analogWrite(throttle, 2048);
	}
	// -------------------------------------------------------------
	if (acommet.hasPassed(100, true)) {
		Serial2.println(" \f motor rpm " + String(m_rpm*torpm) + " target rpm " +
						String(t_rpm*torpm) + " motor set " + String(o_set));
		Serial2.println("Kp: " + String(Kp/torpm) + " Ki: " + String(Ki/torpm) + " Kd: "+String(Kd/torpm) +" Kf: "+String(Kf) );
		Serial2.println("Filtconst: "+String(filt)+ " Khz " + String(1.0 / (freq / 1000.0))+" tm "+String(shbtimeout));
		Serial2.println("use letters p, i, d, f, c, m, h  value to set parameters");
		// Serial2.print("mr,"+String(m_rpm)+'\n');
		// Serial2.print("mt,"+String(t_rpm)+'\n');
		// Serial2.print("mo,"+String(o_set)+'\n');
		
		// Serial2.print("kp,"+String(Kp)+'\n');
		// Serial2.print("ki,"+String(Ki)+'\n');
		// Serial2.print("kd,"+String(Kd)+'\n');
		
		// Serial2.print("to,"+String(shbtimeout)+'\n');
		// Serial2.print("hz,"+String(1.0 / (freq / 1000.0))+'\n');
		// Serial2.flush();
	}
	freq = 0;
	if (Serial2.available() > 1) {

		switch (Serial2.read()) {
		case /* value */ 'p':
			Kp = Serial2.parseFloat()*torpm;
			myPID.SetTunings(Kp, Ki, Kd);
			Serial2.println("Kp set to: " + String(Kp));
			break;

		case /* value */ 'i':
			Ki = Serial2.parseFloat()*torpm;
			myPID.SetTunings(Kp, Ki, Kd);
			Serial2.println("Ki set to: " + String(Ki));
			break;

		case /* value */ 'd':
			Kd = Serial2.parseFloat()*torpm;
			myPID.SetTunings(Kp, Ki, Kd);
			Serial2.println("Kd set to: " + String(Kd));
			break;
		case /* value */ 'f':
			Kf = Serial2.parseFloat();
			
			Serial2.println("Kf set to: " + String(Kf));
			break;
	
		case /* value */ 'c':
			filt = Serial2.parseFloat();
			
			Serial2.println("filt set to: " + String(filt));
			break;	
		case /* value */ 'm':
			t_rpm = Serial2.parseInt()/(torpm);
			serialheartbeat.restart();
			Serial2.println("Motor set to: " + String(t_rpm*torpm));
			break;
		case /* value */ 'h':
			shbtimeout = Serial2.parseInt();
			Serial2.println("Timeout set to: " + String(shbtimeout));

			break;
		}
	}
	if (rosmet.hasPassed(20)) {
		rosmet.restart();
		pub_speed.publish(&cart_speed);
		nh.spinOnce();
	}
}

void target_set(const std_msgs::Float32 &v_msg) {
	t_rpm = v_msg.data;
	rosheartbeat.restart();
	digitalWrite(LED_BUILTIN,HIGH);

}

void state_set(const std_msgs::Bool &em_state) {
	if (em_state.data) {
		digitalWrite(relay_pin, HIGH); // relay on for electromagnet
	} else {
		digitalWrite(relay_pin, LOW); // relay off for electromagnet
	}
}

