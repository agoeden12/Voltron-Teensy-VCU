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
float t_rpm = 0;
float m_rpm = 0;
float o_set = 0;

// -------------------------------------------------------------

ros::Publisher
pub_speed("Cart_Speed",
			&cart_speed); // TODO Create custome msg that sends back all data
ros::Subscriber<std_msgs::Float32> sub("/tvelocity", &target_set);
ros::Subscriber<std_msgs::Bool> sub_em("/rc_in_node/cmd_stop", &state_set);

// -------------------------------------------------------------
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
	m_rpm = uint16_t((frame.buf[0] << 8) + frame.buf[1]); // rpm
	cart_speed.data = m_rpm;
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
elapsedMicros freq;
Chrono rosmet;
Chrono controlmet;
Chrono acommet;
Chrono dataqmet;
Chrono rosheartbeat;
Chrono serialheartbeat;
int shbtimeout = 5000;

// -------------------------------------------------------------
double Kp = 2, Ki = 0, Kd = 0;
PID myPID(&m_rpm, &o_set, &t_rpm, Kp, Ki, Kd, DIRECT);

QuadDecode<1> menc;

float countsrev=256.0;
float filtrpm=0;
float filt=0.90;
int stime=10;
float dband=40;


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

        filtrpm = filt*filtrpm+(1-filt)*(memc.calPosn()/countsrev*500*60);//low pass filter
        if(abs(filtrpm)>dband){//dead band
           filtrpm=0;
        }
        m_rpm=filtrpm;// pid input update
    
        memc.zeroFTM();// encoder reset
    	cart_speed.data = filtrpm;// ros speed update
    	myPID.Compute();// pid update
	}


	if (!rosheartbeat.hasPassed(10000) ||
		!serialheartbeat.hasPassed(shbtimeout) || issim) {
		//digitalWrite(LED_BUILTIN,HIGH);
		int output = (int) o_set;
		analogWrite(throttle, output + 2048);
	} else {

		analogWrite(throttle, 2048);
	}
	// -------------------------------------------------------------
	if (acommet.hasPassed(100, true)) {
		Serial2.println(" \f motor rpm " + String(m_rpm) + " target rpm " +
						String(t_rpm) + " motor set " + String(o_set));
		Serial2.println("Kp: " + String(Kp) + " Ki: " + String(Ki) + " Kd: " +
						String(Kd) + " Khz " + String(1.0 / (freq / 1000.0)));
		Serial2.println("use letters p, i, d, m, h  value to set parameters");
	}
	freq = 0;
	if (Serial2.available() > 1) {

		switch (Serial2.read()) {
		case /* value */ 'p':
			Kp = Serial2.parseFloat();
			myPID.SetTunings(Kp, Ki, Kd);
			Serial2.println("Kp set to: " + String(Kp));
			break;

		case /* value */ 'i':
			Ki = Serial2.parseFloat();
			myPID.SetTunings(Kp, Ki, Kd);
			Serial2.println("Ki set to: " + String(Ki));
			break;

		case /* value */ 'd':
			Kd = Serial2.parseFloat();
			myPID.SetTunings(Kp, Ki, Kd);
			Serial2.println("Kd set to: " + String(Kd));
			break;
		case /* value */ 'm':
			t_rpm = Serial2.parseInt();
			serialheartbeat.restart();
			Serial2.println("Motor set to: " + String(t_rpm));
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

