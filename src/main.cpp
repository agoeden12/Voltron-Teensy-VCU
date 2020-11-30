#define USE_TEENSY_HW_SERIAL
#define PID FastFloatPID
#define relay_pin 24
#define ledPin 13
#define USE_USBCON

#include <Arduino.h>
#include <Chrono.h>
#include <FastFloatPID.h>
#include <math.h>

#ifdef WITH_QUAD_DECODE
#include "QuadDecode_def.h"
#endif

#include <SatelliteReceiver.h>
#include <ardupilotmega/mavlink.h>

// ******* TASK MANAGEMENT ****** //
#include <ExecWithParameter.h>
#include <TaskManagerIO.h>

#define throttle A17
#define regen A17

#define TELEM_BAUD_RATE 921600

// ------------------------------------------------------------
static SatelliteReceiver SpektrumRx;
static uint16_t update = 0;

// -------------------------------------------------------------
static elapsedMicros freq;  // used to track totalloop time
static elapsedMillis enct;
static Chrono syncmet;     // used to regulate outer loop tomes to prevent sync
                           // errrors with inner loops
static Chrono rosmet;      // used to regulate ros update rate
static Chrono controlmet;  // used to regulate cotroller loop update rate
static Chrono acommet;     // used to regulat serail update rate
static Chrono rosheartbeat;     // stops cart if no controller input
static Chrono serialheartbeat;  // stops cart if no controller input
static int shbtimeout = 5000;

// -------------------------------------------------------------
static int stime = 10;  // sample time of controll loop and enconder vel
static double Kp = 4.61, Ki = 0, Kd = 0, Kf = 0.98;  // pid vals
static float t_rpm = 0;  // variable the PID controller operates on
static float m_rpm = 0;
static float o_set = 0;

static PID myPID(&m_rpm, &o_set, &t_rpm, Kp, Ki, Kd,
                 DIRECT);  // create pid controller
static int output = 0;

#ifdef WITH_QUAD_DECODE
static QuadDecode<1> menc;  // creates enconder pins 3,4
#endif

static float countsrev = 1042;  // encoder ticks
static float filtrpm = 0;       // creates low pass filter for encoder
static float filt = 0.90;       // filter param
static float fixitnum = 0.83;
static float torpm = 100 * 60 / countsrev * fixitnum;
static float ofiltval = 0;  // creates low pass filter for encoder
static float ofilt = 0.90;  // filter param

static float dband = 200;  // encoder deadband setting
static float maxthrottle = 2048;

//******** MAVLINK DEFS **********//
#define SYSTEM_ID 1
#define COMP_ID 22

static uint8_t rawbuf[MAVLINK_MAX_PACKET_LEN];
static uint8_t readbuf[128];

static bool led_status = 0;

// --------- MAV sender --------- //
static void __mavsend(mavlink_message_t& msg) {
  // prepare message
  uint8_t crc_extra =
      mavlink_get_crc_extra(&msg);  //< get extra crc for given message
  uint8_t lmin =
      mavlink_min_message_length(&msg);  //< get min length for given message

  uint16_t finlen = mavlink_finalize_message_chan(&msg, msg.sysid, msg.compid,
                                                  0, lmin, msg.len, crc_extra);

  uint16_t len = mavlink_msg_to_send_buffer(&rawbuf[0], &msg);

  Serial1.write(rawbuf, MAVLINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)msg.len);
}

bool __reqmsg(uint8_t sysid, uint8_t compid, uint32_t msg_id, uint32_t usecT) {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(sysid, compid, &msg, sysid, MAV_COMP_ID_ALL,
                                MAV_CMD_SET_MESSAGE_INTERVAL, 1, float(msg_id),
                                float(uint16_t(usecT)) /* interval in usecs */,
                                0 /* all */, 0, 0, 0, 0);

  __mavsend(msg);
  return true;
}
// ************** PERIODIC TASKS ************//
class HeartBeatTask : public Executable {
 private:
  uint8_t __sysid;
  uint8_t __compid;
  uint32_t __counter;

 public:
  HeartBeatTask(uint8_t compid, uint8_t sysid = SYSTEM_ID) {
    __compid = compid;
    __sysid = sysid;
    __counter = 0;
  }

  void setSysId(uint8_t sysid) { __sysid = sysid; }
  void setCompId(uint8_t compid) { __compid = compid; }
  void incrCounter() { __counter++; }

  void exec() override {
    mavlink_message_t msg;

    if (Serial1.availableForWrite() > 1) {
      mavlink_msg_heartbeat_pack(__sysid, __compid, &msg, MAV_TYPE_GROUND_ROVER,
                                 MAV_AUTOPILOT_GENERIC, MAV_MODE_PREFLIGHT,
                                 __counter, MAV_STATE_ACTIVE);
      __mavsend(msg);
      //   printf("Sent hb for comp %d\n", int(__compid));
    }
  }
};

static HeartBeatTask heartbeatTask0(COMP_ID, SYSTEM_ID);

static HeartBeatTask heartbeatTask1(45, SYSTEM_ID);

// -------------------------------------------------------------
void setup() {
  delay(1000);

  // -------------------------------------------------------------
  pinMode(relay_pin, OUTPUT);
  // digitalWrite(relay_pin, LOW);
  digitalWrite(relay_pin, HIGH);

  analogWriteResolution(12);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(stime / 1000.0);
  // myPID.SetOutputLimits(-993, 993);
  myPID.SetOutputLimits(-2048, 2048);

#ifdef WITH_QUAD_DECODE
  menc.setup();
  menc.start();
#endif

  // -------------------------------------------------------------
  Serial.begin(TELEM_BAUD_RATE);

  Serial2.begin(TELEM_BAUD_RATE);
  // Serial2.println("init");

  Serial1.begin(TELEM_BAUD_RATE);
  // Serial1.setTimeout(1);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // ----- start periodic tasks ---- //
  taskManager.scheduleFixedRate(500, &heartbeatTask0);  // each 500 millis
  taskManager.scheduleFixedRate(100, &heartbeatTask1);  // each 50 millis

  // ----- request mav messages ---- //
  __reqmsg(SYSTEM_ID, COMP_ID, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 30000);

  delay(20);
  __reqmsg(SYSTEM_ID, COMP_ID, MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 20000);

  delay(20);
  __reqmsg(SYSTEM_ID, COMP_ID, MAVLINK_MSG_ID_ATTITUDE, 20000);
}

void loop() {
  // ---- execute scheduer loop ---- //
  taskManager.runLoop();

  // ---- communicate with others --- //
  mavlink_message_t msg;
  mavlink_status_t status;

  uint32_t mcast_id = -1;
  uint32_t inst_id = -1;

  int to_read = Serial1.available();
  size_t read = Serial1.readBytes(&rawbuf[0], to_read);

  for (int i = 0; i < read; i++) {
    uint8_t ch = rawbuf[i];

    if (mavlink_parse_char(0, ch, &msg, &status) ==
        MAVLINK_FRAMING_OK) {  // message received
      mavlink_message_t* rxmsg = (mavlink_message_t*)(&msg);

      if (rxmsg->msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
        mavlink_local_position_ned_t lpos;
        mavlink_msg_local_position_ned_decode(rxmsg, &lpos);

        // Serial.printf("LPOS %f;%f;%f;%f;%f,%f\n", lpos.x, lpos.y, lpos.z,
        //               lpos.vx, lpos.vy, lpos.vz);

      } else if (rxmsg->msgid == MAVLINK_MSG_ID_ATTITUDE) {
        mavlink_attitude_t att;
        mavlink_msg_attitude_decode(rxmsg, &att);

        Serial.printf("ATT %f;%f;%f;%f;%f,%f\n", att.pitch, att.roll, att.yaw,
                      att.pitchspeed, att.rollspeed, att.yawspeed);
        Serial.flush();
        heartbeatTask1.incrCounter();

      } else if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        // blink
        // led_status = !led_status;
        digitalWrite(ledPin, HIGH);

        mavlink_heartbeat_t heartbeat;
        mavlink_msg_heartbeat_decode(&msg, &heartbeat);

        Serial.printf(
            "HBEAT %s(%d);%d;%d;%d;%d\n",
            (MAV_TYPE_GROUND_ROVER == heartbeat.type) ? "ROVER" : "UNKNOWN",
            heartbeat.type, static_cast<int>(heartbeat.autopilot),
            static_cast<int>(heartbeat.base_mode),
            static_cast<uint32_t>(heartbeat.custom_mode),
            static_cast<int>(heartbeat.system_status));

        heartbeatTask0.incrCounter();

        if (heartbeat.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
          auto base_custom_mode = static_cast<uint32_t>(heartbeat.custom_mode);
          switch (base_custom_mode) {
            case int(ROVER_MODE_ACRO):
              Serial.println("ROVER_MODE_ACRO");
              break;
            case int(ROVER_MODE_STEERING):
              Serial.println("ROVER_MODE_STEERING");
              break;
            case int(ROVER_MODE_HOLD):
              Serial.println("ROVER_MODE_HOLD");
              break;
            case int(ROVER_MODE_LOITER):
              Serial.println("ROVER_MODE_LOITER");
              break;
            case int(ROVER_MODE_AUTO):
              Serial.println("ROVER_MODE_AUTO");
              break;
            case int(ROVER_MODE_RTL):
              Serial.println("ROVER_MODE_RTL");
              break;
            case int(ROVER_MODE_SMART_RTL):
              Serial.println("ROVER_MODE_SMART_RTL");
              break;
            case int(ROVER_MODE_GUIDED):
              Serial.println("ROVER_MODE_GUIDED");
              break;
            case int(ROVER_MODE_INITIALIZING):
              Serial.println("ROVER_MODE_INITIALIZING");
              break;
            case int(ROVER_MODE_MANUAL):
              Serial.println("ROVER_MODE_MANUAL");
              break;
            default:
              break;
          }
        }
        Serial.flush();

        // blink
        digitalWrite(ledPin, LOW);

      } else {
        // Serial.printf("Mavlink MSG ID : %d\n", rxmsg->msgid);
      }

    }  // if
  }
}
