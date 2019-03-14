#include <Arduino.h>

// Sensors
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <TinyGPS++.h>
#include <SD.h>

// Other
#include <Wire.h>  // for I2C communication
#include <utility/imumaths.h>

#define gpsSerial Serial1
#define xtendSerial Serial2
#define DEBUG_MODE true

Adafruit_BMP280 bmp;  // I2C wiring: SCK-->SCL0, SDI-->SDA0
Adafruit_BNO055 bno;  // I2C wiring: SCL-->SCL0, SDA-->SDA0

// These refer to the same sensor. Adafruit's library is used to start it, and TinyGPSPlus is used to read
Adafruit_GPS GPS(&gpsSerial);  // HW serial: TX-->RX, RX-->TX
TinyGPSPlus gps;

// Configuration vars
String file_prefix = "data";
String file_type = ".txt";
const int card_chip_select = BUILTIN_SDCARD;
String header = "Fix,Satellites,Longitude,Latitude,Heading,GPS_speed,GPS_altitude,BMP_altitude,Temperature,Pressure,Speed,Accel_magnitude,Time_passed,New_Characters_Processed,Accel_x,Accel_y,Accel_z,Gyro_x,Gyro_y,Gyro_z,Mag_x,Mag_y,Mag_z";

// Functional vars
String datastring;
String cache;  // cached datastrings
int file_num = 0;  // current file number
int cached_records = 0;  // current number of cached records; used to determine whether or not to write to SD
int file_records = 0;  // total records in the current file
int cache_max = 50;  // cache this many records before writing to the file
int max_records = 500;  // maximum number of records per file
bool using_interrupt = false;  // GPS: false by default

uint32_t t_curr, t_prev;  // timers
float ground_alt, alt_prev;
float accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,temperature,pressure,alt,speed,accel_magnitude;
uint32_t sats,chars_processed,new_chars_processed,prev_chars_processed; bool fix; double gps_speed,gps_alt; float lng,lat,heading;  // GPS vars

int counter = 0;

// Function prototypes to keep the board happy
static int create_file(int file_num);
static void read_data();
static void send_data();
static void store_data();
void use_interrupt(bool);

void setup() {
  Serial.begin(9600); while (!Serial);
  xtendSerial.begin(9600); while (!xtendSerial);

  // GPS setup
  GPS.begin(9600);
  gpsSerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // Comment this line to turn off RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);  // Uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);  // Set the update rate -- TEST TO SEE IF 1HZ or 5HZ WORKS BETTER
  //GPS.sendCommand(PGCMD_ANTENNA);  // Uncomment to request updates on antenna status
  
  #ifdef __arm__
    using_interrupt = false;  // NOTE: we don't want to use interrupts on the Due
  #else
    useInterrupt(true);
  #endif
  delay(1000);
  gpsSerial.println(PMTK_Q_RELEASE);

  // Begin the other sensors
  if (!bmp.begin())
    Serial.println("Error: bmp not detected.");
  if (!bno.begin())
    Serial.println("Error: bno not detected.");
  if (!SD.begin(card_chip_select))
    Serial.println("Error: SD card not detected.");

  file_num = create_file(file_num);

  ground_alt = bmp.readAltitude();
  t_prev = millis();
  alt_prev = ground_alt;
}

void loop() {
  read_data();
  store_data();
}

// Create a new file, usually with an index one higher than the previous
static int create_file(int file_num) {
  int current_num = file_num + 1;
  String file_name;

  while(true) {
    file_name = file_prefix + current_num + file_type;
    char data_file_char_array[file_name.length() + 1];
    file_name.toCharArray(data_file_char_array, file_name.length() + 1);

    // If the file doesn't exist, create it, otherwise try the next number
    if (!SD.exists(data_file_char_array)) {
      File datafile = SD.open(data_file_char_array, FILE_WRITE);
      datafile.println(header);
      datafile.close();
      break;
    } else {
      current_num++;
    }
  }
  return current_num;
}

static void read_data() {
  // ############ Read from the sensors ############
  imu::Vector<3> accel_euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  accel_x = accel_euler.x();
  accel_y = accel_euler.y();
  accel_z = accel_euler.z();

  imu::Vector<3> gyro_euler = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  gyro_x = gyro_euler.x();
  gyro_y = gyro_euler.y();
  gyro_z = gyro_euler.z();

  imu::Vector<3> mag_euler = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  mag_x = mag_euler.x();
  mag_y = mag_euler.y();
  mag_z = mag_euler.z();

  temperature = bmp.readTemperature();
  pressure = bmp.readPressure();
  alt = bmp.readAltitude() - ground_alt;

  // Calculate velocity and magnitude of acceleration
  t_curr = millis();
  speed = (alt - alt_prev)/((t_curr - t_prev)/1000.0);
  accel_magnitude = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
  t_prev = t_curr;
  alt_prev = alt;

  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  chars_processed = gps.charsProcessed();
  new_chars_processed = chars_processed - prev_chars_processed;
  prev_chars_processed = chars_processed;
  
  sats = gps.satellites.value();
  lng = gps.location.lng();
  lat = gps.location.lat();
  heading = gps.course.deg();
  gps_speed = gps.speed.kmph();
  gps_alt = gps.altitude.meters();
  fix = (gps.sentencesWithFix() > 0 && sats > 0)? true:false;

  if (DEBUG_MODE && counter % 30 == 0) {
    Serial.println("##################### NEW TRANSMISSION ######################");
    Serial.print("sats: "); Serial.println(sats);
    Serial.print("lng: "); Serial.println(lng);
    Serial.print("lat: "); Serial.println(lat);
    Serial.print("fix: "); Serial.println(fix);
    Serial.print("new chars: "); Serial.println(new_chars_processed);
    Serial.println();
  }
  counter++;

  // ############ Create the datastring ############
  datastring += String(fix);
  datastring += ",";
  datastring += String(sats, 0);
  datastring += ",";
  datastring += String(lng, 6);
  datastring += ",";
  datastring += String(lat, 6);
  datastring += ",";
  datastring += String(heading, 6);  // in degrees
  datastring += ",";
  datastring += String(gps_speed, 6);  // in kmph
  datastring += ",";
  datastring += String(gps_alt, 6);  // in metres
  datastring += ",";
  datastring += alt;  // from the BMP
  datastring += ",";
  datastring += temperature;
  datastring += ",";
  datastring += pressure;
  datastring += ",";
  datastring += speed;  // calculated from the IMU data
  datastring += ",";
  datastring += accel_magnitude;
  datastring += ",";
  datastring += String(t_curr);  // in milliseconds
  datastring += ",";
  datastring += String(new_chars_processed);
  datastring += ",";

  send_data();  // to the xtend

  // Add data not-to-transmit to the datastring
  datastring += String(accel_x,6);
  datastring += ",";
  datastring += String(accel_y,6);
  datastring += ",";
  datastring += String(accel_z,6);
  datastring += ",";
  datastring += String(gyro_x,6);
  datastring += ",";
  datastring += String(gyro_y,6);
  datastring += ",";
  datastring += String(gyro_z,6);
  datastring += ",";
  datastring += String(mag_x,6);
  datastring += ",";
  datastring += String(mag_y,6);
  datastring += ",";
  datastring += String(mag_z,6);
  datastring += "\n";  // end of record
}

static void send_data() {
  xtendSerial.print("S,");
  xtendSerial.print(datastring);
  xtendSerial.println("E");
}

static void store_data() {
  cache += datastring;
  datastring = "";
  cached_records++;

  // Only empty cache to the SD card if the number of records exceeds the limit  
  if (cached_records >= cache_max) {
    String file_name = file_prefix + file_num + file_type;
    char data_file_char_array[file_name.length() + 1];
    file_name.toCharArray(data_file_char_array, file_name.length() + 1);

    if (SD.exists(data_file_char_array)) {
      File dataFile = SD.open(data_file_char_array, FILE_WRITE);
    
      if (dataFile) {
        dataFile.print(cache);
        dataFile.close();

        // Clear the cache and update the counters
        file_records += cached_records;
        cached_records = 0;
        cache = "";
      }
    }

    if (file_records >= max_records) {
      if (DEBUG_MODE) { Serial.print("File num: "); Serial.println(file_num); }
      file_num = create_file(file_num);
      file_records = 0;
    }
  }
}

#ifdef __AVR__
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = gps.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (DEBUG_MODE)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    using_interrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    using_interrupt = false;
  }
}
#endif //#ifdef__AVR__