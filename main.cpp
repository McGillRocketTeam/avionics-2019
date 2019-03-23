#include <Arduino.h>

// Sensors
#include <Adafruit_GPS.h>
#include <TinyGPS++.h>
#include <SD.h>

#define gpsSerial Serial1
#define xtendSerial Serial2
#define DEBUG_MODE true

// These refer to the same sensor. Adafruit's library is used to start it, and TinyGPSPlus is used to read
Adafruit_GPS GPS(&gpsSerial);  // HW serial: TX-->RX, RX-->TX
TinyGPSPlus gps;

// Configuration vars
String file_prefix = "data";
String file_type = ".txt";
const int card_chip_select = BUILTIN_SDCARD;
String header = "Fix,Satellites,Longitude,Latitude,Heading,GPS_speed,GPS_altitude,Time_passed,New_Characters_Processed";

// Functional vars
String datastring;
String cache;  // cached datastrings
int file_num = 0;  // current file number
int cached_records = 0;  // current number of cached records; used to determine whether or not to write to SD
int file_records = 0;  // total records in the current file
int cache_max = 50;  // cache this many records before writing to the file
int max_records = 500;  // maximum number of records per file
bool using_interrupt = false;  // GPS: false by default

uint32_t t_curr;  // timer
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
  gpsSerial.begin(9600); while (!gpsSerial);
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

  file_num = create_file(file_num);
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
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  chars_processed = gps.charsProcessed();
  new_chars_processed = chars_processed - prev_chars_processed;
  prev_chars_processed = chars_processed;

  t_curr = millis();
  
  sats = gps.satellites.value();
  lng = gps.location.lng();
  lat = gps.location.lat();
  heading = gps.course.deg();
  gps_speed = gps.speed.kmph();
  gps_alt = gps.altitude.meters();
  fix = (gps.sentencesWithFix() > 0 && new_chars_processed > 30)? true:false;

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
  datastring += String(lat, 6);
  datastring += ",";
  datastring += String(lng, 6);
  datastring += ",";
  datastring += alt;  // from the BMP
  datastring += ",";
  datastring += String(t_curr);  // in milliseconds
  datastring += ",";
  datastring += temperature;
  datastring += ",";
  
  datastring += String(gps_speed, 6);  // in kmph
  datastring += ",";
  datastring += accel_magnitude;
  datastring += ",";
  
  datastring += String(sats, 0);
  datastring += ",";

  send_data();  // to the xtend

  datastring += "\n";  // end of record
}

static void send_data() {
  xtendSerial.print("S");
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
