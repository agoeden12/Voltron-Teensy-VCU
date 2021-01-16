#include "VoltronSD.h"

String VoltronSD::get_short_timestamp(){
  String timestamp = "";

  if(hour() < 10)
    timestamp += "0";
  timestamp += hour();
  timestamp += ":";
  if(minute() < 10)
    timestamp += "0";
  timestamp += minute();
  timestamp += ":";
  if(second() < 10)
    timestamp += "0";
  timestamp += second();

  return timestamp;
}

VoltronSD::VoltronSD(){
  pinMode(SD_status, OUTPUT);
}

void VoltronSD::test_sd_card()
{
  InitializeSDcard();
  
  if (ReadWriteTest()){
    Serial.println("OK");
    digitalWrite(SD_status, HIGH);
    delay(10 * 1000);
    digitalWrite(SD_status, LOW);
  }
  else
  {
    Serial.println("ERROR");
    for (int i = 1;i <= 10;i++)
    {
      digitalWrite(SD_status, HIGH);
      delay(500);
      digitalWrite(SD_status, LOW);
      delay(500);
    }
  }
}

void VoltronSD::InitializeSDcard()
{
  if (!SD.begin(BUILTIN_SDCARD)) {
    for (int i = 1;i <= 10;i++)
    {
      digitalWrite(SD_status, HIGH);
      delay(400);
      digitalWrite(SD_status, LOW);
      delay(600);
    }
    return;
  } else {
    digitalWrite(SD_status, HIGH);
  }

  File log_file = SD.open(filename, FILE_WRITE);

  if (log_file) {
    log_file.println("");
    log_file.println("");
    log_file.print("VCU Started  ");
    log_file.println(get_timestamp());
    log_file.close();
  } 
  else 
  {
    // Error writing to the file
    for (int i = 1;i <= 10;i++)
    {
      digitalWrite(SD_status, HIGH);
      delay(100);
      digitalWrite(SD_status, LOW);
      delay(900);
    }
  }
  digitalWrite(SD_status, LOW);
}

void VoltronSD::log_message(String msg)
{
  digitalWrite(SD_status, HIGH);
  File log_file = SD.open(filename, FILE_WRITE);

  if (log_file) {
    log_file.print(get_short_timestamp());
    log_file.print("  ");
    log_file.println(msg);
    log_file.close();
  } 
  else 
  {
    // Error writing to the file
    for (int i = 1;i <= 10;i++)
    {
      digitalWrite(SD_status, HIGH);
      delay(100);
      digitalWrite(SD_status, LOW);
      delay(900);
    }
  }
  digitalWrite(SD_status, LOW);
}

String VoltronSD::get_timestamp(){
  String timestamp = get_short_timestamp();

  timestamp += " ";
  timestamp += month();
  timestamp += "/";
  timestamp += day();
  timestamp += "/";
  timestamp += year();


  return timestamp;
}

void VoltronSD::InfoTest()
{
  Sd2Card SDcard;
  SdVolume volume;
  SdFile root;

  // Card information
  Serial.print("\nCard info: ");
  switch(SDcard.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.print("SD1"); break;
    case SD_CARD_TYPE_SD2:
      Serial.print("SD2"); break;
    case SD_CARD_TYPE_SDHC:
      Serial.print("SDHC"); break;
    default:
      Serial.print("Unknown");
  }

  // Find the volume on the SD card
  if (!volume.init(SDcard)) {
    Serial.println("\nNo FAT16/FAT32 partition.");
    return;
  }

  // FAT type
  uint32_t volumesize;
  Serial.print(", FAT"); Serial.print(volume.fatType(), DEC);
  Serial.print(", ");
  
  // Sector size (or Blocks) is fixed at 512 bytes
  volumesize = volume.blocksPerCluster() * volume.clusterCount() * 512;
  Serial.print(volumesize/1024);
  Serial.println(" Kb"); 

  Serial.println("\nFiles on the SD card: ");
  root.openRoot(volume);
  
  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);
}

bool VoltronSD::ReadWriteTest()
{
  File myFile;
  char filename[] = "testfile.txt";
  char writestring[] = "abcdefghijklmnopqrstuvwxyz1234567890";
  char readstring[40];

  // First remove the file is it already exists
  if (SD.exists(filename)) {
    SD.remove(filename);
  }
  
  // Open file to write
  myFile = SD.open(filename, FILE_WRITE);
  
  // If okay, write to the file
  if (myFile) {
    Serial.print("Writing to file...  ");
    myFile.print(writestring);
    myFile.close();
    Serial.print('[');
    Serial.print(writestring);
    Serial.println("] done.");
  } 
  else 
  {
    // Error writing to the file
    Serial.println("error opening testfile.txt");
  }
  
  // Open file to read. Which is the default option
  myFile = SD.open(filename, FILE_READ);
  if (myFile) {
    Serial.print("Reading from file...");
    int n = 0;
    while (myFile.available()) {
      if (n<39)
      {
        readstring[n] = myFile.read();
        readstring[n+1] = '\0';
      }
      n=n+1;
    }
    myFile.close();
    Serial.print('[');
    Serial.print(readstring);
    Serial.println("] done.");
  } 
  else 
  {
    // Error reading from the file
    Serial.println("error opening testfile.txt");
  }

  // Return true if the two char arrays are equal
  if (strcmp(writestring, readstring) == 0){
    return true;
  }
  return false;
}