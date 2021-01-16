#ifndef VoltronSD_h
#define VoltronSD_h

#include <TimeLib.h>
#include <SD.h>

class VoltronSD {
    public: 
        VoltronSD();
        void InitializeSDcard();
        void test_sd_card();
        void log_message(String str_msg);
    private:
        const int SD_status = 13;
        const char* filename = "LOGS.txt";
        String get_timestamp();
        String get_short_timestamp();
        void InfoTest();
        bool ReadWriteTest();
};

#endif