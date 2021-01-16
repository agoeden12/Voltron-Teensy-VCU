#ifndef VoltronSD_h
#define VoltronSD_h

#include <TimeLib.h>
#include <SD.h>

class VoltronSD {
    public: 
        VoltronSD();
        void InitializeSDcard();
        void InfoTest();
        bool ReadWriteTest();
        void test_sd_card();
        void log_message(String str_msg);
        String get_timestamp();
    private:
        const int SD_status = 13;
        const char* filename = "LOGS.txt";
};

#endif