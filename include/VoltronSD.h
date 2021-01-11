#ifndef VoltronSD_h
#define VoltronSD_h

#include <SD.h>

class VoltronSD {
    public: 
        VoltronSD();
        void InitializeSDcard(int testseconds);
        void InfoTest();
        bool ReadWriteTest();
    private:
        const int SD_status = 13;
};

#endif