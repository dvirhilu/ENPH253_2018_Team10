
#ifndef FrequencyDetection_h
#define FrequencyDetection_h

#include "Arduino.h"

class FrequencyDetection
{
  public:
    FrequencyDetection( int tenkHz, int onekHz, int tinahPin );
    void set10kHzThresh( int thresh );
    void set1kHzThresh( int thresh );
    bool is10kHz();
    bool is1kHz();
    void handle10kHz();
    void handle1kHz();
    void detectFrequency();
  private:
    int pin10kHz;
    int pin1kHz;
    int tinahComPin;
    int thresh10kHz;
    int thresh1kHz;
};

#endif
