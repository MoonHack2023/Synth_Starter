#include <Arduino.h>
#include <U8g2lib.h>
#include <string>
#include <vector>
#include <math.h>

#define _USE_MATH_DEFINES
#define M_PI 3.14159265358979323846

const int32_t freq[12] = {262, 277, 294, 311, 330, 349, 367, 392, 415, 440, 466, 494};

// Phase step sizes
const uint32_t stepSizes [] = {
  51076922, //C4
  54112683, //C#4
  57330004, //D4
  60740598, //D#4
  64352275, //E4 
  68178701, //F4
  72231588, //F#4
  76528508, //G4
  81077269, //G#4
  85899345, //A4
  91006452, //A#4
  96426316, //B4
};

const int LUT_SIZE = 1024;
const int Fs = 22000;

class Wave
{
private:
public:
    int32_t LUT[LUT_SIZE];
    uint16_t diff[13];
};

class Sine : public Wave
{
public:
    Sine()
    {
        for (int i = 0; i < LUT_SIZE; i++)
        {
            LUT[i] = (int32_t)(127 * sinf(2.0 * M_PI * (float)i / LUT_SIZE));
        }
        float diff_float[12];
        for (int i = 0; i < 12; i++)
        {
            diff_float[i] = (freq[i] * LUT_SIZE) / Fs;
            diff[i] = diff_float[i];
        }
    }

   int32_t makeVout(int key, int octave) {
        static uint16_t phase = 0;
        int shift = octave - 4;
        int phaseIncrement = (float)stepSizes[key] / Fs * LUT_SIZE;
        
        if (shift > 0) {
            phaseIncrement <<= shift;
        } else if (shift < 0) {
            phaseIncrement >>= -shift;
        }

        phase += phaseIncrement;
        uint16_t index = phase >> 16;
        int32_t sample = LUT[index];

        return sample;

    }
};

