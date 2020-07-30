#ifndef SCURVEGENERATOR_H
#define SCURVEGENERATOR_H

#include "SCurveGeneratorData.h"

constexpr int sCurveLookupTableSize = sizeof(sCurveLookupTable) / sizeof(sCurveLookupTable[0]);

constexpr float pi = 3.14159265358979f;
constexpr float pi2 = pi * pi;

constexpr float at = 2 * pi2 / sCurveLookupTableSize;

enum class SCurveGeneratorStage
{
    STOP,
    ACCELERATION,
    DECELERATION,
    CRUISE
};

class SCurveGenerator {
public:
    // transitionTimeUS -- time of acceleration / deceleration stage in microseconds
    // cruiseDelayUS -- time between stepper motor steps in microseconds in cruise mode
    SCurveGenerator(uint32_t transitionTimeUS, uint32_t cruiseDelayUS) {
        // transitionTimeUS = 2 * pi * kt
        kt = static_cast<float>(transitionTimeUS) / (2 * pi);

        // cruise_speed_us = (pi * kt / numberOfTransitionSteps);
        numberOfTransitionSteps = (pi * kt) / cruiseDelayUS;

        deltaCruise = (pi * kt / numberOfTransitionSteps);
    }

    uint32_t getDelayUS()
    {
        if (stage == SCurveGeneratorStage::ACCELERATION) {
            if (i == numberOfTransitionSteps) {
                stage = SCurveGeneratorStage::CRUISE;
            } else {
                float d = 2 * pi2 * i / numberOfTransitionSteps;
                t = solve(d) * kt;
                delta = t - tPrevious;
                tPrevious = t;

                ++i;
            }
        } else if (stage == SCurveGeneratorStage::CRUISE) {
            delta = deltaCruise;
        } else if (stage == SCurveGeneratorStage::DECELERATION) {
            if (i == 0) {
                stage = SCurveGeneratorStage::STOP;
            } else {
                float d = 2 * pi2 * i / numberOfTransitionSteps;
                t = solve(d) * kt;
                delta = tPrevious - t;
                tPrevious = t;

                --i;
            }
        } else if (stage == SCurveGeneratorStage::STOP) {
        }

        return delta;
    }

    void start() 
    {
        if (stage == SCurveGeneratorStage::STOP) {
            stage = SCurveGeneratorStage::ACCELERATION;
        }
    }

    void stop()
    {
        if (stage == SCurveGeneratorStage::CRUISE) {
            i = numberOfTransitionSteps;
            stage = SCurveGeneratorStage::DECELERATION;
        }
    }

    SCurveGeneratorStage getStage()
    {
        return stage;
    }

    int getNumberOfTransitionSteps()
    {
        return numberOfTransitionSteps;
    }

private:
    float solve(float t) 
    {
        if (t >= 2 * pi2)
            return sCurveLookupTable[sCurveLookupTableSize - 1];
        int n = (t / at);
        float dt = t - n * at;
        return (sCurveLookupTable[n + 1] - sCurveLookupTable[n]) * dt / at + sCurveLookupTable[n];
    }

    int i = 0;
    int numberOfTransitionSteps; // number of accel/decel steps
    float delta = 0;
    float deltaCruise = 0;
    float t = 0, tPrevious = 0;
    float kt; // time adjust constant, accel time = 2 * pi * kt

    SCurveGeneratorStage stage = SCurveGeneratorStage::STOP;
};

#endif /* SCURVEGENERATOR_H */