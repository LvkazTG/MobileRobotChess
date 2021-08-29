#ifndef FUZZYLITE_DATA
#define FUZZYLITE_DATA

namespace FuzzyliteData
{
    // Funcs to creates fuzzy structure to use
    void loadFuzzyData();

    void loadFuzzyLinearDesloc();

    void loadFuzzyAngDesloc();

    void loadFuzzyObstacleDist();

    void loadFuzzyBackwardAng();

    // Funcs to set values for fuzzy process
    // Calculation of values is done in sequence after insertion
    void setErrorValues(const float valErrorLin, const float valErrorAng);

    void setObstacleDists(const float valObsL, const float valObsF, const float valObsR);

    // Funcs to get value out of fuzzy process
    double getLinSpeed();

    double getAngSpeed();

    double getLinSpeedAvoidObs();

    double getAngSpeedAvoidObs();

    double getAngSpeedBackward();
}

#endif