#include "fuzzylite.h"
#include "fl/Headers.h"
#include <iostream>

using namespace fl;


Engine* engine{nullptr};

InputVariable* eLin{nullptr};
InputVariable* eAng{nullptr};

InputVariable* obstacleDistLeft{nullptr};
InputVariable* obstacleDistFront{nullptr};
InputVariable* obstacleDistRight{nullptr};


OutputVariable* outSpeed{nullptr};
OutputVariable* outAng{nullptr};

OutputVariable* outSpeedObsAvoid{nullptr};
OutputVariable* outAngObsAvoid{nullptr};

OutputVariable* outAngBackward{nullptr};


RuleBlock* rDeslocLin{nullptr};
RuleBlock* rDeslocAng{nullptr};

RuleBlock* rDeslocLinObsAvoid{nullptr};
RuleBlock* rDeslocAngObsAvoid{nullptr};

RuleBlock* rDeslocAngBackward{nullptr};


void FuzzyliteData::loadFuzzyLinearDesloc()
{
    engine = new Engine{};

    engine->setName("DeslocLin");
    engine->setDescription("");

    eLin = new InputVariable{};
    eLin->setName("Error_Linear");
    eLin->setDescription("");
    eLin->setEnabled(true);
    eLin->setRange(0.000, 5.000);
    eLin->setLockValueInRange(true);
    eLin->addTerm(new Triangle{"Z_Close", 0.0, 0.05, 0.15, 2.0});
    eLin->addTerm(new Triangle{"V_Close", 0.1, 0.25, 0.4, 2.0});
    eLin->addTerm(new Triangle{"Close", 0.25, 1.0, 1.2});
    eLin->addTerm(new Triangle{"Far", 0.8, 1.5, 2.0});
    eLin->addTerm(new Triangle{"V_Far", 1.5, 5.0, 5.0});
    engine->addInputVariable(eLin);

    eAng = new InputVariable{};
    eAng->setName("Error_Angular");
    eAng->setDescription("");
    eAng->setEnabled(true);
    eAng->setRange(-M_PI, M_PI);
    eAng->setLockValueInRange(true);
    eAng->addTerm(new Triangle{"V_Neg", -M_PI, -M_PI, -M_PI/6});
    eAng->addTerm(new Triangle{"Neg", -M_PI/3, -M_PI/9, -M_PI/100});
    eAng->addTerm(new Triangle{"Neg_Zero", -M_PI/20, -M_PI/100, 0});
    eAng->addTerm(new Triangle{"Zero", -M_PI/200, 0, M_PI/200});
    eAng->addTerm(new Triangle{"Posi_Zero", 0, M_PI/100, M_PI/20});
    eAng->addTerm(new Triangle{"Posi", M_PI/100, M_PI/9, M_PI/3});
    eAng->addTerm(new Triangle{"V_Posi", M_PI/6, M_PI, M_PI});
    engine->addInputVariable(eAng);

    outSpeed = new OutputVariable{};
    outSpeed->setName("Speed");
    outSpeed->setDescription("");
    outSpeed->setEnabled(true);
    outSpeed->setRange(-0.05, 1.5);
    outSpeed->setLockValueInRange(true);
    outSpeed->setAggregation(new Maximum);
    outSpeed->setDefuzzifier(new Centroid(100));
    outSpeed->setDefaultValue(0.0);
    outSpeed->setLockPreviousValue(true);
    outSpeed->addTerm(new Triangle{"Zero", -0.05, 0.05, 0.2, 3});
    outSpeed->addTerm(new Triangle{"Z_Slow", 0.1, 0.15, 0.25, 2});
    outSpeed->addTerm(new Triangle{"V_Slow", 0.15, 0.3, 0.5, 2});
    outSpeed->addTerm(new Triangle{"Slow", 0.4, 0.6, 0.8});
    outSpeed->addTerm(new Triangle{"Fast", 0.6, 0.8, 0.9});
    outSpeed->addTerm(new Triangle{"V_Fast", 0.8, 1.2, 1.5});
    engine->addOutputVariable(outSpeed);

    rDeslocLin = new RuleBlock{};
    rDeslocLin->setName("rDeslocLin");
    rDeslocLin->setDescription("");
    rDeslocLin->setEnabled(true);
    rDeslocLin->setConjunction(new Minimum{});
    rDeslocLin->setDisjunction(new Maximum{});
    rDeslocLin->setImplication(new AlgebraicProduct);
    rDeslocLin->setActivation(new General);

    rDeslocLin->addRule(Rule::parse("if Error_Angular is V_Neg or Error_Angular is V_Posi then Speed is V_Slow", engine));

    // Fine error handling
    rDeslocLin->addRule(Rule::parse("if Error_Linear is Z_Close then Speed is Zero", engine));

    rDeslocLin->addRule(Rule::parse("if Error_Linear is V_Close then Speed is Z_Slow", engine));

    rDeslocLin->addRule(Rule::parse("if Error_Linear is Close then Speed is V_Slow", engine));

    rDeslocLin->addRule(Rule::parse("if Error_Linear is Far and Error_Angular is Zero then Speed is Fast", engine));
    rDeslocLin->addRule(Rule::parse("if Error_Linear is Far and Error_Angular is not Zero then Speed is Slow", engine));

    rDeslocLin->addRule(Rule::parse("if Error_Linear is V_Far and Error_Angular is Zero then Speed is V_Fast", engine));
    rDeslocLin->addRule(Rule::parse("if Error_Linear is V_Far and Error_Angular is not Zero then Speed is Fast", engine));

    engine->addRuleBlock(rDeslocLin);
}

void FuzzyliteData::loadFuzzyAngDesloc()
{
    outAng = new OutputVariable{};
    outAng->setName("SpeedAng");
    outAng->setDescription("");
    outAng->setEnabled(true);
    outAng->setRange(-M_PI*2, M_PI*2);
    outAng->setLockValueInRange(true);
    outAng->setAggregation(new Maximum);
    outAng->setDefuzzifier(new Centroid(100));
    outAng->setDefaultValue(0.0);
    outAng->setLockPreviousValue(true);
    outAng->addTerm(new Triangle{"Zero", -M_PI/30, 0.0, M_PI/30});
    outAng->addTerm(new Triangle{"Posi_Zero", 0.0, M_PI/30, M_PI/15, 2});
    outAng->addTerm(new Triangle{"Nega_Zero", -M_PI/15, -M_PI/30, 0, 2});
    outAng->addTerm(new Triangle{"Posi", M_PI/30, M_PI/8, M_PI/3, 2});
    outAng->addTerm(new Triangle{"Nega", -M_PI/3, -M_PI/8, -M_PI/30, 2});
    outAng->addTerm(new Triangle{"V_Posi", M_PI/4, M_PI*2, M_PI*2});
    outAng->addTerm(new Triangle{"V_Nega", -M_PI*2, -M_PI*2, -M_PI/4});
    engine->addOutputVariable(outAng);

    rDeslocAng = new RuleBlock{};
    rDeslocAng->setName("rDeslocAng");
    rDeslocAng->setDescription("");
    rDeslocAng->setEnabled(true);
    rDeslocAng->setConjunction(new Minimum{});
    rDeslocAng->setDisjunction(new Maximum{});
    rDeslocAng->setImplication(new AlgebraicProduct);
    rDeslocAng->setActivation(new General);

    rDeslocAng->addRule(Rule::parse("if Error_Angular is Zero then SpeedAng is Zero", engine));
    rDeslocAng->addRule(Rule::parse("if Error_Angular is Posi_Zero then SpeedAng is Posi_Zero", engine));
    rDeslocAng->addRule(Rule::parse("if Error_Angular is Neg_Zero then SpeedAng is Nega_Zero", engine));
    rDeslocAng->addRule(Rule::parse("if Error_Angular is Posi then SpeedAng is Posi", engine));
    rDeslocAng->addRule(Rule::parse("if Error_Angular is Neg then SpeedAng is Nega", engine));
    rDeslocAng->addRule(Rule::parse("if Error_Angular is V_Posi then SpeedAng is V_Posi", engine));
    rDeslocAng->addRule(Rule::parse("if Error_Angular is V_Neg then SpeedAng is V_Nega", engine));

    engine->addRuleBlock(rDeslocAng);
}

void FuzzyliteData::loadFuzzyObstacleDist()
{

    obstacleDistLeft = new InputVariable{};
    obstacleDistLeft->setName("Obs_Dist_L");
    obstacleDistLeft->setDescription("");
    obstacleDistLeft->setEnabled(true);
    obstacleDistLeft->setRange(0.000, 2.000);
    obstacleDistLeft->setLockValueInRange(true);
//    obstacleDistLeft->addTerm(new Triangle{"V_Close", 0.0, 0.1, 0.35, 2.0});
//    obstacleDistLeft->addTerm(new Triangle{"Close", 0.2, 0.5, 1.2});
    obstacleDistLeft->addTerm(new Triangle{"Close_Zero", 0.0, 0.2, 0.25});
//    obstacleDistLeft->addTerm(new Triangle{"V_Close", 0.0, 0.3, 0.5, 2.0});
//    obstacleDistLeft->addTerm(new Triangle{"Close", 0.4, 0.6, 1.2});
    obstacleDistLeft->addTerm(new Triangle{"V_Close", 0.0, 0.25, 0.45, 2.0});
    obstacleDistLeft->addTerm(new Triangle{"Close", 0.35, 0.6, 1.2});
    obstacleDistLeft->addTerm(new Triangle{"Far", 0.6, 1.2, 2.0});
    engine->addInputVariable(obstacleDistLeft);

    // Clone obstacleDistLeft to obstacleDistFront, as it uses same info
    obstacleDistFront = obstacleDistLeft->clone();
    obstacleDistFront->setName("Obs_Dist_F");
    engine->addInputVariable(obstacleDistFront);

    // Clone obstacleDistLeft to obstacleDistRight, as it uses same info
    obstacleDistRight = obstacleDistLeft->clone();
    obstacleDistRight->setName("Obs_Dist_R");
    engine->addInputVariable(obstacleDistRight);

    outSpeedObsAvoid = new OutputVariable{};
    outSpeedObsAvoid->setName("SpeedObsAvoid");
    outSpeedObsAvoid->setDescription("");
    outSpeedObsAvoid->setEnabled(true);
    outSpeedObsAvoid->setRange(-0.200, 0.7); // -0.2 to allow 0 output
    outSpeedObsAvoid->setLockValueInRange(true);
    outSpeedObsAvoid->setAggregation(new Maximum);
    outSpeedObsAvoid->setDefuzzifier(new Centroid(100));
    outSpeedObsAvoid->setDefaultValue(0.0);
    outSpeedObsAvoid->setLockPreviousValue(true);
    outSpeedObsAvoid->addTerm(new Triangle{"Zero", -0.2, 0.0, 0.2, 2});
    outSpeedObsAvoid->addTerm(new Triangle{"V_Slow", 0.1, 0.25, 0.4, 2});
    outSpeedObsAvoid->addTerm(new Triangle{"Slow", 0.25, 0.5, 0.7});
    engine->addOutputVariable(outSpeedObsAvoid);


    outAngObsAvoid = new OutputVariable{};
    outAngObsAvoid->setName("SpeedAngObsAvoid");
    outAngObsAvoid->setDescription("");
    outAngObsAvoid->setEnabled(true);
    outAngObsAvoid->setRange(-M_PI*1.5, M_PI*1.5);
    outAngObsAvoid->setLockValueInRange(true);
    outAngObsAvoid->setAggregation(new Maximum);
    outAngObsAvoid->setDefuzzifier(new Centroid(100));
    outAngObsAvoid->setDefaultValue(0.0);
    outAngObsAvoid->setLockPreviousValue(true);
    outAngObsAvoid->addTerm(new Triangle{"M_Nega", -M_PI*1.5, -M_PI*1.5, -M_PI/2});
    outAngObsAvoid->addTerm(new Triangle{"V_Nega", -M_PI*3/4, -M_PI/2, -M_PI/4});
    outAngObsAvoid->addTerm(new Triangle{"Nega", -M_PI/2, -M_PI/5, 0, 2});
    outAngObsAvoid->addTerm(new Triangle{"Zero", -M_PI/12, 0.0, M_PI/12});
    outAngObsAvoid->addTerm(new Triangle{"Posi", 0, M_PI/5, M_PI/2, 2});
    outAngObsAvoid->addTerm(new Triangle{"V_Posi", M_PI/4, M_PI/2, M_PI*3/4});
    outAngObsAvoid->addTerm(new Triangle{"M_Posi", M_PI/2, M_PI*1.5, M_PI*1.5});
    engine->addOutputVariable(outAngObsAvoid);

    rDeslocLinObsAvoid = new RuleBlock{};
    rDeslocLinObsAvoid->setName("rDeslocLinObsAvoid");
    rDeslocLinObsAvoid->setDescription("");
    rDeslocLinObsAvoid->setEnabled(true);
    rDeslocLinObsAvoid->setConjunction(new Minimum{});
    rDeslocLinObsAvoid->setDisjunction(new Maximum{});
    rDeslocLinObsAvoid->setImplication(new AlgebraicProduct);
    rDeslocLinObsAvoid->setActivation(new General);

    rDeslocLinObsAvoid->addRule(Rule::parse("if Obs_Dist_L is V_Close or "
                                            "Obs_Dist_F is V_Close or "
                                            "Obs_Dist_R is V_Close "
                                            "then SpeedObsAvoid is Zero", engine));

    rDeslocLinObsAvoid->addRule(Rule::parse("if Obs_Dist_L is Close and "
                                            "Obs_Dist_F is not V_Close and "
                                            "Obs_Dist_R is not V_Close "
                                            "then SpeedObsAvoid is V_Slow", engine));

    rDeslocLinObsAvoid->addRule(Rule::parse("if Obs_Dist_F is Close and "
                                            "Obs_Dist_L is not V_Close and "
                                            "Obs_Dist_R is not V_Close "
                                            "then SpeedObsAvoid is V_Slow", engine));

    rDeslocLinObsAvoid->addRule(Rule::parse("if Obs_Dist_R is Close and "
                                            "Obs_Dist_L is not V_Close and "
                                            "Obs_Dist_F is not V_Close "
                                            "then SpeedObsAvoid is V_Slow", engine));

    rDeslocLinObsAvoid->addRule(Rule::parse("if Obs_Dist_L is Far and "
                                            "Obs_Dist_F is Far and "
                                            "Obs_Dist_R is Far "
                                            "then SpeedObsAvoid is Slow", engine));

    engine->addRuleBlock(rDeslocLinObsAvoid);

    rDeslocAngObsAvoid = new RuleBlock{};
    rDeslocAngObsAvoid->setName("rDeslocAngObsAvoid");
    rDeslocAngObsAvoid->setDescription("");
    rDeslocAngObsAvoid->setEnabled(true);
    rDeslocAngObsAvoid->setConjunction(new Minimum{});
    rDeslocAngObsAvoid->setDisjunction(new Maximum{});
    rDeslocAngObsAvoid->setImplication(new AlgebraicProduct);
    rDeslocAngObsAvoid->setActivation(new General);

    // Tries to keep going straight
    rDeslocAngObsAvoid->addRule(Rule::parse("if Obs_Dist_F is Far "
                                            "then SpeedAngObsAvoid is Zero", engine));


    // Gives preference to go to RIGHT move, negative values

    rDeslocAngObsAvoid->addRule(Rule::parse("if Obs_Dist_F is V_Close "
                                            "then SpeedAngObsAvoid is V_Nega", engine));

    rDeslocAngObsAvoid->addRule(Rule::parse("if Obs_Dist_L is V_Close and "
                                            "Obs_Dist_F is not Far "
                                            "then SpeedAngObsAvoid is M_Nega", engine));

    rDeslocAngObsAvoid->addRule(Rule::parse("if Obs_Dist_L is V_Close and "
                                            "Obs_Dist_F is Far "
                                            "then SpeedAngObsAvoid is Nega", engine));

    rDeslocAngObsAvoid->addRule(Rule::parse("if Obs_Dist_L is Close and "
                                            "Obs_Dist_F is not Far "
                                            "then SpeedAngObsAvoid is V_Nega", engine));

    // Moves LEFT only when necessary
    rDeslocAngObsAvoid->addRule(Rule::parse("if Obs_Dist_R is V_Close and "
                                            "Obs_Dist_L is Far "
                                            "then SpeedAngObsAvoid is M_Posi", engine));

    rDeslocAngObsAvoid->addRule(Rule::parse("if Obs_Dist_R is Close and "
                                            "Obs_Dist_L is Far "
                                            "then SpeedAngObsAvoid is V_Posi", engine));

    rDeslocAngObsAvoid->addRule(Rule::parse("if Obs_Dist_R is V_Close and "
                                            "Obs_Dist_L is not Far "
                                            "then SpeedAngObsAvoid is V_Posi", engine));

    rDeslocAngObsAvoid->addRule(Rule::parse("if Obs_Dist_R is Close and "
                                            "Obs_Dist_L is not Far "
                                            "then SpeedAngObsAvoid is Posi", engine));

    engine->addRuleBlock(rDeslocAngObsAvoid);
}

void FuzzyliteData::loadFuzzyBackwardAng()
{
    outAngBackward = outAng->clone();
    outAngBackward->setName("SpeedAngBackward");
    engine->addOutputVariable(outAngBackward);

    rDeslocAngBackward = new RuleBlock{};
    rDeslocAngBackward->setName("rDeslocAngBackward");
    rDeslocAngBackward->setDescription("");
    rDeslocAngBackward->setEnabled(true);
    rDeslocAngBackward->setConjunction(new Minimum{});
    rDeslocAngBackward->setDisjunction(new Maximum{});
    rDeslocAngBackward->setImplication(new AlgebraicProduct);
    rDeslocAngBackward->setActivation(new General);

    std::string commonDistObsLimiterPhrase{"if Obs_Dist_L is not Close_Zero and "
                                           "Obs_Dist_F is not Close_Zero and "
                                           "Obs_Dist_R is not Close_Zero and "};

    rDeslocAngBackward->addRule(Rule::parse(commonDistObsLimiterPhrase + "Error_Angular is Zero then SpeedAngBackward is Zero", engine));
    rDeslocAngBackward->addRule(Rule::parse(commonDistObsLimiterPhrase + "Error_Angular is Posi_Zero then SpeedAngBackward is Posi_Zero", engine));
    rDeslocAngBackward->addRule(Rule::parse(commonDistObsLimiterPhrase + "Error_Angular is Neg_Zero then SpeedAngBackward is Nega_Zero", engine));
    rDeslocAngBackward->addRule(Rule::parse(commonDistObsLimiterPhrase + "Error_Angular is Posi then SpeedAngBackward is Posi", engine));
    rDeslocAngBackward->addRule(Rule::parse(commonDistObsLimiterPhrase + "Error_Angular is Neg then SpeedAngBackward is Nega", engine));
    rDeslocAngBackward->addRule(Rule::parse(commonDistObsLimiterPhrase + "Error_Angular is V_Posi then SpeedAngBackward is V_Posi", engine));
    rDeslocAngBackward->addRule(Rule::parse(commonDistObsLimiterPhrase + "Error_Angular is V_Neg then SpeedAngBackward is V_Nega", engine));

    // Extra rule to ensure when near objects dont move in angular
    rDeslocAngBackward->addRule(Rule::parse("if Obs_Dist_L is Close_Zero or "
                                            "Obs_Dist_F is Close_Zero or "
                                            "Obs_Dist_R is Close_Zero then SpeedAngBackward is Zero", engine));

    engine->addRuleBlock(rDeslocAngBackward);

}


void FuzzyliteData::loadFuzzyData()
{
    FuzzyliteData::loadFuzzyLinearDesloc();
    FuzzyliteData::loadFuzzyAngDesloc();
    FuzzyliteData::loadFuzzyObstacleDist();
    FuzzyliteData::loadFuzzyBackwardAng();

    std::string status;
    if (!engine->isReady(&status))
    {
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);
    }
}

void FuzzyliteData::setErrorValues(const float valErrorLin, const float valErrorAng)
{
    eLin->setValue(valErrorLin);
    eAng->setValue(valErrorAng);

    engine->process();
}

void FuzzyliteData::setObstacleDists(const float valObsL, const float valObsF, const float valObsR)
{
    obstacleDistLeft->setValue(valObsL);
    obstacleDistFront->setValue(valObsF);
    obstacleDistRight->setValue(valObsR);

    engine->process();
}

double FuzzyliteData::getLinSpeed()
{
    const float outVal{outSpeed->getValue()};
    return(0.0 > outVal ? 0.0 : outVal);   
}

double FuzzyliteData::getAngSpeed()
{
    return outAng->getValue();
}

double FuzzyliteData::getLinSpeedAvoidObs()
{
    const float outVal{outSpeedObsAvoid->getValue()};
    return(0.0 > outVal ? 0.0 : outVal);   
}

double FuzzyliteData::getAngSpeedAvoidObs()
{
    return outAngObsAvoid->getValue();
}

double FuzzyliteData::getAngSpeedBackward()
{
    return outAngBackward->getValue();
}
