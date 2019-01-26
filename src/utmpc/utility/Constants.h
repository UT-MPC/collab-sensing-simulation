/*
 * Constants.h
 * Defines constants used in ND and App layers.
 *
 */
#ifndef UTMPC_UTILITY_CONSTANTS_H_
#define UTMPC_UTILITY_CONSTANTS_H_

namespace scentssim {

enum ContextType {
    kTypeAcceleration = 0, kTypeAirQuality,  //1
    kTypeAtmosphericPressure,  //2
    kTypeOrientation,  //3
    kTypeHumidity,  //4
    kTypeAmbientNoise,  //5
    kTypeMotion,  //6
    kTypePosition,  //7
    kTypeAddtionalType1, //8
    kTypeAddtionalType2, //9
    kTypeAddtionalType3, //10
    kTypeAddtionalType4,
    kTypeAddtionalType5,
    kTypeAddtionalType6,
    kTypeAddtionalType7,
    kTypeAddtionalType8,
    kTypeAddtionalType9,
    kTypeAddtionalType10,
    kTypeAddtionalType11,
    kTypeAddtionalType12,
    kTypeAddtionalType13, //20
    kTypeAddtionalType14,
    kTypeAddtionalType15,
    kTypeAddtionalType16,
    kTypeAddtionalType17,
    kTypeAddtionalType18,
    kTypeAddtionalType19,
    kTypeAddtionalType20,
    kTypeAddtionalType21,
    kTypeAddtionalType22,
    kTypeAddtionalType23, //30
    kTypeAddtionalType24,

    kContextTypeSize
};

enum QueryMode {
    kQueryModeComplete = 100,
    kQueryModeNone,
    kQueryModeRandomSubset,
    kQuerySubset,
};

enum CapabilityMode {
    kCapabilityNone = 200,
    kCapabilityOmni,
    kCapabilityRandomSubset,
    kCapabilitySubset,
    kCapabilityRarityTest,
};

enum SignalType {
    kSignalTypeLocalCheck = 300,
    kSignalTypeQuery,
    kSignalTypeRequest,
    kSignalTypeResponse,
    kSignalTypeFailedNoCandidate,
    kSignalTypeFailedBeaconFull,
};

enum BlendMode {
    kUBLEND = 0, kBBLEND, kFBLEND
};

enum NDModuleState {
    kNDStateAdv = 1, kNDStateIdle, kNDStateRecv, kNDStateScan, kNDStateWarmUp
};

enum NDTimer {
    kTimerNull = 0,
    kTimerWarmUp,
    kTimerAdv,
    kTimerScanComplete,
    kTimerEpochStart
};

enum FulfillerStrategy {
    kOptionIndependent, kOptionRandomSelect, kOptionOptimized
};

enum AnswerType {
    kOriginalAnswer, kReusedAnswer, kAnswerTypeUninitialized
};

enum SharingStrategy {
    kStgyNone, kStgyBasicGreedy, kStgyRandomizedGreedy, kStgyRarityWeighted, kCollisionAware
};

enum Framework {
    kFrameworkNone, kFrameworkScents
};

enum ShareDurationOption {
    kDurationFixed, kDurationDynamic
};

enum DemandModel {
    kStaticEquallyImportant,
    kStaticLinearDecay,
    kStaticExponentialDecay,
    kDynamicEquallyImportant,
    kDynamicLinearDecay,
    kDynamicExponentialDecay
};

constexpr double kTimeTransmission = 3.2;   // ms
constexpr int kMaxSlackPeriod = 10;    // ms
constexpr char kDelimiter = ';';
constexpr char kMsgDelimiter = '+';
constexpr char kBcnDelimiter = '|';
constexpr char kBcnInfoDelimiter = '-';
constexpr int kCandidateNotFound = -77;
constexpr int kUninitializedInt = -7;
constexpr double kUninitializedDouble = -7.0;
constexpr double kInitializedDouble = 8.0;
constexpr int kNumEpochBeforeLost = 5;
constexpr int kNumLambdaLost = 3;
constexpr int kProcessedQueueLength = 20;
constexpr float kPacketValidLambda = 1.2;

constexpr int kTTLAcceleration = 10;
constexpr int kTTLAirQuality = 60;
constexpr int kTTLAtmosphericPressure = 60;
constexpr int kTTLOrientation = 10;
constexpr int kTTLHumidity = 30;
constexpr int kTTLAmbientNoise = 20;
constexpr int kTTLMotion = 8;
constexpr int kTTLPosition = 8;

constexpr float kCostAcceleration = 0.359;
constexpr float kCostAirQuality = 0.0043;
constexpr float kCostAtmosphericPressure = 0.000021;
constexpr float kCostOrientation = 0.005433;
constexpr float kCostHumidity = 0.000013;
constexpr float kCostAmbientNoise = 1.114;
constexpr float kCostMotion = 0.005433;
constexpr float kCostPosition = 6.616;
constexpr float kCostTemperature = 1.3;
constexpr int kMaxSensorReading = 3;

constexpr double kCommRangeM = 50.0;
constexpr float kDefaultStability = 0.9;

constexpr int kMaxContextTypes = 32;
constexpr int kMaxNeighborCount = 64;
constexpr int kBeaconValidDuration = 1;

constexpr short kKindNewEpochBeacons = 18;
constexpr int kUncompressedLimit = 1000;
constexpr uint8_t kNumBitsDuration = 3;
constexpr int kDefaultContextTypeSize = 8;

constexpr int kRecordIntegrityGuard = 777; // For reconstructing the records when out-of-order writing happens
constexpr int kMaxPacketCount = 3;

}   // end of scentssim

#endif /* UTMPC_UTILITY_CONSTANTS_H_ */
