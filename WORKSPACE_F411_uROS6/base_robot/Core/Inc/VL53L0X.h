/**
 * @file : VL53L0X.h
 * @brief : VL53L0X API STSW-IMG005 portage
 * Most of the functionality of this library is based on the VL53L0X API
 * provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
 * or paraphrased from the API source code, API user manual (UM2039), and the
 * VL53L0X datasheet.
 */

#define ACTIVE_WHILE 0
#define IO_2V8 1
#define ADDRESS 0x52 /**< Default address of VL53L0X sensor */

//#define bool  uint8_t
#define true  1
#define false 0

#define SYSRANGE_START 0x00
#define SYSTEM_THRESH_HIGH 0x0C
#define SYSTEM_THRESH_LOW 0x0E
#define SYSTEM_SEQUENCE_CONFIG 0x01
#define SYSTEM_RANGE_CONFIG 0x09
#define SYSTEM_INTERMEASUREMENT_PERIOD 0x04


#define SYSTEM_INTERRUPT_GPIO_CONFIG 0x0A

//GPIO Config
#define GPIO_HV_MUX_ACTIVE_HIGH 0x84
#define SYSTEM_INTERRUPT_CLEAR 0x0B
#define I2C_MODE 0x88

// Result registers
#define RESULT_INTERRUPT_STATUS 0x13
#define RESULT_RANGE_STATUS 0x14
#define RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN 0xBC
#define RESULT_CORE_RANGING_TOTAL_EVENTS_RTN 0xC0
#define RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF 0xD0
#define RESULT_CORE_RANGING_TOTAL_EVENTS_REF 0xD4
#define RESULT_PEAK_SIGNAL_RATE_REF 0xB6

//Algo Register
#define ALGO_PART_TO_PART_RANGE_OFFSET_MM 0x28

//Check limit register
#define MSRC_CONFIG_CONTROL 0x60
#define PRE_RANGE_CONFIG_MIN_SNR 0x27
#define PRE_RANGE_CONFIG_VALID_PHASE_LOW 0x56
#define PRE_RANGE_CONFIG_VALID_PHASE_HIGH 0x57
#define PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT 0x64
#define FINAL_RANGE_CONFIG_MIN_SNR 0x67
#define FINAL_RANGE_CONFIG_VALID_PHASE_LOW 0x47
#define FINAL_RANGE_CONFIG_VALID_PHASE_HIGH 0x48
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44

// PRE RANGE registers
#define PRE_RANGE_CONFIG_SIGMA_THRESH_HI 0x61
#define PRE_RANGE_CONFIG_SIGMA_THRESH_LO 0x62
#define PRE_RANGE_CONFIG_VCSEL_PERIOD 0x50
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x51
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO 0x52

//Internal tuning registers
#define INTERNAL_TUNING_1 0x91
#define INTERNAL_TUNING_2 0xFF


//Other registers
#define SYSTEM_HISTOGRAM_BIN 0x81
#define HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT 0x33
#define HISTOGRAM_CONFIG_READOUT_CTRL 0x55
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x71
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO 0x72
#define CROSSTALK_COMPENSATION_PEAK_RATE_MCPS 0x20
#define MSRC_CONFIG_TIMEOUT_MACROP 0x46
#define GLOBAL_CONFIG_SPAD_ENABLES_REF0 0x0B0
#define GLOBAL_CONFIG_SPAD_ENABLES_REF1 0x0B1
#define GLOBAL_CONFIG_SPAD_ENABLES_REF2 0x0B2
#define GLOBAL_CONFIG_SPAD_ENABLES_REF3 0x0B3
#define GLOBAL_CONFIG_SPAD_ENABLES_REF4 0x0B4
#define GLOBAL_CONFIG_SPAD_ENABLES_REF5 0x0B5
#define GLOBAL_CONFIG_REF_EN_START_SELECT 0xB6
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD 0x4E
#define DYNAMIC_SPAD_REF_EN_START_OFFSET 0x4F
#define POWER_MANAGEMENT_GO1_POWER_FORCE 0x80
#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV 0x89
#define ALGO_PHASECAL_LIM 0x30
#define ALGO_PHASECAL_CONFIG_TIMEOUT 0x30


#define SYSTEM_THRESH_HIGH                           0x0C
#define SYSTEM_THRESH_LOW                           0x0E

#define  SYSTEM_SEQUENCE_CONFIG                       0x01
#define  SYSTEM_RANGE_CONFIG                          0x09
#define  SYSTEM_INTERMEASUREMENT_PERIOD               0x04

#define  SYSTEM_INTERRUPT_CONFIG_GPIO                 0x0A

#define  GPIO_HV_MUX_ACTIVE_HIGH                      0x84

#define  SYSTEM_INTERRUPT_CLEAR                       0x0B

#define RESULT_INTERRUPT_STATUS                     0x13
#define  RESULT_RANGE_STATUS                          0x14

#define  RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN        0xBC
#define  RESULT_CORE_RANGING_TOTAL_EVENTS_RTN         0xC0
#define  RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF        0xD0
#define  RESULT_CORE_RANGING_TOTAL_EVENTS_REF         0xD4
#define  RESULT_PEAK_SIGNAL_RATE_REF                  0xB6

#define  ALGO_PART_TO_PART_RANGE_OFFSET_MM            0x28

#define  I2C_SLAVE_DEVICE_ADDRESS                     0x8A
//#define  I2C_SLAVE_DEVICE_ADDRESS                     0x53

#define  MSRC_CONFIG_CONTROL                          0x60

#define  PRE_RANGE_CONFIG_MIN_SNR                     0x27
#define  PRE_RANGE_CONFIG_VALID_PHASE_LOW             0x56
#define  PRE_RANGE_CONFIG_VALID_PHASE_HIGH            0x57
#define  PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT           0x64

#define  FINAL_RANGE_CONFIG_MIN_SNR                   0x67
#define  FINAL_RANGE_CONFIG_VALID_PHASE_LOW           0x47
#define  FINAL_RANGE_CONFIG_VALID_PHASE_HIGH          0x48
#define  FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT  0x44

#define  PRE_RANGE_CONFIG_SIGMA_THRESH_HI             0x61
#define  PRE_RANGE_CONFIG_SIGMA_THRESH_LO             0x62

#define  PRE_RANGE_CONFIG_VCSEL_PERIOD                0x50
#define  PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          0x51
#define  PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO           0x52

#define  SYSTEM_HISTOGRAM_BIN                         0x81
#define  HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT        0x33
#define  HISTOGRAM_CONFIG_READOUT_CTRL                0x55

#define  FINAL_RANGE_CONFIG_VCSEL_PERIOD              0x70
#define  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI         0x71
#define  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO         0x72
#define CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       0x20

#define MSRC_CONFIG_TIMEOUT_MACROP                   0x46

#define  SOFT_RESET_GO2_SOFT_RESET_N                  0xBF
#define  IDENTIFICATION_MODEL_ID                      0xC0
#define  IDENTIFICATION_REVISION_ID                  0xC2

#define  OSC_CALIBRATE_VAL                            0xF8

#define  GLOBAL_CONFIG_VCSEL_WIDTH                    0x32
#define  GLOBAL_CONFIG_SPAD_ENABLES_REF_0             0xB0
#define  GLOBAL_CONFIG_SPAD_ENABLES_REF_1             0xB1
#define  GLOBAL_CONFIG_SPAD_ENABLES_REF_2             0xB2
#define  GLOBAL_CONFIG_SPAD_ENABLES_REF_3            0xB3
#define  GLOBAL_CONFIG_SPAD_ENABLES_REF_4             0xB4
#define  GLOBAL_CONFIG_SPAD_ENABLES_REF_5             0xB5

#define  GLOBAL_CONFIG_REF_EN_START_SELECT            0xB6
#define  DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD          0x4E
#define  DYNAMIC_SPAD_REF_EN_START_OFFSET             0x4F
#define  POWER_MANAGEMENT_GO1_POWER_FORCE             0x80

#define  VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV            0x89

#define  ALGO_PHASECAL_LIM                            0x30
#define  ALGO_PHASECAL_CONFIG_TIMEOUT                 0x30

//------------------------------------------------------------
// Defines
//------------------------------------------------------------
// I use a 8-bit number for the address, LSB must be 0 so that I can
// OR over the last bit correctly based on reads and writes
#define ADDRESS_DEFAULT 0b01010010
#define ADDRESS_DEFAULT2 0b00101001
// Record the current time to check an upcoming timeout against
#define startTimeout() (g_timeoutStartMs = millis())

// Check if timeout is enabled (set to nonzero value) and has expired
#define checkTimeoutExpired() (g_ioTimeout > 0 && ((uint16_t)millis() - g_timeoutStartMs) > g_ioTimeout)

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

// register addresses from API vl53l0x_device.h (ordered as listed there)


typedef enum { VcselPeriodPreRange, VcselPeriodFinalRange }vcselPeriodType;

// Additional info for one measurement
typedef struct{
  uint16_t rawDistance; //uncorrected distance  [mm],   uint16_t
  uint16_t signalCnt;   //Signal  Counting Rate [mcps], uint16_t, fixpoint9.7
  uint16_t ambientCnt;  //Ambient Counting Rate [mcps], uint16_t, fixpoint9.7
  uint16_t spadCnt;     //Effective SPAD return count,  uint16_t, fixpoint8.8
  uint8_t  rangeStatus; //Ranging status (0-15)
} statInfo_t;


//------------------------------------------------------------
// API Functions
//------------------------------------------------------------
// configures chip i2c and lib for `new_addr` (8 bit, LSB=0)
void setAddress(uint8_t new_addr);
// Returns the current I²C address.
uint8_t getAddress(void);

// Iniitializes and configures the sensor.
// If the optional argument io_2v8 is 1, the sensor is configured for 2V8 mode (2.8 V I/O);
// if 0, the sensor is left in 1V8 mode. Returns 1 if the initialization completed successfully.
uint8_t initVL53L0X();

// Sets the return signal rate limit to the given value in units of MCPS (mega counts per second).
// This is the minimum amplitude of the signal reflected from the target and received by the sensor
//  necessary for it to report a valid reading. Setting a lower limit increases the potential range
// of the sensor but also increases the likelihood of getting an inaccurate reading because of
//  reflections from objects other than the intended target. This limit is initialized to 0.25 MCPS
//  by default. The return value is a boolean indicating whether the requested limit was valid.
uint8_t setSignalRateLimit(float limit_Mcps);

// Returns the current return signal rate limit in MCPS.
float getSignalRateLimit(void);

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
uint8_t setMeasurementTimingBudget(uint32_t budget_us);

// Returns the current measurement timing budget in microseconds.
uint32_t getMeasurementTimingBudget(void);

// Sets the VCSEL (vertical cavity surface emitting laser) pulse period for the given period type
// (VcselPeriodPreRange or VcselPeriodFinalRange) to the given value (in PCLKs).
// Longer periods increase the potential range of the sensor. Valid values are (even numbers only):
// Pre: 12 to 18 (initialized to 14 by default)
// Final: 8 to 14 (initialized to 10 by default)
// The return value is a boolean indicating whether the requested period was valid.
uint8_t setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);

// Returns the current VCSEL pulse period for the given period type.
uint8_t getVcselPulsePeriod(vcselPeriodType type);

// Starts continuous ranging measurements. If the argument period_ms is 0,
// continuous back-to-back mode is used (the sensor takes measurements as often as possible);
// if it is nonzero, continuous timed mode is used, with the specified inter-measurement period
// in milliseconds determining how often the sensor takes a measurement.
void startContinuous(uint32_t period_ms);

// Stops continuous mode.
void stopContinuous(void);

// Returns a range reading in millimeters when continuous mode is active.
// Additional measurement data will be copied into `extraStats` if it is non-zero.
uint16_t readRangeContinuousMillimeters(/* statInfo_t *extraStats*/ );

// Performs a single-shot ranging measurement and returns the reading in millimeters.
// Additional measurement data will be copied into `extraStats` if it is non-zero.
uint16_t readRangeSingleMillimeters( /*statInfo_t *extraStats */);

// Sets a timeout period in milliseconds after which read operations will abort
// if the sensor is not ready. A value of 0 disables the timeout.
void setTimeout(uint16_t timeout);

// Returns the current timeout period setting.
uint16_t getTimeout(void);

// Indicates whether a read timeout has occurred since the last call to timeoutOccurred().
bool timeoutOccurred(void);

//---------------------------------------------------------
// I2C communication Functions
//---------------------------------------------------------
void writeReg(uint8_t reg, uint8_t value);        // Write an 8-bit register
void writeReg16Bit(uint8_t reg, uint16_t value);  // Write a 16-bit register
void writeReg32Bit(uint8_t reg, uint32_t value);  // Write a 32-bit register
uint8_t readReg(uint8_t reg);                     // Read an 8-bit register
uint16_t readReg16Bit(uint8_t reg);               // Read a 16-bit register
uint32_t readReg32Bit(uint8_t reg);               // Read a 32-bit register
// Write `count` number of bytes from `src` to the sensor, starting at `reg`
void writeMulti(uint8_t reg, uint8_t const *src, uint8_t count);

// TCC: Target CentreCheck
// MSRC: Minimum Signal Rate Check
// DSS: Dynamic Spad Selection
typedef struct {
  uint8_t tcc, msrc, dss, pre_range, final_range;
}SequenceStepEnables;

typedef struct {
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
}SequenceStepTimeouts;

