#include "defines.h"

// rate in deg/sec for acro mode
#define MAX_RATE 1800
#define MAX_RATEYAW 1800

#define LOW_RATES_MULTI 0.5

#define POLAR_EXPO 0.3 // 0.0 .. linear, 1.0 .. square curve

// max angle for level mode
#define LEVEL_MAX_ANGLE 80

// max rate used by level pid (limit)
#define LEVEL_MAX_RATE 1800

// Change this factor to get a correct battery voltage.
#define ADC_SCALEFACTOR 11.111 // 11.0 for an ideal 10k/1k voltage divider

// Allow automatic cell count detection (up to 6S) and report single cell voltage.
#define BATTERY_CELL_COUNT_DETECTION

// Do not start software if battery is too low. Flashes 2 times repeatedly at startup.
//#define STOP_LOWBATTERY // If below 3.3 Volt

// If enabled, start LED blinking at low battery voltage
#define WARN_ON_LOW_BATTERY 3.6 // Volt
// Voltage hysteresis for WARN_ON_LOW_BATTERY
#define VOLTAGE_HYSTERESIS 0.10 // Volt

// compensation for battery voltage vs throttle drop
#define VDROP_FACTOR 0.7
// calculate above factor automatically
#define AUTO_VDROP_FACTOR

// lower throttle when battery below threshold
#define LVC_LOWER_THROTTLE
#define LVC_LOWER_THROTTLE_VOLTAGE 3.30
#define LVC_LOWER_THROTTLE_VOLTAGE_RAW 2.70
#define LVC_LOWER_THROTTLE_KP 3.0

// Go into DFU Mode when powered over USB without connected battery
#define AUTO_BOOTLOADER

// MPU-60x0 on-chip Gyro LPF filter frequency
// gyro filter 0: 256 Hz, delay 0.98 ms (use this to get 8k gyro update frequency)
// gyro filter 1: 188 Hz, delay 1.9 ms
// gyro filter 2: 98 Hz, delay 2.8 ms
// gyro filter 3: 42 Hz, delay 4.8 ms
#define GYRO_LOW_PASS_FILTER 0

// Gyro Notch Filters

#define RPM_FILTER // requires DSHOT_DMA_BIDIR in hardware.h -- also ensure MOTOR_POLE_COUNT in drv_dshot_bidir.c is correct
#define RPM_FILTER_HZ_MIN 80 // do not apply RPM filtering below RPM_FILTER_HZ_MIN
#define RPM_FILTER_HZ_FADE 40 // gradually increase notch filtering until RPM_FILTER_HZ_MIN + RPM_FILTER_HZ_FADE is reached
#define RPM_FILTER_2ND_HARMONIC true // note, that there are 12 notch filters (4 motors * 3 axes) per harmonic
#define RPM_FILTER_3RD_HARMONIC true
#define RPM_FILTER_Q 6 // -3dB bandwidth = f0 / Q -- but a higher Q also results in a longer settling time

//#define BIQUAD_NOTCH_A_HZ 273 // Dalprop Cyclone T5249C
//#define BIQUAD_NOTCH_A_Q 6

//#define BIQUAD_NOTCH_B_HZ 308 // T-Motor T5147
//#define BIQUAD_NOTCH_B_Q 6

//#define BIQUAD_NOTCH_C_HZ 256 // GemFan WinDacer 51433
//#define BIQUAD_NOTCH_C_Q 6

//#define BIQUAD_AUTO_NOTCH // Performs an FFT of the last second of stored gyro data with the RRR gesture.
//#define BIQUAD_AUTO_NOTCH_Q 6 // (frequency bin resolution is defined via FFT_SIZE in fft.h)

#define BIQUAD_SDFT_NOTCH // Sliding DFT dynamic notch filters. Two per filtered axis.

//#define SDFT_GYRO_FILTER // Experimental frequency domain filtering. Does not work well.

// Dynamic Gyro first and second order LPFs

//#define GYRO_LPF_1ST_HZ_BASE 120 // Filter frequency at zero throttle.
//#define GYRO_LPF_1ST_HZ_MAX 120 // A higher filter frequency than loopfrequency/2.4 causes ripples.
//#define GYRO_LPF_1ST_HZ_THROTTLE 0.25 // MAX reached at 1/4 throttle.

//#define GYRO_LPF_2ND_HZ_BASE 240 //* ( aux[ FN_INVERTED ] ? 0.75f : 1.0f )
//#define GYRO_LPF_2ND_HZ_MAX 240
//#define GYRO_LPF_2ND_HZ_THROTTLE 0.25

// Static Gyro first order LPF
//#define GYRO_LPF_1ST_HZ 240

// Additional static Gyro first order LPF on yaw only
//#define GYRO_YAW_LPF_1ST_HZ 240

// Kalman gyro filter. The specified value is not Hz but affects the process noise covariance.
//#define GYRO_KALMAN_q 100 // Higher value is less filtering

// Dynamic D-Term second order LPF
//#define DTERM_LPF_2ND_HZ_BASE 60 //* ( aux[ FN_INVERTED ] ? 0.75f : 1.0f )
//#define DTERM_LPF_2ND_HZ_MAX 60
//#define DTERM_LPF_2ND_HZ_THROTTLE 0.5

// Whether to use Bessel type filter for D-Term instead of PT2.
//#define DTERM_BESSEL_FILTER

// Static D-Term first order LPFs
#define DTERM_LPF_1ST_A_HZ 60
#define DTERM_LPF_1ST_B_HZ 120

// D-Term peak
//#define BIQUAD_PEAK_HZ 12
//#define BIQUAD_PEAK_Q 3
//#define BIQUAD_PEAK_GAIN 1.2

// If enabled, the D-Term filters use the LPF-filtered gyro signal from above. (RPM-, Notch-, and SDFT-filtering is always applied, if enabled.)
//#define CASCADE_GYRO_AND_DTERM_FILTER

// Motor first order LPFs
//#define MOTOR_FILTER_A_HZ 120
//#define MOTOR_FILTER_B_HZ 240
//#define MOTOR_FILTER_HZ_MULTIPLIER 1 // Multiply motor filter frequency by MOTOR_FILTER_HZ_MULTIPLIER
//#define MOTOR_FILTER_THROTTLE_BREAKPOINT 0.25 // at and above MOTOR_FILTER_THROTTLE_BREAKPOINT.

// Limit maximum motor speed change rate to MIX_CHANGE_LIMIT
#define MIX_CHANGE_LIMIT 50 // 50/s == 50%/10ms
// Post-MIX_CHANGE_LIMIT motor filter
//#define MIX_FILTER_HZ 120 // MIX_FILTER_HZ at zero to 2 * MIX_FILTER_HZ at full motor speed
// Kalman motor filter. The specified value is not Hz but affects the process noise covariance.
#define MOTOR_KALMAN_q 100 // Higher value is less filtering

// Switch function selection

#define RATES DEVO_CHAN_9 // LOW_RATES_MULTI gets applied when RATES is 0.

#define LEVELMODE DEVO_CHAN_10 // Set this to CH_ON for level mode always on. Comment it out for just acro.

#define LEDS_ON DEVO_CHAN_7

// To stop the motors on ground a switch on the remote control is necessary.
#define THROTTLE_KILL_SWITCH DEVO_CHAN_5

// enable inverted (3D) flight code
#define INVERTED_ENABLE // goes together with BIDIRECTIONAL in drv_dshot.c / drv_dshot_dma.c / drv_dshot_bidir.c
#define FN_INVERTED DEVO_CHAN_6
//#define LEVEL_MODE_INVERTED_ENABLE // be careful when enabling this

// Two switchable channels via gestures: CH_AUX1 and CH_AUX2
// Channel CH_AUX1 changed via gestures LLU -> 1 and LLD -> 0
// Channel CH_AUX2 changed via gestures RRU -> 1 and RRD -> 0
//#define AUX1_START_ON // CH_AUX1 channel starts on if this is defined, otherwise off.
//#define AUX2_START_ON // CH_AUX2 channel starts on if this is defined, otherwise off.

// lost quad beeps using motors (60 sec timeout or via channel)
#define MOTOR_BEEPS
#define MOTOR_BEEPS_CHANNEL DEVO_CHAN_12

// Send maximum measured g-force in the telemetry data.
//#define DISPLAY_MAX_G_INSTEAD_OF_VOLTAGE
// Send fly time (! onground) in the telemetry data.
#define DISPLAY_FLY_TIME_INSTEAD_OF_VOLTAGE
// Send maximum looptime in the telemetry data.
#define DISPLAY_MAX_USED_LOOP_TIME_INSTEAD_OF_RX_PACKETS
// Send PID values in the telemetry data.
#define DISPLAY_PID_VALUES

// Radio module and protocol selection (only Bayang protocol implemented)
#define RX_NRF24_BAYANG_TELEMETRY // For nRF24L01+ radio module
//#define RX_XN297_BAYANG_TELEMETRY // For XN297 radio module harvested from toy TX

#define RADIO_XN297 // also enable RX_HARDSPI or RX_SOFTSPI_4WIRE in hardware.h
//#define RADIO_XN297L // also enable RX_SOFTSPI_3WIRE in hardware.h

#define TX_POWER 3 // 0 .. 3 (use 1 when using an nRF24L01+PA+LNA module)

#define AUX_ANALOG_DESTMAX 255 // only for custom DeviationTX builds

// led brightness 0 .. 15 (used for solid lights only)
#define LED_BRIGHTNESS 15

// a filter which makes throttle feel faster (aka Throttle Boost) (not active in LOW_RATES or lowbatt)
//#define THROTTLE_TRANSIENT_COMPENSATION_FACTOR 3.0

// Compensate throttle for sagging battery voltage
#define THROTTLE_VOLTAGE_COMPENSATION

// For more consistent motor reversing in 3D flight
#define THROTTLE_REVERSING_KICK 0.15 // 4S
#define THROTTLE_REVERSING_DEADTIME 20000 // 20 ms (increase this in case of over-propped motors)
//#define THROTTLE_STARTUP_KICK 0.05

// Continue stick movement with the current stick velocity in case of lost packets
#define RX_PREDICTOR
// Add linear interpolation between the otherwise 5 ms staircase steps of the RX signal
#define RX_SMOOTHING
// Limit maximum stick velocity from STICK_VELOCITY_LIMIT around center to 2 * STICK_VELOCITY_LIMIT at full deflection
#define STICK_VELOCITY_LIMIT 7 // deflection/s (It takes 1/STICK_VELOCITY_LIMIT seconds to reach full stick deflection)
// Apply LPF to roll, pitch, and yaw sticks
//#define STICK_FILTER_HZ 10

// Betaflight like mix scaling (aka Airmode)
#define MIX_SCALING
// Mix increasing yields a more crisp response but also a more jumpy quad at low RPM
#define ALLOW_MIX_INCREASING
// A higher value means a shorter active increasing period (shorter bouncy period)
#define TRANSIENT_MIX_INCREASING_HZ 2.0
// Can be used to limit maximum motor RPM, i.e. tone down a too fast quad.
#define MIX_RANGE_LIMIT 1.0 // aux[ DEVO_CHAN_11 ] ? 0.75 : 1.0

// Adjust MOTOR_IDLE_OFFSET so that the motors still spin reliably under all circumstances.
#define MOTOR_IDLE_OFFSET 0.03 // 4S
// Throttle value when throttle stick is at zero to have some headroom for pid mixing at zero throttle.
#define THROTTLE_ZERO_VALUE 0.00 // 4S

// Use a square root motor curve to counteract thrust ~ RPM^2
//#define THRUST_LINEARIZATION 0.4 // 0.0 .. no compensation, 1.0 .. full square root curve
#define ALTERNATIVE_THRUST_LINEARIZATION 0.6 // different shape from the above curve; do not enable both together

// A deadband can be used to eliminate stick center jitter and non-returning to exactly 0.
#define STICKS_DEADBAND 0.02

// throttle direct to motors for thrust measure
//#define MOTORS_TO_THROTTLE

// throttle direct to motors for thrust measure as a flight mode
#define MOTORS_TO_THROTTLE_MODE CH_AUX1

// PID tuning by gestures and/or stick position
//#define PID_GESTURE_TUNING
#define PID_STICK_TUNING
#define COMBINE_PITCH_ROLL_PID_TUNING

// Save gyro calibration (together with accelerometer calibration) to flash (gesture DDD)
#define PERSISTENT_GYRO_CAL

// Unscaled PID values tuned for 4S. Note: Compared to traditional SilverWare PID_KP values must be multiplied by 10.
//             { roll  pitch yaw }
#define PID_KP { 0.30, 0.30, 0.4 }
#define PID_KI { 0.40, 0.40, 2.0 }
#define PID_KD { 0.15, 0.15, 0.0 }

// Multiply PID by this value when flying inverted
#define PID_SCALE_INVERTED 1.5

// Multiply P and D by this value at full yaw to avoid oscillations
//#define PD_SCALE_YAW_STABILIZER 0.8f

// Compensate PID values for sagging battery voltage
#define PID_VOLTAGE_COMPENSATION

// Invert yaw pid. Necessary when spinning props outwards.
#define INVERT_YAW_PID

// Rotate I-term vector for a stable yaw axis (aka iTerm Rotation)
#define PID_ROTATE_ERRORS

// Remove roll and pitch bounce back after flips (aka iTerm Relax)
#define TRANSIENT_WINDUP_PROTECTION
// Remove bounce back when quickly stopping a roll/pitch/yaw movement (but it is mostly there for yaw)
#define DYNAMIC_ITERM_RESET

// Full smoothing at zero throttle, gradually less smoothing with increasing throttle, no smoothing at and above RFS_THROTTLE_BREAKPOINT.
#define ROLL_FLIP_SMOOTHER // Scale P, I, and D on roll and pitch axes according to gyro speed.
#define RFS_RATE_MIN 180 // °/s, No scaling below RFS_RATE_MIN. Start scaling at RFS_RATE_MIN.
#define RFS_RATE_MAX 720 // °/s, Linear transition to full scaling at and above RFS_RATE_MAX.
#define RFS_P_SCALER 0.5 // Scale P by this factor at and above RFS_RATE_MAX.
#define RFS_I_SCALER 0.0 // Scale I by this factor at and above RFS_RATE_MAX.
#define RFS_D_SCALER 0.5 // Scale D by this factor at and above RFS_RATE_MAX.
#define RFS_THROTTLE_BREAKPOINT 0.5 // No smoothing at and above RFS_THROTTLE_BREAKPOINT.

// Attenuate P and D on roll and pitch axes linearly with rising throttle until TPDA_VALUE is reached at TPDA_BREAKPOINT.
//#define THROTTLE_PD_ATTENUATION // No scaling at zero throttle. No scaling for inverted flying.
#define TPDA_VALUE 0.8 // Scale P and D by this factor at TPDA_BREAKPOINT.
#define TPDA_BREAKPOINT 0.5 // Constant scaling with TPDA_VALUE at and above this throttle value.

// Scale P and D on roll and pitch axes in case gyro shows activity (prop wash) without setpoint (stick) movement.
//#define PROP_WASH_REDUCER
#define PROP_WASH_P_SCALER 1.0
#define PROP_WASH_D_SCALER 1.5

// Cross-attenuate roll/pitch pidoutput on high pitch/roll pidoutput respectively.
//#define MIX_CROSS_ATTENUATION
#define MCA_STEEPNESS 200 // 0 .. is identical to no attenuation. A higher value results in stronger attenuation.
#define MCA_MIN 0.5 // Do not scale below MCA_MIN at high pidoutput.

// Feed fast roll/pitch-stick changes directly to the motors to give a snappier response
// 0.0f (or commented out) equates D-term on measurement, 1.0f equates D-term on error.
//#define FEED_FORWARD_STRENGTH 1.0f
//#define SMART_FF
// Feedforward for yaw. It's an absolute value, not related to the 0 .. 1 from above.
//#define FEED_FORWARD_YAW 0.2f

// Loop time in us
#define LOOPTIME 250 // 125 us (8k) needs DSHOT 600 in drv_dshot_bidir.c and an STM32F405 board.
// Correction for ceramic resonator frequency variation
#define WALLTIME_CORRECTION_FACTOR 1.000 // set to >1 to compensate for a too slow resonator

// Failsafe time in us. Sets stick inputs to zero after FAILSAFETIME no RX signal. Keeps quad stabilized.
#define FAILSAFETIME 150000 // 0.15 seconds
// Motors failsafe time in us. Shuts motors down after additional MOTORS_FAILSAFETIME.
#define MOTORS_FAILSAFETIME 3000000 // 3 seconds

// Gyro orientation:
// The expected orientation is with the dot on the chip in the front-left corner.
// Specify by how much you have to rotate the board so that the dot is front left.
// The rotations are performed in order and cumulated.
// Note, the motors don't get rotated, so they have to be referenced to the new gyro position.
//#define SENSOR_ROTATE_45_CCW
//#define SENSOR_ROTATE_45_CW
//#define SENSOR_ROTATE_90_CW
//#define SENSOR_ROTATE_90_CCW
//#define SENSOR_ROTATE_180
//#define SENSOR_INVERT // Necessary if the gyro is mounted upside down. For an inverted gyro,
// the expected orientation is with the dot on the chip in the front-right corner.

#if defined FC_BOARD_OMNIBUS || defined FC_BOARD_F4XSD
	#define SENSOR_ROTATE_90_CCW
#elif defined FC_BOARD_NOXE
	#define SENSOR_ROTATE_180
#elif defined FC_BOARD_NOXE_V1
	// no rotation needed
#else
	#error "FC_BOARD_xxx must be defined by the toolchain, e.g. in the Keil project file."
#endif

// Motor order
#define MOTOR_BL 2
#define MOTOR_FL 1
#define MOTOR_BR 4
#define MOTOR_FR 3

// For BIDIRECTIONAL motor direction, the motors can be reversed below.
#define REVERSE_MOTOR_BL false
#define REVERSE_MOTOR_FL false
#define REVERSE_MOTOR_BR false
#define REVERSE_MOTOR_FR false

// Disable the check for known gyro that causes the 4 times LED flash.
#define GYRO_CHECK

// Disable the check for development without RX module (3 times LED flash).
#define RADIO_CHECK

// Logs various information to an externally connected OpenLager logger.
//#define BLACKBOX_LOGGING
#define CRAFT_NAME "SILF4WARE" // Used as filename of the converted logfile.

// OSD usage requires uploading the custom SilF4ware.mcm font from the Utilities folder to the FC using Betaflight Configurator.
// Display battery status, flight time, and RSSI info in the OSD.
//#define OSD_ENABLE NTSC // PAL or NTSC
// Artificial horizon for OSD.
#define ARTIFICIAL_HORIZON // Comment out to disable the artificial horizon completely.
#define CAMERA_FOV 130 // Horizontal camera field of view in ° (needed for correct artificial horizon movement).
