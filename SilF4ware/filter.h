void lpf( float * out, float in, float alpha );
void lpf_hz( float * out, float in, float f_hz );
float rpm_filter( float in, int num );
float notch_a_filter( float in, int num );
float notch_b_filter( float in, int num );
float notch_c_filter( float in, int num );
float auto_notch_filter( float in, int num );
float sdft_notch_filter( float in, int num );
float peak_filter( float in, int num );
float gyro_lpf_filter( float in, int num );
float gyro_lpf2_filter( float in, int num );
float dterm_filter( float in, int num );
float throttle_hpf( float in );
void throttle_hpf_reset( int holdoff_time_ms );
float kalman_filter( float in, int num );
void kalman_set( float in, int num );

// With constants as parameters this should be precalculated by the compiler.
// This is used to calculate alpha. filtertime = 1 / filter-cutoff-frequency.
// The approximation is good only for filtertime >> dT, maybe filtertime > dT * 5.
#define ALPHACALC( dT, filtertime ) ( ( 6.6f * (float)dT ) / ( 4.2f * (float)dT + (float)filtertime ) )

#define SDFT_AXES 3 // 2 .. only for roll and pitch, 3 .. roll, pitch, and yaw
