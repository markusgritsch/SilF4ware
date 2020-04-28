#define FFT_SIZE 4096 // Gives about 0.98 Hz resolution at 4k loop frequency.
// #define FFT_SIZE 2048 // Gives about 1.95 Hz resolution at 4k loop frequency.

int get_max_amplitude_index( float array[], int start_index );
