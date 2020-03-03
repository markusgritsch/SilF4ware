void pid_precalc( void );
void pid( int x );
void rotateErrors( void );
void set_current_pid_term( int pid_term );
int next_pid_term( void );
int next_pid_axis( void );
void multiply_current_pid_value( float mutiplier );
int increase_pid( void );
int decrease_pid( void );
