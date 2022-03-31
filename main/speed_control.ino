float pd(float kp,float kd,float error,float pev_error) {
    return kp * error + kd * (error - pev_error); }

float pd_sum(float kp,float kd,float error,float pev_error,float current_value) {
    return (current_value + ((kp * error) + (kd * (error - pev_error)))); }