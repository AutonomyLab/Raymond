#ifndef RobotKit_h
#define RobotKit_h

typedef struct {
 float lin_speed;// [m/s]
 float orientation;// [rad]
 float rot_speed;// [rad/s]
} data_t;

void read_and_send_odometry(void);
void polygone(unsigned int length, unsigned int driving_speed, int rot_speed, unsigned int nb_sides);
void blinkLED(void);
int send_message(int message[4]);
int receive_message(data_t* data, int length);
void send_uart_to_computer(void);

#endif 
