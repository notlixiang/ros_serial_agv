
#ifndef _STRUCTSERIAL_H_
#define _STRUCTSERIAL_H_

#define COMMAND_DATA_LENGTH sizeof(struct_command_data)

#define CHECK_FRONT_CMD 	23579
#define CHECK_BACK_CMD 		74348

typedef struct struct_command_data {
	int check_front_cmd;
	
	float speed_cmd[3];
	float pos_cmd[3];
	uint8_t  qr_scan_cmd;
	
	int check_back_cmd;
} struct_command_data, *struct_command_data_ptr;


#define FEEDBACK_DATA_LENGTH sizeof(struct_feedback_data)
	
#define CHECK_FRONT_FBK 	61516
#define CHECK_BACK_FBK 		23275

typedef struct struct_feedback_data {
	int check_front_fbk;	
	
	float speed_fbk[3];	
	float pos_fbk[3];	
	
	float a_fbk[3];
	float g_fbk[3];
	float yaw_fbk;
	
	float ultra_sound_signal_fbk[12];

	float voltage_fbk;
	uint8_t charging_status_fbk;
	
	char qr_scan_fbk[10];
	
	int check_back_fbk;
} struct_feedback_data, *struct_feedback_data_ptr;


void send_struct_feedback_serial(void);

#endif