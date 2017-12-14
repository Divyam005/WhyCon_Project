
#pragma once

#define TOTAL_CMDS 25

typedef enum{
	TAKE_OFF,
	LAND,
	SLEEP,
	ARM_MOTORS,
	GO_TO_WAY_POINT,
	DISARM_MOTORS,
	LOOP_FROM_START
} cmd_type;

typedef struct command{
	cmd_type id;
	int16_t pos_x;
	int16_t pos_y;
	int16_t height;
} cmd_def;

void command_def(cmd_def *cmd);

bool command_verify(void);

void command_next(void);

void command_previous(void);

void command_run(uint32_t current_time);

void command_jump(uint8_t jump);