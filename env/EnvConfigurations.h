#ifndef __ENV_CONFIGURATIONS_H__
#define __ENV_CONFIGURATIONS_H__

// database_map
#define WALK_DATABASE "mocap-walk/"
#define SIT_DATABASE "mocap-sit/"

// autoencoder network
#define AUTOENCODER_NETWORK_NAME "cnn_pretrained"

// terminal condition
#define TERMINAL_ROOT_VEL 0.03
#define TERMINAL_PENETRATION_THRESHOLD 0.04

// initial
#define INITIAL_WALK_FRAME_COUNT 30

// reward related
#define ROOT_VEL_THRESHOLD 0.045
#define ROOT_VEL_FRAME_COUNT 60
#define EDIT_BODY_WEIGHT 0.0
#define TARGET_REWARD_AMPLIFY_MAX 3 
#define TARGET_REWARD_AMPLIFY_MIN 1.5 
#define PENE_LEG_WEIGHT 0.7
#define PENE_FRAME_COUNT 5
#define PENE_EXP_WEIGHT 0.35

// motion editing related
#define EDIT_POSTPROCESS_EPOCH 500

// scene bbox
#define BBOX_SIZE 1.0

#endif
