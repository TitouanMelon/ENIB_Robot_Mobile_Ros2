#ifndef DEF_MICROROS
#define DEF_MICROROS

	#include "main.h"
	#define ARRAY_LEN 100
	#define CAPTEUR_DIR_TOPIC "capteur/dir"
	#define ETAT_MODE_TOPIC "etat/mode"
	#define ETAT_SPEED_TOPIC "etat/speed"
	#define CAMERA_X_TOPIC "camera/x"
	#define CAMERA_Y_TOPIC "camera/y"
	#define TELECOMMANDE_DIR_TOPIC "telecommande/dir"
	#define CONFIG_MODE_TOPIC "config/mode"
	#define CONFIG_SPEED_TOPIC "config/speed"

	void createPublisher(rcl_publisher_t* publisher,
			  const rcl_node_t* node,
			  const rosidl_message_type_support_t* type_support,
			  const char* topic_name,
			  std_msgs__msg__Int32* msg);

	void createSubscriber(rcl_subscription_t* subscription,
			  const rcl_node_t* node,
			  const rosidl_message_type_support_t* type_support,
			  const char* topic_name,
			  std_msgs__msg__Int32* msg);

#endif //DEF_MICROROS
