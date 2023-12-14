/**
 * @file : microROS.h
 * @brief : Contain microROS topic and default custom creator for subscriber and publisher
 */

#ifndef DEF_MICROROS
#define DEF_MICROROS

	#include "main.h"
	#define ARRAY_LEN 100 /**< Length of string messages */
	#define CAPTEUR_DIR_TOPIC "capteur/dir" /**< Topic name of direction publisher */
	#define ETAT_MODE_TOPIC "etat/mode" /**< Topic name of mode publisher */
	#define ETAT_SPEED_TOPIC "etat/speed" /**< Topic name of speed publisher */
	#define CAMERA_X_TOPIC "camera/X" /**< Topic name of x camera subscriber */
	#define CAMERA_Y_TOPIC "camera/Y" /**< Topic name of y camera subscriber */
	#define TELECOMMANDE_DIR_TOPIC "direction" /**< Topic name of manual subscriber */ //"telecommande/dir"
	#define CONFIG_MODE_TOPIC "mode" /**< Topic name of mode subscriber */ //"config/mode"
	#define CONFIG_SPEED_TOPIC "speed" /**< Topic name of speed subscriber */ //"config/speed"

	/**
	 * Create a publisher with default options
	 * @param publisher microRos structure that represent a publisher
	 * @param node microRos structure that represent a node
	 * @param type_support microRos structure that represent the type of message
	 * @param topic_name The name of the topic
	 * @param msg microRos structure that represent the message
	 */
	void createPublisher(rcl_publisher_t* publisher,
			  const rcl_node_t* node,
			  const rosidl_message_type_support_t* type_support,
			  const char* topic_name,
			  std_msgs__msg__Int32* msg);

	/**
	 * Create a subscriber with default options
	 * @param subscription microRos structure that represent a subscriber
	 * @param node microRos structure that represent a node
	 * @param type_support microRos structure that represent the type of message
	 * @param topic_name The name of the topic
	 * @param msg microRos structure that represent the message
	 */
	void createSubscriber(rcl_subscription_t* subscription,
			  rcl_node_t* node,
			  const rosidl_message_type_support_t* type_support,
			  const char* topic_name,
			  std_msgs__msg__Int32* msg);

#endif //DEF_MICROROS
