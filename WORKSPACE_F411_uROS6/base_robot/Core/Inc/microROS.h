#ifndef DEF_MICROROS
#define DEF_MICROROS

	#include "main.h"
	#define ARRAY_LEN 100

	typedef struct MicroRosPubMsg MicroRosPubMsg;
	struct MicroRosPubMsg
	{
		char dir;
		int mode;
		int speed;
		int isGood;
	};

	typedef struct MicroRosSubMsg MicroRosSubMsg;
	struct MicroRosSubMsg
	{
		int dir;
		int x;
		int y;
		int mode;
		int speed;
	};

	void CHECKMRRET(rcl_ret_t ret, char* msg);
	void SubscriberStringCallbackFunction(const void *msgin);
	void SubscriberCallbackFunction(const void *msgin);

	void createPublisher(rcl_publisher_t* publisher,
			  const rcl_node_t* node,
			  const rosidl_message_type_support_t* type_support,
			  const char* topic_name,
			  std_msgs__msg__Int32* msg);

	void createSubscriber(rcl_subscription_t* subscription,
			  rcl_node_t* node,
			  const rosidl_message_type_support_t* type_support,
			  const char* topic_name,
			  std_msgs__msg__Int32* msg);

#endif //DEF_MICROROS
