#include "main.h"

#define STRING 0

void createPublisher(rcl_publisher_t* publisher,
	const rcl_node_t* node,
	const rosidl_message_type_support_t* type_support,
	const char* topic_name,
	std_msgs__msg__Int32* msg)
{
	rcl_ret_t ret = rclc_publisher_init_default(publisher, node, type_support, topic_name);
	printf("Publisher %s is created with result %d\r\n", topic_name, (int)ret);

#if STRING == 1
	(*msg).data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
	(*msg).data.size = 0;
	(*msg).data.capacity = ARRAY_LEN;
#else
	(*msg).data = 0;
#endif

}

void createSubscriber(rcl_subscription_t* subscription,
	rcl_node_t* node,
	const rosidl_message_type_support_t* type_support,
	const char* topic_name,
	std_msgs__msg__Int32* msg)
{
	*subscription = rcl_get_zero_initialized_subscription();

	rcl_ret_t ret = rclc_subscription_init_default(subscription, node,
		type_support, topic_name);
	printf("Subscription %s is created with result %d\r\n", topic_name, (int)ret);

#if STRING == 1
	(*msg).data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
	(*msg).data.size = 0;
	(*msg).data.capacity = ARRAY_LEN;
#else
	(*msg).data = 0;
#endif
}
