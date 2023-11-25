// https://github.com/lFatality/stm32_micro_ros_setup
#include "main.h"

#include "motorCommand.h"
#include "quadEncoder.h"
#include "captDistIR.h"
#include "VL53L0X.h"
#include "groveLCD.h"

#define SAMPLING_PERIOD_ms 5

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

extern I2C_HandleTypeDef hi2c1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

//#######################################################
#define EXSTARTUP 0 //Kerhoas code
#define EXTEST_UART2 1 //Just printf and scanf fonction
#define EXCORRECTOR 2
#define EXTEST_VL53 3 //send value of VL53 captor on uart2
#define EXTEST_MICROROS 4 //Test send and receive Int32 msg
#define EXFINAL 5 //final code

#define SYNCHRO_EX EXFINAL

enum {MODE_OBS, MODE_ZIG, MODE_CAM, LAST_MODE};
enum {STOP_VIT, LOW, FAST, SONIC, LAST_SPEED};
enum {AVANT, GAUCHE, RECULE, DROITE, STOP, AVANT_GAUCHE, AVANT_DROITE, RECULE_GAUCHE, RECULE_DROITE, LAST_DIR};

#define ROS_DOMAIN_ID 0
#define NB 200
#define TEST_CORRECTOR_SPEED 150
#define TEST_LEFT_MOTOR 1
//#######################################################
#define LCD 0
#define VL53 0
#define MICROROS 1
#define DEBUG_PRINTF 0
#define DEBUG_MOTOR 0
//#######################################################
#define CMD 1000
#define VITESSE_KART CMD/2
#define VITESSE_OBS CMD
#define VITESSE_CAM CMD
//#######################################################
#define CAMERA_X_MIN 0
#define CAMERA_X_MAX 1024
#define CAMERA_Y_MIN 0
#define CAMERA_Y_MAX 1024
#define CAMERA_X_TIER (CAMERA_X_MAX-CAMERA_X_MIN)/3
#define CAMERA_Y_TIER (CAMERA_Y_MAX-CAMERA_Y_MIN)/3
//#######################################################
#define Te 5

#define LKp 0.001
#define LKi (5.0/(0.1*40.0))

#define RKp 0.001
#define RKi (5.0/(0.1*40.0))
//#######################################################

// Déclaration des objets synchronisants !! Ne pas oublier de les créer
xSemaphoreHandle xSem_Supervision = NULL;
xSemaphoreHandle xSem_MicroRos = NULL;
xQueueHandle q_mot_L = NULL;
xQueueHandle q_mot_R = NULL;

xQueueHandle qhMR_sub = NULL;
xQueueHandle qhMR_pub = NULL;
xQueueHandle qhLCD = NULL;
xQueueHandle qhVl53 = NULL;

int16_t tab_speed[NB];

typedef struct AMessage AMessage;
struct AMessage
{
	char command;
	int data;
};

typedef struct MicroRosPubMsg MicroRosPubMsg;
struct MicroRosPubMsg
{
	char dir;
	int mode;
	int speed;
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

//Test function
void test_print_uart2(void *pvParameters);
void test_vl53(void *pvParameters);
void test_motor(void *pvParameters);

//Robot function
void SystemClock_Config(void);

//Micro-Ros function
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);


void CHECKMRRET(rcl_ret_t ret, char* msg){
	if (ret != RCL_RET_OK){ printf("Error : %d\r\nMsg : %s\r\n", (int)ret, msg); }
}

void SubscriberCallbackFunction(const void *msgin){
#if SYNCHRO_EX == EXTEST_MICROROS
	std_msgs__msg__String * msg = (std_msgs__msg__String * )msgin;
	printf("\r\nMessage recue : %s\r\n", msg->data->data);
#elif SYNCHRO_EX == EXFINAL
	std_msgs__msg__Int32 * msg = (std_msgs__msg__Int32 * )msgin;
	printf("\r\nMessage recue : %ld\r\n", msg->data);
#endif //SYNCHRO_EX
}



void microros_task(void *argument)
{
	// micro-ROS app variable
	rclc_support_t support;
	rcl_allocator_t allocator;
	rcl_node_t node;
	rcl_node_options_t node_opt;
	rclc_executor_t executor;

	// micro-ROS configuration
	rmw_uros_set_custom_transport(
		true,
		(void *) &huart1,
		cubemx_transport_open,
		cubemx_transport_close,
		cubemx_transport_write,
		cubemx_transport_read);

	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		printf("Error on default allocators (line %d)\r\n", __LINE__);
	}

	allocator = rcl_get_default_allocator();
	//create init_options
	CHECKMRRET(rclc_support_init(&support, 0, NULL, &allocator), "error on init support");
	// create node
	//CHECKMRRET(rclc_node_init_default(&node, "STM32_node", "", &support), "error on init node");
	node_opt = rcl_node_get_default_options();
	node_opt.domain_id = ROS_DOMAIN_ID;
	CHECKMRRET(rclc_node_init_with_options(&node, "STM32_node", "", &support, &node_opt), "error on init node");


#if SYNCHRO_EX == EXSTARTUP
	static int counter = 0;
	rcl_ret_t ret;
	rcl_publisher_t publisher;
	std_msgs__msg__String msg;

	CHECKMRRET(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "cubemx_publisher"),
			"Error when create publisher");

	for (;;)
	{
		sprintf(msg.data.data, "Hello from micro-ROS #%d", counter++);
		msg.data.size = strlen(msg.data.data);
		ret = rcl_publish(&publisher, &msg, NULL);
		if (ret != RCL_RET_OK)
			printf("Error publishing (line %d)\r\n", __LINE__);
		vTaskDelay(SAMPLING_PERIOD_ms);
	}
#elif SYNCHRO_EX == EXTEST_MICROROS
	//micro-ros topic variable
	rcl_ret_t ret;
	rcl_publisher_t publisher;
	rcl_subscription_t subscriber;
	std_msgs__msg__String msg;

	/* Default publisher */
	CHECKMRRET(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "cubemx_publisher"),
			"Error when create publisher");
		/* ---------------------- */
	/* Default subscriber */
	subscriber = rcl_get_zero_initialized_subscription();
	CHECKMRRET(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),"cubemx_subscriber"),
			"Error when create subscriber");
	/* ----------------------- */
	/* Init string msg */
	msg.data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
	msg.data.size = 0;
	msg.data.capacity = ARRAY_LEN;
	/* ---------------- */
	// Init executor and add subscriber to it
	CHECKMRRET(rclc_executor_init(&executor, &support.context, 1, &allocator), "Error on init executor");
	CHECKMRRET(rclc_executor_add_subscription(&executor, &subscriber, &msg, &SubscriberCallbackFunction, ON_NEW_DATA), "error add subscriber");

	for (;;)
	{
		ret = rclc_executor_spin_some(&executor, 100*1000*1000);
		vTaskDelay(SAMPLING_PERIOD_ms);
	}
#elif SYNCHRO_EX == EXFINAL
	MicroRosPubMsg MsgToPub = {'N', 0, 0};
	MicroRosSubMsg SubToMsg = {STOP, 0, 0, MODE_OBS, LOW};
	/* PUBLISHER */
	//Use to publish the direction of robot in sensor mode
	rcl_publisher_t capteur_dir_pub;
	char* capteur_dir_topic = "capteur/dir";
	std_msgs__msg__Int32 capteur_dir_msg;
	//Use to publish the actual mode of the robot
	rcl_publisher_t etat_mode_pub;
	char* etat_mode_topic = "etat/mode";
	std_msgs__msg__Int32 etat_mode_msg;
	//Use to publish the actual speed of the robot
	rcl_publisher_t etat_speed_pub;
	char* etat_speed_topic = "etat/speed";
	std_msgs__msg__Int32 etat_speed_msg;

	/* SUBSCRIBER */
	//Use to receive the x position of object see by the camera
	rcl_subscription_t camera_x_sub;
	char* camera_x_topic = "camera/x";
	std_msgs__msg__Int32 camera_x_msg;
	//Use to receive the y position of object see by the camera
	rcl_subscription_t camera_y_sub;
	char* camera_y_topic = "camera/y";
	std_msgs__msg__Int32 camera_y_msg;
	//Use to receive the remote control in remote mode
	rcl_subscription_t telecommande_dir_sub;
	char* telecommande_dir_topic = "telecommande/dir";
	std_msgs__msg__Int32 telecommande_dir_msg;
	//Use to receive the mode config
	rcl_subscription_t config_mode_sub;
	char* config_mode_topic = "config/mode";
	std_msgs__msg__Int32 config_mode_msg;
	//Use to receive the speed config
	rcl_subscription_t config_speed_sub;
	char* config_speed_topic = "config/speed";
	std_msgs__msg__Int32 config_speed_msg;

	// create publisher
	createPublisher(&capteur_dir_pub, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		capteur_dir_topic, &capteur_dir_msg);

	createPublisher(&etat_mode_pub, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		etat_mode_topic, &etat_mode_msg);

	createPublisher(&etat_speed_pub, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		etat_speed_topic, &etat_speed_msg);

	//create subscriber
	createSubscriber(&camera_x_sub,&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		camera_x_topic, &camera_x_msg);

	createSubscriber(&camera_y_sub,&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			camera_y_topic, &camera_y_msg);

	createSubscriber(&telecommande_dir_sub,&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			telecommande_dir_topic, &telecommande_dir_msg);

	createSubscriber(&config_mode_sub,&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			config_mode_topic, &config_mode_msg);

	createSubscriber(&config_speed_sub,&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			config_speed_topic, &config_speed_msg);

	CHECKMRRET(rclc_executor_init(&executor, &support.context, 5, &allocator), "Error on init executor");
	CHECKMRRET(rclc_executor_add_subscription(&executor, &camera_x_sub, &camera_x_msg, &SubscriberCallbackFunction, ON_NEW_DATA), "error add camera_x_sub");
	CHECKMRRET(rclc_executor_add_subscription(&executor, &camera_y_sub, &camera_y_msg, &SubscriberCallbackFunction, ON_NEW_DATA), "error add camera_y_sub");
	CHECKMRRET(rclc_executor_add_subscription(&executor, &telecommande_dir_sub, &telecommande_dir_msg, &SubscriberCallbackFunction, ON_NEW_DATA), "error add telecommande_dir_sub");
	CHECKMRRET(rclc_executor_add_subscription(&executor, &config_mode_sub, &config_mode_msg, &SubscriberCallbackFunction, ON_NEW_DATA), "error add config_mode_sub");
	CHECKMRRET(rclc_executor_add_subscription(&executor, &config_speed_sub, &config_speed_msg, &SubscriberCallbackFunction, ON_NEW_DATA), "error add config_speed_sub");

	for(;;)
	{
		if (!uxQueueMessagesWaiting(qhMR_sub))
			xQueueSend(qhMR_sub, ( void * ) &SubToMsg, portMAX_DELAY);
		rclc_executor_spin_some(&executor, 1*1000*1000);
		SubToMsg.dir = telecommande_dir_msg.data;
		SubToMsg.x = camera_x_msg.data;
		SubToMsg.y = camera_y_msg.data;
		SubToMsg.mode = config_mode_msg.data;
		SubToMsg.speed = config_speed_msg.data;

		if (uxQueueMessagesWaiting(qhMR_pub))
		{
			xQueueReceive(qhMR_pub, &MsgToPub, portMAX_DELAY);
			capteur_dir_msg.data = (int)MsgToPub.dir;
			etat_mode_msg.data = MsgToPub.mode;
			etat_speed_msg.data = MsgToPub.speed;
			CHECKMRRET(rcl_publish(&capteur_dir_pub, &capteur_dir_msg, NULL), "erreur publish capteur_dir_pub");
			CHECKMRRET(rcl_publish(&etat_mode_pub, &etat_mode_msg, NULL), "erreur publish etat_mode_pub");
			CHECKMRRET(rcl_publish(&etat_speed_pub, &etat_speed_msg, NULL), "erreur publish etat_speed_pub");
			#if DEBUG_PRINTF
			printf("\r\nReceive from decision :\r\nDirection : %d\r\nMode : %d\r\nSpeed : %d\r\n", capteur_dir_msg.data, etat_mode_msg.data, etat_speed_msg.data);
			#endif //DEBUG_PRINTF
		}
		vTaskDelay(SAMPLING_PERIOD_ms);
	}
#endif //SYNCHRO_EX
}
//========================================================================
static void task_Motor_Left(void *pvParameters)
{
	int16_t consigne = 0;

	float ui = 0.0;
	float up = 0.0;
	int err = 0;
	int speed = 0;

	int state = 0;

	for (;;)
	{
		xQueueReceive(q_mot_L, &consigne, portMAX_DELAY);

		speed = quadEncoder_GetSpeedL();
		err=consigne-speed;
		up=LKp*(float)err;
		ui=ui+LKp*LKi*(float)err;

		motorLeft_SetDuty(100+(int)(up+ui));

		xSemaphoreGive(xSem_Supervision);
		vTaskDelay(SAMPLING_PERIOD_ms);
	}
}

//=========================================================================
static void task_Motor_Right(void *pvParameters)
{
	int16_t consigne = 0;

	float ui= 0.0;
	float up = 0.0;
	int err = 0;
	int speed = 0;

	for (;;)
	{
		xQueueReceive(q_mot_R, &consigne, portMAX_DELAY);

		speed = quadEncoder_GetSpeedR();
		err=consigne-speed;
		up=RKp*(float)err;
		ui=ui+RKp*RKi*(float)err;

		motorRight_SetDuty(100+(int)(up+ui));

		xSemaphoreGive(xSem_Supervision);
		vTaskDelay(SAMPLING_PERIOD_ms);
	}
}

//=========================================================================
#if VL53
static void task_VL53(void *pvParameters)
{
	static uint16_t dist;
	static const int SEUIL = 20;
	int obs = 0;

	for(;;)
	{
		dist = readRangeSingleMillimeters()/10;
		if (dist < SEUIL && dist != 0)
			obs = 1;
		else
			obs = 0;

		if (!uxQueueMessagesWaiting(qhVl53))
			xQueueSend(qhVl53, (void *)&obs, portMAX_DELAY);

		vTaskDelay(SAMPLING_PERIOD_ms);
	}
}
#endif

//=========================================================================
#if LCD
static void task_Grove_LCD(void *pvParameters)
{
#if SYNCHRO_EX == EXFINAL
	AMessage pxRxedMessage;

	for(;;)
	{
		if (uxQueueMessagesWaiting(qhLCD))
		{
			xQueueReceive(qhLCD, &pxRxedMessage, portMAX_DELAY);
			int mode = pxRxedMessage.data;
			char direction=pxRxedMessage.command;
			groveLCD_setCursor(0,0);
			if (mode == MODE_OBS)
				groveLCD_term_printf("M:Obstacle  D:%c", direction);
			else if (mode == MODE_ZIG)
				groveLCD_term_printf("M:Zigbee        ");
			else if (mode == MODE_CAM)
				groveLCD_term_printf("M:Camera        ");
		}

		vTaskDelay(SAMPLING_PERIOD_ms);
	}
#elif SYNCHRO_EX == EXSTARTUP
	for (;;)
	{
		 groveLCD_setCursor(0,0);
		 groveLCD_term_printf("TEST LCD");
		 vTaskDelay(100);
	}
#endif
}
#endif

//=========================================================================
static void task_Supervision(void *pvParameters)
{
#if SYNCHRO_EX == EXSTARTUP
	int16_t consigne_G=0;
	int16_t consigne_D=0;

	int tab_mes_ir[2];
	uint16_t mes_vl53=0;

	vTaskDelay(100);
	for (;;)
	{
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);

	    captDistIR_Get(tab_mes_ir);
	    //mes_vl53 = readRangeSingleMillimeters()/10;

	    if((tab_mes_ir[0]>2000)||(tab_mes_ir[1]>2000))
	    { // !! obstacle
	    	consigne_G=0;
	    	consigne_D=0;
	    }
	    else
	    {
	    	consigne_G=1000;
	    	consigne_D=1000;
	    }

		xQueueSend( q_mot_L, ( void * ) &consigne_G,  portMAX_DELAY );
		xSemaphoreTake( xSem_Supervision, portMAX_DELAY );

		xQueueSend( q_mot_R, ( void * ) &consigne_D,  portMAX_DELAY );
		xSemaphoreTake( xSem_Supervision, portMAX_DELAY );

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
		vTaskDelay(SAMPLING_PERIOD_ms);
	}
#elif SYNCHRO_EX == EXFINAL
	int16_t speedLeft;
	int16_t speedRight;

	int table[2];
	int vl53 = 0;

	static int obs = 0;
	static char dir = 'f';
	static int direction = STOP;
	static int speed = LOW;
	static int mode = MODE_OBS;
	static int x = 0;
	static int y = 0;

#if LCD
	 AMessage pxMessage;
#endif

#if MICROROS
	MicroRosSubMsg SubToMsg;
	MicroRosPubMsg MsgToPub;
#endif //MICROROS

	for (;;)
	{
		#if MICROROS
		if (uxQueueMessagesWaiting(qhMR_sub))
		{
			xQueueReceive(qhMR_sub, &SubToMsg, portMAX_DELAY);
			if (SubToMsg.mode >= 0 && SubToMsg.mode < LAST_MODE)
				mode = SubToMsg.mode;
			if (SubToMsg.dir >= 0 && SubToMsg.dir < LAST_DIR)
				direction = SubToMsg.dir;
			if (SubToMsg.speed > 0 && SubToMsg.speed < LAST_SPEED)
				speed = SubToMsg.speed;
			x = SubToMsg.x;
			y = SubToMsg.y;
			#if DEBUG_PRINTF
			printf("%cc%c[2J%c[0;0HVariable to make decision : \n\rDirection : %d\r\nMode: %d\r\nSpeed : %d\r\nX: %d\r\nY : %d\r\n", 0x1b, 0x1b, 0x1b, direction, mode, speed, x, y);
			#endif //DEBUG_PRINTF
		}
		#endif //MICROROS

		if (mode == MODE_ZIG)
		{
			dir = 'N';
			obs = 0;
			switch(direction)
			{
				case STOP:
					speedLeft = 0;
					speedRight = 0;
					break;
				case AVANT:
					speedLeft = VITESSE_KART*speed;
					speedRight = VITESSE_KART*speed;
					break;
				case RECULE:
					speedLeft = -VITESSE_KART*speed;
					speedRight = -VITESSE_KART*speed;
					break;
				case DROITE:
					speedLeft = VITESSE_KART*speed;
					speedRight = -VITESSE_KART*speed;
					break;
				case GAUCHE:
					speedLeft = -VITESSE_KART*speed;
					speedRight = VITESSE_KART*speed;
					break;
				case AVANT_GAUCHE:
					speedLeft = (VITESSE_KART/2)*speed;
					speedRight = VITESSE_KART*speed;
					break;
				case AVANT_DROITE:
					speedLeft = VITESSE_KART*speed;
					speedRight = (VITESSE_KART/2)*speed;
					break;
				case RECULE_GAUCHE:
					speedLeft = -VITESSE_KART*speed;
					speedRight = (-VITESSE_KART/2)*speed;
					break;
				case RECULE_DROITE:
					speedLeft = (-VITESSE_KART/2)*speed;
					speedRight = -VITESSE_KART*speed;
					break;
				default:
					speedLeft = 0;
					speedRight = 0;
					break;
			}
		}
		else if (mode == MODE_OBS)
		{
			captDistIR_Get(table);
			#if VL53
			if (uxQueueMessagesWaiting(qhVl53))
				xQueueReceive(qhVl53, &vl53, portMAX_DELAY);
			else
				vl53 = 0;

			if (vl53 == 1) //Il y a un obstacle
			{
				speedLeft = 0;
				speedRight = 0;
				dir = 'S';
				obs = 1;
			}
			else
			#endif //VL53
			if (table[0] > 1000 || table[1] > 1000)
			{
				if (obs > 10)
				{
					speedLeft = VITESSE_OBS;
					speedRight = -VITESSE_OBS/2;
					dir = 'G';
				}
				else
				{
					speedLeft = 0;
					speedRight = 0;

					if (table[0] > table[1] && table[0] > 1000)
					{
						dir = 'G';
						speedLeft = VITESSE_OBS/2;
						speedRight = -VITESSE_OBS/2;
						if (obs%2 == 0)
							obs++;
					}
					else if (table[0] < table[1] && table[1] > 1000)
					{
						dir = 'D';
						speedLeft = -VITESSE_OBS/2;
						speedRight = VITESSE_OBS/2;
						if (obs%2 == 1)
							obs++;
					}
				}
			}
			else
			{
				speedLeft = VITESSE_OBS;
				speedRight = VITESSE_OBS;
				dir = 'F';
				obs = 0;
			}
		}
		else if (mode == MODE_CAM)
		{
			dir = 'N';
			obs = 0;

			if (x > CAMERA_X_MIN+CAMERA_X_TIER && x < CAMERA_X_MAX-CAMERA_X_TIER && y > CAMERA_Y_MIN && y <CAMERA_Y_MIN+CAMERA_Y_TIER) //AVANT
			{
				speedLeft = VITESSE_CAM;
				speedRight = VITESSE_CAM;
			}
			else if (x > CAMERA_X_MAX-CAMERA_X_TIER && x < CAMERA_X_MAX && y > CAMERA_Y_MIN && y < CAMERA_Y_MIN+CAMERA_Y_TIER) //AVANT_DROITE:
			{
				speedLeft = VITESSE_CAM;
				speedRight = VITESSE_CAM/2;
			}
			else if (x > CAMERA_X_MIN && x < CAMERA_X_MIN+CAMERA_X_TIER && y > CAMERA_Y_MIN && y < CAMERA_Y_MIN+CAMERA_Y_TIER) //AVANT_GAUCHE:
			{
				speedLeft = VITESSE_CAM/2;
				speedRight = VITESSE_CAM;
			}
			else if (x > CAMERA_X_MIN+CAMERA_X_TIER && x < CAMERA_X_MAX-CAMERA_X_TIER && y > CAMERA_Y_MIN+CAMERA_Y_TIER && y <CAMERA_Y_MAX-CAMERA_Y_TIER) //STOP
			{
				speedLeft = 0;
				speedRight = 0;
			}
			else if (x > CAMERA_X_MAX-CAMERA_X_TIER && x < CAMERA_X_MAX && y > CAMERA_Y_MIN+CAMERA_Y_TIER && y <CAMERA_Y_MAX-CAMERA_Y_TIER) //DROITE
			{
				speedLeft = VITESSE_CAM;
				speedRight = -VITESSE_CAM;
			}
			else if (x > CAMERA_X_MIN && x < CAMERA_X_MIN+CAMERA_X_TIER && y > CAMERA_Y_MIN+CAMERA_Y_TIER && y <CAMERA_Y_MAX-CAMERA_Y_TIER) //GAUCHE
			{
				speedLeft = -VITESSE_CAM;
				speedRight = VITESSE_CAM;
			}
			else if (x > CAMERA_X_MIN+CAMERA_X_TIER && x < CAMERA_X_MAX-CAMERA_X_TIER && y > CAMERA_Y_MAX-CAMERA_Y_TIER && y < CAMERA_Y_MAX) //RECULE:
			{
				speedLeft = -VITESSE_CAM;
				speedRight = -VITESSE_CAM;
			}
			else if (x > CAMERA_X_MAX-CAMERA_X_TIER && x < CAMERA_X_MAX && y > CAMERA_Y_MAX-CAMERA_Y_TIER && y < CAMERA_Y_MAX) //RECULE_DROITE:
			{
				speedLeft = -VITESSE_CAM/2;
				speedRight = -VITESSE_CAM;
			}
			else if (x > CAMERA_X_MIN && x < CAMERA_X_MIN+CAMERA_X_TIER && y > CAMERA_Y_MAX-CAMERA_Y_TIER && y < CAMERA_Y_MAX) //RECULE_GAUCHE:
			{
				speedLeft = -VITESSE_CAM;
				speedRight = -VITESSE_CAM/2;
			}
			else
			{
				speedLeft = 0;
				speedRight = 0;
			}
		}

		#if DEBUG_MOTOR
		printf("Motor L : %d || R : %d\r\n", speedLeft, speedRight);
		#endif

		xQueueSend( q_mot_L, ( void * ) &speedLeft,  portMAX_DELAY );
		xSemaphoreTake( xSem_Supervision, portMAX_DELAY );

		xQueueSend( q_mot_R, ( void * ) &speedRight,  portMAX_DELAY );
		xSemaphoreTake( xSem_Supervision, portMAX_DELAY );

	#if MICROROS
		MsgToPub.dir = dir;
		MsgToPub.mode = mode;
		MsgToPub.speed = speed;
		if (!uxQueueMessagesWaiting(qhMR_pub))
			xQueueSend(qhMR_pub, ( void * ) &MsgToPub, portMAX_DELAY);
	#endif //MICROROS

	#if LCD
		if (!uxQueueMessagesWaiting(qhLCD))
		{
			pxMessage.data=mode;
			pxMessage.command=dir;
			xQueueSend( qhLCD, ( void * ) &pxMessage, portMAX_DELAY);
		}
	#endif //LCD

		vTaskDelay(SAMPLING_PERIOD_ms);
	}
#endif //SYNCHRO_EX
}

//=========================================================================
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  RetargetInit(&huart2); //make printf and scanf work with uart2
  printf("%cc%c[2J%c[0;0HTitouan//Jeremy//Louanne\r\n", 0x1b, 0x1b, 0x1b);

  motorCommand_Init();
  quadEncoder_Init();
  captDistIR_Init();

  HAL_Delay(500);

#if VL53
  initVL53L0X();
  for (int i=0 ; i<20 ; i++)
  {
	  printf("%d\r\n", readRangeSingleMillimeters()/10);
  }
  HAL_Delay(500);
#endif

  // Test Ecran LCD
#if LCD
  groveLCD_begin(16,2,0); // !! cette fonction prend du temps
  HAL_Delay(100);
  groveLCD_setCursor(0,0);
  groveLCD_setColor(1);
  groveLCD_term_printf("Titouan//Jeremy//Louanne");
  HAL_Delay(1000);
#endif

  osKernelInitialize();
  //defaultTaskHandle = osThreadNew(microros_task, NULL, &defaultTask_attributes);

#if SYNCHRO_EX == EXSTARTUP
	#if MICROROS
  	xTaskCreate( microros_task, ( const portCHAR * ) "microros_task", 3000 /* stack size */, NULL, 24, NULL);
	#endif //MICROROS
  	xTaskCreate( task_Supervision, ( const portCHAR * ) "task Supervision", 128 /* stack size */, NULL, 27, NULL);
	xTaskCreate( task_Motor_Left, ( const portCHAR * ) "task Mot L", 128 /* stack size */, NULL, 25, NULL);
	xTaskCreate( task_Motor_Right, ( const portCHAR * ) "task Mot R", 128 /* stack size */, NULL, 26, NULL);
	#if LCD
	xTaskCreate( task_Grove_LCD, ( const portCHAR * ) "task Mot R", 128 /* stack size */, NULL, 23, NULL);
	#endif
#elif SYNCHRO_EX == EXTEST_UART2
	xTaskCreate(test_uart2, ( const portCHAR * ) "task print uart 2", 128 /* stack size */, NULL, tskIDLE_PRIORITY, NULL);
#elif SYNCHRO_EX == EXCORRECTOR
	xTaskCreate(test_motor, ( const portCHAR * ) "task test motor", 128 /* stack size */, NULL, tskIDLE_PRIORITY, NULL);
#elif SYNCHRO_EX == EXTEST_VL53
	xTaskCreate(test_vl53, ( const portCHAR * ) "test_vl53", 128 /* stack size */, NULL, tskIDLE_PRIORITY, NULL);
#elif SYNCHRO_EX == EXTEST_MICROROS
    xTaskCreate(microros_task, ( const portCHAR * ) "microros_task", 3000 /* stack size */, NULL, tskIDLE_PRIORITY, NULL);
#elif SYNCHRO_EX == EXFINAL
	#if MICROROS
    xTaskCreate(microros_task, ( const portCHAR * ) "microros_task", 3000 /* stack size */, NULL, 24, NULL);
	#endif //MICROROS
    xTaskCreate(task_Supervision, ( const portCHAR * ) "task Supervision", 128 /* stack size */, NULL, 27, NULL);
	xTaskCreate(task_Motor_Left, ( const portCHAR * ) "task Motor Left", 128 /* stack size */, NULL, 25, NULL);
	xTaskCreate(task_Motor_Right, ( const portCHAR * ) "task Motor Right", 128 /* stack size */, NULL, 26, NULL);

	#if VL53
	xTaskCreate(task_VL53, ( const portCHAR * ) "task VL53", 128 /* stack size */, NULL, 23, NULL);
	#endif

	#if LCD
	xTaskCreate(task_Grove_LCD, ( const portCHAR * ) "task LCD", 128 /* stack size */, NULL, 23, NULL);
	#endif
#endif

    vSemaphoreCreateBinary(xSem_Supervision);
    xSemaphoreTake( xSem_Supervision, portMAX_DELAY );

    q_mot_L = xQueueCreate(1, sizeof(int16_t));
    q_mot_R = xQueueCreate(1, sizeof(int16_t));
    qhVl53 = xQueueCreate(1, sizeof(int));

    qhMR_sub = xQueueCreate(1, sizeof(MicroRosSubMsg));
    qhMR_pub = xQueueCreate(1, sizeof(MicroRosPubMsg));
    qhLCD = xQueueCreate(1, sizeof(AMessage));

  osKernelStart();
  while(1)
  {

  }
}

//=========================================================================
void test_uart2(void *pvParameters)
{
	char buf[100] = "";
	for(;;)
	{
		printf("Veuillez saisir votre nom :\r\n");
		scanf("%s", buf);
		printf("bonjour et bienvenue %s\r\n", buf);
		vTaskDelay(SAMPLING_PERIOD_ms);
	}
}

void test_vl53(void *pvParameters)
{
	uint16_t val;

	for(;;)
	{
		val = readRangeSingleMillimeters()/10;
		printf("Distance capteur : %d\r\n", val);
	}
}

void test_motor(void *pvParameters)
{
	int16_t  consigne = TEST_CORRECTOR_SPEED;
	if (consigne < 0 || consigne > 200)
		consigne = 150;
	int speed = 0;
	int i = 0;

	for (;;)
	{
		#if TEST_LEFT_MOTOR
		motorLeft_SetDuty(consigne);
		speed = quadEncoder_GetSpeedL();
		#else
		motorRight_SetDuty(consigne);
		speed = quadEncoder_GetSpeedR();
		#endif

		if(i<NB)
		{
			tab_speed[i]=speed;
			i++;
		}
		else
			printf("sampling end");
		vTaskDelay(SAMPLING_PERIOD_ms);
	}
}

//=========================================================================
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
}
//=========================================================================
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {}
}
//=========================================================================
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
