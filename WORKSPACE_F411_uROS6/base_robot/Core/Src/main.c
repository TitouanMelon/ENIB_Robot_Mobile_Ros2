/**
 * @file main.c
 * @brief file that contain the main code
 */
#include "main.h"

#include "motorCommand.h"
#include "quadEncoder.h"
#include "captDistIR.h"
#include "VL53L0X.h"
#include "groveLCD.h"

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

/** enumerate mode of robot */
enum {MODE_OBS, MODE_ZIG, MODE_CAM, LAST_MODE};
/** enumerate speed */
enum {STOP_VIT, LOW, FAST, SONIC, LAST_SPEED=100};
/** enumerate direction */
enum {AVANT, GAUCHE, RECULE, DROITE, STOP, AVANT_GAUCHE, AVANT_DROITE, RECULE_GAUCHE, RECULE_DROITE, LAST_DIR};

#define SAMPLING_PERIOD_ms 5 /**< Define the delay beetween two execution of the same task */
/**@{ @name config exo */
#define EXSTARTUP 0 /**< startup code */
#define EXTEST_UART2 1 /**< Test printf and scanf function */
#define EXCORRECTOR 2 /**< Code to calibrate your correcteur */
#define EXTESTCORRECTOR 3 /**< Code to test your correcteur */
#define EXTEST_VL53 4 /**< Test VL530X sensor */
#define EXTEST_MICROROS 5 /**< Test Micro ROS subscriber and publisher */
#define EXFINAL 6 /**< Final code */

#define SYNCHRO_EX EXFINAL /**< Define wich config are executed */
/** @} */
/** @{ @name config robot */
#define SEUIL_DIST_SENSOR 800 /**< Define the trigger for forward sensors4 */
#define ROS_DOMAIN_ID 0 /**< Define ROS domain id*/
#define LCD 1 /**< Activate LCD task */
#define VL53 1 /**< Activate VL530X task */
#define MICROROS 1 /**< Activate MicroROS task*/
#define DEBUG_PRINTF 0 /**< Activate debug print */
#define DEBUG_MOTOR 0 /**< Activate motor debug print */
/** @} */
/** @{ @name config correcteur */
#define Te SAMPLING_PERIOD_ms /**< */
#define LKp 0.001 /**< Kp factor for the left motor */
#define LKi (5.0/(0.1*40.0)) /**< Ki factor for the left motor */
#define RKp 0.001 /**< Kp factor for the right motor */
#define RKi (5.0/(0.1*40.0)) /**< Kp factor for the right motor */
/** @} */
/** @{ @name config default speed for each mode */
#define CMD 1000 /**< Can be use as default speed */
#define VITESSE_KART CMD/2 /**< Default speed for manual mode */
#define VITESSE_OBS CMD /**< Default speed for obstacle mode */
#define VITESSE_CAM CMD/3 /**< Default speed for camera mode */
/** @} */
/** @{ @name config camera settings */
#define CAMERA_X_MIN 0 /**< Define minimal x position return by camera */
#define CAMERA_X_MAX 640 /**< Define maximal x position return by camera */
#define CAMERA_Y_MIN 0 /**< Define minimal y position return by camera */
#define CAMERA_Y_MAX 480 /**< Define maximal y position return by camera */
//#define CAMERA_X_TIER (CAMERA_X_MAX-CAMERA_X_MIN)/3 /**< */
//#define CAMERA_Y_TIER (CAMERA_Y_MAX-CAMERA_Y_MIN)/3 /**< */
/** @} */
/** @{ @name config default behaviour */
#define DEFAULT_MODE MODE_ZIG /**< Default mode at startup */
#define DEFAULT_SPEED LOW /**< Default speed at startup */
#define DEFAULT_DIR STOP /**< Default direction at startup */
/** @} */
/** @{ @name config test value */
#define NB 200 /**< Number of samples in correcteur calibration task */
#define TEST_CORRECTOR_DUTY 150 /**< Duty cycle to apply to calibrate the correcteur */
#define TEST_CORRECTOR_SPEEDL -100 /**< Speed to test the left correcteur */
#define TEST_CORRECTOR_SPEEDR -100 /**< Speed to test the right correcteur */
#define TEST_LEFT_MOTOR 1 /**< Calibrate left motor correcteur or not */
/** @} */

// Déclaration des objets synchronisants !! Ne pas oublier de les créer
/** @{ @name semaphore */
xSemaphoreHandle xSem_Supervision = NULL; /**< Semaphore use in decision task */
/** @} */
/** @{ @name queueHandle */
xQueueHandle q_mot_L = NULL; /**< Queue to communicate with left motor task */
xQueueHandle q_mot_R = NULL; /**< Queue to communicate with right motor task */
xQueueHandle qhMR_sub = NULL; /**< Queue to get information from microRos task */
xQueueHandle qhMR_pub = NULL; /**< Queue to communicate with microRos task */
xQueueHandle qhLCD = NULL; /**< Queue to communicate with LCD task */
xQueueHandle qhVl53 = NULL; /**< Queue to communicate with VL53 task */
/** @} */

int16_t tab_speed[NB]; /**< use to store speed of motor during calibration of the correcteur */

/** @{ @name queue messages structures */
/**
 * Use to send data to lcd's task
 */
typedef struct
{
	char command; /**< Represent the direction of the robot */
	int data; /**< Represent the mode of the robot */
} AMessage;

/**
 * Use to send information from the task decision to microRos task
 */
typedef struct
{
	char dir; /**< Represent the direction of the robot */
	int mode; /**< Represent the mode of the robot */
	int speed; /**< Represent the speed of the robot */
} MicroRosPubMsg;

/**
 * Use to send information get by microRos to decision task
 */
typedef struct
{
	int dir; /**< Represent the direction send by the IHM */
	int x; /**< Represent the x position send by the camera */
	int y; /**< Represent the y position send by the camera */
	int mode; /**< Represent the mode send by the IHM */
	int speed; /**< Represent the speed send by the IHM */
} MicroRosSubMsg;
/** @} */

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

void CHECKMRRET(rcl_ret_t ret, char* msg){if (ret != RCL_RET_OK){ if (DEBUG_PRINTF){printf("Error : %d\r\nMsg : %s\r\n", (int)ret, msg); }}}

void SubscriberCallbackFunction(const void *msgin){
#if SYNCHRO_EX == EXTEST_MICROROS
	std_msgs__msg__String * msg = (std_msgs__msg__String * )msgin;
	printf("\r\nMessage recue : %s\r\n", msg->data->data);
#elif SYNCHRO_EX == EXFINAL
	std_msgs__msg__Int32 * msg = (std_msgs__msg__Int32 * )msgin;
	if (DEBUG_PRINTF)
		printf("\r\nMessage recue : %ld\r\n", msg->data);
#endif //SYNCHRO_EX
}

// https://github.com/lFatality/stm32_micro_ros_setup
void microros_task(void *argument)
{
	// micro-ROS app variable
	rclc_support_t support; //Contain information about how config microros
	rcl_allocator_t allocator; //Contain information about how microRos can allocate memory
	rcl_node_t node; //microRos structure wich represent a node ROS
	rcl_node_options_t node_opt; //microRos structure wich represent option of a node ROS
	rclc_executor_t executor; //microRos structure wich represent an executor wich can be use to receive message

	// micro-ROS configuration with freertos
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
	node_opt = rcl_node_get_default_options(); //Get default node options
	node_opt.domain_id = ROS_DOMAIN_ID; //Set the ROS_DOMAIN_ID
	CHECKMRRET(rclc_node_init_with_options(&node, "STM32_node", "", &support, &node_opt), "error on init node");


#if SYNCHRO_EX == EXSTARTUP
	static int counter = 0;
	rcl_ret_t ret; //Use to store the return of microRos function
	rcl_publisher_t publisher; //microRos structure wich represent a publisher
	std_msgs__msg__String msg; //microRos structure wich represent a String ROS message

	CHECKMRRET(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "cubemx_publisher"),
			"Error when create publisher"); //Create a default publisher wich publish on topic named "cubemx_publisher"

	//Allocate memory for string message
	msg.data.data = (char * ) malloc(100 * sizeof(char));
	msg.data.size = 0;
	msg.data.capacity = 100; //Capacity need to be less than or equal to the allocution memory space

	for (;;)
	{
		sprintf(msg.data.data, "Hello from micro-ROS #%d", counter++); //Write string in message
		msg.data.size = strlen(msg.data.data); //Set the size of the message
		ret = rcl_publish(&publisher, &msg, NULL); //Publish the message
		if (ret != RCL_RET_OK)
			printf("Error publishing (line %d)\r\n", __LINE__); //If the message are not publish print an error
		vTaskDelay(SAMPLING_PERIOD_ms);
	}
#elif SYNCHRO_EX == EXTEST_MICROROS
	//micro-ros topic variable
	rcl_ret_t ret; //Use to store the return of microRos function
	rcl_publisher_t publisher; //microRos structure wich represent a publisher
	rcl_subscription_t subscriber; //microRos structure wich represent a subsriber
	std_msgs__msg__String msg; //microRos structure wich represent a String ROS message

	//create default publisher wich publish on topic named "cubemx_publisher"
	CHECKMRRET(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "cubemx_publisher"),
			"Error when create publisher");

	//create default subscriber wich listen to the topic named "cubemx_subscriber"
	subscriber = rcl_get_zero_initialized_subscription();
	CHECKMRRET(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),"cubemx_subscriber"),
			"Error when create subscriber");

	//Init string msg
	msg.data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
	msg.data.size = 0;
	msg.data.capacity = ARRAY_LEN;

	// Init executor by indicate how many subscriber we will put in it
	CHECKMRRET(rclc_executor_init(
			&executor, //executor structure
			&support.context,
			1, //number of subscriber that will be add
			&allocator), "Error on init executor");
	//Add subsciber to the executor
	CHECKMRRET(rclc_executor_add_subscription(&executor, //executor structure
			&subscriber, //subscriber structure
			&msg, //msg structure
			&SubscriberCallbackFunction, ON_NEW_DATA), "error add subscriber");

	for (;;)
	{
		//Execute the executor to receive message
		ret = rclc_executor_spin_some(&executor, 100*1000*1000);
		vTaskDelay(SAMPLING_PERIOD_ms);
	}
#elif SYNCHRO_EX == EXFINAL
	//Init the queue mesage
	MicroRosPubMsg MsgToPub = {'N', 0, 0};
	MicroRosSubMsg SubToMsg = {DEFAULT_DIR, 0, 0, DEFAULT_MODE, DEFAULT_SPEED};
	/* PUBLISHER */
	//Use to publish the direction of robot in sensor mode
	rcl_publisher_t capteur_dir_pub;
	char* capteur_dir_topic = CAPTEUR_DIR_TOPIC;
	std_msgs__msg__Int32 capteur_dir_msg;
	//Use to publish the actual mode of the robot
	rcl_publisher_t etat_mode_pub;
	char* etat_mode_topic = ETAT_MODE_TOPIC;
	std_msgs__msg__Int32 etat_mode_msg;
	//Use to publish the actual speed of the robot
	rcl_publisher_t etat_speed_pub;
	char* etat_speed_topic = ETAT_SPEED_TOPIC;
	std_msgs__msg__Int32 etat_speed_msg;
	/* SUBSCRIBER */
	//Use to receive the x position of object see by the camera
	rcl_subscription_t camera_x_sub;
	char* camera_x_topic = CAMERA_X_TOPIC;
	std_msgs__msg__Int32 camera_x_msg;
	//Use to receive the y position of object see by the camera
	rcl_subscription_t camera_y_sub;
	char* camera_y_topic = CAMERA_Y_TOPIC;
	std_msgs__msg__Int32 camera_y_msg;
	//Use to receive the remote control in remote mode
	rcl_subscription_t telecommande_dir_sub;
	char* telecommande_dir_topic = TELECOMMANDE_DIR_TOPIC;
	std_msgs__msg__Int32 telecommande_dir_msg;
	//Use to receive the mode config
	rcl_subscription_t config_mode_sub;
	char* config_mode_topic = CONFIG_MODE_TOPIC;
	std_msgs__msg__Int32 config_mode_msg;
	//Use to receive the speed config
	rcl_subscription_t config_speed_sub;
	char* config_speed_topic = CONFIG_SPEED_TOPIC;
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
	createSubscriber(&camera_x_sub, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		camera_x_topic, &camera_x_msg);

	createSubscriber(&camera_y_sub, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			camera_y_topic, &camera_y_msg);

	createSubscriber(&telecommande_dir_sub, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			telecommande_dir_topic, &telecommande_dir_msg);

	createSubscriber(&config_mode_sub, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			config_mode_topic, &config_mode_msg);

	createSubscriber(&config_speed_sub, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			config_speed_topic, &config_speed_msg);

	//Init the executor
	CHECKMRRET(rclc_executor_init(&executor, &support.context, 5, &allocator), "Error on init executor");
	/*Add subscriber to executor to let it check if message is receive on this
	topic and store the data on the message structure after call the callback*/
	CHECKMRRET(rclc_executor_add_subscription(&executor, &camera_x_sub, &camera_x_msg, &SubscriberCallbackFunction, ON_NEW_DATA), "error add camera_x_sub");
	CHECKMRRET(rclc_executor_add_subscription(&executor, &camera_y_sub, &camera_y_msg, &SubscriberCallbackFunction, ON_NEW_DATA), "error add camera_y_sub");
	CHECKMRRET(rclc_executor_add_subscription(&executor, &telecommande_dir_sub, &telecommande_dir_msg, &SubscriberCallbackFunction, ON_NEW_DATA), "error add telecommande_dir_sub");
	CHECKMRRET(rclc_executor_add_subscription(&executor, &config_mode_sub, &config_mode_msg, &SubscriberCallbackFunction, ON_NEW_DATA), "error add config_mode_sub");
	CHECKMRRET(rclc_executor_add_subscription(&executor, &config_speed_sub, &config_speed_msg, &SubscriberCallbackFunction, ON_NEW_DATA), "error add config_speed_sub");

	for(;;)
	{
		if (!uxQueueMessagesWaiting(qhMR_sub)) //If no message in 'output' queue
			xQueueSend(qhMR_sub, ( void * ) &SubToMsg, portMAX_DELAY); //Send queue message
		rclc_executor_spin_some(&executor, 1*1000*1000); //Execute executor

		//Put the receive data into the queue message structure
		SubToMsg.dir = telecommande_dir_msg.data;
		SubToMsg.x = camera_x_msg.data;
		SubToMsg.y = camera_y_msg.data;
		SubToMsg.mode = config_mode_msg.data;
		SubToMsg.speed = config_speed_msg.data;

		if (uxQueueMessagesWaiting(qhMR_pub)) //If no message in 'input' queue
		{
			xQueueReceive(qhMR_pub, &MsgToPub, portMAX_DELAY); //Receive data
			capteur_dir_msg.data = (int)MsgToPub.dir;
			etat_mode_msg.data = MsgToPub.mode;
			etat_speed_msg.data = MsgToPub.speed;

			//Publish data
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

void task_Motor_Left(void *pvParameters)
{
	int16_t consigne = 0; //Store the desirate speed

	float ui = 0.0; //Integral term of the correcteur
	float up = 0.0; //Proportionnal term of the correcteur
	int err = 0; //Error term of the correcteur
	int speed = 0; //Actual speed of motor

	for (;;)
	{
		xQueueReceive(q_mot_L, &consigne, portMAX_DELAY); //receive wanted speed

		speed = quadEncoder_GetSpeedL(); //Get actual speed
		//Calculate term of correcteur
		err=consigne-speed;
		up=LKp*(float)err;
		ui=ui+LKp*LKi*(float)err;

		motorLeft_SetDuty(100+(int)(up+ui)); //Set duty cycle of the motor

		xSemaphoreGive(xSem_Supervision); //Give semaphore to liberate the decision task
		vTaskDelay(SAMPLING_PERIOD_ms);
	}
}

void task_Motor_Right(void *pvParameters)
{
	int16_t consigne = 0; //Store the desirate speed

	float ui = 0.0; //Integral term of the correcteur
	float up = 0.0; //Proportionnal term of the correcteur
	int err = 0; //Error term of the correcteur
	int speed = 0; //Actual speed of motor

	for (;;)
	{
		xQueueReceive(q_mot_R, &consigne, portMAX_DELAY); //receive wanted speed

		speed = quadEncoder_GetSpeedR(); //Get actual speed
		//Calculate term of correcteur
		err=consigne-speed;
		up=RKp*(float)err;
		ui=ui+RKp*RKi*(float)err;

		motorRight_SetDuty(100+(int)(up+ui)); //Set duty cycle of the motor

		xSemaphoreGive(xSem_Supervision);//Give semaphore to liberate the decision task
		vTaskDelay(SAMPLING_PERIOD_ms);
	}
}

#if VL53
void task_VL53(void *pvParameters)
{
	static uint16_t dist;
	static const int SEUIL = 20; //Trigger
	int obs = 0; //Bool to indicate if we detect an obstacle or not

	for(;;)
	{
		dist = readRangeSingleMillimeters()/10; //Get the distance from the sensor

		if (dist < SEUIL && dist != 0) //If distance is less than the trigger
			obs = 1; //We detect an obstacle
		else
			obs = 0; //We do not detect an obstacle

		if (!uxQueueMessagesWaiting(qhVl53)) //If no data in queue
			xQueueSend(qhVl53, (void *)&obs, portMAX_DELAY); //Send data

		vTaskDelay(SAMPLING_PERIOD_ms);
	}
}
#endif //VL53


#if LCD
void task_Grove_LCD(void *pvParameters)
{
#if SYNCHRO_EX == EXSTARTUP
	for (;;)
	{
		 groveLCD_setCursor(0,0); //Set cursor position to 0,0
		 groveLCD_term_printf("TEST LCD"); //Write TEST LCD on the screen
		 vTaskDelay(100);
	}
#elif SYNCHRO_EX == EXFINAL
	AMessage pxRxedMessage;

	for(;;)
	{
		if (uxQueueMessagesWaiting(qhLCD)) //If data in the queue
		{
			xQueueReceive(qhLCD, &pxRxedMessage, portMAX_DELAY); //Receive data
			int mode = pxRxedMessage.data;
			char direction=pxRxedMessage.command;
			groveLCD_setCursor(0,0);
			//Write on screen information about mode
			if (mode == MODE_OBS)
				groveLCD_term_printf("M:Obstacle  D:%c", direction);
			else if (mode == MODE_ZIG)
				groveLCD_term_printf("M:Manuel        ");
			else if (mode == MODE_CAM)
				groveLCD_term_printf("M:Camera        ");
		}

		vTaskDelay(SAMPLING_PERIOD_ms);
	}
#endif //SYNCHRO_EX
}
#endif //LCD

void task_Supervision(void *pvParameters)
{
#if SYNCHRO_EX == EXSTARTUP
	int16_t consigne_G=0; //Motor left speed
	int16_t consigne_D=0; //Motor rigth speed

	int tab_mes_ir[2]; //VL53L0X sensors values
	uint16_t mes_vl53=0; //VL530X sensor value

	vTaskDelay(100);
	for (;;)
	{
		//Get sensor value
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

		vTaskDelay(SAMPLING_PERIOD_ms);
	}
#elif SYNCHRO_EX == EXTESTCORRECTOR
	int16_t speedLeft = TEST_CORRECTOR_SPEEDL;
	int16_t speedRight = TEST_CORRECTOR_SPEEDR;

	for (;;)
	{
		xQueueSend(q_mot_L, (void *)&speedLeft, portMAX_DELAY);
		xSemaphoreTake(xSem_Supervision, portMAX_DELAY);

		xQueueSend(q_mot_R, (void *)&speedRight, portMAX_DELAY);
		xSemaphoreTake(xSem_Supervision, portMAX_DELAY);

		vTaskDelay(SAMPLING_PERIOD_ms);
	}
#elif SYNCHRO_EX == EXFINAL
	int16_t speedLeft; //Motor left speed
	int16_t speedRight; //Motor rigth speed

	int table[2]; //VL53L0X sensors values
	#if VL53
	int vl53 = 0; //VL530X detect an obstacle or not
	#endif //VL53

	static int obs = 0; //store the number of different obstacle detected without break
	static char dir = 'f'; //represent the direction of the robot in obstacle mode
	static int direction = DEFAULT_DIR; //default direction of the robot
	static int speed = DEFAULT_SPEED; //default speed of the robot
	static int mode = DEFAULT_MODE; //default mode of the robot
	static int x = 0; //position x of the object detect by the camera
	static int y = 0; //position y of the object detect by the camera

#if LCD
	 AMessage pxMessage; //LCD queue message
#endif

#if MICROROS
	MicroRosSubMsg SubToMsg; //ROS subscriber queue message
	MicroRosPubMsg MsgToPub; //ROS publisher queue message
#endif //MICROROS

	for (;;)
	{
		#if MICROROS
		if (uxQueueMessagesWaiting(qhMR_sub)) //If data  are in the the queue
		{
			xQueueReceive(qhMR_sub, &SubToMsg, portMAX_DELAY); //Receive data
			//Set mode, speed and direction if the data is correct
			if (SubToMsg.mode >= 0 && SubToMsg.mode < LAST_MODE)
				mode = SubToMsg.mode;
			if (SubToMsg.dir >= 0 && SubToMsg.dir < LAST_DIR)
				direction = SubToMsg.dir;
			if (SubToMsg.speed > 0 && SubToMsg.speed < LAST_SPEED)
				speed = SubToMsg.speed;
			//Set x and y position
			x = SubToMsg.x;
			y = SubToMsg.y;
			#if DEBUG_PRINTF
			printf("%cc%c[2J%c[0;0HVariable to make decision : \n\rDirection : %d\r\nMode: %d\r\nSpeed : %d\r\nX: %d\r\nY : %d\r\n", 0x1b, 0x1b, 0x1b, direction, mode, speed, x, y);
			#endif //DEBUG_PRINTF
		}
		#endif //MICROROS

		if (mode == MODE_ZIG) //Mode manual
		{
			dir = 'N'; //No direction information
			obs = 0; //No obstacle
			switch(direction) //Set the motor speed depending of the direction variable
			{
				case STOP:
					speedLeft = 0;
					speedRight = 0;
					break;
				case AVANT:
					speedLeft = VITESSE_KART+(8*(speed-50));
					speedRight = VITESSE_KART+(8*(speed-50));
					break;
				case RECULE:
					speedLeft = -(VITESSE_KART+(8*(speed-50)));
					speedRight = -(VITESSE_KART+(8*(speed-50)));
					break;
				case DROITE:
					speedLeft = VITESSE_KART+(8*(speed-50));
					speedRight = -(VITESSE_KART+(8*(speed-50)));
					break;
				case GAUCHE:
					speedLeft = -(VITESSE_KART+(8*(speed-50)));
					speedRight = VITESSE_KART+(8*(speed-50));
					break;
				case AVANT_GAUCHE:
					speedLeft = (VITESSE_KART/2)+(8*(speed-50));
					speedRight = VITESSE_KART+(8*(speed-50));
					break;
				case AVANT_DROITE:
					speedLeft = VITESSE_KART+(8*(speed-50));
					speedRight = (VITESSE_KART/2)+(8*(speed-50));
					break;
				case RECULE_GAUCHE:
					speedLeft = -(VITESSE_KART+(8*(speed-50)));
					speedRight = -((VITESSE_KART/2)+(8*(speed-50)));
					break;
				case RECULE_DROITE:
					speedLeft = -((VITESSE_KART/2)+(8*(speed-50)));
					speedRight = -(VITESSE_KART+(8*(speed-50)));
					break;
				default:
					speedLeft = 0;
					speedRight = 0;
					break;
			}
		}
		else if (mode == MODE_OBS) //Mode obstacle
		{
			//Get sensors informations
			captDistIR_Get(table);
			#if VL53
			if (uxQueueMessagesWaiting(qhVl53))
				xQueueReceive(qhVl53, &vl53, portMAX_DELAY);
			else
				vl53 = 0;

			if (vl53 == 1) //if an obstacle is detected on the back we stop
			{
				if (dir != 'S')
					printf("Detection d'un obstacle à l'arrièrre");
				speedLeft = 0;
				speedRight = 0;
				dir = 'S';
				obs = 1;
			}
			else
			#endif //VL53
			if (table[0] > SEUIL_DIST_SENSOR || table[1] > SEUIL_DIST_SENSOR) //We have an obstacle in front of the robot
			{
				if (obs > 10) //If we detect more than 10 different obstacle we turn on the left until they are no more obstacle
				{
					speedLeft = VITESSE_OBS/2;
					speedRight = -VITESSE_OBS/2;
					dir = 'G';
				}
				else
				{
					speedLeft = 0;
					speedRight = 0;

					if (table[0] > table[1] && table[0] > SEUIL_DIST_SENSOR) //We have an obstacle on our right
					{
						dir = 'G';
						speedLeft = -VITESSE_OBS/2;
						speedRight = VITESSE_OBS/2;
						if (obs%2 == 0)
							obs++;
					}
					else if (table[0] < table[1] && table[1] > SEUIL_DIST_SENSOR) //We have an obstacle on left right
					{
						dir = 'D';
						speedLeft = VITESSE_OBS/2;
						speedRight = -VITESSE_OBS/2;
						if (obs%2 == 1)
							obs++;
					}
				}
			}
			else //No obstacle
			{
				speedLeft = VITESSE_OBS;
				speedRight = VITESSE_OBS;
				dir = 'F';
				obs = 0;
			}
		}
		else if (mode == MODE_CAM) //Mode camera
		{
			dir = 'N';
			obs = 0;

			if(x < 0 || y < 0) //No object
			{
				speedLeft = 0;
				speedRight = 0;
			}
			else //Try to keep the object on the center
			{
				speedLeft = VITESSE_CAM - ((CAMERA_X_MAX/2 - x))/3; // (int) (((float) ((x-CAMERA_X_MAX/2)/CAMERA_X_MAX))*500);
				speedRight = VITESSE_CAM + ((CAMERA_X_MAX/2 - x))/3; // (int) (((float) (x/CAMERA_X_MAX))*500);
			}
		}

		#if DEBUG_MOTOR
		printf("Motor L : %d || R : %d\r\n", speedLeft, speedRight);
		#endif

		xQueueSend( q_mot_L, ( void * ) &speedLeft,  portMAX_DELAY ); //Send motor left speed
		xSemaphoreTake( xSem_Supervision, portMAX_DELAY );

		xQueueSend( q_mot_R, ( void * ) &speedRight,  portMAX_DELAY ); //Send motor right speed
		xSemaphoreTake( xSem_Supervision, portMAX_DELAY );

	#if MICROROS
		MsgToPub.dir = dir;
		MsgToPub.mode = mode;
		MsgToPub.speed = speed;
		if (!uxQueueMessagesWaiting(qhMR_pub)) //If no data in queue
			xQueueSend(qhMR_pub, ( void * ) &MsgToPub, portMAX_DELAY); //Send data
	#endif //MICROROS

	#if LCD
		if (!uxQueueMessagesWaiting(qhLCD)) //If no data in queue
		{
			pxMessage.data=mode;
			pxMessage.command=dir;
			xQueueSend( qhLCD, ( void * ) &pxMessage, portMAX_DELAY); //Send data
		}
	#endif //LCD

		vTaskDelay(SAMPLING_PERIOD_ms);
	}
#endif //SYNCHRO_EX
}

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
  printf("%cc%c[2J%c[0;0HTitouan//Jeremy//Louanne//Donald\r\n", 0x1b, 0x1b, 0x1b);

  motorCommand_Init();
  quadEncoder_Init();
  captDistIR_Init();

  HAL_Delay(500);

#if VL53
  initVL53L0X();
  HAL_Delay(500);
#endif //VL53

  // Test Ecran LCD
#if LCD
  groveLCD_begin(16,2,0); // !! cette fonction prend du temps
  HAL_Delay(100);
  groveLCD_setCursor(0,0);
  groveLCD_setColor(1);
  groveLCD_term_printf("Titouan//Jeremy//Louanne");
  HAL_Delay(1000);
#endif //LCD

  osKernelInitialize();
  //defaultTaskHandle = osThreadNew(microros_task, NULL, &defaultTask_attributes);

  //Create the diffrent task depending of the config
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
#elif SYNCHRO_EX == EXTESTCORRECTOR
	xTaskCreate(task_Supervision, ( const portCHAR * ) "task Supervision", 128 /* stack size */, NULL, 27, NULL);
	xTaskCreate(task_Motor_Left, ( const portCHAR * ) "task Motor Left", 128 /* stack size */, NULL, 25, NULL);
	xTaskCreate(task_Motor_Right, ( const portCHAR * ) "task Motor Right", 128 /* stack size */, NULL, 26, NULL);
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
	#endif //VL53

	#if LCD
	xTaskCreate(task_Grove_LCD, ( const portCHAR * ) "task LCD", 128 /* stack size */, NULL, 23, NULL);
	#endif //LCD
#endif //SYNCHRO_EX

	//Create the semaphore
    vSemaphoreCreateBinary(xSem_Supervision);

    //Init all the queue
    q_mot_L = xQueueCreate(1, sizeof(int16_t));
    q_mot_R = xQueueCreate(1, sizeof(int16_t));
    qhVl53 = xQueueCreate(1, sizeof(int));

    qhMR_sub = xQueueCreate(1, sizeof(MicroRosSubMsg));
    qhMR_pub = xQueueCreate(1, sizeof(MicroRosPubMsg));
    qhLCD = xQueueCreate(1, sizeof(AMessage));

    osKernelStart();
    while(1){}
}

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
	int16_t  consigne = TEST_CORRECTOR_DUTY;
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
/*
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
/*
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
