#include "./../include/mainwindow.hpp"
//#include "./ui_mainwindow.h"

// ros2 topic pub --once /cubemx_publisher std_msgs/msg/String "data: Hello !"


std::shared_ptr<rclcpp::Node> node = nullptr;
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_dir;
rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_rgb_low;
rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_rgb_high;
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_mode;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_start;
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_speed;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_img;
//=========================================================================
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Form)
{

    //QWidget *zoneCentrale = new QWidget;
    //setCentralWidget(zoneCentrale);
//---------------------------------------------------------------------
    //m_bout_publish = new QPushButton("publish");
//---------------------------------------------------------------------
    //m_layout = new QGridLayout();
    //m_layout->addWidget(m_bout_publish,0,0);

    //zoneCentrale->setLayout(m_layout);

    ui->setupUi(this);
    //ui->horizontalLayout_11->setAlignment(Qt::AlignCenter);


    node = std::make_shared<rclcpp::Node>("my_node");
    publisher_start = node->create_publisher<std_msgs::msg::Bool>("start", 10);
    (*ui).horizontalSlider_red_low->setMaximum(255);
    (*ui).horizontalSlider_red_high->setMaximum(255);
    (*ui).horizontalSlider_green_low->setMaximum(255);
    (*ui).horizontalSlider_green_high->setMaximum(255);
    (*ui).horizontalSlider_blue_low->setMaximum(255);
    (*ui).horizontalSlider_blue_high->setMaximum(255);
    (*ui).ON_OFF->setChecked(false);
    connect ((*ui).ON_OFF, SIGNAL(stateChanged(int)), this, SLOT(on_ON_OFF_stateChanged(int)));
    on_ON_OFF_stateChanged(false);
//---------------------------------------------------------------------
    timer_chrono = new QTimer();
//---------------------------------------------------------------------
    connect( timer_chrono, SIGNAL(timeout()), this, SLOT(onTimer_Tick()));
    connect( m_bout_publish, SIGNAL(clicked()), this, SLOT(onButton_Publish()));
    connect ((*ui).radioButton_Man, SIGNAL(stateChanged(int)), this, SLOT(on_radioButton_Man_toggled(int)));
    connect((*ui).horizontalSlider_red_low, SIGNAL(valueChanged(int)), this, SLOT(rgb_onSliderValueChanged(int)));
    connect((*ui).horizontalSlider_red_high, SIGNAL(valueChanged(int)), this, SLOT(rgb_onSliderValueChanged(int)));
    connect((*ui).horizontalSlider_green_low, SIGNAL(valueChanged(int)), this, SLOT(rgb_onSliderValueChanged(int)));
    connect((*ui).horizontalSlider_green_high, SIGNAL(valueChanged(int)), this, SLOT(rgb_onSliderValueChanged(int)));
    connect((*ui).horizontalSlider_blue_low, SIGNAL(valueChanged(int)), this, SLOT(rgb_onSliderValueChanged(int)));
    connect((*ui).horizontalSlider_blue_high, SIGNAL(valueChanged(int)), this, SLOT(rgb_onSliderValueChanged(int)));


    //node = std::make_shared<rclcpp::Node>("my_node");
    publisher_dir = node->create_publisher<std_msgs::msg::Int32>("direction", 10);
    publisher_mode = node->create_publisher<std_msgs::msg::Int32>("mode", 10);
    publisher_speed = node->create_publisher<std_msgs::msg::Int32>("speed", 10);
    publisher_rgb_low = node->create_publisher<std_msgs::msg::UInt8MultiArray>("/camera/hsv_low", 10);
    publisher_rgb_high = node->create_publisher<std_msgs::msg::UInt8MultiArray>("/camera/hsv_high", 10);
    //publisher_start = node->create_publisher<std_msgs::msg::Bool>("start", 10);

    subscriber_img = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/IMAGE_frame", 1000,std::bind(&MainWindow::img_callback, this, std::placeholders::_1));//"/topic/IMAGE_frame"
    timer_chrono -> start(5); // 5 ms
    /*
    graphicsScene = new QGraphicsScene(this);
    QGraphicsView *graphicsView = (*ui).graphicsView;
    graphicsScene->setSceneRect(graphicsView->rect());
    graphicsView ->setScene(graphicsScene);
    //*/
    //*
    graphicsScene = new QGraphicsScene(this);
    QGraphicsView *graphicsView = (*ui).graphicsView;
    //QRectF graphicsSceneRect(0, 0, 1080, 400);
    QBrush background_scene(QColor(0, 0, 0));
    //graphicsScene->setSceneRect(graphicsView->rect());
    graphicsScene->setBackgroundBrush(background_scene);
    graphicsView ->setScene(graphicsScene);
    //*/
    RCLCPP_INFO(node->get_logger(), "Robot Gui Started");
}
//=========================================================================
void MainWindow::onTimer_Tick()
{
    rclcpp::spin_some(node);
}
//=================/camera/IMAGE_frame ========================================================
void MainWindow::on_pushButton_Up_clicked(){
    publishNewsUp();
}
//=========================================================================
void MainWindow::publishNewsUp(){
    auto message = std_msgs::msg::Int32();
    message.data = 0;
    RCLCPP_INFO(node->get_logger(), "Publishing: '%d'", message.data);
    publisher_dir->publish(message);

}
//=========================================================================
void MainWindow::on_pushButton_Down_clicked(){
    publishNewsDown();
}
//=========================================================================
void MainWindow::publishNewsDown(){
    auto message = std_msgs::msg::Int32();
    message.data = 2;
    RCLCPP_INFO(node->get_logger(), "Publishing: '%d'", message.data);
    publisher_dir->publish(message);

}
//=========================================================================
void MainWindow::on_pushButton_Left_clicked(){
    publishNewsLeft();
}
//=========================================================================
void MainWindow::publishNewsLeft(){
    auto message = std_msgs::msg::Int32();
    message.data = 3;
    RCLCPP_INFO(node->get_logger(), "Publishing: '%d'", message.data);
    publisher_dir->publish(message);

}
//=========================================================================
void MainWindow::on_pushButton_Right_clicked(){
    publishNewsRight();
}
//=========================================================================
void MainWindow::publishNewsRight(){
    auto message = std_msgs::msg::Int32();
    message.data = 1;
    RCLCPP_INFO(node->get_logger(), "Publishing: '%d'", message.data);
    publisher_dir->publish(message);

}
//=========================================================================
MainWindow::~MainWindow()
{
    delete ui;
}
//=========================================================================

void MainWindow::on_radioButton_Auto_toggled(bool checked)
{
    if(checked){
        (*ui).pushButton_Up->setEnabled(!checked);
        (*ui).pushButton_Down->setEnabled(!checked);
        (*ui).pushButton_Left->setEnabled(!checked);
        (*ui).pushButton_Right->setEnabled(!checked);
        (*ui).horizontalSlider_Speed->setEnabled(!checked);
        (*ui).verticalLayout_rgb_lcd->setEnabled(!checked);
        (*ui).horizontalSlider_red_low->setEnabled(!checked);
        (*ui).horizontalSlider_red_high->setEnabled(!checked);
        (*ui).horizontalSlider_green_low->setEnabled(!checked);
        (*ui).horizontalSlider_green_high->setEnabled(!checked);
        (*ui).horizontalSlider_blue_low->setEnabled(!checked);
        (*ui).horizontalSlider_blue_high->setEnabled(!checked);
        publishModeAuto();
    }

}
void MainWindow::publishModeAuto(){
    auto message = std_msgs::msg::Int32();
    message.data = 0;
    RCLCPP_INFO(node->get_logger(), "Publishing: '%d'", message.data);
    publisher_mode->publish(message);

}

//=========================================================================

void MainWindow::on_radioButton_Man_toggled(bool checked)
{
    if(checked){
        (*ui).pushButton_Up->setEnabled(checked);
        (*ui).pushButton_Down->setEnabled(checked);
        (*ui).pushButton_Left->setEnabled(checked);
        (*ui).pushButton_Right->setEnabled(checked);
        (*ui).horizontalSlider_Speed->setEnabled(checked);
        (*ui).lcdNumber_Speed->setEnabled(checked);
        (*ui).horizontalSlider_red_low->setEnabled(!checked);
        (*ui).horizontalSlider_red_high->setEnabled(!checked);
        (*ui).horizontalSlider_green_low->setEnabled(!checked);
        (*ui).horizontalSlider_green_high->setEnabled(!checked);
        (*ui).horizontalSlider_blue_low->setEnabled(!checked);
        (*ui).horizontalSlider_blue_high->setEnabled(!checked);
        publishModeMan();
    }
}
void MainWindow::publishModeMan(){
    auto message = std_msgs::msg::Int32();
    auto command_stop = std_msgs::msg::Int32();
    command_stop.data = 4;
    message.data = 1;
    RCLCPP_INFO(node->get_logger(), "Publishing manual mode: '%d' and stop command '%d'", message.data, command_stop.data);
    publisher_mode->publish(message);
    publisher_dir->publish(command_stop);

}
//=========================================================================

void MainWindow::on_radioButton_Track_toggled(bool checked)
{
    if(checked){
        (*ui).pushButton_Up->setEnabled(!checked);
        (*ui).pushButton_Down->setEnabled(!checked);
        (*ui).pushButton_Left->setEnabled(!checked);
        (*ui).pushButton_Right->setEnabled(!checked);
        (*ui).horizontalSlider_Speed->setEnabled(!checked);
        (*ui).horizontalSlider_red_low->setEnabled(checked);
        (*ui).horizontalSlider_red_high->setEnabled(checked);    
        (*ui).horizontalSlider_green_low->setEnabled(checked);
        (*ui).horizontalSlider_green_high->setEnabled(checked);   
        (*ui).horizontalSlider_blue_low->setEnabled(checked);
        (*ui).horizontalSlider_blue_high->setEnabled(checked);

        (*ui).horizontalSlider_red_low->setValue(99);
        (*ui).horizontalSlider_green_low->setValue(91);
        (*ui).horizontalSlider_blue_low->setValue(29);
        (*ui).horizontalSlider_red_high->setValue(120);
        (*ui).horizontalSlider_green_high->setValue(255);
        (*ui).horizontalSlider_blue_high->setValue(255);
        publishModeTrack();
    }

}
void MainWindow::publishModeTrack(){
    auto message = std_msgs::msg::Int32();
    message.data = 2;
    RCLCPP_INFO(node->get_logger(), "Publishing: '%u'", message.data);
    publisher_mode->publish(message);

}
//=========================================================================

void MainWindow::on_horizontalSlider_Speed_valueChanged(int value)
{
    publishSpeed(value);
    (*ui).lcdNumber_Speed->display(value);
}
void MainWindow::publishSpeed(uint8_t value){
    auto message = std_msgs::msg::Int32();
    message.data = value;
    RCLCPP_INFO(node->get_logger(), "Publishing: '%u'", message.data);
    publisher_speed->publish(message);

}
//=========================================================================
void MainWindow::rgb_onSliderValueChanged(int value){
    auto rgb_data_low = std_msgs::msg::UInt8MultiArray();
    auto rgb_data_high = std_msgs::msg::UInt8MultiArray();


    static uint8_t rgb_low[3] = {0};
    static uint8_t rgb_high[3] = {0};


    QObject *senderObj = sender();
    if(senderObj){
        QString senderName = senderObj->objectName();
        //(*ui).lcdNumber_rgb->setText(QString("%1: %2").arg(senderName.arg(value)));
        if(senderName == QString("horizontalSlider_red_low")){
            QSlider *senderSlider = qobject_cast<QSlider *>(senderObj);
            if(senderSlider){
                rgb_low[0] = value;
                (*ui).lcdNumber_rgb->display(value);
            }
        }
        else if(senderName == QString("horizontalSlider_green_low")){
            QSlider *senderSlider = qobject_cast<QSlider *>(senderObj);
            if(senderSlider){
                rgb_low[1] = value;
                (*ui).lcdNumber_rgb->display(value);
            }
        }
        else if(senderName == QString("horizontalSlider_blue_low")){
            QSlider *senderSlider = qobject_cast<QSlider *>(senderObj);
            if(senderSlider){
                rgb_low[2] = value;
                (*ui).lcdNumber_rgb->display(value);
            }
        }
        else if(senderName == QString("horizontalSlider_red_high")){
            QSlider *senderSlider = qobject_cast<QSlider *>(senderObj);
            if(senderSlider){
                rgb_high[0] = value;
                (*ui).lcdNumber_rgb->display(value);
            }
        }
        else if(senderName == QString("horizontalSlider_green_high")){
            QSlider *senderSlider = qobject_cast<QSlider *>(senderObj);
            if(senderSlider){
                rgb_high[1] = value;
                (*ui).lcdNumber_rgb->display(value);
            }
        }
        else if(senderName == QString("horizontalSlider_blue_high")){
            QSlider *senderSlider = qobject_cast<QSlider *>(senderObj);
            if(senderSlider){
                rgb_high[2] = value;
                (*ui).lcdNumber_rgb->display(value);
            }
        }
        /*
        for(int i = 0; i < 3; i++)
            rgb_data_low.data[i] = rgb_low[i];

        for(int j = 0; j < 3; j++)
            rgb_data_high.data[j] = rgb_high[j];
        //*/
        rgb_data_low.data = {rgb_low[0], rgb_low[1], rgb_low[2]};
        rgb_data_high.data = {rgb_high[0], rgb_high[1], rgb_high[2]};


        RCLCPP_INFO(node->get_logger(), "Publishing rgb_low_values: %s, %s, %s", std::to_string(rgb_data_low.data[0]).c_str(), std::to_string(rgb_data_low.data[1]).c_str(), std::to_string(rgb_data_low.data[2]).c_str());
        RCLCPP_INFO(node->get_logger(), "Publishing rgb_high_values: %s, %s, %s", std::to_string(rgb_data_high.data[0]).c_str(), std::to_string(rgb_data_high.data[1]).c_str(), std::to_string(rgb_data_high.data[2]).c_str());

        publisher_rgb_low -> publish(rgb_data_low);
        publisher_rgb_high ->publish(rgb_data_high);

    }
}
//=========================================================================
void MainWindow::img_callback(const sensor_msgs::msg::Image::SharedPtr img)
{
    //Converting the ROS image to a Qimage (encoding: RGB8)
    QImage image(img->data.data(), img->width, img->height, QImage::Format_RGB888);

    //Display Image in QGraphicsScene
    graphicsScene->clear();
    graphicsScene->addPixmap(QPixmap::fromImage(image));

}

void MainWindow::on_ON_OFF_stateChanged(int arg1)
{
    if(arg1 == Qt::Checked){
        (*ui).pushButton_Up->setEnabled(true);
        (*ui).pushButton_Down->setEnabled(true);
        (*ui).pushButton_Left->setEnabled(true);
        (*ui).pushButton_Right->setEnabled(true);
        (*ui).radioButton_Auto->setEnabled(true);
        (*ui).radioButton_Man->setEnabled(true);
        (*ui).radioButton_Track->setEnabled(true);
        (*ui).graphicsView->setEnabled(true);
        (*ui).horizontalSlider_Speed->setEnabled(true);
        (*ui).horizontalSlider_red_low->setEnabled(true);
        (*ui).horizontalSlider_red_high->setEnabled(true);
        (*ui).horizontalSlider_green_low->setEnabled(true);
        (*ui).horizontalSlider_green_high->setEnabled(true);
        (*ui).horizontalSlider_blue_low->setEnabled(true);
        (*ui).horizontalSlider_blue_high->setEnabled(true);
        (*ui).radioButton_Man->setChecked(true);
        publishStart(true);
        on_radioButton_Man_toggled(true);

        //connect( (*ui).radioButton_Man, SIGNAL(QRadioButton::toggled()), this, SLOT(on_radioButton_Man_toggled()));
    }
    else{
        (*ui).pushButton_Up->setEnabled(false);
        (*ui).pushButton_Down->setEnabled(false);
        (*ui).pushButton_Left->setEnabled(false);
        (*ui).pushButton_Right->setEnabled(false);
        (*ui).radioButton_Auto->setEnabled(false);
        (*ui).radioButton_Man->setEnabled(false);
        (*ui).radioButton_Track->setEnabled(false);
        (*ui).horizontalSlider_Speed->setEnabled(false);
        (*ui).horizontalSlider_red_low->setEnabled(false);
        (*ui).horizontalSlider_red_high->setEnabled(false);
        (*ui).horizontalSlider_green_low->setEnabled(false);
        (*ui).horizontalSlider_green_high->setEnabled(false);
        (*ui).horizontalSlider_blue_low->setEnabled(false);
        (*ui).horizontalSlider_blue_high->setEnabled(false);
        (*ui).graphicsView->setEnabled(false);
        publishStart(false);
    }
}
void MainWindow::publishStart(bool value){
    auto message = std_msgs::msg::Bool();
    message.data = value;
    RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", message.data ? "On" : "OFF");
    publisher_start->publish(message);

}

