#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "./ui_mainwindow.h"

#include <QMainWindow>
#include <QPushButton>
#include <QGridLayout>
#include <QLCDNumber>
#include <QTimer>
#include <QGraphicsScene>
#include <QBrush>
#include <QColor>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>




class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void img_callback(const sensor_msgs::msg::Image::SharedPtr img);

public slots:

    void onTimer_Tick();

private slots:
    void on_pushButton_Up_clicked();

    void on_pushButton_Down_clicked();

    void on_pushButton_Left_clicked();

    void on_pushButton_Right_clicked();

    void on_radioButton_Auto_toggled(bool checked);

    void on_radioButton_Man_toggled(bool checked);

    void on_radioButton_Track_toggled(bool checked);

    void on_horizontalSlider_Speed_valueChanged(int value);

    void rgb_onSliderValueChanged(int value);

    void on_ON_OFF_stateChanged(int arg1);

private:
    void publishNewsUp();

    void publishNewsDown();

    void publishNewsLeft();

    void publishNewsRight();

    void publishModeAuto();

    void publishModeMan();

    void publishModeTrack();

    void publishSpeed(uint8_t value);
    void publishStart(bool value);

    Ui::Form *ui;
    QGraphicsScene* graphicsScene;
    QTimer *timer_chrono;

    QPushButton *m_bout_publish;
    QGridLayout *m_layout;


};

#endif // MAINWINDOW_H
