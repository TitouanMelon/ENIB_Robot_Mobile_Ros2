/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Form
{
public:
    QWidget *widget;
    QWidget *widget1;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_10;
    QHBoxLayout *horizontalLayout_2;
    QVBoxLayout *verticalLayout_2;
    QRadioButton *radioButton_Auto;
    QRadioButton *radioButton_Man;
    QRadioButton *radioButton_Track;
    QVBoxLayout *verticalLayout_3;
    QLCDNumber *lcdNumber_Speed;
    QSlider *horizontalSlider_Speed;
    QHBoxLayout *horizontalLayout;
    QPushButton *pushButton_Left;
    QSpacerItem *horizontalSpacer;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton_Up;
    QSpacerItem *verticalSpacer;
    QPushButton *pushButton_Down;
    QPushButton *pushButton_Right;
    QVBoxLayout *verticalLayout_rgb_lcd;
    QVBoxLayout *verticalLayout_rgb_low;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label;
    QSlider *horizontalSlider_red_low;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_2;
    QSlider *horizontalSlider_green_low;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_3;
    QSlider *horizontalSlider_blue_low;
    QVBoxLayout *verticalLayout_rgb_high;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_4;
    QSlider *horizontalSlider_red_high;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label_5;
    QSlider *horizontalSlider_green_high;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_6;
    QSlider *horizontalSlider_blue_high;
    QLCDNumber *lcdNumber_rgb;
    QHBoxLayout *horizontalLayout_3;
    QSpacerItem *horizontalSpacer_2;
    QCheckBox *ON_OFF;
    QGraphicsView *graphicsView;

    void setupUi(QWidget *Form)
    {
        if (Form->objectName().isEmpty())
            Form->setObjectName(QString::fromUtf8("Form"));
        Form->resize(1016, 595);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(6);
        sizePolicy.setVerticalStretch(6);
        sizePolicy.setHeightForWidth(Form->sizePolicy().hasHeightForWidth());
        Form->setSizePolicy(sizePolicy);
        Form->setMinimumSize(QSize(0, 0));
        Form->setMaximumSize(QSize(16777215, 16777215));
        Form->setSizeIncrement(QSize(2, 2));
        Form->setLayoutDirection(Qt::LeftToRight);
        Form->setAutoFillBackground(false);
        widget = new QWidget(Form);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setGeometry(QRect(-9, -5, 1031, 611));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(4);
        sizePolicy1.setVerticalStretch(4);
        sizePolicy1.setHeightForWidth(widget->sizePolicy().hasHeightForWidth());
        widget->setSizePolicy(sizePolicy1);
        widget1 = new QWidget(widget);
        widget1->setObjectName(QString::fromUtf8("widget1"));
        widget1->setGeometry(QRect(10, 398, 1011, 201));
        verticalLayout_4 = new QVBoxLayout(widget1);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalLayout_4->setSizeConstraint(QLayout::SetNoConstraint);
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        horizontalLayout_10->setSizeConstraint(QLayout::SetNoConstraint);
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setSizeConstraint(QLayout::SetNoConstraint);
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setSizeConstraint(QLayout::SetNoConstraint);
        radioButton_Auto = new QRadioButton(widget1);
        radioButton_Auto->setObjectName(QString::fromUtf8("radioButton_Auto"));
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(radioButton_Auto->sizePolicy().hasHeightForWidth());
        radioButton_Auto->setSizePolicy(sizePolicy2);
        radioButton_Auto->setChecked(false);

        verticalLayout_2->addWidget(radioButton_Auto);

        radioButton_Man = new QRadioButton(widget1);
        radioButton_Man->setObjectName(QString::fromUtf8("radioButton_Man"));
        sizePolicy2.setHeightForWidth(radioButton_Man->sizePolicy().hasHeightForWidth());
        radioButton_Man->setSizePolicy(sizePolicy2);
        radioButton_Man->setChecked(true);

        verticalLayout_2->addWidget(radioButton_Man);

        radioButton_Track = new QRadioButton(widget1);
        radioButton_Track->setObjectName(QString::fromUtf8("radioButton_Track"));
        sizePolicy2.setHeightForWidth(radioButton_Track->sizePolicy().hasHeightForWidth());
        radioButton_Track->setSizePolicy(sizePolicy2);

        verticalLayout_2->addWidget(radioButton_Track);


        horizontalLayout_2->addLayout(verticalLayout_2);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setSizeConstraint(QLayout::SetNoConstraint);
        lcdNumber_Speed = new QLCDNumber(widget1);
        lcdNumber_Speed->setObjectName(QString::fromUtf8("lcdNumber_Speed"));
        sizePolicy2.setHeightForWidth(lcdNumber_Speed->sizePolicy().hasHeightForWidth());
        lcdNumber_Speed->setSizePolicy(sizePolicy2);
        lcdNumber_Speed->setSegmentStyle(QLCDNumber::Flat);

        verticalLayout_3->addWidget(lcdNumber_Speed);

        horizontalSlider_Speed = new QSlider(widget1);
        horizontalSlider_Speed->setObjectName(QString::fromUtf8("horizontalSlider_Speed"));
        sizePolicy2.setHeightForWidth(horizontalSlider_Speed->sizePolicy().hasHeightForWidth());
        horizontalSlider_Speed->setSizePolicy(sizePolicy2);
        horizontalSlider_Speed->setOrientation(Qt::Horizontal);

        verticalLayout_3->addWidget(horizontalSlider_Speed);


        horizontalLayout_2->addLayout(verticalLayout_3);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setSizeConstraint(QLayout::SetNoConstraint);
        pushButton_Left = new QPushButton(widget1);
        pushButton_Left->setObjectName(QString::fromUtf8("pushButton_Left"));
        QSizePolicy sizePolicy3(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(pushButton_Left->sizePolicy().hasHeightForWidth());
        pushButton_Left->setSizePolicy(sizePolicy3);

        horizontalLayout->addWidget(pushButton_Left);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setSizeConstraint(QLayout::SetNoConstraint);
        pushButton_Up = new QPushButton(widget1);
        pushButton_Up->setObjectName(QString::fromUtf8("pushButton_Up"));
        sizePolicy3.setHeightForWidth(pushButton_Up->sizePolicy().hasHeightForWidth());
        pushButton_Up->setSizePolicy(sizePolicy3);

        verticalLayout->addWidget(pushButton_Up);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        pushButton_Down = new QPushButton(widget1);
        pushButton_Down->setObjectName(QString::fromUtf8("pushButton_Down"));
        sizePolicy3.setHeightForWidth(pushButton_Down->sizePolicy().hasHeightForWidth());
        pushButton_Down->setSizePolicy(sizePolicy3);

        verticalLayout->addWidget(pushButton_Down);


        horizontalLayout->addLayout(verticalLayout);

        pushButton_Right = new QPushButton(widget1);
        pushButton_Right->setObjectName(QString::fromUtf8("pushButton_Right"));
        sizePolicy3.setHeightForWidth(pushButton_Right->sizePolicy().hasHeightForWidth());
        pushButton_Right->setSizePolicy(sizePolicy3);

        horizontalLayout->addWidget(pushButton_Right);


        horizontalLayout_2->addLayout(horizontalLayout);


        horizontalLayout_10->addLayout(horizontalLayout_2);

        verticalLayout_rgb_lcd = new QVBoxLayout();
        verticalLayout_rgb_lcd->setObjectName(QString::fromUtf8("verticalLayout_rgb_lcd"));
        verticalLayout_rgb_lcd->setSizeConstraint(QLayout::SetNoConstraint);
        verticalLayout_rgb_low = new QVBoxLayout();
        verticalLayout_rgb_low->setObjectName(QString::fromUtf8("verticalLayout_rgb_low"));
        verticalLayout_rgb_low->setSizeConstraint(QLayout::SetNoConstraint);
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalLayout_4->setSizeConstraint(QLayout::SetNoConstraint);
        label = new QLabel(widget1);
        label->setObjectName(QString::fromUtf8("label"));
        sizePolicy2.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy2);

        horizontalLayout_4->addWidget(label);

        horizontalSlider_red_low = new QSlider(widget1);
        horizontalSlider_red_low->setObjectName(QString::fromUtf8("horizontalSlider_red_low"));
        sizePolicy2.setHeightForWidth(horizontalSlider_red_low->sizePolicy().hasHeightForWidth());
        horizontalSlider_red_low->setSizePolicy(sizePolicy2);
        horizontalSlider_red_low->setMaximum(255);
        horizontalSlider_red_low->setOrientation(Qt::Horizontal);

        horizontalLayout_4->addWidget(horizontalSlider_red_low);


        verticalLayout_rgb_low->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        horizontalLayout_5->setSizeConstraint(QLayout::SetNoConstraint);
        label_2 = new QLabel(widget1);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        sizePolicy2.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy2);

        horizontalLayout_5->addWidget(label_2);

        horizontalSlider_green_low = new QSlider(widget1);
        horizontalSlider_green_low->setObjectName(QString::fromUtf8("horizontalSlider_green_low"));
        sizePolicy2.setHeightForWidth(horizontalSlider_green_low->sizePolicy().hasHeightForWidth());
        horizontalSlider_green_low->setSizePolicy(sizePolicy2);
        horizontalSlider_green_low->setMaximum(255);
        horizontalSlider_green_low->setOrientation(Qt::Horizontal);

        horizontalLayout_5->addWidget(horizontalSlider_green_low);


        verticalLayout_rgb_low->addLayout(horizontalLayout_5);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        horizontalLayout_6->setSizeConstraint(QLayout::SetNoConstraint);
        label_3 = new QLabel(widget1);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        sizePolicy2.setHeightForWidth(label_3->sizePolicy().hasHeightForWidth());
        label_3->setSizePolicy(sizePolicy2);

        horizontalLayout_6->addWidget(label_3);

        horizontalSlider_blue_low = new QSlider(widget1);
        horizontalSlider_blue_low->setObjectName(QString::fromUtf8("horizontalSlider_blue_low"));
        sizePolicy2.setHeightForWidth(horizontalSlider_blue_low->sizePolicy().hasHeightForWidth());
        horizontalSlider_blue_low->setSizePolicy(sizePolicy2);
        horizontalSlider_blue_low->setMaximum(255);
        horizontalSlider_blue_low->setOrientation(Qt::Horizontal);

        horizontalLayout_6->addWidget(horizontalSlider_blue_low);


        verticalLayout_rgb_low->addLayout(horizontalLayout_6);


        verticalLayout_rgb_lcd->addLayout(verticalLayout_rgb_low);

        verticalLayout_rgb_high = new QVBoxLayout();
        verticalLayout_rgb_high->setObjectName(QString::fromUtf8("verticalLayout_rgb_high"));
        verticalLayout_rgb_high->setSizeConstraint(QLayout::SetNoConstraint);
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        horizontalLayout_7->setSizeConstraint(QLayout::SetNoConstraint);
        label_4 = new QLabel(widget1);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        sizePolicy2.setHeightForWidth(label_4->sizePolicy().hasHeightForWidth());
        label_4->setSizePolicy(sizePolicy2);

        horizontalLayout_7->addWidget(label_4);

        horizontalSlider_red_high = new QSlider(widget1);
        horizontalSlider_red_high->setObjectName(QString::fromUtf8("horizontalSlider_red_high"));
        sizePolicy2.setHeightForWidth(horizontalSlider_red_high->sizePolicy().hasHeightForWidth());
        horizontalSlider_red_high->setSizePolicy(sizePolicy2);
        horizontalSlider_red_high->setMaximum(255);
        horizontalSlider_red_high->setOrientation(Qt::Horizontal);

        horizontalLayout_7->addWidget(horizontalSlider_red_high);


        verticalLayout_rgb_high->addLayout(horizontalLayout_7);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        horizontalLayout_8->setSizeConstraint(QLayout::SetNoConstraint);
        label_5 = new QLabel(widget1);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        sizePolicy2.setHeightForWidth(label_5->sizePolicy().hasHeightForWidth());
        label_5->setSizePolicy(sizePolicy2);

        horizontalLayout_8->addWidget(label_5);

        horizontalSlider_green_high = new QSlider(widget1);
        horizontalSlider_green_high->setObjectName(QString::fromUtf8("horizontalSlider_green_high"));
        sizePolicy2.setHeightForWidth(horizontalSlider_green_high->sizePolicy().hasHeightForWidth());
        horizontalSlider_green_high->setSizePolicy(sizePolicy2);
        horizontalSlider_green_high->setMaximum(255);
        horizontalSlider_green_high->setOrientation(Qt::Horizontal);

        horizontalLayout_8->addWidget(horizontalSlider_green_high);


        verticalLayout_rgb_high->addLayout(horizontalLayout_8);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        horizontalLayout_9->setSizeConstraint(QLayout::SetNoConstraint);
        label_6 = new QLabel(widget1);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        sizePolicy2.setHeightForWidth(label_6->sizePolicy().hasHeightForWidth());
        label_6->setSizePolicy(sizePolicy2);

        horizontalLayout_9->addWidget(label_6);

        horizontalSlider_blue_high = new QSlider(widget1);
        horizontalSlider_blue_high->setObjectName(QString::fromUtf8("horizontalSlider_blue_high"));
        sizePolicy2.setHeightForWidth(horizontalSlider_blue_high->sizePolicy().hasHeightForWidth());
        horizontalSlider_blue_high->setSizePolicy(sizePolicy2);
        horizontalSlider_blue_high->setMaximum(255);
        horizontalSlider_blue_high->setOrientation(Qt::Horizontal);

        horizontalLayout_9->addWidget(horizontalSlider_blue_high);


        verticalLayout_rgb_high->addLayout(horizontalLayout_9);


        verticalLayout_rgb_lcd->addLayout(verticalLayout_rgb_high);

        lcdNumber_rgb = new QLCDNumber(widget1);
        lcdNumber_rgb->setObjectName(QString::fromUtf8("lcdNumber_rgb"));
        sizePolicy2.setHeightForWidth(lcdNumber_rgb->sizePolicy().hasHeightForWidth());
        lcdNumber_rgb->setSizePolicy(sizePolicy2);
        lcdNumber_rgb->setSegmentStyle(QLCDNumber::Flat);

        verticalLayout_rgb_lcd->addWidget(lcdNumber_rgb);


        horizontalLayout_10->addLayout(verticalLayout_rgb_lcd);


        verticalLayout_4->addLayout(horizontalLayout_10);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalLayout_3->setSizeConstraint(QLayout::SetNoConstraint);
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_2);

        ON_OFF = new QCheckBox(widget1);
        ON_OFF->setObjectName(QString::fromUtf8("ON_OFF"));
        sizePolicy2.setHeightForWidth(ON_OFF->sizePolicy().hasHeightForWidth());
        ON_OFF->setSizePolicy(sizePolicy2);

        horizontalLayout_3->addWidget(ON_OFF);


        verticalLayout_4->addLayout(horizontalLayout_3);

        graphicsView = new QGraphicsView(widget);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));
        graphicsView->setGeometry(QRect(10, 11, 1011, 381));
        sizePolicy2.setHeightForWidth(graphicsView->sizePolicy().hasHeightForWidth());
        graphicsView->setSizePolicy(sizePolicy2);
        graphicsView->setMaximumSize(QSize(16777215, 1200));

        retranslateUi(Form);

        QMetaObject::connectSlotsByName(Form);
    } // setupUi

    void retranslateUi(QWidget *Form)
    {
        Form->setWindowTitle(QCoreApplication::translate("Form", "ROS2 ROBOT GUI", nullptr));
        radioButton_Auto->setText(QCoreApplication::translate("Form", "AUTO", nullptr));
        radioButton_Man->setText(QCoreApplication::translate("Form", "MAN", nullptr));
        radioButton_Track->setText(QCoreApplication::translate("Form", "TRACK", nullptr));
        pushButton_Left->setText(QCoreApplication::translate("Form", "LEFT", nullptr));
        pushButton_Up->setText(QCoreApplication::translate("Form", "UP", nullptr));
        pushButton_Down->setText(QCoreApplication::translate("Form", "DOWN", nullptr));
        pushButton_Right->setText(QCoreApplication::translate("Form", "RIGHT", nullptr));
        label->setText(QCoreApplication::translate("Form", "Hue_Low", nullptr));
        label_2->setText(QCoreApplication::translate("Form", "Saturation_Low", nullptr));
        label_3->setText(QCoreApplication::translate("Form", "Value_Low", nullptr));
        label_4->setText(QCoreApplication::translate("Form", "Hue_High", nullptr));
        label_5->setText(QCoreApplication::translate("Form", "Saturation_High", nullptr));
        label_6->setText(QCoreApplication::translate("Form", "Value_High", nullptr));
        ON_OFF->setText(QCoreApplication::translate("Form", "ON-OFF", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Form: public Ui_Form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
