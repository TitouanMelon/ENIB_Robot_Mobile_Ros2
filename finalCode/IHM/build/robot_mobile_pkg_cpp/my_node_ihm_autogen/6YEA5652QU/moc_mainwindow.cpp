/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/robot_mobile_pkg_cpp/include/mainwindow.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[16];
    char stringdata0[322];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 12), // "onTimer_Tick"
QT_MOC_LITERAL(2, 24, 0), // ""
QT_MOC_LITERAL(3, 25, 24), // "on_pushButton_Up_clicked"
QT_MOC_LITERAL(4, 50, 26), // "on_pushButton_Down_clicked"
QT_MOC_LITERAL(5, 77, 26), // "on_pushButton_Left_clicked"
QT_MOC_LITERAL(6, 104, 27), // "on_pushButton_Right_clicked"
QT_MOC_LITERAL(7, 132, 27), // "on_radioButton_Auto_toggled"
QT_MOC_LITERAL(8, 160, 7), // "checked"
QT_MOC_LITERAL(9, 168, 26), // "on_radioButton_Man_toggled"
QT_MOC_LITERAL(10, 195, 28), // "on_radioButton_Track_toggled"
QT_MOC_LITERAL(11, 224, 38), // "on_horizontalSlider_Speed_val..."
QT_MOC_LITERAL(12, 263, 5), // "value"
QT_MOC_LITERAL(13, 269, 24), // "rgb_onSliderValueChanged"
QT_MOC_LITERAL(14, 294, 22), // "on_ON_OFF_stateChanged"
QT_MOC_LITERAL(15, 317, 4) // "arg1"

    },
    "MainWindow\0onTimer_Tick\0\0"
    "on_pushButton_Up_clicked\0"
    "on_pushButton_Down_clicked\0"
    "on_pushButton_Left_clicked\0"
    "on_pushButton_Right_clicked\0"
    "on_radioButton_Auto_toggled\0checked\0"
    "on_radioButton_Man_toggled\0"
    "on_radioButton_Track_toggled\0"
    "on_horizontalSlider_Speed_valueChanged\0"
    "value\0rgb_onSliderValueChanged\0"
    "on_ON_OFF_stateChanged\0arg1"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   69,    2, 0x0a /* Public */,
       3,    0,   70,    2, 0x08 /* Private */,
       4,    0,   71,    2, 0x08 /* Private */,
       5,    0,   72,    2, 0x08 /* Private */,
       6,    0,   73,    2, 0x08 /* Private */,
       7,    1,   74,    2, 0x08 /* Private */,
       9,    1,   77,    2, 0x08 /* Private */,
      10,    1,   80,    2, 0x08 /* Private */,
      11,    1,   83,    2, 0x08 /* Private */,
      13,    1,   86,    2, 0x08 /* Private */,
      14,    1,   89,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    8,
    QMetaType::Void, QMetaType::Bool,    8,
    QMetaType::Void, QMetaType::Bool,    8,
    QMetaType::Void, QMetaType::Int,   12,
    QMetaType::Void, QMetaType::Int,   12,
    QMetaType::Void, QMetaType::Int,   15,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainWindow *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->onTimer_Tick(); break;
        case 1: _t->on_pushButton_Up_clicked(); break;
        case 2: _t->on_pushButton_Down_clicked(); break;
        case 3: _t->on_pushButton_Left_clicked(); break;
        case 4: _t->on_pushButton_Right_clicked(); break;
        case 5: _t->on_radioButton_Auto_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->on_radioButton_Man_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->on_radioButton_Track_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->on_horizontalSlider_Speed_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->rgb_onSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->on_ON_OFF_stateChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_MainWindow.data,
    qt_meta_data_MainWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
