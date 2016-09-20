#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort/QtSerialPort>
#include <QLabel>
#include <QTimer>
#include <QThread>
#include <QFileDialog>
#include <QScrollBar>

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>
#include <QAbstractButton>
#include <QButtonGroup>
#include <QtCharts>

typedef struct
{
    int8_t  rotational_direction;
    uint8_t tim_ch;
} motor;

typedef int8_t matrix[3][3];

typedef struct {
    uint8_t magic;
    uint8_t pad1;
    uint8_t pad2;
    uint8_t pad3;
    float pidvars[9];
    float l_pidvars[9];
    float rate[3];
    motor motor_1;
    motor motor_2;
    motor motor_3;
    motor motor_4;
    matrix sensor_orient;
    uint8_t pad4;
    uint8_t pad5;
    uint8_t pad6;
    float aspect_ratio;
    uint8_t rc_thrust;
    uint8_t rc_roll;
    uint8_t rc_nick;
    uint8_t rc_gier;
    uint8_t rc_arm;
    uint8_t rc_mode;
    uint8_t rc_beep;
    uint8_t rc_prog;
    uint8_t rc_var;
    uint8_t rc_write;
    uint8_t rc_aux1;
    uint8_t rc_aux2;
    uint8_t receiver;
} settings;

enum { RKp, RKi, RKd, NKp, NKi, NKd, GKp, GKi, GKd }; // pidvars index
enum { th, ro, ni, gi }; // motor index
enum { roll, nick, gier }; // rate axis index
enum { SBUS, SRXL };
enum { cw_radioButton = 201, ccw_radioButton = 202};
enum { CW = 1, CCW = -1 };
enum { rc_thrust, rc_roll, rc_nick, rc_gier, rc_arm, rc_mode, rc_beep, rc_prog, rc_var, rc_write, rc_aux1, rc_aux2 };

enum { // for sensor orientation rotation
    sensor_rot_x_plus_pushButton = 101,
    sensor_rot_x_minus_pushButton = 102,
    sensor_rot_y_plus_pushButton = 103,
    sensor_rot_y_minus_pushButton = 104,
    sensor_rot_z_plus_pushButton = 105,
    sensor_rot_z_minus_pushButton = 106
};

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();


private slots:
    void on_connect_pushButton_clicked();
    void on_disconnect_pushButton_clicked();
    void on_start_bootloader_pushButton_clicked();
    void on_pull_settings_pushButton_clicked();
    void on_push_settings_pushButton_clicked();
    void on_reboot_pushButton_clicked();
    void timer_elapsed();
    void serialPortError(QSerialPort::SerialPortError error);
    void browseFiles();
    void dfuFlashBinary();
    void dfuListDevices();
    void dfuCommandStatus();
    void dfuCommandComplete( int exitCode );
    void serialReadyRead();
    void set_sensor_orientation(int id);
    void set_rotational_direction(int id);

private:
    Ui::MainWindow *ui;
    QSerialPort *serial;
    QGraphicsScene *config_scene;
    QGraphicsScene *channels_scene;
    QGraphicsRectItem *rectangle;
    QGraphicsEllipseItem *ellipse;
    QGraphicsTextItem *text;
    QGraphicsLineItem *line;
    QGraphicsPolygonItem *arrow;
    QTimer *timer;
    QLabel *StatusLabel;
    QProcess dfuUtilProcess;
    QString binaryPath;
    QByteArray settings_data;
    QList<int> rc_channels;
    void refreshSerialDevices();
    void showStatusInfo(QString info);
    void display_config_scene(int rotation);
    void display_channels_scene();
    void displayVector(int direction);
    bool checkDFU( QFile *dfuUtil );
    bool serial_to_be_closed;
    bool settings_to_be_read;
    bool settings_to_be_write;
    bool found_our_port;
    bool pulled;
    qint64 bytes_written;
    motor motor_1;
    motor motor_2;
    motor motor_3;
    motor motor_4;
    int8_t rotational_direction;
    int delay_counter;
};

#endif // MAINWINDOW_H
