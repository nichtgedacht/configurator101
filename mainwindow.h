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

typedef struct
{
    uint8_t number;
    uint8_t rev;
} rc_channel;

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
    rc_channel rc_func[13];
    uint8_t pad7;
    rc_channel rc_ch[13];
    uint8_t pad8;
    uint8_t receiver;
} settings;


enum { RKp, RKi, RKd, NKp, NKi, NKd, GKp, GKi, GKd }; // pidvars index
enum { th, ro, ni, gi }; // motor index
enum { roll, nick, gier }; // rate axis index
enum { SBUS, SRXL };
enum { cw_radioButton = 201, ccw_radioButton = 202};
enum { CW = 1, CCW = -1 };
enum { r_thrust = 1,
       r_roll = 2,
       r_nick = 3,
       r_gier = 4,
       r_arm = 5,
       r_mode = 6,
       r_beep = 7,
       r_prog = 8,
       r_var = 9,
       r_aux1 = 10,
       r_aux2 = 11,
       r_aux3 = 12
     };

enum { // for sensor orientation rotation
    sensor_rot_x_plus_pushButton = 101,
    sensor_rot_x_minus_pushButton = 102,
    sensor_rot_y_plus_pushButton = 103,
    sensor_rot_y_minus_pushButton = 104,
    sensor_rot_z_plus_pushButton = 105,
    sensor_rot_z_minus_pushButton = 106
};

enum {
    thrust_rev = 301,
    roll_rev = 302,
    nick_rev = 303,
    gier_rev = 304,
    arm_rev = 305,
    mode_rev = 306,
    beep_rev = 307,
    prog_rev = 308,
    var_rev = 309,
    aux1_rev = 310,
    aux2_rev = 311,
    aux3_rev = 312
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
    void on_default_settings_pushButton_clicked();

    void on_rc_thrust_spinBox_valueChanged(int value);
    void on_rc_roll_spinBox_valueChanged(int value);
    void on_rc_nick_spinBox_valueChanged(int value);
    void on_rc_gier_spinBox_valueChanged(int value);
    void on_rc_arm_spinBox_valueChanged(int value);
    void on_rc_mode_spinBox_valueChanged(int value);
    void on_rc_beep_spinBox_valueChanged(int value);
    void on_rc_prog_spinBox_valueChanged(int value);
    void on_rc_var_spinBox_valueChanged(int value);
    void on_rc_aux1_spinBox_valueChanged(int value);
    void on_rc_aux2_spinBox_valueChanged(int value);
    void on_rc_aux3_spinBox_valueChanged(int value);
    void set_rev(int index);
    void on_tab_currentChanged(int index);
    void on_reboot_pushButton_clicked();
    void timer_elapsed();
    void serialPortError(QSerialPort::SerialPortError error);
    void browseFiles();
    void browse_saveFile();
    void dfuFlashBinary();
    void dfuSaveBinary();
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
    bool channels_to_be_read;
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
    //uint8_t rc_rev[12];
    rc_channel rc_func[13] = { {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0}, {8, 0}, {9, 0}, {10, 0}, {11, 0}, {12, 0} };
    rc_channel rc_ch[13] = { {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0}, {8, 0}, {9, 0}, {10, 0}, {11, 0}, {12, 0} };
};

#endif // MAINWINDOW_H
