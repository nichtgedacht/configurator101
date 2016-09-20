#include "mainwindow.h"
#include "ui_mainwindow.h"

namespace {

// these are rotation matrices
// simplified to 90 degree rotations
matrix rot_x_plus = {
                    { 1, 0,  0 },
                    { 0, 0, -1 },
                    { 0, 1,  0 }
                    };

matrix rot_x_minus = {
                    { 1,  0,  0 },
                    { 0,  0,  1 },
                    { 0, -1,  0 }
                    };

matrix rot_y_plus = {
                    {  0, 0, 1  },
                    {  0, 1, 0  },
                    { -1, 0, 0  }
                    };

matrix rot_y_minus = {
                     {  0, 0, -1  },
                     {  0, 1,  0  },
                     {  1, 0,  0  }
                     };

matrix rot_z_plus = {
                    {  0, -1, 0  },
                    {  1,  0, 0  },
                    {  0,  0, 1  }
                    };

matrix rot_z_minus = {
                     {  0,  1,  0 },
                     { -1,  0,  0 },
                     {  0,  0,  1 }
                     };

// normal orientation
// +X front side
// +Y left side (in flight direction)
// +Z top side
matrix sensor_orientation = { // Front   Left   Top
                            {     1,      0,     0  }, // x
                            {     0,      1,     0  }, // y
                            {     0,      0,     1  }  // z
                            };
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //globally used objects
    serial = new QSerialPort(this);
    config_scene = new QGraphicsScene(this);
    channels_scene = new QGraphicsScene(this);
    text = new QGraphicsTextItem;
    timer = new QTimer(this);
    timer->start(100);
    StatusLabel = new QLabel(this);

    // connect any slot which can't be connected by on_NAME_SIGNAL()
    connect(timer, SIGNAL(timeout()), this, SLOT(timer_elapsed()));
    connect(serial, SIGNAL(error(QSerialPort::SerialPortError)), this, SLOT(serialPortError(QSerialPort::SerialPortError)));
    connect( ui->fw_select_pushButton, SIGNAL( released() ), this, SLOT( browseFiles() ) );
    connect( ui->flash_pushButton, SIGNAL( released() ), this, SLOT( dfuFlashBinary() ) );
    connect( ui->show_dfu_pushButton, SIGNAL( released() ), this, SLOT( dfuListDevices() ) );
    connect( &dfuUtilProcess, SIGNAL( readyReadStandardOutput() ), this, SLOT( dfuCommandStatus() ) );
    connect( &dfuUtilProcess, SIGNAL( finished( int, QProcess::ExitStatus ) ), this, SLOT( dfuCommandComplete( int ) ) );
    connect( serial, SIGNAL(readyRead() ), this, SLOT(serialReadyRead() ) );
    connect( ui->sensor_set_buttonGroup, SIGNAL(buttonClicked(int)), this, SLOT( set_sensor_orientation(int) ) );
    connect( ui->rot_dir_buttonGroup, SIGNAL(buttonClicked(int)), this, SLOT(set_rotational_direction(int) ) );

    // Only use the included dfu-util
    binaryPath = QFileInfo( QCoreApplication::applicationFilePath() ).dir().absolutePath();
    dfuUtilProcess.setWorkingDirectory( binaryPath );

    // Merge the output channels? //No just stdout
    //dfuUtilProcess.setProcessChannelMode( QProcess::MergedChannels );
    dfuUtilProcess.setReadChannel(QProcess::StandardOutput);

    // set ui defaults
    ui->statusBar->addPermanentWidget(StatusLabel,1);
    ui->result_textEdit->setReadOnly( true );
    ui->start_bootloader_pushButton->setDisabled( true );
    ui->connect_pushButton->setDisabled( true );
    ui->reboot_pushButton->setDisabled( false );
    ui->pull_settings_pushButton->setDisabled( true );
    ui->push_settings_pushButton->setDisabled( true );
    ui->rx_select_comboBox->addItems(QStringList() << "SBUS" << "SRXL");

    // set IDs
    ui->sensor_set_buttonGroup->setId(ui->sensor_rot_x_plus_pushButton, 101);
    ui->sensor_set_buttonGroup->setId(ui->sensor_rot_x_minus_pushButton, 102);
    ui->sensor_set_buttonGroup->setId(ui->sensor_rot_y_plus_pushButton, 103);
    ui->sensor_set_buttonGroup->setId(ui->sensor_rot_y_minus_pushButton, 104);
    ui->sensor_set_buttonGroup->setId(ui->sensor_rot_z_plus_pushButton, 105);
    ui->sensor_set_buttonGroup->setId(ui->sensor_rot_z_minus_pushButton, 106);

    ui->rot_dir_buttonGroup->setId(ui->cw_radioButton, 201);
    ui->rot_dir_buttonGroup->setId(ui->ccw_radioButton, 202);

    // set some globally used variables
    settings_to_be_read = false;
    settings_to_be_write = false;
    pulled = false;
    bytes_written = 0;
    rotational_direction = CW;
    rc_channels << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0;

    // will be refreshed only if changed
    display_config_scene(CW);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::timer_elapsed()
{

    qint64 res;

    refreshSerialDevices();

    display_channels_scene();

    if ( serial_to_be_closed == true)
    {
        serial->close();
        StatusLabel->setText("Not Connected");

        serial_to_be_closed = false;
    }
    else
    {
        if (serial->isOpen())
        {
            if (  settings_to_be_write )
            {
                ui->push_settings_pushButton->setDisabled( true );

                if ( bytes_written < 1024 )
                {
                    res = serial->write( settings_data.constData() +  bytes_written, 1024 - bytes_written);
                    if (res > -1)
                    {
                        bytes_written += res;
                    }
                    else
                    {
                        settings_to_be_write = false;
                        bytes_written = 0;
                    }
                }
                else
                {
                    settings_to_be_write = false;
                    bytes_written = 0;
                }
            }
            else  if (pulled)
            {
                ui->push_settings_pushButton->setDisabled( false );
            }

            StatusLabel->setText("Connected");
            ui->start_bootloader_pushButton->setDisabled( false );

            ui->reboot_pushButton->setDisabled( false );

            // enable again after pulled data read
            if (settings_to_be_read == false)
            {
                ui->pull_settings_pushButton->setDisabled( false );
            }

            ui->connect_pushButton->setDisabled( true );
            ui->disconnect_pushButton->setDisabled( false );
        }
        else
        {
            StatusLabel->setText("Not Connected");
            ui->start_bootloader_pushButton->setDisabled( true );
            ui->pull_settings_pushButton->setDisabled( true );
            ui->push_settings_pushButton->setDisabled( true );
            ui->disconnect_pushButton->setDisabled( true );
            ui->reboot_pushButton->setDisabled( true );

            if (found_our_port)
            {
                ui->connect_pushButton->setDisabled( false );
            }
        }
    }

    // After "pull_settings" sent, the device stops sending live data
    // within 20 ms then waits > 300 ms before sending the requested
    // settings.
    // We are reading data continously if available but append arrived
    // data to the cleaned settings read buffer not before a time between
    // 100 - 200 ms has elapsed.
    // | | | |  1. send "pull_request"
    // | | | |     20 ms max device side reaction
    // | | | |----------------------------------------------------------
    // | | |    2. device has stopped sending live date
    // | | |    3. Our Reader swallows not settings data on wire
    // | | |    4. After 100-200 ready to begin append to settings buffer
    // | | |------------------------------------------------------------
    // | |      5. After > 300 ms device senda settings data
    // | ---------------------------------------------------------------
    // |      6. Append to settings buffer begins immediately
    // -----------------------------------------------------------------
    if ( settings_to_be_read )
    {
        delay_counter++;
    }
}

void MainWindow::refreshSerialDevices()
{
    ui->availports_comboBox->clear();
    found_our_port = false;

    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    foreach(const QSerialPortInfo &port, ports) {
        QString name = port.portName();
        int index = ui->availports_comboBox->count();
        // put STMicroelectronics device first in list
        if(port.manufacturer() == "STMicroelectronics" && port.productIdentifier() == 0x5740 ) {
            name.insert(0, "");
            index = 0;
            found_our_port = true;
        }
        ui->availports_comboBox->insertItem(index, name, port.systemLocation());
    }
    ui->availports_comboBox->setCurrentIndex(0);

    if ( found_our_port )
    {
        ui->flash_pushButton->setDisabled( true );
        ui->show_dfu_pushButton->setDisabled( true );
    }
    else
    {
        ui->connect_pushButton->setDisabled( true );
        ui->flash_pushButton->setDisabled( false );
        ui->show_dfu_pushButton->setDisabled( false);
    }
}

/*
void MainWindow::on_tabWidget_currentChanged(int index)
{

    switch (index)
    {
    case 0:
        serial->write("stop_live", 10);
        break;

    case 1:
        serial->write("stop_live", 10);
        break;

    case 2:
        serial->write("stop_live", 10);
        break;

    case 3:
        serial->write("send_live", 10);
        break;
    }
}
*/

void MainWindow::on_pull_settings_pushButton_clicked()
{
    serial->clear();
    settings_data.clear();
    serial->write("pull_settings", 14);

    ui->pull_settings_pushButton->setDisabled( true );

    settings_to_be_read = true;
}

void MainWindow::on_push_settings_pushButton_clicked()
{
    settings *ps;
    int i,j;

    serial->clear();
    serial->write("push_settings", 14);

    ps = (settings*) settings_data.data();

    ps->pidvars[RKp] = ui->roll_kp->value();
    ps->pidvars[RKi] = ui->roll_ki->value();
    ps->pidvars[RKd] = ui->roll_kd->value();

    ps->pidvars[NKp] = ui->nick_kp->value();
    ps->pidvars[NKi] = ui->nick_ki->value();
    ps->pidvars[NKd] = ui->nick_kd->value();

    ps->pidvars[GKp] = ui->gier_kp->value();
    ps->pidvars[GKi] = ui->gier_ki->value();
    ps->pidvars[GKd] = ui->gier_kd->value();

    ps->l_pidvars[RKp] = ui->l_roll_kp->value();
    ps->l_pidvars[RKi] = ui->l_roll_ki->value();
    ps->l_pidvars[RKd] = ui->l_roll_kd->value();

    ps->l_pidvars[NKp] = ui->l_nick_kp->value();
    ps->l_pidvars[NKi] = ui->l_nick_ki->value();
    ps->l_pidvars[NKd] = ui->l_nick_kd->value();

    ps->l_pidvars[GKp] = ui->l_gier_kp->value();
    ps->l_pidvars[GKi] = ui->l_gier_ki->value();
    ps->l_pidvars[GKd] = ui->l_gier_kd->value();

    ps->rate[roll] = ui->roll_rate->value();
    ps->rate[nick] = ui->nick_rate->value();
    ps->rate[gier] = ui->gier_rate->value();

    ps->motor_1.tim_ch = ui->motor1_ch_spinBox->value();
    ps->motor_2.tim_ch = ui->motor2_ch_spinBox->value();
    ps->motor_3.tim_ch = ui->motor3_ch_spinBox->value();
    ps->motor_4.tim_ch = ui->motor4_ch_spinBox->value();

    if (rotational_direction == CW)
    {
        ps->motor_1.rotational_direction = CCW;
        ps->motor_2.rotational_direction = CW;
        ps->motor_3.rotational_direction = CW;
        ps->motor_4.rotational_direction = CCW;
    }
    else
    {
        ps->motor_1.rotational_direction = CW;
        ps->motor_2.rotational_direction = CCW;
        ps->motor_3.rotational_direction = CCW;
        ps->motor_4.rotational_direction = CW;
    }

    ps->rc_thrust = ui->rc_thrust_spinBox->value();
    ps->rc_roll = ui->rc_roll_spinBox->value();
    ps->rc_nick = ui->rc_nick_spinBox->value();
    ps->rc_gier = ui->rc_gier_spinBox->value();
    ps->rc_arm = ui->rc_arm_spinBox->value();
    ps->rc_mode = ui->rc_mode_spinBox->value();
    ps->rc_beep = ui->rc_beep_spinBox->value();
    ps->rc_prog = ui->rc_prog_spinBox->value();
    ps->rc_var = ui->rc_var_spinBox->value();
    ps->rc_write = ui->rc_write_spinBox->value();
    ps->rc_aux1 = ui->rc_aux1_spinBox->value();
    ps->rc_aux2 = ui->rc_aux2_spinBox->value();

    for(i=0; i<3; ++i)
        for(j=0; j<3; ++j)
        {
            ps->sensor_orient[i][j] = sensor_orientation[i][j];
        }

    ps->aspect_ratio = ui->aspect_ratio_doubleSpinBox->value();

    ps->receiver = ui->rx_select_comboBox->currentIndex();

    settings_to_be_write = true;
}

void MainWindow::on_reboot_pushButton_clicked()
{
    settings_data.clear();
    serial->write("reboot", 7);
}

void MainWindow::on_start_bootloader_pushButton_clicked()
{
    ui->start_bootloader_pushButton->setDisabled( true );
    ui->connect_pushButton->setDisabled( true );
    serial->write("bootloader", 11);
}

void MainWindow::on_connect_pushButton_clicked()
{
    if(serial->isOpen()) {
        return;
    }

    serial->setPortName(ui->availports_comboBox->currentData().toString());

    // for proxy test
    //QString qstr;
    //qstr.append("/dev/ttyACM5");
    //serial->setPortName(qstr);
    // usage:
    // socat -x -v PTY,link=/dev/ttyACM5 /dev/ttyACM0,raw
    // prepare in other terminal: chmod 666 /dev/ttyACM5
    // and stty "1:0:18b2:0:3:1c:7f:15:4:5:1:0:11:13:1a:0:12:f:17:16:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0" -F /dev/ttyACM5
    // after connection traffic is shown in both directions in hex

    serial->open(QIODevice::ReadWrite);

    if(!serial->isOpen()) {
        return;
    }

    serial->setBaudRate(QSerialPort::Baud115200);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
}

void MainWindow::on_disconnect_pushButton_clicked()
{
    if (serial->isOpen()) {
        serial->close();
    }
}

void MainWindow::set_rotational_direction(int id)
{
    if (id == cw_radioButton)
    {
        rotational_direction = CW;
    }
    else
    {
        rotational_direction = CCW;
    }

    display_config_scene(rotational_direction);
}


void MainWindow::set_sensor_orientation(int id)
{

    int i, j, k;
    int8_t new_orient[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

    switch(id)
    {
    case sensor_rot_x_plus_pushButton:

        for(i=0; i<3; ++i)
            for(j=0; j<3; ++j)
                for(k=0; k<3; ++k)
                {
                    new_orient[i][j]+=sensor_orientation[i][k] * rot_x_plus[k][j];
                }
        break;

    case sensor_rot_x_minus_pushButton:

        for(i=0; i<3; ++i)
            for(j=0; j<3; ++j)
                for(k=0; k<3; ++k)
                {
                    new_orient[i][j]+=sensor_orientation[i][k] * rot_x_minus[k][j];
                }
        break;

    case sensor_rot_y_plus_pushButton:

        for(i=0; i<3; ++i)
            for(j=0; j<3; ++j)
                for(k=0; k<3; ++k)
                {
                    new_orient[i][j]+=sensor_orientation[i][k] * rot_y_plus[k][j];
                }
        break;

    case sensor_rot_y_minus_pushButton:

        for(i=0; i<3; ++i)
            for(j=0; j<3; ++j)
                for(k=0; k<3; ++k)
                {
                    new_orient[i][j]+=sensor_orientation[i][k] * rot_y_minus[k][j];
                }
        break;

    case sensor_rot_z_plus_pushButton:
        for(i=0; i<3; ++i)
            for(j=0; j<3; ++j)
                for(k=0; k<3; ++k)
                {
                    new_orient[i][j]+=sensor_orientation[i][k] * rot_z_plus[k][j];
                }
        break;

    case sensor_rot_z_minus_pushButton:
        for(i=0; i<3; ++i)
            for(j=0; j<3; ++j)
                for(k=0; k<3; ++k)
                {
                    new_orient[i][j]+=sensor_orientation[i][k] * rot_z_minus[k][j];
                }
        break;
    }

    for(i=0; i<3; ++i)
        for(j=0; j<3; ++j)
        {
            sensor_orientation[i][j] = new_orient[i][j];
        }

    display_config_scene(rotational_direction);
}

void MainWindow::serialReadyRead()
{
    settings *ps;
    int i,j;

    /*
    // Alternative Version
    // This slot is connected to QSerialPort::readyRead()
    void QSerialPortClass::readyReadSlot()
    {
        while (!port.atEnd()) {
            QByteArray data = port.read(100);
            ....
        }
    }
    */

    if ( settings_to_be_read && delay_counter > 1)
    {
        settings_data.append(serial->read(1024));

        if ( settings_data.size() >= 1024 )
        {
            ps = (settings*) settings_data.data();

            ui->roll_kp->setValue(ps->pidvars[RKp]);
            ui->roll_ki->setValue(ps->pidvars[RKi]);
            ui->roll_kd->setValue(ps->pidvars[RKd]);
            ui->nick_kp->setValue(ps->pidvars[NKp]);
            ui->nick_ki->setValue(ps->pidvars[NKi]);
            ui->nick_kd->setValue(ps->pidvars[NKd]);
            ui->gier_kp->setValue(ps->pidvars[GKp]);
            ui->gier_ki->setValue(ps->pidvars[GKi]);
            ui->gier_kd->setValue(ps->pidvars[GKd]);

            ui->l_roll_kp->setValue(ps->l_pidvars[RKp]);
            ui->l_roll_ki->setValue(ps->l_pidvars[RKi]);
            ui->l_roll_kd->setValue(ps->l_pidvars[RKd]);
            ui->l_nick_kp->setValue(ps->l_pidvars[NKp]);
            ui->l_nick_ki->setValue(ps->l_pidvars[NKi]);
            ui->l_nick_kd->setValue(ps->l_pidvars[NKd]);
            ui->l_gier_kp->setValue(ps->l_pidvars[GKp]);
            ui->l_gier_ki->setValue(ps->l_pidvars[GKi]);
            ui->l_gier_kd->setValue(ps->l_pidvars[GKd]);

            ui->roll_rate->setValue(ps->rate[roll]);
            ui->nick_rate->setValue(ps->rate[nick]);
            ui->gier_rate->setValue(ps->rate[gier]);

            ui->motor1_ch_spinBox->setValue( ps->motor_1.tim_ch);
            ui->motor2_ch_spinBox->setValue( ps->motor_2.tim_ch);
            ui->motor3_ch_spinBox->setValue( ps->motor_3.tim_ch);
            ui->motor4_ch_spinBox->setValue( ps->motor_4.tim_ch);

            if (ps->motor_2.rotational_direction == CW)
            {
                ui->cw_radioButton->setChecked(true);
                ui->ccw_radioButton->setChecked(false);
            }
            else
            {
                ui->ccw_radioButton->setChecked(true);
                ui->cw_radioButton->setChecked(false);
            }

            ui->aspect_ratio_doubleSpinBox->setValue(ps->aspect_ratio);

            ui->rx_select_comboBox->setCurrentIndex(ps->receiver);

            for(i=0; i<3; ++i)
                for(j=0; j<3; ++j)
                {
                    sensor_orientation[i][j] = ps->sensor_orient[i][j];
                }

            ui->rc_thrust_spinBox->setValue(ps->rc_thrust);
            ui->rc_roll_spinBox->setValue(ps->rc_roll);
            ui->rc_nick_spinBox->setValue(ps->rc_nick);
            ui->rc_gier_spinBox->setValue(ps->rc_gier);
            ui->rc_arm_spinBox->setValue(ps->rc_arm);
            ui->rc_mode_spinBox->setValue(ps->rc_mode);
            ui->rc_beep_spinBox->setValue(ps->rc_beep);
            ui->rc_prog_spinBox->setValue(ps->rc_prog);
            ui->rc_var_spinBox->setValue(ps->rc_var);
            ui->rc_write_spinBox->setValue(ps->rc_write);
            ui->rc_aux1_spinBox->setValue(ps->rc_aux1);
            ui->rc_aux2_spinBox->setValue(ps->rc_aux2);

            rotational_direction = ps->motor_2.rotational_direction;

            display_config_scene(rotational_direction);

            delay_counter = 0;
            settings_to_be_read = false;

            pulled = true;            

        }
    }
    else
    {

        QString rc_data_string = serial->readLine(61);

        //QStringList list = serial->readLine(70).

        QStringList list = rc_data_string.split(' ');

        if ( list.count() == 12 )
        {
            QListIterator<QString> iter(list);
            for (i=0; i<12; i++)
            {
                rc_channels.replace(i, iter.next().toInt() );
            }
        }

        /*
        printf("%d %d %d %d %d %d %d %d %d %d %d %d\n",
               rc_channels.at(0), rc_channels.at(1),  rc_channels.at(2),
               rc_channels.at(3), rc_channels.at(4), rc_channels.at(5),
               rc_channels.at(6), rc_channels.at(7), rc_channels.at(8),
               rc_channels.at(9), rc_channels.at(10), rc_channels.at(11) );
        */
    }
}


void MainWindow::serialPortError(QSerialPort::SerialPortError error)
{

    QString message;
    switch (error) {
    case QSerialPort::NoError:
        break;
    case QSerialPort::DeviceNotFoundError:
        message = tr("Device not found");
        break;
    case QSerialPort::OpenError:
        message = tr("Can't open device");
        break;
    case QSerialPort::NotOpenError:
        message = tr("Not open error");
        break;
    case QSerialPort::ResourceError:
        message = tr("Port disconnected");
        break;
    case QSerialPort::UnknownError:
        message = tr("Unknown error");
        break;
    default:
        message = "Serial port error: " + QString::number(error);
        break;
    }

    if(!message.isEmpty()) {

        showStatusInfo(message);

        // crash if closed in this slot
        serial_to_be_closed = true;
    }
}

void MainWindow::showStatusInfo(QString info)
{
    StatusLabel->setText(info);
}

void MainWindow::browseFiles()
{
    ui->fw_select_lineEdit->setText(
        QFileDialog::getOpenFileName(
            this,
            tr("Select dfu binary"),
            QString(),
            tr("DFU Binary ( *.dfu.bin *.bin );;All Files ( * )")
        )
    );
}

bool MainWindow::checkDFU( QFile *dfuUtil )
{
    // Make sure dfu-util exists
    if ( !dfuUtil->exists() )
    {
        // Error, dfu-util not installed locally
        QString output = tr("dfu-util cannot be found. Either build dfu-util and copy the binary to this directory or symlink it.\ne.g. ln -s /usr/bin/dfu-util %1/.").arg( binaryPath );
        ui->result_textEdit->appendPlainText(output );

        return false;
    }

    return true;
}

void MainWindow::dfuFlashBinary()
{
    // Check if file exists
    QString dfuCmd;
    QFile flashFile( ui->fw_select_lineEdit->text() );
    if ( !flashFile.exists() )
    {
        // Error if no file selected
        if ( flashFile.fileName() == QString() )
        {
            QString output = tr("No file selected...");
            ui->result_textEdit->appendPlainText( output );
        }
        // Error if it doesn't exist
        else
        {
            QString output = tr("'%1' does not exist...").arg( flashFile.fileName() );
            ui->result_textEdit->appendPlainText( output );
        }

        return;
    }

#ifdef WIN32
    QFile dfuUtil( binaryPath + "/" + "dfu-util.exe");
#else
    QFile dfuUtil( binaryPath + "/" + "dfu-util" );
#endif

    // Only run dfu-util if it exists
    if ( !checkDFU( &dfuUtil ) )
    {
        return;
    }

    // Run dfu-util command
    if (ui->restart_radioButton->isChecked())
    {
        dfuCmd = QString("%1 -s %2 -D %3").arg( dfuUtil.fileName(), "0x08004000:leave", flashFile.fileName() );
    }
    else
    {
        dfuCmd = QString("%1 -s %2 -D %3").arg( dfuUtil.fileName(), "0x08004000", flashFile.fileName() );
    }

    dfuUtilProcess.start( dfuCmd );

    // Disable the flash button while command is running
    ui->flash_pushButton->setDisabled( true );
    ui->show_dfu_pushButton->setDisabled( true );
}

void MainWindow::dfuListDevices()
{
#ifdef WIN32
    QFile dfuUtil( binaryPath + "/" + "dfu-util.exe");
#else
    QFile dfuUtil( binaryPath + "/" + "dfu-util" );
#endif
    QFile pprintf("/usr/bin/printf");

    // Only run dfu-util if it exists
    if ( !checkDFU( &dfuUtil ) )
    {
        return;
    }

    // Run dfu-util command
    QString dfuCmd = QString("%1 -l").arg( dfuUtil.fileName() );

    dfuUtilProcess.start( dfuCmd );

    // Disable the flash button while command is running
    ui->flash_pushButton->setDisabled( true );
    ui->show_dfu_pushButton->setDisabled( true );
}

void MainWindow::dfuCommandStatus()
{

    // Ihr macht mich wahnsinnig
    QString out;
    static QTextCursor cursor;
    out = dfuUtilProcess.readAll();
    static bool successor = false;
    static bool predecessor = true;

    if (out.contains("\r") )
    {

        out.replace('\r', ' ');

        if (predecessor)
        {
            // first line progress bar must not overwrite the previous line
            ui->result_textEdit->appendPlainText (out);
            predecessor = false;
        }
        else
        {
            // lines of progress bar must delete each previous line
            successor = true;
            cursor = ui->result_textEdit->textCursor();
            cursor.select(QTextCursor::LineUnderCursor);
            cursor.removeSelectedText();
            cursor.deletePreviousChar();
            ui->result_textEdit->appendPlainText (out);
            ui->result_textEdit->setTextCursor( cursor);
        }
    }
    else
    {
        if ( successor )
        {
            // last line progress bar has no '\r' but must overwrite the previous one
            cursor = ui->result_textEdit->textCursor();
            cursor.select(QTextCursor::LineUnderCursor);
            cursor.removeSelectedText();
            cursor.deletePreviousChar();
            ui->result_textEdit->appendPlainText (out);
            ui->result_textEdit->setTextCursor( cursor);
            successor = false;
        }
        else
        {
            ui->result_textEdit->appendPlainText (out);
            predecessor = true;
        }
    }

    // Scroll to bottom
    ui->result_textEdit->verticalScrollBar()->setValue( ui->result_textEdit->verticalScrollBar()->maximum() );
}

void MainWindow::dfuCommandComplete( int exitCode )
{
    // Re-enable button after command completes
    ui->flash_pushButton->setDisabled( false );
    ui->show_dfu_pushButton->setDisabled( false );

    // Append return code
    QString output = tr("Return Code: %1").arg( exitCode );
    ui->result_textEdit->appendPlainText( output );
}

void MainWindow::displayVector(int direction)
{
    enum { up, right, down, left };

    QPolygonF triangle;
    QPen outlinePen(Qt::black);
    QBrush blackBrush(Qt::black);

    switch (direction)
    {
    case up:
        outlinePen.setWidth(2);
        line = config_scene->addLine( 330, 80, 330, 100, outlinePen);
        triangle.clear();
        triangle.append(QPointF(325, 80));
        triangle.append(QPointF(335, 80));
        triangle.append(QPointF(330, 70));
        triangle.append(QPointF(325, 80));
        outlinePen.setWidth(1);
        arrow =  config_scene->addPolygon(triangle, outlinePen, blackBrush);
        break;

    case right:
        outlinePen.setWidth(2);
        line = config_scene->addLine( 350, 120, 370, 120, outlinePen);
        triangle.clear();
        triangle.append(QPointF(370, 115));
        triangle.append(QPointF(370, 125));
        triangle.append(QPointF(380, 120));
        triangle.append(QPointF(370, 115));
        outlinePen.setWidth(1);
        arrow =  config_scene->addPolygon(triangle, outlinePen, blackBrush);
        break;

    case down:
        outlinePen.setWidth(2);
        line = config_scene->addLine( 330, 140, 330, 160, outlinePen);
        triangle.clear();
        triangle.append(QPointF(325, 160));
        triangle.append(QPointF(335, 160));
        triangle.append(QPointF(330, 170));
        triangle.append(QPointF(325, 160));
        outlinePen.setWidth(1);
        arrow =  config_scene->addPolygon(triangle, outlinePen, blackBrush);
        break;

    case left:
        outlinePen.setWidth(2);
        line = config_scene->addLine( 290, 120, 310, 120, outlinePen);
        triangle.clear();
        triangle.append(QPointF(290, 115));
        triangle.append(QPointF(290, 125));
        triangle.append(QPointF(280, 120));
        triangle.append(QPointF(290, 115));
        outlinePen.setWidth(1);
        arrow =  config_scene->addPolygon(triangle, outlinePen, blackBrush);
        break;
    }
}

void MainWindow::display_channels_scene()
{
    int i;
    int ch;


    channels_scene->clear();


    ui->rc_channels_graphicsView->setScene(channels_scene);
    QPen outlinePen(Qt::black);
    outlinePen.setWidth(1);
    QBrush grayBrush(Qt::gray);

    line = channels_scene->addLine( 10, 0, 10, 329, outlinePen);
    line = channels_scene->addLine( 214, 0, 214, 329, outlinePen);
    line = channels_scene->addLine( 418, 0, 418, 329, outlinePen);

    ch = 0;
    for (i=0; i<320; i+=29)
    {
        rectangle = channels_scene->addRect( 10, i, rc_channels.at(ch) / 10, 10, outlinePen, grayBrush );
        ch++;
    }

    /*
    rectangle = channels_scene->addRect( 10, 33, rc_channels.at(rc_roll) / 10, 15, outlinePen, grayBrush );
    rectangle = channels_scene->addRect( 10, 61, rc_channels.at(rc_nick) / 10, 15, outlinePen, grayBrush );
    rectangle = channels_scene->addRect( 10, 89, rc_channels.at(rc_gier) / 10, 15, outlinePen, grayBrush );
    rectangle = channels_scene->addRect( 10, 117, rc_channels.at(rc_arm) / 10, 15, outlinePen, grayBrush );
    rectangle = channels_scene->addRect( 10, 145, rc_channels.at(rc_mode) / 10, 15, outlinePen, grayBrush );
    rectangle = channels_scene->addRect( 10, 173, rc_channels.at(rc_beep) / 10, 15, outlinePen, grayBrush );
    rectangle = channels_scene->addRect( 10, 201, rc_channels.at(rc_prog) / 10, 15, outlinePen, grayBrush );
    rectangle = channels_scene->addRect( 10, 229, rc_channels.at(rc_var) / 10, 15, outlinePen, grayBrush );
    rectangle = channels_scene->addRect( 10, 257, rc_channels.at(rc_write) / 10, 15, outlinePen, grayBrush );
    rectangle = channels_scene->addRect( 10, 285, rc_channels.at(rc_aux1) / 10, 15, outlinePen, grayBrush );
    rectangle = channels_scene->addRect( 10, 313, rc_channels.at(rc_aux2) / 10, 15, outlinePen, grayBrush );
    */
}


void MainWindow::display_config_scene(int rotation)
{

    enum { up, right, down, left };

    int i, j;

    config_scene->clear();

    ui->mix_graphicsView->setScene(config_scene);

    QPen outlinePen(Qt::black);
    outlinePen.setWidth(2);
    QBrush grayBrush(Qt::gray);
    // Motor arms
    rectangle = config_scene->addRect( 35, 68, 160, 15, outlinePen, grayBrush );
    rectangle = config_scene->addRect( 35, 152, 160, 15, outlinePen, grayBrush );
    // chassis
    rectangle = config_scene->addRect( 95, 40, 40, 150, outlinePen, grayBrush );

    // propeller circles
    ellipse = config_scene->addEllipse(10, 35, 80, 80, outlinePen, grayBrush);
    ellipse = config_scene->addEllipse(10, 120, 80, 80, outlinePen, grayBrush);
    ellipse = config_scene->addEllipse(140, 35, 80, 80, outlinePen, grayBrush);
    ellipse = config_scene->addEllipse(140, 120, 80, 80, outlinePen, grayBrush);

    // Flight direction pointer
    QBrush blackBrush(Qt::black);
    outlinePen.setWidth(4);
    line = config_scene->addLine(115, 90, 115, 165, outlinePen);
    outlinePen.setWidth(1);
    QPolygonF triangle;
    triangle.append(QPointF(110, 90));
    triangle.append(QPointF(120, 90));
    triangle.append(QPointF(115, 70));
    triangle.append(QPointF(110, 90));
    outlinePen.setWidth(1);
    arrow =  config_scene->addPolygon(triangle, outlinePen, blackBrush);

    // Rotation direction pointer
    triangle.clear();
    triangle.append(QPointF(18, 43));
    triangle.append(QPointF(28, 53));
    if (rotation == 1)
    {
        triangle.append(QPointF(13, 57));
    }
    else
    {
        triangle.append(QPointF(33, 38));
    }
    triangle.append(QPointF(18, 43));
    arrow =  config_scene->addPolygon(triangle, outlinePen, blackBrush);


    // Motor labels
    text = config_scene->addText("2", QFont("Arial", 20) );
    text->setPos(40,55);
    text = config_scene->addText("1", QFont("Arial", 20) );
    text->setPos(40,140);
    text = config_scene->addText("3", QFont("Arial", 20) );
    text->setPos(170,140);
    text = config_scene->addText("4", QFont("Arial", 20) );
    text->setPos(170,55);

    // Distance labels
    text = config_scene->addText("A", QFont("Arial", 12) );
    text->setPos(105, 5);
    text = config_scene->addText("B", QFont("Arial", 12) );
    text->setPos(232, 105);

    // Distance lines
    outlinePen.setWidth(1);
    line = config_scene->addLine( 50, 15, 100, 15, outlinePen);
    line = config_scene->addLine( 130, 15, 180, 15, outlinePen);
    line = config_scene->addLine( 50, 10, 50, 75, outlinePen);
    line = config_scene->addLine( 180, 10, 180, 75, outlinePen);
    line = config_scene->addLine( 180, 75, 245, 75, outlinePen);
    line = config_scene->addLine( 180, 160, 245, 160, outlinePen);
    line = config_scene->addLine( 240, 75, 240, 100, outlinePen);
    line = config_scene->addLine( 240, 135, 240, 160, outlinePen);

    // Oriented sensor following
    text = config_scene->addText("Sensor", QFont("Arial", 14) );
    text->setPos(295, 35);

    // Body
    outlinePen.setWidth(2);
    rectangle = config_scene->addRect( 310, 100, 40, 40, outlinePen, grayBrush );

    // Showing front, left and top vectors according sensor_orientation
    // Because rotation is always 90 Degrees any vector can have one component only
    for(i=0; i<3; ++i)    // i is colum meaning vector
    {
        for(j=0; j<3; ++j) // j ist row meaning component
        {
            if ( sensor_orientation[i][j] != 0 )
            {
                switch (i)
                {
                case 0: // Front Vector

                    switch (j)
                    {
                    case 0: // x component

                        if (sensor_orientation[i][j] > 0)
                        {
                            // Front Vector +X (up) direction
                            displayVector(up);
                            text = config_scene->addText("X", QFont("Arial", 12) );
                            text->setPos(335, 65);
                        }
                        else
                        {
                            // Front Vector -X (down) direction
                            displayVector(down);
                            text = config_scene->addText("X", QFont("Arial", 12) );
                            text->setPos(308, 152);
                        }

                        break;

                    case 1: // y component

                        if (sensor_orientation[i][j] > 0)
                        {
                            // Front Vector +Y (left) direction
                            displayVector(left);
                            text = config_scene->addText("X", QFont("Arial", 12) );
                            text->setPos(278, 95);
                        }
                        else
                        {
                            // Front Vector -Y (right) direction
                            displayVector(right);
                            text = config_scene->addText("X", QFont("Arial", 12) );
                            text->setPos(365, 122);
                        }

                        break;

                    case 2: // z component

                        if (sensor_orientation[i][j] > 0)
                        {
                            // Front Vector +Z (top) direction
                            text = config_scene->addText("X+", QFont("Arial", 12) );
                            text->setPos(318, 108);
                        }
                        else
                        {
                            // Front Vector -Z (bottom) direction
                            text = config_scene->addText("X-", QFont("Arial", 12) );
                            text->setPos(318, 108);
                        }

                        break;
                    }

                    break;

                case 1: // Left Vector

                    switch (j)
                    {
                    case 0: // x component

                        if (sensor_orientation[i][j] > 0)
                        {
                            // Left Vector +X (up) direction
                            displayVector(up);
                            text = config_scene->addText("Y", QFont("Arial", 12) );
                            text->setPos(335, 65);
                        }
                        else
                        {
                            // Left Vector -X (down) direction
                            displayVector(down);
                            text = config_scene->addText("Y", QFont("Arial", 12) );
                            text->setPos(308, 152);
                        }

                        break;

                    case 1: // y component

                        if (sensor_orientation[i][j] > 0)
                        {
                            // Left Vector +Y (left) direction
                            displayVector(left);
                            text = config_scene->addText("Y", QFont("Arial", 12) );
                            text->setPos(278, 95);
                        }
                        else
                        {
                            // Left Vector -Y (right) direction
                            displayVector(right);
                            text = config_scene->addText("Y", QFont("Arial", 12) );
                            text->setPos(365, 122);
                        }

                        break;

                    case 2: // z component

                        if (sensor_orientation[i][j] > 0)
                        {
                            // Left Vector +Z (top) direction
                            text = config_scene->addText("Y+", QFont("Arial", 12) );
                            text->setPos(318, 108);
                        }
                        else
                        {
                            // Left Vector -Z (bottom) direction
                            text = config_scene->addText("Y-", QFont("Arial", 12) );
                            text->setPos(318, 108);
                        }

                        break;
                    }

                    break;

                case 2: // Top Vector

                    switch (j)
                    {
                    case 0: // x component

                        if (sensor_orientation[i][j] > 0)
                        {
                            // Top Vector +X (up) direction
                            displayVector(up);
                            text = config_scene->addText("Z", QFont("Arial", 12) );
                            text->setPos(335, 65);
                        }
                        else
                        {
                            // Top Vector -X (down) direction
                            displayVector(down);
                            text = config_scene->addText("Z", QFont("Arial", 12) );
                            text->setPos(308, 152);
                        }

                        break;

                    case 1: // y component

                        if (sensor_orientation[i][j] > 0)
                        {
                            // Top Vector +Y (left) direction
                            displayVector(left);
                            text = config_scene->addText("Z", QFont("Arial", 12) );
                            text->setPos(278, 95);
                        }
                        else
                        {
                            // Top Vector -Y (right) direction
                            displayVector(right);
                            text = config_scene->addText("Z", QFont("Arial", 12) );
                            text->setPos(365, 122);
                        }

                        break;

                    case 2: // z component

                        if (sensor_orientation[i][j] > 0)
                        {
                            // Top Vector +Z (top) direction
                            text = config_scene->addText("Z+", QFont("Arial", 12) );
                            text->setPos(318, 108);
                        }
                        else
                        {
                            // Top Vector -Z (bottom) direction
                            text = config_scene->addText("Z-", QFont("Arial", 12) );
                            text->setPos(318, 108);
                        }

                        break;
                    }

                    break;
                }
            }
        }
    }
}

