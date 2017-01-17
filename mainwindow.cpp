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
    one_shot_timer = new QTimer(this);
    one_shot_timer->setSingleShot(true);
    StatusLabel = new QLabel(this);

    // connect any slot which can't be connected by on_NAME_SIGNAL()
    connect(timer, SIGNAL(timeout()), this, SLOT(timer_elapsed()));
    connect(one_shot_timer, SIGNAL(timeout()), this, SLOT(update_settings_read_delay()));
    connect(serial, SIGNAL(error(QSerialPort::SerialPortError)), this, SLOT(serialPortError(QSerialPort::SerialPortError)));
    connect( ui->fw_select_pushButton, SIGNAL( released() ), this, SLOT( browseFiles() ) );
    connect( ui->fw_save_select_pushButton, SIGNAL( released() ), this, SLOT( browse_saveFile() ) );
    connect( ui->flash_pushButton, SIGNAL( released() ), this, SLOT( dfuFlashBinary() ) );
    connect( ui->fw_save_pushButton, SIGNAL( released() ), this, SLOT( dfuSaveBinary() ) );
    connect( ui->show_dfu_pushButton, SIGNAL( released() ), this, SLOT( dfuListDevices() ) );
    connect( &dfuUtilProcess, SIGNAL( readyReadStandardOutput() ), this, SLOT( dfuCommandStatus() ) );
    connect( &dfuUtilProcess, SIGNAL( finished( int, QProcess::ExitStatus ) ), this, SLOT( dfuCommandComplete( int ) ) );
    connect( serial, SIGNAL(readyRead() ), this, SLOT(serialReadyRead() ) );
    connect( ui->sensor_set_buttonGroup, SIGNAL(buttonClicked(int)), this, SLOT( set_sensor_orientation(int) ) );
    connect( ui->rot_dir_buttonGroup, SIGNAL(buttonClicked(int)), this, SLOT(set_rotational_direction(int) ) );
    connect( ui->rev_buttonGroup, SIGNAL(buttonClicked(int)), this, SLOT(set_rev(int) ) );
    connect(ui->motors_min_max_buttonGroup, SIGNAL(buttonClicked(int)), this, SLOT( motors_set_master_slider(int) ) );
    connect(ui->live_check_buttonGroup, SIGNAL(buttonClicked(int)), this, SLOT( live_graph_enable(int) ) );

    connect( ui->save_settings_pushButton, SIGNAL( released() ), this, SLOT( save_settings() ) );
    connect( ui->restore_settings_pushButton, SIGNAL( released() ), this, SLOT( restore_settings() ) );

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
    ui->reboot_pushButton->setDisabled( true );
    ui->pull_settings_pushButton->setDisabled( true );
    ui->default_settings_pushButton->setDisabled( true );
    ui->push_settings_pushButton->setDisabled( true );    
    ui->save_settings_pushButton->setDisabled( true );
    ui->restore_settings_pushButton->setDisabled( true );
    ui->motors_enable_checkBox->setDisabled( true );
    ui->rx_select_comboBox->addItems(QStringList() << "SBUS" << "SRXL");
    ui->rev_buttonGroup->setExclusive(false);
    ui->live_check_buttonGroup->setExclusive(false);

    // acc graphs
    ui->qcustomplot_widget->addGraph();
    ui->qcustomplot_widget->graph(0)->setPen(QPen(QColor(158, 158, 0)));
    ui->qcustomplot_widget->addGraph();
    ui->qcustomplot_widget->graph(1)->setPen(QPen(QColor(0, 144, 144)));
    ui->qcustomplot_widget->addGraph();
    ui->qcustomplot_widget->graph(2)->setPen(QPen(QColor(255, 0, 255)));

    // gy graphs
    ui->qcustomplot_widget->addGraph();
    ui->qcustomplot_widget->graph(3)->setPen(QPen(QColor(0, 0, 255)));
    ui->qcustomplot_widget->addGraph();
    ui->qcustomplot_widget->graph(4)->setPen(QPen(QColor(255, 0, 0)));
    ui->qcustomplot_widget->addGraph();
    ui->qcustomplot_widget->graph(5)->setPen(QPen(QColor(0, 255, 0)));

    // ang graphs
    ui->qcustomplot_widget->addGraph();
    ui->qcustomplot_widget->graph(6)->setPen(QPen(QColor(255, 170, 0)));
    ui->qcustomplot_widget->addGraph();
    ui->qcustomplot_widget->graph(7)->setPen(QPen(QColor(170, 85, 255)));
    ui->qcustomplot_widget->addGraph();
    ui->qcustomplot_widget->graph(8)->setPen(QPen(QColor(0, 85, 0)));

    //QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    //timeTicker->setTimeFormat("%h:%m:%s");
    //ui->qcustomplot_widget->xAxis->setTicker(timeTicker);
    ui->qcustomplot_widget->xAxis->setRangeReversed(true);
    ui->qcustomplot_widget->axisRect()->setupFullAxesBox();
    ui->qcustomplot_widget->yAxis->setRange(-1.2, 1.2);

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->qcustomplot_widget->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->qcustomplot_widget->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->qcustomplot_widget->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->qcustomplot_widget->yAxis2, SLOT(setRange(QCPRange)));

    plot_timer = new QTimer(this);
    connect(plot_timer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));

    // set IDs
    ui->sensor_set_buttonGroup->setId(ui->sensor_rot_x_plus_pushButton, 101);
    ui->sensor_set_buttonGroup->setId(ui->sensor_rot_x_minus_pushButton, 102);
    ui->sensor_set_buttonGroup->setId(ui->sensor_rot_y_plus_pushButton, 103);
    ui->sensor_set_buttonGroup->setId(ui->sensor_rot_y_minus_pushButton, 104);
    ui->sensor_set_buttonGroup->setId(ui->sensor_rot_z_plus_pushButton, 105);
    ui->sensor_set_buttonGroup->setId(ui->sensor_rot_z_minus_pushButton, 106);

    ui->rot_dir_buttonGroup->setId(ui->cw_radioButton, 201);
    ui->rot_dir_buttonGroup->setId(ui->ccw_radioButton, 202);

    ui->rev_buttonGroup->setId(ui->rc_thrust_rev_checkBox, 301);
    ui->rev_buttonGroup->setId(ui->rc_roll_rev_checkBox, 302);
    ui->rev_buttonGroup->setId(ui->rc_nick_rev_checkBox, 303);
    ui->rev_buttonGroup->setId(ui->rc_gier_rev_checkBox, 304);
    ui->rev_buttonGroup->setId(ui->rc_arm_rev_checkBox, 305);
    ui->rev_buttonGroup->setId(ui->rc_mode_rev_checkBox, 306);
    ui->rev_buttonGroup->setId(ui->rc_beep_rev_checkBox, 307);
    ui->rev_buttonGroup->setId(ui->rc_prog_rev_checkBox, 308);
    ui->rev_buttonGroup->setId(ui->rc_var_rev_checkBox, 309);
    ui->rev_buttonGroup->setId(ui->rc_aux1_rev_checkBox, 310);
    ui->rev_buttonGroup->setId(ui->rc_aux2_rev_checkBox, 311);
    ui->rev_buttonGroup->setId(ui->rc_aux3_rev_checkBox, 312);

    ui->motors_min_max_buttonGroup->setId(ui->motors_min_pushButton, 401);
    ui->motors_min_max_buttonGroup->setId(ui->motors_max_pushButton, 402);

    ui->live_check_buttonGroup->setId(ui->acc_roll_checkBox, 501);
    ui->live_check_buttonGroup->setId(ui->acc_nick_checkBox, 502);
    ui->live_check_buttonGroup->setId(ui->acc_gier_checkBox, 503);
    ui->live_check_buttonGroup->setId(ui->gy_roll_checkBox, 504);
    ui->live_check_buttonGroup->setId(ui->gy_nick_checkBox, 505);
    ui->live_check_buttonGroup->setId(ui->gy_gier_checkBox, 506);
    ui->live_check_buttonGroup->setId(ui->ang_roll_checkBox, 507);
    ui->live_check_buttonGroup->setId(ui->ang_nick_checkBox, 508);
    ui->live_check_buttonGroup->setId(ui->ang_gier_checkBox, 509);

    for (int i=0; i<9; i++ )
    {
        ui->qcustomplot_widget->graph(i)->setVisible(false);
    }

    // set some globally used variables
    settings_to_be_read = false;
    channels_to_be_read = false;
    live_to_be_read = false;
    settings_to_be_write = false;
    settings_written = false;
    settings_saved = false;
    serial_to_be_closed = false;
    switch_state = 42;
    found_our_port = false;
    pulled = false;
    motors_to_be_write = false;
    motors_receipt = false;
    ok_push = false;
    bytes_written = 0;
    delay200 = false;
    motors_write_state_counter = 0;
    push_pending = false;

    rotational_direction = CW;
    rc_channels << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0;
    live_values << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0;
    motor_data = "4000,4000,4000,4000";

    // will be refreshed only if changed
    display_config_scene(CW);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::live_graph_enable(int id)
{
    if (ui->live_check_buttonGroup->button(id)->isChecked())
    {
        ui->qcustomplot_widget->graph(id - 501)->setVisible(true);
    }
    else
    {
       ui->qcustomplot_widget->graph(id - 501)->setVisible(false);
    }
}

void MainWindow::realtimeDataSlot()
{
    // reset of lastPointKey
    // restart of QTime plot_time (see below) and QTimer plot_timer
    // which drives this slot will be done in state_switcher function

    //static QTime time(QTime::currentTime());
    // calculate two new data points:
    double key = plot_time.elapsed()/1000.0; // time elapsed since start, in seconds

    if ( key - lastPointKey > 0.005 ) // at most add point every 5 ms
    {
        // add data to lines:

        for ( int i=0; i<9; i++ )
        {
           ui->qcustomplot_widget->graph(i)->addData(key, live_values.at(i) );
        }

        // scale only by values from visible graphs
        ui->qcustomplot_widget->yAxis->rescale(true);

       // rescale value (vertical) axis to fit the current data:
       // ui->qcustomplot_widget->graph(0)->rescaleValueAxis();
       // ui->qcustomplot_widget->graph(1)->rescaleValueAxis();
       // ui->qcustomplot_widget->graph(2)->rescaleValueAxis();

        lastPointKey = key;
    }

    // make key axis range scroll with the data (at a constant range size of 8 meaning seconds here)
    ui->qcustomplot_widget->xAxis->setRange(key, 8, Qt::AlignRight);
    ui->qcustomplot_widget->replot();
}

void MainWindow::update_settings_read_delay()
{
    // After "pull_settings" sent, the device stops sending live data
    // within 20 ms then waits > 300 ms before sending the requested
    // settings.
    // We are reading data continously if available but append arrived
    // data to the cleaned settings read buffer not before a time between
    // 200 ms has elapsed.
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

    delay200 = true;
}

void MainWindow::timer_elapsed() // 100 ms period
{
    qint64 res;

    refreshSerialDevices();
    display_channels_scene();

    if ( serial_to_be_closed == true)
    {
        serial->close();
        live_values.clear();
        live_values << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0;
        plot_timer->stop();
        StatusLabel->setText("Not Connected");
        serial_to_be_closed = false;
    }
    else
    {
        if (serial->isOpen())
        {
            StatusLabel->setText("Connected");
            ui->connect_pushButton->setDisabled( true );

            if ( settings_to_be_write == true)
            {
                if ( bytes_written < 1024 )
                {
                    res = serial->write( settings_data.constData() +  bytes_written, 1024 - bytes_written);
                    if (res > -1)
                    {
                        bytes_written += res;
                    }
                    else
                    {
                        // need error handler
                        settings_to_be_write = false;
                        bytes_written = 0;
                    }
                }
                else
                {
                    settings_to_be_write = false;
                    bytes_written = 0;
                    settings_written = true;
                }
            }
            else if ( motors_to_be_write == true)
            {
                if ( motors_receipt == true )
                {
                    serial->write(motor_data.constData(), 20);
                    motors_receipt = false;
                }
            }
            else if (settings_to_be_read == false && push_pending == false)
            {
                ui->start_bootloader_pushButton->setDisabled( false );
                ui->reboot_pushButton->setDisabled( false );
                ui->motors_enable_checkBox->setDisabled( false );
                ui->disconnect_pushButton->setDisabled( false );

                if ( live_to_be_read == false )
                {
                    ui->default_settings_pushButton->setDisabled( false );
                    ui->pull_settings_pushButton->setDisabled( false );

                    if (pulled )
                    {
                        ui->push_settings_pushButton->setDisabled( false );
                        ui->save_settings_pushButton->setDisabled( false );
                    }
                    if ( settings_saved )
                    {
                        ui->restore_settings_pushButton->setDisabled( false );
                    }
                }
            }
        }
        else
        {
            StatusLabel->setText("Not Connected");
            ui->start_bootloader_pushButton->setDisabled( true );
            ui->pull_settings_pushButton->setDisabled( true );
            ui->default_settings_pushButton->setDisabled( true );
            ui->save_settings_pushButton->setDisabled( true );
            ui->restore_settings_pushButton->setDisabled( true );
            ui->push_settings_pushButton->setDisabled( true );
            ui->disconnect_pushButton->setDisabled( true );
            ui->reboot_pushButton->setDisabled( true );
            ui->motors_enable_checkBox->setDisabled( true );

            if (found_our_port)
            {
                ui->connect_pushButton->setDisabled( false );
            }
        }
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
        ui->fw_save_pushButton->setDisabled( true );
        ui->show_dfu_pushButton->setDisabled( true );
    }
    else
    {
        ui->connect_pushButton->setDisabled( true );
        ui->flash_pushButton->setDisabled( false );
        ui->fw_save_pushButton->setDisabled( false );
        ui->show_dfu_pushButton->setDisabled( false);
    }
}

void MainWindow::state_switch(int state)
{

    live_values.clear();
    live_values << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0;
    serial->clear();

    switch (state)
    {

    case Firmware:
        serial->write("fw_tab", 7);
        channels_to_be_read = false;
        motors_to_be_write = false;
        live_to_be_read = false;
        plot_timer->stop();
        break;

    case Configuration:
        serial->write("config_tab", 11);
        channels_to_be_read = true;
        motors_to_be_write = false;
        live_to_be_read = false;
        plot_timer->stop();
        break;

    case Motor_test:
        serial->write("motors_tab", 11);
        channels_to_be_read = false;
        live_to_be_read = false;
        plot_timer->stop();
        if ( ui->motors_enable_checkBox->isChecked() )
        {
            ui->motors_enable_checkBox->click();
        }
        break;

    case Flight_setup:
        serial->write("flight_tab", 11);
        channels_to_be_read = false;
        motors_to_be_write = false;
        live_to_be_read = false;
        plot_timer->stop();
        break;

    case Live_plots:
        serial->write("live_tab", 9);

        ui->pull_settings_pushButton->setDisabled( true );
        ui->default_settings_pushButton->setDisabled( true );
        ui->save_settings_pushButton->setDisabled( true );
        ui->restore_settings_pushButton->setDisabled( true );
        ui->push_settings_pushButton->setDisabled( true );

        channels_to_be_read = false;
        motors_to_be_write = false;
        live_to_be_read = true;
        for ( int i=0; i<9; i++ )
        {
            ui->qcustomplot_widget->graph(i)->data().data()->clear();
        }
        lastPointKey = 0;
        plot_timer->start(0);
        plot_time.start();
        break;

    // not used jet
    case suspend:
        serial->write("suspend", 8);
        channels_to_be_read = false;
        motors_to_be_write = false;
        live_to_be_read = false;
        plot_timer->stop();
        break;
    }

    switch_state = state;
}

void MainWindow::on_connect_pushButton_clicked()
{
    if(serial->isOpen())
    {
        return;
    }

    serial->setPortName(ui->availports_comboBox->currentData().toString());

    // for testing with socat as proxy
    //QString qstr;
    //qstr.append("/dev/ttyACM5");
    //serial->setPortName(qstr);

    // usage (as root):
    // socat -x -v PTY,link=/dev/ttyACM5 /dev/ttyACM0,raw
    // prepare in other terminal: chmod 666 /dev/ttyACM5
    // and stty "1:0:18b2:0:3:1c:7f:15:4:5:1:0:11:13:1a:0:12:f:17:16:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0" -F /dev/ttyACM5
    // after connection traffic in both directions in hex will be shown in the terminal where socat have been started

    serial->open(QIODevice::ReadWrite);

    if(!serial->isOpen()) {
        return;
    }

    serial->setBaudRate(QSerialPort::Baud115200);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    state_switch(ui->tab->currentIndex());
}

void MainWindow::on_disconnect_pushButton_clicked()
{
    live_values.clear();
    live_values << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0;

    plot_timer->stop();
    if (serial->isOpen()) {

        if ( ui->motors_enable_checkBox->isChecked() )
        {
            ui->motors_enable_checkBox->click();
        }
        serial->close();
    }
}

void MainWindow::on_tab_currentChanged(int index)
{
    if (serial->isOpen())
    {
        serial->clear();
        state_switch(index);
    }
}

void MainWindow::set_rev(int index)
{
    int status, i;
    int func = index - 300;

    // find status of current clicked channel reverse checkbox
    status = ui->rev_buttonGroup->button(index)->isChecked();

    // set this status in the position of rc_ch array wich
    // is assigned to the current position of the rc_func array
    rc_ch[rc_func[func].number].rev = status;

    // set this status in any position of rc_func
    // with the same channel number as the current one
    for ( i = 1; i < 13; i++)
    {
        if ( rc_func[i].number ==  rc_func[func].number )
        {
             rc_func[i].rev = status;
             ui->rev_buttonGroup->button(300 + i)->setChecked( status );
        }
    }
}

// in each of this functions assign the changed channel number value
// to the corresponding position in the rc_func array
// this sets also the reverse field in rc_func from rc_ch
// then check or uncheck the corresponding reverse checkbox
// memorized at the corresponding rc_ch position by the rev field
void MainWindow::on_rc_thrust_spinBox_valueChanged(int value)
{
    rc_func[r_thrust] = rc_ch[value];
    ui->rc_thrust_rev_checkBox->setChecked( rc_ch[value].rev );
}

void MainWindow::on_rc_roll_spinBox_valueChanged(int value)
{
    rc_func[r_roll] = rc_ch[value];
    ui->rc_roll_rev_checkBox->setChecked( rc_ch[value].rev );
}

void MainWindow::on_rc_nick_spinBox_valueChanged(int value)
{
    rc_func[r_nick] = rc_ch[value];
    ui->rc_nick_rev_checkBox->setChecked( rc_ch[value].rev );
}

void MainWindow::on_rc_gier_spinBox_valueChanged(int value)
{
    rc_func[r_gier] = rc_ch[value];
    ui->rc_gier_rev_checkBox->setChecked( rc_ch[value].rev );
}

void MainWindow::on_rc_arm_spinBox_valueChanged(int value)
{
    rc_func[r_arm] = rc_ch[value];
    ui->rc_arm_rev_checkBox->setChecked( rc_ch[value].rev );
}

void MainWindow::on_rc_mode_spinBox_valueChanged(int value)
{
    rc_func[r_mode] = rc_ch[value];
    ui->rc_mode_rev_checkBox->setChecked( rc_ch[value].rev );
}

void MainWindow::on_rc_beep_spinBox_valueChanged(int value)
{
    rc_func[r_beep] = rc_ch[value];
     ui->rc_beep_rev_checkBox->setChecked( rc_ch[value].rev );
}

void MainWindow::on_rc_prog_spinBox_valueChanged(int value)
{
    rc_func[r_prog] = rc_ch[value];
    ui->rc_prog_rev_checkBox->setChecked( rc_ch[value].rev );
}

void MainWindow::on_rc_var_spinBox_valueChanged(int value)
{
    rc_func[r_var] = rc_ch[value];
    ui->rc_var_rev_checkBox->setChecked( rc_ch[value].rev );
}

void MainWindow::on_rc_aux1_spinBox_valueChanged(int value)
{
    rc_func[r_aux1] = rc_ch[value];
    ui->rc_aux1_rev_checkBox->setChecked( rc_ch[value].rev );
}

void MainWindow::on_rc_aux2_spinBox_valueChanged(int value)
{
    rc_func[r_aux2] = rc_ch[value];
    ui->rc_aux2_rev_checkBox->setChecked(  rc_ch[value].rev );
}

void MainWindow::on_rc_aux3_spinBox_valueChanged(int value)
{
    rc_func[r_aux3] = rc_ch[value];
    ui->rc_aux3_rev_checkBox->setChecked( rc_ch[value].rev );
}

void MainWindow::motors_set_master_slider(int id)
{
    switch (id)
    {
       case min:
       ui->motor_value_master_verticalSlider->setValue(4000);
       on_motor_value_master_verticalSlider_valueChanged(4000);
       break;

       case max:
       ui->motor_value_master_verticalSlider->setValue(8000);
       on_motor_value_master_verticalSlider_valueChanged(8000);
       break;
    }
}

void MainWindow::on_motors_enable_checkBox_clicked(bool checked)
{
    ui->motor_value_master_verticalSlider->setValue(4000);
    ui->motor1_value_verticalSlider->setValue(4000);
    ui->motor2_value_verticalSlider->setValue(4000);
    ui->motor3_value_verticalSlider->setValue(4000);
    ui->motor4_value_verticalSlider->setValue(4000);

    if ( checked == false )
    {
        ui->motors_max_pushButton->setDisabled( true );
        ui->motors_min_pushButton->setDisabled( true );

        ui->motor_value_master_verticalSlider->setDisabled( true );
        ui->motor1_value_verticalSlider->setDisabled( true );
        ui->motor2_value_verticalSlider->setDisabled( true );
        ui->motor3_value_verticalSlider->setDisabled( true );
        ui->motor4_value_verticalSlider->setDisabled( true );

    }
    else
    {
        ui->motors_max_pushButton->setDisabled( false );
        ui->motors_min_pushButton->setDisabled( false );

        ui->motor_value_master_verticalSlider->setDisabled( false );
        ui->motor1_value_verticalSlider->setDisabled( false );
        ui->motor2_value_verticalSlider->setDisabled( false );
        ui->motor3_value_verticalSlider->setDisabled( false );
        ui->motor4_value_verticalSlider->setDisabled( false );

        ui->start_bootloader_pushButton->setDisabled( true );
        ui->reboot_pushButton->setDisabled( true );
        ui->default_settings_pushButton->setDisabled( true );
        ui->save_settings_pushButton->setDisabled( true );
        ui->restore_settings_pushButton->setDisabled( true );
        ui->pull_settings_pushButton->setDisabled( true );
        ui->push_settings_pushButton->setDisabled( true );
        ui->disconnect_pushButton->setDisabled( true );

        motors_to_be_write = true;
        motors_receipt = true;
    }
}

void MainWindow::on_motor_value_master_verticalSlider_valueChanged(int value)
{
    motor1_value = motor2_value = motor3_value = motor4_value = value;

    ui->motor1_value_verticalSlider->setValue(value);
    ui->motor2_value_verticalSlider->setValue(value);
    ui->motor3_value_verticalSlider->setValue(value);
    ui->motor4_value_verticalSlider->setValue(value);

    QString motor1_data_string;
    QString motor2_data_string;
    QString motor3_data_string;
    QString motor4_data_string;

    QTextStream(&motor1_data_string) << motor1_value / 4;
    QTextStream(&motor2_data_string) << motor2_value / 4;
    QTextStream(&motor3_data_string) << motor3_value / 4;
    QTextStream(&motor4_data_string) << motor4_value / 4;


    ui->motor1_value_label->setText(motor1_data_string);
    ui->motor2_value_label->setText(motor2_data_string);
    ui->motor3_value_label->setText(motor3_data_string);
    ui->motor4_value_label->setText(motor4_data_string);

    QString motor_data_string;

    QTextStream(&motor_data_string) << motor1_value << "," <<  motor2_value <<  "," << motor3_value <<  "," << motor4_value;

    motor_data.clear();
    motor_data.append(motor_data_string.toLocal8Bit());
}

void MainWindow::on_motor1_value_verticalSlider_valueChanged(int value)
{
    motor1_value = value;
    QString motor1_data_string;
    QTextStream(&motor1_data_string) << motor1_value / 4;
    ui->motor1_value_label->setText(motor1_data_string);

    QString motor_data_string;
    QTextStream(&motor_data_string) << motor1_value << "," <<  motor2_value <<  "," << motor3_value <<  "," << motor4_value;

    motor_data.clear();
    motor_data.append(motor_data_string.toLocal8Bit());
}

void MainWindow::on_motor2_value_verticalSlider_valueChanged(int value)
{
    motor2_value = value;
    QString motor2_data_string;
    QTextStream(&motor2_data_string) << motor2_value / 4;
    ui->motor2_value_label->setText(motor2_data_string);

    QString motor_data_string;
    QTextStream(&motor_data_string) << motor1_value << "," <<  motor2_value <<  "," << motor3_value <<  "," << motor4_value;

    motor_data.clear();
    motor_data.append(motor_data_string.toLocal8Bit());
}

void MainWindow::on_motor3_value_verticalSlider_valueChanged(int value)
{
    motor3_value = value;
    QString motor3_data_string;
    QTextStream(&motor3_data_string) << motor3_value / 4;
    ui->motor3_value_label->setText(motor3_data_string);

    QString motor_data_string;
    QTextStream(&motor_data_string) << motor1_value << "," <<  motor2_value <<  "," << motor3_value <<  "," << motor4_value;

    motor_data.clear();
    motor_data.append(motor_data_string.toLocal8Bit());
}

void MainWindow::on_motor4_value_verticalSlider_valueChanged(int value)
{
    motor4_value = value;
    QString motor4_data_string;
    QTextStream(&motor4_data_string) << motor4_value / 4;
    ui->motor4_value_label->setText(motor4_data_string);

    QString motor_data_string;
    QTextStream(&motor_data_string) << motor1_value << "," <<  motor2_value <<  "," << motor3_value <<  "," << motor4_value;

    motor_data.clear();
    motor_data.append(motor_data_string.toLocal8Bit());
}

void MainWindow::ui_to_settings_data()
{
    settings *ps;
    int i,j;

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

    for ( i = 0; i < 13; i++ )
    {
        ps->rc_func[i] = rc_func[i];
        ps->rc_ch[i] = rc_ch[i];
    }

    for(i=0; i<3; ++i)
        for(j=0; j<3; ++j)
        {
            ps->sensor_orient[i][j] = sensor_orientation[i][j];
        }

    ps->aspect_ratio = ui->aspect_ratio_doubleSpinBox->value();

    ps->receiver = ui->rx_select_comboBox->currentIndex();
}

bool MainWindow::settings_data_to_ui()
{
    settings *ps;
    int i,j;

    ps = (settings*) settings_data.data();

    if ( ps->magic == 0xdb )
    {

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

        ui->rc_thrust_spinBox->setValue(ps->rc_func[r_thrust].number);
        ui->rc_roll_spinBox->setValue(ps->rc_func[r_roll].number);
        ui->rc_nick_spinBox->setValue(ps->rc_func[r_nick].number);
        ui->rc_gier_spinBox->setValue(ps->rc_func[r_gier].number);
        ui->rc_arm_spinBox->setValue(ps->rc_func[r_arm].number);
        ui->rc_mode_spinBox->setValue(ps->rc_func[r_mode].number);
        ui->rc_beep_spinBox->setValue(ps->rc_func[r_beep].number);
        ui->rc_prog_spinBox->setValue(ps->rc_func[r_prog].number);
        ui->rc_var_spinBox->setValue(ps->rc_func[r_var].number);
        ui->rc_aux1_spinBox->setValue(ps->rc_func[r_aux1].number);
        ui->rc_aux2_spinBox->setValue(ps->rc_func[r_aux2].number);
        ui->rc_aux3_spinBox->setValue(ps->rc_func[r_aux3].number);

        ui->rc_thrust_rev_checkBox->setChecked( ps->rc_func[r_thrust].rev );
        ui->rc_roll_rev_checkBox->setChecked( ps->rc_func[r_roll].rev );
        ui->rc_nick_rev_checkBox->setChecked( ps->rc_func[r_nick].rev );
        ui->rc_gier_rev_checkBox->setChecked( ps->rc_func[r_gier].rev );
        ui->rc_arm_rev_checkBox->setChecked( ps->rc_func[r_arm].rev );
        ui->rc_mode_rev_checkBox->setChecked( ps->rc_func[r_mode].rev );
        ui->rc_beep_rev_checkBox->setChecked( ps->rc_func[r_beep].rev );
        ui->rc_prog_rev_checkBox->setChecked( ps->rc_func[r_prog].rev );
        ui->rc_var_rev_checkBox->setChecked( ps->rc_func[r_var].rev );
        ui->rc_aux1_rev_checkBox->setChecked( ps->rc_func[r_aux1].rev );
        ui->rc_aux2_rev_checkBox->setChecked( ps->rc_func[r_aux2].rev );
        ui->rc_aux3_rev_checkBox->setChecked( ps->rc_func[r_aux3].rev );

        for ( i = 0; i < 13; i++ )
        {
            rc_func[i] = ps->rc_func[i];
            rc_ch[i] = ps->rc_ch[i];
        }

        rotational_direction = ps->motor_2.rotational_direction;

        display_config_scene(rotational_direction);

        return true;
    }
    return false;
}

void MainWindow::on_pull_settings_pushButton_clicked()
{
    serial->clear();
    settings_data.clear();
    serial->write("pull_settings", 14);

    channels_to_be_read = false;

    one_shot_timer->start(150);

    ui->pull_settings_pushButton->setDisabled( true );
    ui->push_settings_pushButton->setDisabled( true );
    ui->default_settings_pushButton->setDisabled( true );
    ui->save_settings_pushButton->setDisabled( true );
    ui->restore_settings_pushButton->setDisabled( true );
    ui->reboot_pushButton->setDisabled( true );
    ui->disconnect_pushButton->setDisabled( true );

    settings_to_be_read = true;
}

void MainWindow::on_push_settings_pushButton_clicked()
{
    ui->push_settings_pushButton->setDisabled( true );
    ui->default_settings_pushButton->setDisabled( true );
    ui->save_settings_pushButton->setDisabled( true );
    ui->restore_settings_pushButton->setDisabled( true );
    ui->reboot_pushButton->setDisabled( true );
    ui->pull_settings_pushButton->setDisabled( true );
    ui->disconnect_pushButton->setDisabled( true );

    serial->clear();
    serial->write("push_settings", 14);

    channels_to_be_read = false;

    //state_switch(suspend);

    ui_to_settings_data();

    push_pending = true;

}

void MainWindow::on_default_settings_pushButton_clicked()
{
    serial->write("load_defaults", 14);
    pulled = false;
    ui->push_settings_pushButton->setDisabled( true );
}

void MainWindow::save_settings()
{
    QFile settings_file;
    QString filename = "settings.bin";
    settings_file.setFileName(filename);

    settings_saved = false;

    if ( settings_file.open(QIODevice::WriteOnly) == true )
    {
        ui_to_settings_data();

        if ( settings_file.write(settings_data.data(), 1024) == 1024 )
        {
            settings_saved = true;
            ui->restore_settings_pushButton->setText("Restore Settings");
        }

        settings_file.close();
    }
    else
    {
        ui->save_settings_pushButton->setText("failed open settings.bin");
    }
}

void MainWindow::restore_settings()
{

    QFile settings_file;
    QString filename = "settings.bin";
    settings_file.setFileName(filename);

    if ( settings_file.open(QIODevice::ReadOnly) )
    {

        settings_data.clear();

        settings_data = settings_file.readAll();

        if ( settings_data_to_ui() == true )
        {
            ui->restore_settings_pushButton->setText("Restore Settings");
        }
        else
        {
            ui->restore_settings_pushButton->setText("failed Settings invalid");
        }

        settings_file.close();
    }
    else
    {
        ui->restore_settings_pushButton->setText("failed open settings.bin");
    }
}

void MainWindow::on_reboot_pushButton_clicked()
{
    if ( ui->motors_enable_checkBox->isChecked() )
    {
        ui->motors_enable_checkBox->click();
    }

    serial->write("reboot", 7);
    live_to_be_read = 0;
    plot_timer->stop();
    live_values.clear();
    live_values << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0;
    pulled = false;
}

void MainWindow::on_start_bootloader_pushButton_clicked()
{
    ui->start_bootloader_pushButton->setDisabled( true );
    if ( ui->motors_enable_checkBox->isChecked() )
    {
        ui->motors_enable_checkBox->click();
    }
    ui->connect_pushButton->setDisabled( true );
    serial->write("bootloader", 11);
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
//    settings *ps;
//    int i,j;
    int i;

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

    if ( settings_to_be_read && delay200 )
    {
        settings_data.append(serial->read(1024));

        if ( settings_data.size() >= 1024 )
        {
            //ps = (settings*) settings_data.data();

            if ( settings_data_to_ui() == true)
            {
                delay200 = 0;
                settings_to_be_read = false;

                pulled = true;
                ui->pull_settings_pushButton->setText("Pull Settings");

                state_switch(switch_state);
            }
            else
            {
                delay200 = 0;
                settings_to_be_read = false;
                pulled = false;
                ui->pull_settings_pushButton->setText("failed Pull Settings again!");
            }
        }
    }
    else if (channels_to_be_read == 1)
    {
        QString rc_data_string = serial->readLine(61);
        QStringList list = rc_data_string.trimmed().split(' ');

        if ( list.count() == 12 )
        {
            ui->rc_ch_01_label->setText(list.at(0));
            ui->rc_ch_02_label->setText(list.at(1));
            ui->rc_ch_03_label->setText(list.at(2));
            ui->rc_ch_04_label->setText(list.at(3));
            ui->rc_ch_05_label->setText(list.at(4));
            ui->rc_ch_06_label->setText(list.at(5));
            ui->rc_ch_07_label->setText(list.at(6));
            ui->rc_ch_08_label->setText(list.at(7));
            ui->rc_ch_09_label->setText(list.at(8));
            ui->rc_ch_10_label->setText(list.at(9));
            ui->rc_ch_11_label->setText(list.at(10));
            ui->rc_ch_12_label->setText(list.at(11));

            QListIterator<QString> iter(list);
            for (i=0; i<12; i++)
            {
                rc_channels.replace(i, iter.next().toInt() );
            }

            QString labetext;

            //labetext = rc_func[r_thrust].number == 0 ? "disabled" : rc_channels.at(0) < 2048 ? "low    " : "high   ";
            labetext = rc_func[r_thrust].number == 0 ? "disabled" : rc_channels.at(0) < 1400 ? "low    " : rc_channels.at(0) > 2700 ? "high   " : "middle ";
            ui->rc_thrust_label->setText(labetext);

            //labetext = rc_func[r_roll].number == 0 ? "disabled" :  rc_channels.at(1) < 2048 ? "left   " : "right  ";
            labetext = rc_func[r_roll].number == 0 ? "disabled" : rc_channels.at(1) < 1400 ? "left   " : rc_channels.at(1) > 2700 ? "right  " : "middle ";
            ui->rc_roll_label->setText(labetext);

            //labetext = rc_func[r_nick].number == 0 ? "disabled" : rc_channels.at(2) < 2048 ? "up     " : "down   ";
            labetext = rc_func[r_nick].number == 0 ? "disabled" : rc_channels.at(2) < 1400 ? "up     " : rc_channels.at(2) > 2700 ? "down   " : "middle ";
            ui->rc_nick_label->setText(labetext);

            //labetext = rc_func[r_gier].number == 0 ? "disabled" : rc_channels.at(3) < 2048 ? "left   " : "right  ";
            labetext = rc_func[r_gier].number == 0 ? "disabled" : rc_channels.at(3) < 1400 ? "left   " : rc_channels.at(3) > 2700 ? "right  " : "middle ";
            ui->rc_gier_label->setText(labetext);

            labetext = rc_func[r_arm].number == 0 ? "disabled" : rc_channels.at(4) < 2700 ? "stop   " : "armed  ";
            ui->rc_arm_label->setText(labetext);

            labetext = rc_func[r_mode].number == 0 ? "disabled" : rc_channels.at(5) < 1400 ? "mode 1 " : rc_channels.at(5) > 2700 ? "mode 3 " : "mode 2 ";
            ui->rc_mode_label->setText(labetext);

            labetext = rc_func[r_beep].number == 0 ? "disabled" : rc_channels.at(6) < 2700 ? "off    " : "beep   ";
            ui->rc_beep_label->setText(labetext);

            labetext = rc_func[r_prog].number == 0 ? "disabled" : rc_channels.at(7) < 1400 ? "off    " : rc_channels.at(7) > 2700 ? "write  " : "prog   ";
            ui->rc_prog_label->setText(labetext);

            //labetext = rc_func[r_var].number == 0 ? "disabled" : rc_channels.at(8) < 2048 ? "low    " : "high   ";
            labetext = rc_func[r_var].number == 0 ? "disabled" : rc_channels.at(8) < 1400 ? "low    " : rc_channels.at(8) > 2700 ? "high   " : "middle ";
            ui->rc_var_label->setText(labetext);

            //labetext = rc_func[r_aux1].number == 0 ? "disabled" : rc_channels.at(9) < 2048 ? "low    " : "high   ";
            labetext = rc_func[r_aux1].number == 0 ? "disabled" : rc_channels.at(9) < 1400 ? "low    " : rc_channels.at(9) > 2700 ? "high   " : "middle ";
            ui->rc_aux1_label->setText(labetext);

            //labetext = rc_func[r_aux2].number == 0 ? "disabled" : rc_channels.at(10) < 2048 ? "low    " : "high   ";
            labetext = rc_func[r_aux2].number == 0 ? "disabled" : rc_channels.at(10) < 1400 ? "low    " : rc_channels.at(10) > 2700 ? "high   " : "middle ";
            ui->rc_aux2_label->setText(labetext);

            //labetext = rc_func[r_aux3].number == 0 ? "disabled" : rc_channels.at(11) < 2048 ? "low    " : "high   ";
            labetext = rc_func[r_aux3].number == 0 ? "disabled" : rc_channels.at(11) < 1400 ? "low    " : rc_channels.at(11) > 2700 ? "high   " : "middle ";
            ui->rc_aux3_label->setText(labetext);


            if ( channels_to_be_read == 1 ) // stop ASAP
            {
                serial->write("channels_receipt", 17);
            }
        }
    }
    else if (live_to_be_read == 1)
    {
        QString live_data_string = serial->readLine(100);
        QStringList live_list = live_data_string.trimmed().split(' ');

        if ( live_list.count() == 9 )
        {
            QListIterator<QString> iter(live_list);
            for (i=0; i<9; i++)
            {
                if ( i >= 6 )
                {
                    live_values.replace(i, iter.next().toDouble() * 180/M_PI );
                }
                else
                {
                    live_values.replace(i, iter.next().toDouble() );
                }
            }

            serial->write("live_receipt", 13);

            //qDebug(live_data_string.toLatin1() );

            //printf("%1.3f  %1.3f  %1.3f  %4.3f  %4.3f  %4.3f\n",
            //live_values.at(0), live_values.at(1), live_values.at(2),
            //live_values.at(3), live_values.at(4), live_values.at(5) );

        }
        else
        {
            printf("%d\n",live_list.count() );
        }

    }
    else if (motors_to_be_write == 1)
    {
        // This also eats up spurious lines from channels_to_be_read state after such transition
        // So it have to read up to the channels line length
        QString receipt_string = serial->readLine(61).trimmed();

        if (strcmp( receipt_string.toStdString().c_str(), (const char *) "motors_receipt") == 0 )
        {
            motors_receipt = 1;
        }
        else
        {
            //printf ("unrelated input: %s\n", motors_receipt_string.toStdString().c_str() );
        }
    }
    else if (push_pending == true)
    {
        QString receipt_string = serial->readLine(61).trimmed();
        if (strcmp( receipt_string.toStdString().c_str(), (const char *) "ok_push") == 0 )
        {
            push_pending = false;
            settings_to_be_write = 1;
        }
    }
    else if ( settings_written == true)
    {
        QString receipt_string = serial->readLine(61).trimmed();
        if (strcmp( receipt_string.toStdString().c_str(), (const char *) "settings_rcvd") == 0 )
        {
            settings_written = false;
            state_switch(switch_state);
        }
    }
    else
    {
        // printf ("swallow\n");
        serial->readAll();
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

    if( !message.isEmpty() )
    {
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

void MainWindow::browse_saveFile()
{
    ui->fw_save_select_lineEdit->setText(
        QFileDialog::getSaveFileName(
            this,
            tr("Save File"),
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

void MainWindow::dfuSaveBinary()
{
    QString dfuCmd;
    QFile saveFile( ui->fw_save_select_lineEdit->text() );

#ifdef WIN32
    QFile dfuUtil( binaryPath + "/" + "dfu-util.exe");
#else
    QFile dfuUtil( binaryPath + "/" + "dfu-util" );
#endif

    // Error if no file selected
    if ( saveFile.fileName() == QString() )
    {
        QString output = tr("No file for saving the Firmware was selected...");
        ui->result_textEdit->appendPlainText( output );
    }

    // Only run dfu-util if it exists
    if ( !checkDFU( &dfuUtil ) )
    {
        return;
    }

    if ( saveFile.exists() )
    {
        saveFile.remove();
    }

    // Run dfu-util command
    if (ui->restart_checkBox->isChecked())
    {
        if ( ui->incl_settings_checkBox->isChecked() )
        {
           dfuCmd = QString("%1 -s %2 -U %3").arg( dfuUtil.fileName(), "0x08004000:leave", saveFile.fileName() );
        }
        else
        {
            dfuCmd = QString("%1 -s %2 -U %3").arg( dfuUtil.fileName(), "0x08004000:102400:leave", saveFile.fileName() );
        }
    }
    else
    {
        if ( ui->incl_settings_checkBox->isChecked() )
        {
            dfuCmd = QString("%1 -s %2 -U %3").arg( dfuUtil.fileName(), "0x08004000", saveFile.fileName() );
        }
        else
        {
            dfuCmd = QString("%1 -s %2 -U %3").arg( dfuUtil.fileName(), "0x08004000:102400", saveFile.fileName() );
        }
    }

    dfuUtilProcess.start( dfuCmd );

    // Disable the flash button while command is running
    ui->flash_pushButton->setDisabled( true );
    ui->fw_save_pushButton->setDisabled( true );
    ui->show_dfu_pushButton->setDisabled( true );

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
            QString output = tr("No file containing the Firmware was selected...");
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
    if (ui->restart_checkBox->isChecked())
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
    ui->fw_save_pushButton->setDisabled( true );
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
    ui->fw_save_pushButton->setDisabled( true );
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

    // Append return code ?
    // no don't bother the user
    //QString output = tr("Return Code: %1").arg( exitCode );
    //ui->result_textEdit->appendPlainText( output );
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
    line = channels_scene->addLine( 215, 0, 215, 329, outlinePen);
    line = channels_scene->addLine( 419, 0, 419, 329, outlinePen);

    line = channels_scene->addLine( 150, 0, 150, 329, outlinePen);
    line = channels_scene->addLine( 280, 0, 280, 329, outlinePen);

    ch = 0;
    for (i=0; i<320; i+=29)
    {
        rectangle = channels_scene->addRect( 10, i, rc_channels.at(ch) / 10, 10, outlinePen, grayBrush );
        ch++;
    }
}

// Helper for display_config_scene
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

