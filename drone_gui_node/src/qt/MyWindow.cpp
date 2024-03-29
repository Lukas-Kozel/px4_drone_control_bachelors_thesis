#include "MyWindow.h"

MyWindow::MyWindow(rclcpp::Node::SharedPtr node, ConnectionManager* connectionManager, QWidget *parent) : QMainWindow(parent), node(node), connectionManager(connectionManager)
{

    QWidget* centralWidget = new QWidget(this);
    QHBoxLayout* mainLayout = new QHBoxLayout(centralWidget);
    connectionIndicator = new QLabel(centralWidget);
    stateIndicator = new QLabel(centralWidget);

    DroneVisualWidget *droneVisual = new DroneVisualWidget(node,connectionManager,this);

    QVBoxLayout* leftLayout = new QVBoxLayout();
    QSplitter* leftSplitter = new QSplitter(Qt::Vertical);
    leftSplitter->addWidget(droneVisual);
    leftLayout->addWidget(leftSplitter);
    mainLayout->addLayout(leftLayout, 6);

    rightLayout = new QVBoxLayout();

    switchOffboardModeButton = new QPushButton("Switch to Offboard Mode");
    connect(switchOffboardModeButton, &QPushButton::clicked, this, &MyWindow::onSwitchToOffboardMode);

    turnOffboardModeOffButton = new QPushButton("Turn the Offboard Mode off");
    turnOffboardModeOffButton->setDisabled(true);
    connect(turnOffboardModeOffButton, &QPushButton::clicked, this, &MyWindow::turnOffboardModeOff);

    armButton = new QPushButton("Arm");
    connect(armButton, &QPushButton::clicked, this, &MyWindow::onSwitchToArmedMode);

    takeoffButton = new QPushButton("Takeoff");
    connect(takeoffButton, &QPushButton::clicked, this, &MyWindow::onTakeOffMode);

    landingButton = new QPushButton("Land");
    connect(landingButton, &QPushButton::clicked, this, &MyWindow::onLandingMode);

    controllerButton = new QPushButton("Controller");
    controllerButton->setDisabled(true);
    connect(controllerButton,&QPushButton::clicked, this,&MyWindow::onControllerStart);

    QPushButton* restartSimulationButton = new QPushButton("restart simulation");
    connect(restartSimulationButton,&QPushButton::clicked, this,&MyWindow::onRestartSimulation);

    environmentSetupButton = new QPushButton("setup the environment");
    connect(environmentSetupButton,&QPushButton::clicked, this,&MyWindow::onEnvironmentSetup);

    QGridLayout* gridLayout = new QGridLayout();

    int buttonWidth = 250;
    int buttonHeight = 30;

    switchOffboardModeButton->setFixedSize(buttonWidth, buttonHeight);
    turnOffboardModeOffButton->setFixedSize(buttonWidth, buttonHeight);
    armButton->setFixedSize(buttonWidth, buttonHeight);
    takeoffButton->setFixedSize(buttonWidth, buttonHeight);
    landingButton->setFixedSize(buttonWidth, buttonHeight);
    controllerButton->setFixedSize(buttonWidth, buttonHeight);
    restartSimulationButton->setFixedSize(buttonWidth, buttonHeight);
    environmentSetupButton->setFixedSize(buttonWidth, buttonHeight);

    gridLayout->addWidget(restartSimulationButton, 1, 0);
    gridLayout->addWidget(environmentSetupButton, 0, 0);
    gridLayout->addWidget(armButton, 2, 0);
    gridLayout->addWidget(takeoffButton, 3, 0);

    gridLayout->addWidget(switchOffboardModeButton, 0, 1);
    gridLayout->addWidget(turnOffboardModeOffButton, 1,1);
    gridLayout->addWidget(controllerButton, 2, 1);
    gridLayout->addWidget(landingButton, 3, 1);

    rightLayout->addLayout(gridLayout);

    dronePoseLabel = new QLabel(centralWidget);
    loadImuLabel = new QLabel(centralWidget);
    loadAngleLabel = new QLabel(centralWidget);
    droneVelocityLabel = new QLabel(centralWidget);
    QGridLayout* labels = new QGridLayout();
    labels->addWidget(dronePoseLabel, 0, 0, Qt::AlignCenter);
    labels->addWidget(loadImuLabel, 0, 1, Qt::AlignCenter); 
    labels->addWidget(droneVelocityLabel, 1, 0, Qt::AlignCenter);
    labels->addWidget(loadAngleLabel, 1, 1, Qt::AlignCenter);
    rightLayout->addLayout(labels);



    graphSetup();
    mainLayout->addLayout(rightLayout, 4);

    QMenuBar* menuBar = setupMenuBar();
    setMenuBar(menuBar);
    setCentralWidget(centralWidget);
    resize(1500, 750);
    statusMessageLabel = new QLabel(this);
    statusMessageLabel->setStyleSheet("background-color: lightgrey; border-radius: 5px; padding: 2px 5px;");
    statusMessageLabel->setAlignment(Qt::AlignCenter);
    statusMessageLabel->hide();

    connect(connectionManager, &ConnectionManager::dronePoseReceived, this, &MyWindow::updateDronePose);
    connect(connectionManager, &ConnectionManager::loadImuReceived, this, &MyWindow::updateLoadImu);
    connect(connectionManager, &ConnectionManager::loadAngleReceived, this, &MyWindow::updateLoadAngle);
    connect(connectionManager, &ConnectionManager::droneVelocityReceived, this, &MyWindow::updateDroneVelocity);
    connect(connectionManager, &ConnectionManager::connectionStatusChanged, this, &MyWindow::updateConnectionIndicator);
    connect(connectionManager, &ConnectionManager::stateReceived, this, &MyWindow::updateStateIndicator);

   //spinning node to update data every 1 sec
    timer1 = new QTimer(this);
    connect(timer1, &QTimer::timeout, [this]() {
        rclcpp::spin_some(this->node);
    });
    connect(timer1, &QTimer::timeout, this, &MyWindow::updateGraph);
    connect(timer1,&QTimer::timeout, this, &MyWindow::updateDataTable);
    timer1->start(1000);

}

MyWindow::~MyWindow()
{
    rclcpp::shutdown();
}

void MyWindow::updateDronePose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
{
    drone_pose_x = msg->pose.position.x;
    drone_pose_y = msg->pose.position.y;
    drone_pose_z = msg->pose.position.z;
}

void MyWindow::updateLoadImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
{
    load_angular_velocity_x = msg->angular_velocity.x;
    load_angular_velocity_y = msg->angular_velocity.y;
    load_angular_velocity_z = msg->angular_velocity.z;
}

void MyWindow::updateLoadAngle(const angle_stamped_msg::msg::AngleStamped::ConstSharedPtr& msg)
{
    load_angle_y = msg->angle.angle_y;
    load_angle_z = msg->angle.angle_z;
    load_angle_x = msg->angle.angle_x;

}

void MyWindow::updateDroneVelocity(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg)
{
    drone_velocity_x = msg->twist.linear.x;
    drone_velocity_y = msg->twist.linear.y;
    drone_velocity_z = msg->twist.linear.z;
}

void MyWindow::updateGraph() {
    droneVelocitySetX->replace(0, drone_velocity_x);
    droneVelocitySetY->replace(0, drone_velocity_y);
    droneVelocitySetZ->replace(0, drone_velocity_z);
    loadAngularVelocitySetX->replace(0, load_angular_velocity_x);
    loadAngularVelocitySetY->replace(0, load_angular_velocity_y);
    loadAngularVelocitySetZ->replace(0, load_angular_velocity_z);
}

void MyWindow::updateDataTable(){
    if(isConnected){
QString position;
position.sprintf("Drone position: (%.3f, %.3f, %.3f)", drone_pose_x, drone_pose_y, drone_pose_z);
dronePoseLabel->setText(position);

QString drone_velocity;
drone_velocity.sprintf("Drone velocity: (%.3f, %.3f, %.3f)", drone_velocity_x, drone_velocity_y, drone_velocity_z);
droneVelocityLabel->setText(drone_velocity);

QString load_angle;
load_angle.sprintf("Load angle: (%.3f, %.3f, %.3f)", load_angle_x, load_angle_y, load_angle_z);
loadAngleLabel->setText(load_angle);

QString imu;
imu.sprintf("Load angular velocity: (%.3f, %.3f, %.3f)", load_angular_velocity_x, load_angular_velocity_y, load_angular_velocity_z);
loadImuLabel->setText(imu);

    }
    else{
    dronePoseLabel->setText("Drone position: Data unavailable");
    loadImuLabel->setText("Load angular velocity: Data unavailable");
    loadAngleLabel->setText("Load angle: Data unavailable");
    droneVelocityLabel->setText("Drone velocity: Data unavailable");
    }
}

void MyWindow::graphSetup(){
    droneVelocitySetX = new QtCharts::QBarSet("drone's velocity x");
    droneVelocitySetY = new QtCharts::QBarSet("drone's velocity y");
    droneVelocitySetZ = new QtCharts::QBarSet("drone's velocity z");
    //init with some default value
    *droneVelocitySetX << 1;
    *droneVelocitySetY << 1;
    *droneVelocitySetZ << 1;
    
    QtCharts::QBarSeries* droneSeries = new QtCharts::QBarSeries();
    droneSeries->append(droneVelocitySetX);
    droneSeries->append(droneVelocitySetY);
    droneSeries->append(droneVelocitySetZ);
    droneSeries->setBarWidth(0.2);

    QtCharts::QChart* droneChart = new QtCharts::QChart();
    droneChart->addSeries(droneSeries);
    setupAxis(droneChart, droneSeries, "Velocity (m/s)",-10,10);
    
    droneVelocityGraph = new QtCharts::QChartView(droneChart);
    droneVelocityGraph->setRenderHint(QPainter::Antialiasing);
    
    loadAngularVelocitySetX = new QtCharts::QBarSet("load's angular velocity x");
    loadAngularVelocitySetY = new QtCharts::QBarSet("load's angular velocity y");
    loadAngularVelocitySetZ = new QtCharts::QBarSet("load's angular velocity z");

    //init with some default value
    *loadAngularVelocitySetX << 1;
    *loadAngularVelocitySetY << 1;
    *loadAngularVelocitySetZ << 1;
    
    QtCharts::QBarSeries* loadSeries = new QtCharts::QBarSeries();
    loadSeries->append(loadAngularVelocitySetX);
    loadSeries->append(loadAngularVelocitySetY);
    loadSeries->append(loadAngularVelocitySetZ);
    loadSeries->setBarWidth(0.2);

    QtCharts::QChart* loadChart = new QtCharts::QChart();
    loadChart->addSeries(loadSeries);
    setupAxis(loadChart, loadSeries, "Angular Velocity (rad/s)",-3,3);
    
    loadAngularVelocityGraph = new QtCharts::QChartView(loadChart);
    loadAngularVelocityGraph->setRenderHint(QPainter::Antialiasing);
    
    rightLayout->addWidget(droneVelocityGraph);
    rightLayout->addWidget(loadAngularVelocityGraph);
}

void MyWindow::setupAxis(QtCharts::QChart* chart, QtCharts::QBarSeries* series, const QString &AxisText, qreal rangeStart, qreal rangeEnd){
    QtCharts::QValueAxis* axisX = new QtCharts::QValueAxis;
    axisX->setGridLineVisible(true);
    axisX->setTitleText("");
    axisX->setLabelsVisible(false);
    chart->addAxis(axisX, Qt::AlignBottom);
    series->attachAxis(axisX);

    QtCharts::QValueAxis* axisY = new QtCharts::QValueAxis;
    axisY->setTitleText(AxisText);
    axisY->setGridLineVisible(true);
    axisY->setRange(rangeStart, rangeEnd);
    chart->addAxis(axisY, Qt::AlignLeft);
    series->attachAxis(axisY);
}

void MyWindow::updateConnectionIndicator(bool connected) {
    QString color = connected ? "green" : "red";
    connectionIndicator->setStyleSheet("background-color: " + color + "; color: white");
    connectionIndicator->setText(connected ? "Connected" : "Disconnected");
    isConnected = connected;
    if(!connected) updateStateIndicator("unknown");
    buttonManager();
}

void MyWindow::updateStateIndicator(std::string mode) {
    QString qmode = QString::fromStdString(mode);
    QString color = isConnected ? "green" : "red";
    stateIndicator->setStyleSheet("background-color: " + color + "; color: white");
    stateIndicator->setText(qmode);
}


void MyWindow::onSwitchToOffboardMode()
{
    if(isArmed){
    if (connectionManager->switchToOffboardMode())
    {
        switchOffboardModeButton->setDisabled(true);
        turnOffboardModeOffButton->setDisabled(false);
        showStatusMessage("Successfully switched to offboard mode.");
    }
    else
    {
        showStatusMessage("Failed to switch to offboard mode");
    }
    }
    else{
        QMessageBox msgBox;
        msgBox.setWindowTitle("Offboard mode issue");
        msgBox.setText("Drone needs to be armed first");
        msgBox.exec();
    }
}

void MyWindow::turnOffboardModeOff(){
    if (connectionManager->switchTheOffboardModeOff())
    {
        turnOffTheController();
        switchOffboardModeButton->setDisabled(false);
        turnOffboardModeOffButton->setDisabled(true);
        showStatusMessage("Successfully switched off the offboard mode.");
    }
    else
    {
        showStatusMessage("Failed to switch off the offboard mode.");
    }
}

void MyWindow::onSwitchToArmedMode()
{
    if (connectionManager->switchToArmedMode())
    {   
        showStatusMessage("Successfully switched to armed mode.");
        isArmed = true;
        controllerButton->setDisabled(false);
        landingButton->setDisabled(false);
        takeoffButton->setDisabled(false);
    }
    else
    {
        showStatusMessage("Failed to switch to armed mode.");
    }
}

QMenuBar* MyWindow::setupMenuBar()
{
    QMenuBar* menuBar = new QMenuBar(this);
    stateIndicator->setAutoFillBackground(true);
    stateIndicator->setMinimumSize(150, 30);
    stateIndicator->setStyleSheet("background-color: gray; color: white;");
    stateIndicator->setText("unknown");
    stateIndicator->setAlignment(Qt::AlignCenter);

    connectionIndicator->setAutoFillBackground(true);
    connectionIndicator->setMinimumSize(120, 30);
    connectionIndicator->setStyleSheet("background-color: gray; color: white;");
    connectionIndicator->setText("unknown");
    connectionIndicator->setAlignment(Qt::AlignCenter);

    QWidget* rightSpacer = new QWidget(menuBar);
    rightSpacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    QHBoxLayout* rightLayout = new QHBoxLayout(rightSpacer);
    rightLayout->addWidget(connectionIndicator);
    rightSpacer->setLayout(rightLayout);
    menuBar->setCornerWidget(rightSpacer, Qt::TopRightCorner);

    QWidget* leftSpacer = new QWidget(menuBar);
    leftSpacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    QHBoxLayout* leftLayout = new QHBoxLayout(leftSpacer);
    leftLayout->addWidget(stateIndicator);
    leftSpacer->setLayout(leftLayout);
    menuBar->setCornerWidget(leftSpacer, Qt::TopLeftCorner);

    return menuBar;
}

void MyWindow::onControllerStart(){
    QDialog *popup = new QDialog(this);
    popup->setWindowTitle("Controller settings");

    // Main layout
    QVBoxLayout *mainLayout = new QVBoxLayout(popup);
    QRegExpValidator *validator = new QRegExpValidator(QRegExp("^-?((100(\\.0{1,2})?)|([1-9]?\\d(\\.\\d{1,2})?))$"), this);

    // Setup for the left side (Circle)
    QGroupBox *leftGroup = new QGroupBox("Circle Settings", popup);
    QVBoxLayout *leftLayout = new QVBoxLayout();
    QRadioButton *leftRadio = new QRadioButton("Circle", leftGroup);
    QHBoxLayout *leftInputLayout = new QHBoxLayout();
    QLabel *radiusLabel = new QLabel("Radius [m]:", leftGroup);
    QLineEdit *radiusText = new QLineEdit();
    radiusText->setValidator(validator); 

    leftInputLayout->addWidget(radiusLabel);
    leftInputLayout->addWidget(radiusText);
    leftLayout->addWidget(leftRadio);
    leftLayout->addLayout(leftInputLayout);
    leftGroup->setLayout(leftLayout);

    // Setup for the right side (Waypoint)
    QGroupBox *rightGroup = new QGroupBox("Waypoint Settings", popup);
    QVBoxLayout *rightLayout = new QVBoxLayout();
    QRadioButton *rightRadio = new QRadioButton("Waypoint", rightGroup);
    QHBoxLayout *rightInput1Layout = new QHBoxLayout();
    QLabel *xCoordLabel = new QLabel("Coordinate x [m]:", rightGroup);
    QLineEdit *xCoordText = new QLineEdit();
    xCoordText->setValidator(validator);
    QHBoxLayout *rightInput2Layout = new QHBoxLayout();
    QLabel *yCoordLabel = new QLabel("Coordinate y [m]:", rightGroup);
    QLineEdit *yCoordText = new QLineEdit();
    yCoordText->setValidator(validator);

    rightInput1Layout->addWidget(xCoordLabel);
    rightInput1Layout->addWidget(xCoordText);
    rightInput2Layout->addWidget(yCoordLabel);
    rightInput2Layout->addWidget(yCoordText);
    rightLayout->addWidget(rightRadio);
    rightLayout->addLayout(rightInput1Layout);
    rightLayout->addLayout(rightInput2Layout);
    rightGroup->setLayout(rightLayout);

    mainLayout->addWidget(leftGroup);
    mainLayout->addWidget(rightGroup);

    // Buttons layout
    QHBoxLayout *buttonsLayout = new QHBoxLayout();
    QPushButton *runButton = new QPushButton("Run", popup);
    QPushButton *cancelButton = new QPushButton("Cancel", popup);
    buttonsLayout->addWidget(runButton);
    buttonsLayout->addWidget(cancelButton);
    mainLayout->addLayout(buttonsLayout); // Add buttons under the sections

    // Ensure mutual exclusivity of radio buttons
    QButtonGroup *buttonGroup = new QButtonGroup(popup);
    buttonGroup->addButton(leftRadio);
    buttonGroup->addButton(rightRadio);

    // Initially disable text fields
    radiusText->setEnabled(false);
    xCoordText->setEnabled(false);
    yCoordText->setEnabled(false);

    // Enable text fields when the corresponding radio button is selected
    connect(leftRadio, &QRadioButton::toggled, radiusText, &QLineEdit::setEnabled);
    connect(rightRadio, &QRadioButton::toggled, [xCoordText, yCoordText](bool checked){
        xCoordText->setEnabled(checked);
        yCoordText->setEnabled(checked);
    });

    // Button click actions
    connect(runButton, &QPushButton::clicked, [this, popup, radiusText, leftRadio, xCoordText, yCoordText](){
        QString command;
if(leftRadio->isChecked()) {
    double radius = radiusText->text().toDouble(); // Convert text to double
    command = QString("ros2 launch lqr_controller lqr_controller_launch.py trajectory_type:=circle radius:=%1").arg(QString::number(radius, 'f', 1)); // Convert back to QString with one decimal place
    QString message = QString("Successfully started controller with circle trajectory of radius %1.").arg(radius, 0, 'f', 2);
    showStatusMessage(message);
} else {
    double xCoord = xCoordText->text().toDouble(); // Convert x coordinate text to double
    double yCoord = yCoordText->text().toDouble(); // Convert y coordinate text to double
    command = QString("ros2 launch lqr_controller lqr_controller_launch.py trajectory_type:=waypoint waypoint_x:=%1 waypoint_y:=%2")
                .arg(QString::number(xCoord, 'f', 1)) // Convert back to QString with one decimal place
                .arg(QString::number(yCoord, 'f', 1)); // Convert back to QString with one decimal place
    QString message = QString("Successfully started controller with waypoint trajectory of coordinates [%1,%2].")
                  .arg(xCoord, 0, 'f', 2)
                  .arg(yCoord, 0, 'f', 2);
    showStatusMessage(message);
}
        QProcess::startDetached("gnome-terminal", QStringList() << "--" << "/bin/bash" << "-c" << command);
        popup->accept(); // Close the popup after running the command
    });

    connect(cancelButton, &QPushButton::clicked, popup, &QDialog::reject);

    popup->setLayout(mainLayout);
    popup->exec();
}

void MyWindow::turnOffTheController() {
    QProcess process;
    process.start("bash", QStringList() << "-c" << "ps aux | grep lqr_controller | grep -v grep | awk '{print $2}'");

    if (!process.waitForFinished()) {
        qDebug() << "Failed to execute command";
        return;
    }

    QString output = process.readAllStandardOutput().trimmed();

    QStringList pids = output.split(QRegExp("[\r\n]"), QString::SkipEmptyParts);

    for (const QString& pid : pids) {
        bool ok;
        int pidNum = pid.toInt(&ok);
        if (ok) {
            qDebug() << "Killing PID:" << pidNum;
            QProcess::execute("kill", QStringList() << QString::number(pidNum));
        } else {
            qDebug() << "Invalid PID:" << pid;
        }
    }
}


void MyWindow::handleProcessError(){
    QMessageBox msgBox;
    msgBox.setWindowTitle("Controller node issue");
    msgBox.setText("controller node can not be loaded");
    msgBox.exec();
}

void MyWindow::onEnvironmentSetup(){
    QProcess *process = new QProcess(this);
    QString scriptPath = ("/home/luky/mavros_ros2_ws/scripts/test.sh");
    QStringList arguments;
    arguments << "--" << "bash" << "-c" << scriptPath;
    process->start("gnome-terminal", arguments);
    connect(process, &QProcess::errorOccurred, this, &MyWindow::handleScriptExecutionError);
    connect(process, static_cast<void(QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished), this, &MyWindow::onProcessFinished); //QProcess::finished has 2 overloads. The correct one needs to be specified
    showStatusMessage("Environment setup started successfully.");
}
void MyWindow::onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus){
    qDebug() << exitCode << " + " << exitStatus;
    if(exitCode ==0 && exitStatus == QProcess::NormalExit){
        armButton->setDisabled(false);
    }
}

void MyWindow::handleScriptExecutionError(){
    QMessageBox msgBox;
    msgBox.setWindowTitle("Environment setup issue");
    msgBox.setText("script could not be executed");
    msgBox.exec();
}

void MyWindow::onTakeOffMode(){
    if (connectionManager->takeOffMode())
    {
        showStatusMessage("Successfully switched to TakeOff mode.");
        isArmed = true;
        switchOffboardModeButton->setDisabled(false);
        landingButton->setDisabled(false);
        controllerButton->setDisabled(false);
    }
    else
    {
        showStatusMessage("Failed to switched to TakeOff mode.");
    }   
}

void MyWindow::onLandingMode(){
    if (connectionManager->droneLanding())
    {
        controllerButton->setDisabled(true);
        landingButton->setDisabled(true);
        armButton->setDisabled(false);
        showStatusMessage("Successfully switched to landing mode.");
    }
    else
    {
        showStatusMessage("Failed to switched to landing mode.");
    } 
}
void MyWindow::buttonManager(){
    if(!isConnected){
        environmentSetupButton->setDisabled(false);
        controllerButton->setDisabled(true);
        switchOffboardModeButton->setDisabled(true);
        turnOffboardModeOffButton->setDisabled(true);
        landingButton->setDisabled(true);
        takeoffButton->setDisabled(true);
        armButton->setDisabled(true);
    }
    else{
        armButton->setDisabled(false);
        environmentSetupButton->setDisabled(true);
    }
}

void MyWindow::onRestartSimulation(){
    QProcess *process = new QProcess(this);
    QString scriptPath = "/home/luky/mavros_ros2_ws/scripts/restartSimulation.sh";
    process->start("/bin/bash", QStringList() << scriptPath);
    showStatusMessage("Successfully restarted simulation.");
}

void MyWindow::showStatusMessage(const QString &message, int timeoutMs) {
    statusMessageLabel->setText(message);
    statusMessageLabel->adjustSize(); 
    statusMessageLabel->move(10, this->height() - statusMessageLabel->height() - 10);

    if (message.contains("Failed")) {
        // Set text color to white and background color to darker red
        statusMessageLabel->setStyleSheet("QLabel { color: #ffffff; background-color: #8B0000; }");
    } else {
        // Set text color to white and background color to darker green
        statusMessageLabel->setStyleSheet("QLabel { color: #ffffff; background-color: #006400; }");
    }

    statusMessageLabel->show();
    QTimer::singleShot(timeoutMs, statusMessageLabel, &QLabel::hide);
}



void MyWindow::closeEvent(QCloseEvent *event) {
    QProcess *process = new QProcess(this);
    QString scriptPath = "/home/luky/mavros_ros2_ws/scripts/clean_up_script.sh";
    process->start("/bin/bash", QStringList() << scriptPath);
    process->waitForFinished(-1);
    QMainWindow::closeEvent(event);
    std::ofstream ofs;
    ofs.open("/home/luky/mavros_ros2_ws/scripts/px4_log.txt", std::ofstream::out | std::ofstream::trunc);
    ofs.close();   
}