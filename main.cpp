#include <QCoreApplication>
#include <QSerialPortInfo>
#include <QDateTime>
#include <QFile>
#include <signal.h>
#include <QProcess>
#include <QCommandLineParser>
#include <QJsonDocument>
#include <QJsonObject>
#include "WayWise/core/simplewatchdog.h"
#include "WayWise/vehicles/carstate.h"
#include "WayWise/vehicles/controller/carmovementcontroller.h"
#include "WayWise/sensors/imu/bno055orientationupdater.h"
#include "WayWise/sensors/gnss/ubloxrover.h"
#include "WayWise/autopilot/waypointfollower.h"
#include "WayWise/autopilot/purepursuitwaypointfollower.h"
#include "WayWise/vehicles/controller/vescmotorcontroller.h"
#include "WayWise/sensors/camera/depthaicamera.h"
#include "WayWise/sensors/fusion/sdvpvehiclepositionfuser.h"
#include "WayWise/sensors/gnss/rtcmclient.h"
#include "WayWise/communication/mavsdkvehicleserver.h"
#include "WayWise/communication/parameterserver.h"
#include "WayWise/logger/logger.h"
#include "WayWise/sensors/angle/as5600updater.h"
#include "WayWise/vehicles/truckstate.h"
#include "WayWise/vehicles/trailerstate.h"

static void terminationSignalHandler(int signal) {
    qDebug() << "Shutting down";
    if (signal==SIGINT || signal==SIGTERM || signal==SIGQUIT || signal==SIGHUP)
        qApp->quit();
}

// ----------------------------------------------------
// Config struct to hold all settings
// ----------------------------------------------------
struct TruckConfig {
    int truckId = 1;
    int trailerId = 25;
    bool attachTrailer = false;
    QString configPath;

    // Additional configurable parameters
    double truckLength = 0.5;
    double truckWidth = 0.21;
    double trailerLength = 0.96;
    double trailerWidth = 0.21;
    double trailerWheelBase = 0.64;
    double axisDistance = 0.3;
    double turnRadius = 0.67;
    double servoCenter = 0.5;
    double servoRange = 0.50;
    double angleSensorOffset = 94.043;
    double purePursuitRadius = 1.0;
    double speedToRPMFactor = 5190;
    bool adaptiveRadius = true;
    bool repeatRoute = false;
    bool useVescIMU = true;
    int updateVehicleStatePeriodMs = 25;
    QString controlTowerIP = "127.0.0.1";
    int controlTowerPort = 14540;
    QString rtcmInfoFile = "./rtcmServerInfo.txt";
};

// ----------------------------------------------------
// Helper functions
// ----------------------------------------------------
TruckConfig loadConfigFromJson(const QString &path)
{
    TruckConfig config;
    QFile file(path);
    if (file.exists() && file.open(QIODevice::ReadOnly)) {
        QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
        file.close();
        if (doc.isObject()) {
            QJsonObject obj = doc.object();
            config.truckId = obj.value("truck_id").toInt(config.truckId);
            config.trailerId = obj.value("trailer_id").toInt(config.trailerId);
            config.attachTrailer = obj.value("attach_trailer").toBool(config.attachTrailer);

            config.truckLength = obj.value("truck_length").toDouble(config.truckLength);
            config.truckWidth = obj.value("truck_width").toDouble(config.truckWidth);
            config.trailerLength = obj.value("trailer_length").toDouble(config.trailerLength);
            config.trailerWidth = obj.value("trailer_width").toDouble(config.trailerWidth);
            config.trailerWheelBase = obj.value("trailer_wheelbase").toDouble(config.trailerWheelBase);
            config.axisDistance = obj.value("axis_distance").toDouble(config.axisDistance);
            config.turnRadius = obj.value("turn_radius").toDouble(config.turnRadius);
            config.servoCenter = obj.value("servo_center").toDouble(config.servoCenter);
            config.servoRange = obj.value("servo_range").toDouble(config.servoRange);
            config.angleSensorOffset = obj.value("angle_sensor_offset").toDouble(config.angleSensorOffset);
            config.purePursuitRadius = obj.value("pure_pursuit_radius").toDouble(config.purePursuitRadius);
            config.speedToRPMFactor = obj.value("speed_to_rpm_factor").toDouble(config.speedToRPMFactor);
            config.adaptiveRadius = obj.value("adaptive_radius").toBool(config.adaptiveRadius);
            config.repeatRoute = obj.value("repeat_route").toBool(config.repeatRoute);
            config.useVescIMU = obj.value("use_vesc_imu").toBool(config.useVescIMU);
            config.updateVehicleStatePeriodMs = obj.value("update_period_ms").toInt(config.updateVehicleStatePeriodMs);
            config.controlTowerIP = obj.value("control_tower_ip").toString(config.controlTowerIP);
            config.controlTowerPort = obj.value("control_tower_port").toInt(config.controlTowerPort);
            config.rtcmInfoFile = obj.value("rtcm_info_file").toString(config.rtcmInfoFile);
        }
    }
    return config;
}

TruckConfig parseArguments(QCoreApplication &app)
{
    TruckConfig config;

    QCommandLineParser parser;
    parser.setApplicationDescription("RC Truck configuration");
    parser.addHelpOption();

    QCommandLineOption configFileOption({"c", "config"}, "Path to JSON config file.", "file");
    parser.addOption(configFileOption);

    QCommandLineOption truckIDOption({"i", "truck-id"}, "Truck ID.", "id", QString::number(config.truckId));
    parser.addOption(truckIDOption);

    QCommandLineOption trailerIDOption({"l", "trailer-id"}, "Trailer ID.", "id", QString::number(config.trailerId));
    parser.addOption(trailerIDOption);

    QCommandLineOption attachTrailerOption({"a", "attach-trailer"}, "Attach trailer to truck (flag).");
    parser.addOption(attachTrailerOption);

    parser.process(app);

    config.configPath = parser.value(configFileOption);
    if (!config.configPath.isEmpty())
        config = loadConfigFromJson(config.configPath);

    if (parser.isSet(truckIDOption))
        config.truckId = parser.value(truckIDOption).toInt();

    if (parser.isSet(trailerIDOption))
        config.trailerId = parser.value(trailerIDOption).toInt();

    if (parser.isSet(attachTrailerOption))
        config.attachTrailer = true;

    return config;
}


// ----------------------------------------------------
// Main
// ----------------------------------------------------
int main(int argc, char *argv[])
{
    Logger::initVehicle();

    QCoreApplication app(argc, argv);

    TruckConfig config = parseArguments(app);

    // --- Vehicle setup ---
    QTimer mUpdateVehicleStateTimer;
    QSharedPointer<TruckState> mTruckState = QSharedPointer<TruckState>::create(config.truckId);
    mTruckState->setLength(config.truckLength);
    mTruckState->setWidth(config.truckWidth);
    mTruckState->setAxisDistance(config.axisDistance);
    mTruckState->setMaxSteeringAngle(atan(config.axisDistance / config.turnRadius));

    QSharedPointer<TrailerState> mTrailerState;
    if (config.attachTrailer) {
        mTrailerState = QSharedPointer<TrailerState>::create(config.trailerId);
        mTrailerState->setWheelBase(config.trailerWheelBase);
        mTrailerState->setLength(config.trailerLength);
        mTrailerState->setWidth(config.trailerWidth);
        mTruckState->setTrailingVehicle(mTrailerState);
    }

    MavsdkVehicleServer mavsdkVehicleServer(mTruckState, QHostAddress(config.controlTowerIP), config.controlTowerPort);

    // --- Lower-level control setup ---
    QSharedPointer<CarMovementController> mCarMovementController(new CarMovementController(mTruckState));
    mCarMovementController->setSpeedToRPMFactor(config.speedToRPMFactor);
    // setup and connect VESC, simulate movements if unable to connect
    QSharedPointer<VESCMotorController> mVESCMotorController(new VESCMotorController());
    foreach(const QSerialPortInfo &portInfo, QSerialPortInfo::availablePorts()) {
        if (portInfo.description().toLower().replace("-", "").contains("chibios")) { // assumption: Serial device with ChibiOS in description is VESC
            mVESCMotorController->connectSerial(portInfo);
            qDebug() << "VESCMotorController connected to:" << portInfo.systemLocation();
        }
    }
    if (mVESCMotorController->isSerialConnected()) {
        mCarMovementController->setMotorController(mVESCMotorController);

        // VESC is a special case that can also control the servo
        const auto servoController = mVESCMotorController->getServoController();
        servoController->setInvertOutput(true);
        // NOTE: HEADSTART rc car (values read from sdvp pcb)
        servoController->setServoRange(config.servoRange);
        servoController->setServoCenter(config.servoCenter);
        mCarMovementController->setServoController(servoController);
    } else {
        QObject::connect(&mUpdateVehicleStateTimer, &QTimer::timeout, [&](){
            mCarMovementController->simulationStep(config.updateVehicleStatePeriodMs);
        });
        mUpdateVehicleStateTimer.start(config.updateVehicleStatePeriodMs);
    }

    // --- Positioning setup ---
    // Position Fuser
    SDVPVehiclePositionFuser positionFuser;

    // GNSS (with fused IMU when using u-blox F9R)
    QSharedPointer<GNSSReceiver> mGNSSReceiver;
    foreach(const QSerialPortInfo &portInfo, QSerialPortInfo::availablePorts()) {
        if (portInfo.manufacturer().toLower().replace("-", "").contains("ublox")) {
            QSharedPointer<UbloxRover> mUbloxRover(new UbloxRover(mTruckState));
            if (mUbloxRover->connectSerial(portInfo)) {
                qDebug() << "UbloxRover connected to:" << portInfo.systemLocation();

                mUbloxRover->setChipOrientationOffset(0.0, 0.0, 0.0);
                QObject::connect(mUbloxRover.get(), &UbloxRover::updatedGNSSPositionAndYaw, &positionFuser, &SDVPVehiclePositionFuser::correctPositionAndYawGNSS);

                mUbloxRover->setReceiverVariant(RECEIVER_VARIANT::UBLX_ZED_F9R); // or UBLX_ZED_F9P
                mavsdkVehicleServer.setUbloxRover(mUbloxRover);

                // -- NTRIP/TCP client setup for feeding RTCM data into GNSS receiver
                RtcmClient rtcmClient;
                QObject::connect(mUbloxRover.get(), &UbloxRover::gotNmeaGga, &rtcmClient, &RtcmClient::forwardNmeaGgaToServer);
                QObject::connect(&rtcmClient, &RtcmClient::rtcmData, mUbloxRover.get(), &UbloxRover::writeRtcmToUblox);
                QObject::connect(&rtcmClient, &RtcmClient::baseStationPosition, mUbloxRover.get(), &UbloxRover::setEnuRef);
                if (rtcmClient.connectWithInfoFromFile(config.rtcmInfoFile))
                    qDebug() << "RtcmClient: connected to" << QString(rtcmClient.getCurrentHost()+ ":" + QString::number(rtcmClient.getCurrentPort()));
                else
                    qDebug() << "RtcmClient: not connected";

                mGNSSReceiver = mUbloxRover;
            }
        }
    }

    // IMU
    QSharedPointer<IMUOrientationUpdater> mIMUOrientationUpdater;
    if (config.useVescIMU)
        mIMUOrientationUpdater = mVESCMotorController->getIMUOrientationUpdater(mTruckState);
    else
        mIMUOrientationUpdater.reset(new BNO055OrientationUpdater(mTruckState, "/dev/i2c-1"));
    QObject::connect(mIMUOrientationUpdater.get(), &IMUOrientationUpdater::updatedIMUOrientation, &positionFuser, &SDVPVehiclePositionFuser::correctPositionAndYawIMU);

    // Angle Sensor
    QSharedPointer<AngleSensorUpdater> mAngleSensorUpdater;
    mAngleSensorUpdater.reset(new AS5600Updater(mTruckState, config.angleSensorOffset));
    mTruckState->setSimulateTrailer(!mAngleSensorUpdater->isConnected());

    // Odometry
    QObject::connect(mCarMovementController.get(), &CarMovementController::updatedOdomPositionAndYaw, &positionFuser, &SDVPVehiclePositionFuser::correctPositionAndYawOdom);

    // --- Autopilot ---
    QSharedPointer<PurepursuitWaypointFollower> mWaypointFollower(new PurepursuitWaypointFollower(mCarMovementController));
    mWaypointFollower->setPurePursuitRadius(config.purePursuitRadius);
    mWaypointFollower->setRepeatRoute(config.repeatRoute);
    mWaypointFollower->setAdaptivePurePursuitRadiusActive(config.adaptiveRadius);

    // Setup MAVLINK communication towards ControlTower
    mavsdkVehicleServer.setMovementController(mCarMovementController);
    mavsdkVehicleServer.setWaypointFollower(mWaypointFollower);

    // Advertise parameters
    mTruckState->provideParametersToParameterServer();
    if (config.attachTrailer) {
        mTrailerState->provideParametersToParameterServer();
    }
    mavsdkVehicleServer.provideParametersToParameterServer();
    mWaypointFollower->provideParametersToParameterServer();

    // Watchdog that warns when EventLoop is slowed down
    SimpleWatchdog watchdog;

    // Perform safe shutdown
    signal(SIGINT, terminationSignalHandler);
    QObject::connect(&app, &QCoreApplication::aboutToQuit, [&](){
        mGNSSReceiver->aboutToShutdown();
        ParameterServer::getInstance()->saveParametersToXmlFile("vehicle_parameters.xml");
    });
    QObject::connect(&mavsdkVehicleServer, &MavsdkVehicleServer::shutdownOrRebootOnboardComputer, [&](bool isShutdown){
        qApp->quit();
        if (isShutdown) {
           qDebug() << "\nSystem shutdown...";
           QProcess::startDetached("sudo", QStringList() << "shutdown" << "-P" << "now");
        }else {
           qDebug() << "\nSystem reboot...";
           QProcess::startDetached("sudo", QStringList() << "shutdown" << "-r" << "now");
        }
    });

    qDebug() << "                    _________________________________________________";
    qDebug() << "            /|     |                                                 |";
    qDebug() << "            ||     |                                                 |";
    qDebug() << "       .----|-----,|                                                 |";
    qDebug() << "       ||  ||   ==||                                                 |";
    qDebug() << "  .-----'--'|   ==||                                                 |";
    qDebug() << "  |)-      ~|     ||_________________________________________________|";
    qDebug() << "  | ___     |     |____...==..._  >\\______________________________|";
    qDebug() << " [_/.-.\\---\\\\-----  //.-.  .-.\\\\    |/            \\\\ .-.  .-. //";
    qDebug() << "   ( o )   ===~~~~   ( o )( o )     o               ( o )( o )";
    qDebug() << "    '-'               '-'  '-'                       '-'  '-'\n";

    return app.exec();
}
