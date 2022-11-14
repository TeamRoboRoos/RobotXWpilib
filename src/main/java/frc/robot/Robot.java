// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.PropulsionModule.PROPULSION_STATE;
import frc.robot.subsystems.ShooterCanForComms;
import frc.robot.subsystems.ShooterCanForReal;
import frc.robot.subsystems.WeatherCan;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs
 * the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  AHRS ahrs;

  private WeatherCan weatherCan;


  // private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  // private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);
  // private final DifferentialDrive m_robotDrive = new
  // DifferentialDrive(m_leftMotor, m_rightMotor);
  private final Joystick m_stick = new Joystick(0);
  private final Joystick m_throtle = new Joystick(1);

  private final int ESTOP_ID = 0;

  Servo exampleServo = new Servo(9);

  int servoPosition = 0;
  int servoDirecion = 1;

  DigitalOutput redLight = new DigitalOutput(6);
  DigitalOutput greenLight = new DigitalOutput(7);
  DigitalOutput amberLight = new DigitalOutput(8);

  DigitalInput eStop = new DigitalInput(ESTOP_ID);

  // private final PropulsionModule starbFwd = new PropulsionModule(10, 7);

  // private final PropulsionModule portAft = new PropulsionModule(9, 5);
  
  // private final PropulsionModule starbAft = new PropulsionModule(11, 8);

  // private final PropulsionModule portFwd = new PropulsionModule(12, 6);

  // flip left/right for required movement
  private final PropulsionModule starbFwd = new PropulsionModule(12, 6);

  private final PropulsionModule portAft = new PropulsionModule(11, 8);
  
  private final PropulsionModule starbAft = new PropulsionModule(9, 5);

  private final PropulsionModule portFwd = new PropulsionModule(10, 7);

  // private final Drivebase drivebase = new Drivebase(new Translation2d(1.10, 0.6), new Translation2d(1.10, -0.6),
  //     new Translation2d(-2, 1), new Translation2d(-2, -1), portFwd, starbFwd, portAft, starbAft);

      // private final Drivebase drivebase = new Drivebase(
      //   new Translation2d(1.10, 0.6),   //portFwd
      //   new Translation2d(1.10, -0.6),  //starbFwd
      //   new Translation2d(-2, 1),       //portAft
      //   new Translation2d(-2, -1),      //starbAft
      //   portFwd, starbFwd, portAft, starbAft);

        private final Drivebase drivebase = new Drivebase(
          new Translation2d(1, 1),   //portFwd
          new Translation2d(1, -1),  //starbFwd
          new Translation2d(-1, 1),       //portAft
          new Translation2d(-1, -1),      //starbAft
          portFwd, starbFwd, portAft, starbAft);

        // new Translation2d(-0.6, 1.10), //portFwd
        // new Translation2d(0.6, 1.10),  //starbFwd
        // new Translation2d(-1, -2),     //portAft
        // new Translation2d(1, -2),      //starbAft

  private final NavCAN navCAN = new NavCAN(20); // XXX Nav CAN Device ID

  private ShooterCanForComms shooterCan;
  private ShooterCanForReal shooter;

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // m_rightMotor.setInverted(true);

    this.shooter = new ShooterCanForReal();
    this.shooterCan = new ShooterCanForComms(this.drivebase, this.shooter);

    CameraServer.startAutomaticCapture();

    this.redLight.set(false);
    this.greenLight.set(false);
    this.amberLight.set(false);

    try {
      /* Communicate w/navX MXP via the MXP SPI Bus. */
      /* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
      /*
       * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
       * details.
       */
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      //System.out.println("Error instantiating navX MXP:  " + ex.getMessage());
    }

    this.weatherCan = new WeatherCan();
  }

  @Override
  public void robotPeriodic() {
    this.redLight.set(this.eStop.get());
    
    navCAN.refresh_nav_data();

    NavCAN.TelemetryData telemetryData = navCAN.getTelemetryData();
    SmartDashboard.putNumber("Nav/Telemetry_Lat", telemetryData.telem_lat);
    SmartDashboard.putNumber("Nav/Telemetry_Lon", telemetryData.telem_lon);
    SmartDashboard.putNumber("Nav/Telemetry_Heading", telemetryData.telem_hdg);
    SmartDashboard.putNumber("Nav/Telemetry_Speed", telemetryData.telem_spd);

    NavCAN.NavData navData = navCAN.getNavData();
    SmartDashboard.putNumber("Nav/Navigation_Cur_Heading", navData.cur_hdg);
    SmartDashboard.putNumber("Nav/Navigation_Wpt_Heading", navData.wpt_hdg);
    SmartDashboard.putNumber("Nav/Navigation_Wpt_Dst", navData.wpt_dst);
    SmartDashboard.putNumber("Nav/Navigation_X_Spd", navData.x_spd);
    SmartDashboard.putNumber("Nav/Navigation_Y_Spd", navData.y_spd);
    SmartDashboard.putNumber("Nav/Navigation_Z_Spd", navData.z_spd);
    SmartDashboard.putNumber("Nav/Navigation_Speed_Override", navData.spd_ovr);
    SmartDashboard.putNumber("Nav/Navigation_Waypoint_Fin", navData.wpt_fin);

    /* Display 6-axis Processed Angle Data */
    //SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
    SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
    //SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
    //SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
    //SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());

    /* Display tilt-corrected, Magnetometer-based heading (requires */
    /* magnetometer calibration to be useful) */

    SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());

    /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
    //SmartDashboard.putNumber("IMU_FusedHeading", ahrs.getFusedHeading());

    /* These functions are compatible w/the WPI Gyro Class, providing a simple */
    /* path for upgrading from the Kit-of-Parts gyro to the navx MXP */

    //SmartDashboard.putNumber("IMU_TotalYaw", ahrs.getAngle());
    //SmartDashboard.putNumber("IMU_YawRateDPS", ahrs.getRate());

    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

    //SmartDashboard.putNumber("IMU_Accel_X", ahrs.getWorldLinearAccelX());
    //SmartDashboard.putNumber("IMU_Accel_Y", ahrs.getWorldLinearAccelY());
    //SmartDashboard.putBoolean("IMU_IsMoving", ahrs.isMoving());
    //SmartDashboard.putBoolean("IMU_IsRotating", ahrs.isRotating());

    /* Display estimates of velocity/displacement. Note that these values are */
    /* not expected to be accurate enough for estimating robot position on a */
    /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
    /* of these errors due to single (velocity) integration and especially */
    /* double (displacement) integration. */

    //SmartDashboard.putNumber("Velocity_X", ahrs.getVelocityX());
    //SmartDashboard.putNumber("Velocity_Y", ahrs.getVelocityY());
    //SmartDashboard.putNumber("Displacement_X", ahrs.getDisplacementX());
    //SmartDashboard.putNumber("Displacement_Y", ahrs.getDisplacementY());

    /* Display Raw Gyro/Accelerometer/Magnetometer Values */
    /* NOTE: These values are not normally necessary, but are made available */
    /* for advanced users. Before using this data, please consider whether */
    /* the processed data (see above) will suit your needs. */

    //SmartDashboard.putNumber("RawGyro_X", ahrs.getRawGyroX());
    //SmartDashboard.putNumber("RawGyro_Y", ahrs.getRawGyroY());
    //SmartDashboard.putNumber("RawGyro_Z", ahrs.getRawGyroZ());
    //SmartDashboard.putNumber("RawAccel_X", ahrs.getRawAccelX());
    //SmartDashboard.putNumber("RawAccel_Y", ahrs.getRawAccelY());
    //SmartDashboard.putNumber("RawAccel_Z", ahrs.getRawAccelZ());
    //SmartDashboard.putNumber("RawMag_X", ahrs.getRawMagX());
    //SmartDashboard.putNumber("RawMag_Y", ahrs.getRawMagY());
    //SmartDashboard.putNumber("RawMag_Z", ahrs.getRawMagZ());
    //SmartDashboard.putNumber("IMU_Temp_C", ahrs.getTempC());

    /* Omnimount Yaw Axis Information */
    /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
    //AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
    //SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
    //SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());

    /* Sensor Board Information */
    //SmartDashboard.putString("FirmwareVersion", ahrs.getFirmwareVersion());

    /* Quaternion Data */
    /* Quaternions are fascinating, and are the most compact representation of */
    /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
    /* from the Quaternions. If interested in motion processing, knowledge of */
    /* Quaternions is highly recommended. */
    //SmartDashboard.putNumber("QuaternionW", ahrs.getQuaternionW());
    //SmartDashboard.putNumber("QuaternionX", ahrs.getQuaternionX());
    //SmartDashboard.putNumber("QuaternionY", ahrs.getQuaternionY());
    //SmartDashboard.putNumber("QuaternionZ", ahrs.getQuaternionZ());

    /* Sensor Data Timestamp */
    //SmartDashboard.putNumber("SensorTimestamp", ahrs.getLastSensorTimestamp());

    /* Connectivity Debugging Support */
    //SmartDashboard.putNumber("IMU_Byte_Count", ahrs.getByteCount());
    //SmartDashboard.putNumber("IMU_Update_Count", ahrs.getUpdateCount());


    this.weatherCan.update();
    this.weatherCan.displayData();
    this.shooterCan.update();
    this.shooterCan.displayData();
  }

  @Override
  public void disabledPeriodic() {
    this.greenLight.set(false);
    this.amberLight.set(false);

    NavCAN.NavData navData = navCAN.getNavData();
    System.out.println(navData.toString());
  }

  @Override
  public void autonomousPeriodic() {
    this.greenLight.set(true);
    this.amberLight.set(false);
/*
    NavCAN.NavData navData = navCAN.getNavData(); // TODO Alan+Francis here
    if (navData.spd_ovr == 1)
      this.drivebase.drive(new ChassisSpeeds(navData.y_spd, navData.x_spd, navData.z_spd), 1);
    else if (navData.spd_ovr == 0)
      this.drivebase.drive(WaypointSeeker.seek(navData), 1);
*/
    this.drivebase.update();
  }

  @Override
  public void teleopPeriodic() {

    this.greenLight.set(false);
    this.amberLight.set(true);

    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    // m_robotDrive.arcadeDrive(-m_stick.getY(), m_stick.getX());

    double deg = 0;

    double deltaX = m_stick.getX();
    double deltaY = -m_stick.getY();
    double rad = Math.atan2(deltaY, deltaX);
    deg = ((rad * (180 / Math.PI)) * -1) + 90;
    deg = (deg + 360) % 360;

    if (Math.abs(m_stick.getX()) < 0.05 && Math.abs(m_stick.getY()) < 0.05 && Math.abs(m_stick.getRawAxis(4)) < 0.05) {
      deg = 0;
    }

    double power = this.m_throtle.getX();
/*
    System.out.println("" + power);

    System.out.println("" + m_stick.getX() + "x" + m_stick.getY() + " - " + deg);
    System.out.println("" + deg);
    System.out.println("" + power);
*/
    if (m_stick.getRawButtonPressed(2)) {
      System.out.println("Initialising");
      // this.drivebase.setState(PROPULSION_STATE.UNINITIALISED);
      this.drivebase.initialise();
    }

    if (m_stick.getRawButtonPressed(1)) {
      System.out.println("Shooter Enabled");
      this.shooterCan.setEnabled(true);
    }

    if (this.drivebase.isState(PROPULSION_STATE.STOPPED) || this.drivebase.isState(PROPULSION_STATE.DRIVING)) {
      this.drivebase.drive(new ChassisSpeeds(-m_stick.getY(), m_stick.getX(), m_stick.getRawAxis(4)),
          (m_throtle.getX() + 1) / 2);
    }

    // System.out.println(m_stick.getRawAxis(4));

    servoPosition += servoDirecion;
    if (servoPosition == 180)
      servoDirecion = -servoDirecion;
    exampleServo.setAngle(servoPosition);

    this.drivebase.update();
  }
}
