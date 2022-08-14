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
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.PropulsionModule.PROPULSION_STATE;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  AHRS ahrs;

  //private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  //private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);
  //private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
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

  private final PropulsionModule starbFwd = new PropulsionModule(10, 7);
  private final PropulsionModule portAft = new PropulsionModule(9, 5);
  private final PropulsionModule starbAft = new PropulsionModule(11, 8);
  private final PropulsionModule portFwd = new PropulsionModule(12, 6);
  private final Drivebase drivebase = new Drivebase(new Translation2d(1.10, 0.6), new Translation2d(1.10, -0.6), new Translation2d(-2, 1), new Translation2d(-2, -1), portFwd, starbFwd, portAft, starbAft);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_rightMotor.setInverted(true);

    CameraServer.startAutomaticCapture();

    this.redLight.set(false);
    this.greenLight.set(false);
    this.amberLight.set(false);

    try {
      /* Communicate w/navX MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex ) {
        System.out.println("Error instantiating navX MXP:  " + ex.getMessage());
    }
  }

  @Override 
  public void robotPeriodic() {
    this.redLight.set(this.eStop.get());

    /* Display 6-axis Processed Angle Data                                      */
    SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
    SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
    SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
    SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
    SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
    
    /* Display tilt-corrected, Magnetometer-based heading (requires             */
    /* magnetometer calibration to be useful)                                   */
    
    SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());
    
    /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
    SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

    /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
    /* path for upgrading from the Kit-of-Parts gyro to the navx MXP            */
    
    SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
    SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());

    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
    
    SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
    SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
    SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
    SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());

    /* Display estimates of velocity/displacement.  Note that these values are  */
    /* not expected to be accurate enough for estimating robot position on a    */
    /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
    /* of these errors due to single (velocity) integration and especially      */
    /* double (displacement) integration.                                       */
    
    SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
    SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
    SmartDashboard.putNumber(   "Displacement_X",       ahrs.getDisplacementX());
    SmartDashboard.putNumber(   "Displacement_Y",       ahrs.getDisplacementY());
    
    /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
    /* NOTE:  These values are not normally necessary, but are made available   */
    /* for advanced users.  Before using this data, please consider whether     */
    /* the processed data (see above) will suit your needs.                     */
    
    SmartDashboard.putNumber(   "RawGyro_X",            ahrs.getRawGyroX());
    SmartDashboard.putNumber(   "RawGyro_Y",            ahrs.getRawGyroY());
    SmartDashboard.putNumber(   "RawGyro_Z",            ahrs.getRawGyroZ());
    SmartDashboard.putNumber(   "RawAccel_X",           ahrs.getRawAccelX());
    SmartDashboard.putNumber(   "RawAccel_Y",           ahrs.getRawAccelY());
    SmartDashboard.putNumber(   "RawAccel_Z",           ahrs.getRawAccelZ());
    SmartDashboard.putNumber(   "RawMag_X",             ahrs.getRawMagX());
    SmartDashboard.putNumber(   "RawMag_Y",             ahrs.getRawMagY());
    SmartDashboard.putNumber(   "RawMag_Z",             ahrs.getRawMagZ());
    SmartDashboard.putNumber(   "IMU_Temp_C",           ahrs.getTempC());
    
    /* Omnimount Yaw Axis Information                                           */
    /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
    AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
    SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
    SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
    
    /* Sensor Board Information                                                 */
    SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());
    
    /* Quaternion Data                                                          */
    /* Quaternions are fascinating, and are the most compact representation of  */
    /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
    /* from the Quaternions.  If interested in motion processing, knowledge of  */
    /* Quaternions is highly recommended.                                       */
    SmartDashboard.putNumber(   "QuaternionW",          ahrs.getQuaternionW());
    SmartDashboard.putNumber(   "QuaternionX",          ahrs.getQuaternionX());
    SmartDashboard.putNumber(   "QuaternionY",          ahrs.getQuaternionY());
    SmartDashboard.putNumber(   "QuaternionZ",          ahrs.getQuaternionZ());
    
    /* Sensor Data Timestamp */
    SmartDashboard.putNumber(   "SensorTimestamp",		ahrs.getLastSensorTimestamp());
    
    /* Connectivity Debugging Support                                           */
    SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
    SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());
  }

  @Override
  public void disabledPeriodic() {
    this.greenLight.set(false);
    this.amberLight.set(false);
  }

  @Override
  public void autonomousPeriodic() {
    this.greenLight.set(true);
    this.amberLight.set(false);
  }

  @Override
  public void teleopPeriodic() {

    this.greenLight.set(false);
    this.amberLight.set(true);

    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    //m_robotDrive.arcadeDrive(-m_stick.getY(), m_stick.getX());

    double deg = 0;

    double deltaX = m_stick.getX();
    double deltaY = -m_stick.getY();
    double rad = Math.atan2(deltaY, deltaX);
    deg = ((rad * (180 / Math.PI)) * -1) + 90;
    deg = (deg + 360) % 360;

    if (Math.abs(m_stick.getX()) < 0.05 && Math.abs(m_stick.getY()) < 0.05) {
      deg = 0;
    }

    //double power = this.m_throtle.getX();

    // System.out.println("" + power);

    //System.out.println("" + m_stick.getX() + "x" + m_stick.getY() + " - " + deg);
    //System.out.println("" + deg);
    // System.out.println("" + power);

    try {
      if (this.starbFwd.getState() == PROPULSION_STATE.UNINITIALISED) {
        this.starbFwd.setState(PROPULSION_STATE.INITIALISING);
      }
      if (this.starbFwd.getState() == PROPULSION_STATE.STOPPED) {
        // this.starbFwd.drive(power, deg);
      }
    } catch (Exception e) {
      System.out.println(e.getMessage());
    }

    try {
      if (this.portAft.getState() == PROPULSION_STATE.UNINITIALISED) {
        this.portAft.setState(PROPULSION_STATE.INITIALISING);
      }
      if (this.portAft.getState() == PROPULSION_STATE.STOPPED) {
        // this.portAft.drive(0, deg);
      }
    } catch (Exception e) {
      System.out.println(e.getMessage());
    }

    try {
      if (this.portFwd.getState() == PROPULSION_STATE.UNINITIALISED) {
        this.portFwd.setState(PROPULSION_STATE.INITIALISING);
      }
      if (this.portFwd.getState() == PROPULSION_STATE.STOPPED) {
        // this.portFwd.drive(0, deg);
      }
    } catch (Exception e) {
      System.out.println(e.getMessage());
    }
    
    try {
      if (this.starbAft.getState() == PROPULSION_STATE.UNINITIALISED) {
        this.starbAft.setState(PROPULSION_STATE.INITIALISING);
      }
      if (this.starbAft.getState() == PROPULSION_STATE.STOPPED) {
        // this.starbAft.drive(0, deg);
      }
    } catch (Exception e) {
      System.out.println(e.getMessage());
    }

    if (this.portFwd.getState() == PROPULSION_STATE.STOPPED &&
        this.starbFwd.getState() == PROPULSION_STATE.STOPPED &&
        this.portAft.getState() == PROPULSION_STATE.STOPPED &&
        this.starbAft.getState() == PROPULSION_STATE.STOPPED) {
          this.drivebase.drive(new ChassisSpeeds(m_stick.getY(), -m_stick.getX(), -m_stick.getRawAxis(4)), (m_throtle.getX()+1)/2);
    }    
    System.out.println(m_stick.getRawAxis(4));

    servoPosition += servoDirecion;
    if (servoPosition == 180) servoDirecion = -servoDirecion;
    exampleServo.setAngle(servoPosition);

    this.starbFwd.update();
    this.portAft.update();
    this.portFwd.update();
    this.starbAft.update();
  }
}
