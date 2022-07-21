// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.PropulsionModule.PROPULSION_STATE;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {


  //private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  //private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);
  //private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final Joystick m_stick = new Joystick(0);
  private final Joystick m_throtle = new Joystick(1);

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
  }

  @Override
  public void teleopPeriodic() {
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

    double power = this.m_throtle.getX();

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
          this.drivebase.drive(new ChassisSpeeds(-m_stick.getY(), -m_stick.getX(), m_stick.getZ()), (m_throtle.getX()+1)/2);
    }    
    // System.out.println((m_throtle.getX()+1)/2);

    this.starbFwd.update();
    this.portAft.update();
    this.portFwd.update();
    this.starbAft.update();
  }
}
