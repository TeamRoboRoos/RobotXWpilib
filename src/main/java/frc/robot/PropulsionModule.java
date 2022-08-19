// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.LinkedList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class PropulsionModule {
    /**
     *
     */
    private int turnMotorCanID;
    private int driveMotorCanID;

    private CANSparkMax m_turningMotor;
    private SparkMaxPIDController m_turningPidController;
    private SparkMaxLimitSwitch m_leftLimit;
    private SparkMaxLimitSwitch m_rightLimit;

    private TalonSRX m_driveMotor;

    public enum PROPULSION_STATE {
        DEAD, INITIALISING, UNINITIALISED, STOPPED, DRIVING
    }

    private PROPULSION_STATE state;

    public enum DIRECTION {
        STOPPED, CLOCKWISE, ANTICLOCKWISE
    }

    private boolean leftInitialised;
    private boolean rightInitialised;

    private double minEncoderValue = 0;
    private double maxEncoderValue = 0;

    private double tolerance = 0.5;
    private double maxPower = 0.07;

    private LinkedList<Double> speeds;
    private double currentSpeed;
    private double speedSum;
    private final int averageNum = 100;

    // letf is positive
    // right is negative

    public PropulsionModule(int turnMotorCanID, int driveMotorCanID) {
        this.state = PROPULSION_STATE.UNINITIALISED;

        this.turnMotorCanID = turnMotorCanID;
        this.driveMotorCanID = driveMotorCanID;

        this.m_turningMotor = new CANSparkMax(this.turnMotorCanID, MotorType.kBrushless);
        this.m_turningMotor.restoreFactoryDefaults();
        this.m_turningPidController = m_turningMotor.getPIDController();
        this.m_turningPidController.setP(0.2);
        this.m_turningPidController.setI(0.000005);
        this.m_turningPidController.setD(1);
        this.m_turningPidController.setIZone(0);
        this.m_turningPidController.setFF(0);
        this.m_turningPidController.setOutputRange(-maxPower, maxPower);

        this.m_leftLimit = this.m_turningMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        this.m_rightLimit = this.m_turningMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        this.m_driveMotor = new TalonSRX(this.driveMotorCanID);
        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.peakCurrentLimit = 40; // the peak current, in amps
        config.peakCurrentDuration = 1500; // the time at the peak current before the limit triggers, in ms
        config.continuousCurrentLimit = 30; // the current to maintain if the peak limit is triggered
        this.m_driveMotor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder

        this.speeds = new LinkedList<Double>();
        this.currentSpeed = 0;
        this.speedSum = 0;
    }

    public boolean getLeftLimit() {
        return this.m_leftLimit.isPressed();
    }

    public boolean getRightLimit() {
        return this.m_rightLimit.isPressed();
    }

    public double getEncoderValue() {
        return this.m_turningMotor.getEncoder().getPosition();
    }

    public boolean drive(double power, double angle) {

        // Convert degrees to encoder positions

        if (angle > 90 && angle < 270) {
            power = -power;
            angle += 180;
        }

        double target = angle + 90;
        if (target > 360)
            target = target - 360;
        if (target == 360)
            target = 0;
        double degreeRatio = this.maxEncoderValue / 180.0;
        target = this.maxEncoderValue - (target * degreeRatio);

        // this.m_driveMotor.set(ControlMode.PercentOutput, power);
        this.currentSpeed = power;

        try {
            this.setState(PROPULSION_STATE.DRIVING);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        // if (angle > 90 && angle < 270) target = target - this.maxEncoderValue;

        // System.out.println("" + angle + " " + target + ":" +
        // this.m_turningMotor.getEncoder().getPosition());

        this.m_turningPidController.setReference(target, CANSparkMax.ControlType.kPosition);
        if (this.m_turningMotor.getEncoder().getPosition() < target + this.tolerance &&
                this.m_turningMotor.getEncoder().getPosition() > target - this.tolerance) {
            return true;
        }
        return false;
    }

    private void setPower(double power) {
        this.speeds.offer(power);
        this.speedSum += power;
        if (this.speeds.size() > 100) {
            this.speedSum -= this.speeds.poll();
        }
    }

    public boolean driveFromState(SwerveModuleState state, double thrust) {
        // System.out.println(this.turnMotorCanID + " " + state.angle.getDegrees());
        // Capped speed at 0.1 for testing purposes
        // Modulo may need to be modified because getDegrees may return value less than
        // -360
        return this.drive(Math.max(Math.min(state.speedMetersPerSecond * thrust, 0.1), -0.1),
                (state.angle.getDegrees() % 360 + 360) % 360);
    }

    public boolean rotateMotor(DIRECTION direction, double power) {
        // if (this.m_turningMotor.

        this.m_turningMotor.getPIDController().setReference(0, com.revrobotics.CANSparkMax.ControlType.kDutyCycle);

        if (direction == DIRECTION.CLOCKWISE) {
            power = -power;
            if (!this.getRightLimit()) {
                this.m_turningMotor.set(power);
                return true;
            } else {
                this.m_turningMotor.stopMotor();
                this.rightInitialised = true;
                this.m_turningMotor.getEncoder().setPosition(0);
                return false;
            }
        } else if (direction == DIRECTION.ANTICLOCKWISE) {
            if (!this.getLeftLimit()) {
                this.m_turningMotor.set(power);
                return true;
            } else {
                this.m_turningMotor.stopMotor();
                this.leftInitialised = true;
                this.maxEncoderValue = this.m_turningMotor.getEncoder().getPosition();
                return false;
            }
        }

        this.m_turningMotor.stopMotor();
        return false;
    }

    public PROPULSION_STATE getState() {
        return this.state;
    }

    /**
     * Changes the state of the propulsion module. The rule is that the propulsion
     * cannot be used unless it has been initalised - if it is unintialised the only
     * state that can be set is initialisation. Once initialised, other states are
     * options, although generally the only two states we will used are to
     * reinitialise
     * or to stop.
     * 
     * @param state
     * @throws Exception
     */
    public void setState(PROPULSION_STATE state) throws Exception {
        // 1. We are initialising the propulsion. This can be set at any time.
        if (state == PROPULSION_STATE.INITIALISING) {
            if (this.state != PROPULSION_STATE.INITIALISING) {
                this.leftInitialised = false;
                this.rightInitialised = false;
                this.state = state;
            }
        }

        // 2. The robot is not initialised and not initialising
        else if (this.state != PROPULSION_STATE.UNINITIALISED &&
                this.state != PROPULSION_STATE.INITIALISING) {
            this.state = state;
        }

        else if (this.state == PROPULSION_STATE.UNINITIALISED) {
            throw new Exception("Error: Propulsion uninitialised");
        } else if (this.state == PROPULSION_STATE.INITIALISING) {
            throw new Exception("Error: Propulsion initialising");
        } else {
            throw new Exception("Error: Unknown error when setting state");
        }
    }

    public void update() {
        switch (this.state) {
            case INITIALISING:
                if (!this.rightInitialised) {
                    this.rotateMotor(DIRECTION.CLOCKWISE, maxPower);
                } else if (!this.leftInitialised) {
                    this.rotateMotor(DIRECTION.ANTICLOCKWISE, maxPower);
                } else if (this.drive(0, 0)) {
                    this.state = PROPULSION_STATE.STOPPED;
                }
                break;
            case UNINITIALISED:
                break;
            case STOPPED:
                this.currentSpeed = 0;
                break;
            case DRIVING:
                this.m_driveMotor.set(ControlMode.PercentOutput, this.speedSum / averageNum);
                break;
            default:
                break;
        }
        this.setPower(this.currentSpeed);
        this.currentSpeed = 0;
    }

}
