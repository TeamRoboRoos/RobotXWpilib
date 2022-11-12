package frc.robot.subsystems;
    
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.fasterxml.jackson.annotation.JsonCreator.Mode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterCanForReal {
    private final int SHOOTER_ID = 25;
    private final float SHOOTER_DEADBAND = 0.01f;
    private final int SHOOTER_TIMEOUTS = 30;
    private final double GAINS_KP = 0.16;
    private final double GAINS_KI = 0.000001;
    private final double GAINS_KD = 4;
    private final double GAINS_KF = 0.02;// (1023.0 * 0.35) / 20660.0;
    private final int SHOOT_IDX = 0;
    private final int MOTOR_VELOCITY = 3000;
    private final int MOTOR_REVERSE_VELOCITY = -200;
    private final double MOTOR_BACK_POWER = -0.1;

    public static final double GAINS_KP_BALL = 0.1;
    public static final double GAINS_KI_BALL = 0.001;
    public static final double GAINS_KD_BALL = 5;
    public static final double GAINS_KF_BALL = (1023.0 * 0.35) / 20660.0;

    private TalonFX shooterMotor;
    private int shootCount;

    /** Creates a new ExampleSubsystem. */
    public ShooterCanForReal() {
        initHardware();
        // SmartDashboard.putNumber("ShootKP", 0);
        // SmartDashboard.putNumber("ShootKI", 0);
        // SmartDashboard.putNumber("ShootKD", 0);
        // SmartDashboard.putNumber("ShootKFF", 0);
        // SmartDashboard.putNumber("ShooterRPM", 0);
    }

    private void initHardware() {
    this.shooterMotor = new TalonFX(this.SHOOTER_ID);

    this.shooterMotor.configFactoryDefault();
    this.shooterMotor.setInverted(true);
    this.shooterMotor.setNeutralMode(NeutralMode.Coast);
    this.shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    this.shooterMotor.configNeutralDeadband(this.SHOOTER_DEADBAND);

    // Based on code from
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/VelocityClosedLoop/src/main/java/frc/robot/Robot.java
    this.shooterMotor.configNominalOutputForward(0, this.SHOOTER_TIMEOUTS);
    this.shooterMotor.configNominalOutputReverse(0, this.SHOOTER_TIMEOUTS);
    this.shooterMotor.configPeakOutputForward(1, this.SHOOTER_TIMEOUTS);
    this.shooterMotor.configPeakOutputReverse(-1, this.SHOOTER_TIMEOUTS);

    this.shooterMotor.config_kF(this.SHOOT_IDX, this.GAINS_KF, this.SHOOTER_TIMEOUTS);
    this.shooterMotor.config_kP(this.SHOOT_IDX, this.GAINS_KP, this.SHOOTER_TIMEOUTS);
    this.shooterMotor.config_kI(this.SHOOT_IDX, this.GAINS_KI, this.SHOOTER_TIMEOUTS);
    this.shooterMotor.config_kD(this.SHOOT_IDX, this.GAINS_KD, this.SHOOTER_TIMEOUTS);

        this.shooterMotor.setInverted(false);
    }

    public void update() {
    // This method will be called once per scheduler run
    // System.out.println(shootMotorMaster.getSelectedSensorVelocity());

    // shootMotorMaster.config_kF(ShooterConstants.SHOOT_IDX,
    // SmartDashboard.getNumber("ShootKFF", 0),
    // ShooterConstants.SHOOT_TIMEOUTS);
    // shootMotorMaster.config_kP(ShooterConstants.SHOOT_IDX,
    // SmartDashboard.getNumber("ShootKP", 0),
    // ShooterConstants.SHOOT_TIMEOUTS);
    // shootMotorMaster.config_kI(ShooterConstants.SHOOT_IDX,
    // SmartDashboard.getNumber("ShootKI", 0),
    // ShooterConstants.SHOOT_TIMEOUTS);
    // shootMotorMaster.config_kD(ShooterConstants.SHOOT_IDX,
    // SmartDashboard.getNumber("ShootKD", 0),
    // ShooterConstants.SHOOT_TIMEOUTS);

         SmartDashboard.putNumber("ShooterRPMReal",
             this.convertNativeUnitsToRPM(shooterMotor.getSelectedSensorVelocity()));
        SmartDashboard.putBoolean("ShooterAtSpeed",
            this.shooterMotor.getSelectedSensorVelocity() > convertRPMtoNativeUnits(this.MOTOR_VELOCITY - 600));
        SmartDashboard.putBoolean("ShooterRunning", this.shooterMotor.getSupplyCurrent() != 0);
    }

    public void runMotorPower(double power) {
        this.setPIDmodeShoot();
        this.shooterMotor.set(TalonFXControlMode.PercentOutput, power);
        this.shooterMotor.getSelectedSensorVelocity();
    }

    public void runMotorVelocity(int rpm) {
        this.setPIDmodeShoot();
        double velocity = convertRPMtoNativeUnits((int) rpm);
        this.shooterMotor.set(TalonFXControlMode.Velocity, velocity);
        this.shooterMotor.getSelectedSensorVelocity();
    }

    public void setMotorBack() {
        this.setPIDmodeBack();
        this.shooterMotor.set(TalonFXControlMode.PercentOutput, this.MOTOR_BACK_POWER);
    }

    public void stopMotorBack() {
        this.setPIDmodeShoot();
        this.stop();
    }

    private int convertRPMtoNativeUnits(int rpm) {
        return rpm * 2048 / 600;
    }

    private double convertNativeUnitsToRPM(double nativeUnits) {
        return nativeUnits / (2048 / 600);
    }

    public int getShootCount() {
        return shootCount;
    }

    public void stop() {
        this.shooterMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void setBrake() {
        this.shooterMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setCoast() {
        this.shooterMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void setCANSlow() {
        this.shooterMotor.getStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 1000);
        this.shooterMotor.getStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1000);
    }

    public void setCANFast() {
        this.shooterMotor.getStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        this.shooterMotor.getStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    }

    private void setPIDmodeBack() {
        this.shooterMotor.config_kF(this.SHOOT_IDX, this.GAINS_KF_BALL, this.SHOOTER_TIMEOUTS);
        this.shooterMotor.config_kP(this.SHOOT_IDX, this.GAINS_KP_BALL, this.SHOOTER_TIMEOUTS);
        this.shooterMotor.config_kI(this.SHOOT_IDX, this.GAINS_KI_BALL, this.SHOOTER_TIMEOUTS);
        this.shooterMotor.config_kD(this.SHOOT_IDX, this.GAINS_KD_BALL, this.SHOOTER_TIMEOUTS);
    }

    private void setPIDmodeShoot() {
        this.shooterMotor.config_kF(this.SHOOT_IDX, this.GAINS_KF, this.SHOOTER_TIMEOUTS);
        this.shooterMotor.config_kP(this.SHOOT_IDX, this.GAINS_KP, this.SHOOTER_TIMEOUTS);
        this.shooterMotor.config_kI(this.SHOOT_IDX, this.GAINS_KI, this.SHOOTER_TIMEOUTS);
        this.shooterMotor.config_kD(this.SHOOT_IDX, this.GAINS_KD, this.SHOOTER_TIMEOUTS);
    }
}
