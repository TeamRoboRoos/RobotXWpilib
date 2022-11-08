package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.PropulsionModule.PROPULSION_STATE;

public class Drivebase {
    public enum DriveMode {
        SWERVE, DIFFERENTIAL
    }

    private SwerveDriveKinematics kinematics;
    private DifferentialDriveKinematics kinematics_diff;
    private PropulsionModule PortFwd, StarbFwd, PortAft, StarbAft;
    // private SwerveModuleState[] moduleStates;

    public Drivebase(Translation2d PortFwdPos, Translation2d StarbFwdPos, Translation2d PortAftPos,
            Translation2d StarbAftPos,
            PropulsionModule PortFwd, PropulsionModule StarbFwd, PropulsionModule PortAft, PropulsionModule StarbAft) {
        this.kinematics = new SwerveDriveKinematics(PortFwdPos, StarbFwdPos, PortAftPos, StarbAftPos);
        this.kinematics_diff = new DifferentialDriveKinematics(PortFwdPos.getY() - StarbFwdPos.getY());

        this.PortFwd = PortFwd;
        this.StarbFwd = StarbFwd;
        this.PortAft = PortAft;
        this.StarbAft = StarbAft;
    }

    /**
     * Drive the boat in swerve mode
     * 
     * @param chassisSpeeds A ChassisSpeeds object describing the robot relative
     *                      direction of travel
     * @param thrust        The power to drive at
     */
    public void drive(ChassisSpeeds chassisSpeeds, double thrust) {
        drive(chassisSpeeds, thrust, DriveMode.SWERVE);
    }

    public void drive(ChassisSpeeds chassisSpeeds, double thrust, DriveMode mode) {
        chassisSpeeds = speedTolerance(chassisSpeeds);

        if (mode == DriveMode.SWERVE) {
            SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

            PortFwd.driveFromState(moduleStates[0], thrust);
            StarbFwd.driveFromState(moduleStates[1], thrust);
            PortAft.driveFromState(moduleStates[2], thrust);
            StarbAft.driveFromState(moduleStates[3], thrust);
        } else if (mode == DriveMode.DIFFERENTIAL) {
            DifferentialDriveWheelSpeeds wSpeeds = kinematics_diff.toWheelSpeeds(chassisSpeeds);

            PortFwd.driveFromState(new SwerveModuleState(wSpeeds.leftMetersPerSecond, new Rotation2d()), thrust);
            PortAft.driveFromState(new SwerveModuleState(wSpeeds.leftMetersPerSecond, new Rotation2d()), thrust);
            StarbFwd.driveFromState(new SwerveModuleState(wSpeeds.rightMetersPerSecond, new Rotation2d()), thrust);
            StarbAft.driveFromState(new SwerveModuleState(wSpeeds.rightMetersPerSecond, new Rotation2d()), thrust);
        }
    }

    private ChassisSpeeds speedTolerance(ChassisSpeeds speeds) {
        if (Math.abs(speeds.vxMetersPerSecond) < 0.1)
            speeds.vxMetersPerSecond = 0;
        if (Math.abs(speeds.vyMetersPerSecond) < 0.1)
            speeds.vyMetersPerSecond = 0;
        if (Math.abs(speeds.omegaRadiansPerSecond) < 0.1)
            speeds.omegaRadiansPerSecond = 0;
        return speeds;
    }

    public void initialise() {
        try {
            if (this.StarbFwd.getState() == PROPULSION_STATE.UNINITIALISED) {
                this.StarbFwd.setState(PROPULSION_STATE.INITIALISING);
            }
            if (this.StarbFwd.getState() == PROPULSION_STATE.STOPPED) {
                // this.starbFwd.drive(power, deg);
            }
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        try {
            if (this.PortAft.getState() == PROPULSION_STATE.UNINITIALISED) {
                this.PortAft.setState(PROPULSION_STATE.INITIALISING);
            }
            if (this.PortAft.getState() == PROPULSION_STATE.STOPPED) {
                // this.portAft.drive(0, deg);
            }
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        try {
            if (this.PortFwd.getState() == PROPULSION_STATE.UNINITIALISED) {
                this.PortFwd.setState(PROPULSION_STATE.INITIALISING);
            }
            if (this.PortFwd.getState() == PROPULSION_STATE.STOPPED) {
                // this.portFwd.drive(0, deg);
            }
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        try {
            if (this.StarbAft.getState() == PROPULSION_STATE.UNINITIALISED) {
                this.StarbAft.setState(PROPULSION_STATE.INITIALISING);
            }
            if (this.StarbAft.getState() == PROPULSION_STATE.STOPPED) {
                // this.starbAft.drive(0, deg);
            }
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }
    }

    public void update() {
        this.StarbFwd.update();
        this.PortAft.update();
        this.PortFwd.update();
        this.StarbAft.update();
    }

    public boolean isState(PROPULSION_STATE state) {
        if (this.PortFwd.getState() == state &&
                this.StarbFwd.getState() == state &&
                this.PortAft.getState() == state &&
                this.StarbAft.getState() == state) {
            return true;
        }
        return false;
    }

    public void setState(PROPULSION_STATE state) {
        try {
            this.PortFwd.setState(state);
            this.PortAft.setState(state);
            this.StarbFwd.setState(state);
            this.StarbAft.setState(state);
        } catch (Exception e) {

        }
    }
}
