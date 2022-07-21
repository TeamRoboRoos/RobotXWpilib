package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Drivebase {
    private SwerveDriveKinematics kinematics;
    private PropulsionModule PortFwd, StarbFwd, PortAft, StarbAft;
    private SwerveModuleState[] moduleStates;

    public Drivebase(Translation2d PortFwdPos, Translation2d StarbFwdPos, Translation2d PortAftPos, Translation2d StarbAftPos,
                    PropulsionModule PortFwd, PropulsionModule StarbFwd, PropulsionModule PortAft, PropulsionModule StarbAft) {
        this.kinematics = new SwerveDriveKinematics(PortFwdPos, StarbFwdPos, PortAftPos, StarbAftPos);
        this.PortFwd = PortFwd;
        this.StarbFwd = StarbFwd;
        this.PortAft = PortAft;
        this.StarbAft = StarbAft;
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        PortFwd.driveFromState(moduleStates[0]);
        StarbFwd.driveFromState(moduleStates[1]);
        PortAft.driveFromState(moduleStates[2]);
        StarbAft.driveFromState(moduleStates[3]);
    }
}
