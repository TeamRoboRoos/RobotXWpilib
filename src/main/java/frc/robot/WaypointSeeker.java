// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.NavCAN.NavData;

/** Add your docs here. */
public class WaypointSeeker {
    
    public static ChassisSpeeds seek(NavData navData) {

        // double ydiff = navData.target.la - navData.current.la;
        // double xdiff = navData.target.lo - navData.current.lo;
        // double seek_ang = Math.atan(ydiff / xdiff);
        
        // double seek_ang_err = seek_ang - navData.current.hd;
        // double omegaRadiansPerSecond = seek_ang_err * 0.01;

        // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(navData.target.sp, 0.0, omegaRadiansPerSecond);

        return new ChassisSpeeds();
    }
}
