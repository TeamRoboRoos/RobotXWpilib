// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.NavCAN.NavData;

/** Add your docs here. */
public class WaypointSeeker {

    private static double limit(double num) {
        return Math.min(Math.max(num, -1), 1);
    }

    public static ChassisSpeeds seek(NavData navData) {

        double angDiff = navData.wpt_hdg - navData.cur_hdg;

        double ydiff = navData.wpt_dst * Math.sin(angDiff);
        double xdiff = navData.wpt_dst * Math.cos(angDiff);

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(limit(ydiff), limit(xdiff), limit(angDiff));
        // double ydiff = navData.target.la - navData.

        // double seek_ang_err = seek_ang - navData.current.hd;
        // double omegaRadiansPerSecond = seek_ang_err * 0.01;

        //
        //
        // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(navData.target.sp, 0.0,
        // omegaRadiansPerSecond);

        return chassisSpeeds;
    }
}
