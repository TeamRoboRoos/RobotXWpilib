package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivebase;
import frc.robot.subsystems.ShooterCanForReal;
import frc.robot.PropulsionModule.PROPULSION_STATE;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShooterCanForComms {
    private CAN can;

    private Drivebase drivebase;
    private ShooterCanForReal shooter;

    private float xVal;
    private float yVal;
    private float rVal;
    private float pVal;
    private int sRpm;

    private boolean isShooting;
    private boolean isEnabled;
    private boolean isRunning;

    public ShooterCanForComms(Drivebase drivebase, ShooterCanForReal shooter) {
        this.init();
        this.shooter = shooter;
        this.drivebase = drivebase;

      //  SmartDashboard.
      SmartDashboard.getBoolean("Shooter Enabled", isEnabled);

    }

    public void init() {
      this.isRunning = false;
      try {
        this.can = new CAN(1, 8, 16);
        this.isRunning = true;
      }
      catch (Exception e) {
        System.out.println(e.toString());
      }
      this.xVal = 0;
      this.yVal = 0;
      this.rVal = 0;
      this.pVal = 0;
      this.sRpm = 0;
      this.isShooting = false;
      this.isEnabled = false;
      SmartDashboard.putNumber("Data", 0);
      displayData();
    }

    public void update() {
      if (this.isRunning) {
        final CANData data = new CANData();
        can.readPacketLatest(64, data);
        if (data.length > 0) {
          xVal = (float)((short)((data.data[0] << 8) | (data.data[1] & 0xFF)) - 2000) / 1000.0f;
          yVal = (float)((short)((data.data[2] << 8) | (data.data[3] & 0xFF)) - 2000) / 1000.0f;
          rVal = (float)((short)((data.data[4] << 8) | (data.data[5] & 0xFF)) - 2000) / 1000.0f;
          pVal = (float)((short)((data.data[6] << 8) | (data.data[7] & 0xFF)) - 2000) / 1000.0f;
        }
        if (this.drivebase.isState(PROPULSION_STATE.STOPPED) || this.drivebase.isState(PROPULSION_STATE.DRIVING)) {
          //this.drivebase.drive(new ChassisSpeeds(yVal, -xVal, rVal), pVal);
        }
        
        if (this.can.readPacketLatest(36, data)) {
          this.sRpm = (int)((data.data[0] << 8) | (data.data[1] & 0xFF));
          this.isShooting = true;
        }
        
        if (this.can.readPacketLatest(145, data)) {
          System.out.println("Received start");
          this.isEnabled = true;
        }
        
        if (this.can.readPacketLatest(146, data)) {
          this.isEnabled = false;
          System.out.println("Received stop");
        }
/*
        can.readPacketLatest(34, data);
        if (data.length > 0) {
          this.sRpm = 0;
          this.isShooting = false;
        }
*/
        this.shooter.runMotorVelocity(this.sRpm);


        //this.shooter.runMotorVelocity(500);
      }
    }
    //268962048

    public boolean isEnabled() {
      return this.isEnabled;
    }

    public void setEnabled(boolean isEnabled) {
      //this.isEnabled = isEnabled;
      byte[] state = {0};
      if (isEnabled) { 
        System.out.println("Sending start");
        this.can.writePacket(state, 17);
      }
      else { 
        System.out.println("Sending end");
        this.can.writePacket(state, 18);
      }
    }

    public void displayData() {
      SmartDashboard.putBoolean("SC_Shooting", isShooting);
      SmartDashboard.putBoolean("SC_Enabled", isEnabled);
      SmartDashboard.putBoolean("SC_Running", isRunning);

      SmartDashboard.putNumber("xVal", xVal);
      SmartDashboard.putNumber("yVal", yVal);
      SmartDashboard.putNumber("rVal", rVal);
      SmartDashboard.putNumber("pVal", pVal);
      SmartDashboard.putNumber("sRpm", this.sRpm);
    }
}
