package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.hal.CANData;

/** Add your docs here. */
public class WeatherCan {
  private CAN can;
  private boolean isRunning;
  private boolean isReceiving;

  // Weather: 
  //  2 byte wind direction; 
  //  2 byte wind speed (2 decimal places); 
  //  1 byte temperature; 
  //  1 byte humidity; 
  //  2 byte pressure

  private short windDirection;
  private float windSpeed;
  private byte temperature;
  private byte humidity;
  private short pressure;

  public WeatherCan() {
    this.init();
  }

  public void update() {
    if (isRunning) {
      final CANData data = new CANData();
      this.isReceiving = true;
      if (can.readPacketLatest(32, data)) {
        windDirection = (short)((data.data[0] << 8) | (data.data[1] & 0xFF));
        windSpeed = (float)((short)((data.data[2] << 8) | (data.data[3] & 0xFF)) / 100.0f);
        temperature = (byte)(data.data[4]);
        humidity = (byte)(data.data[5]);
        pressure = (short)((data.data[6] << 8) | (data.data[7] & 0xFF));
      }
    }
  }

  public void displayData() {
    SmartDashboard.putNumber("Temperature", temperature);
    SmartDashboard.putNumber("WindDirection", windDirection);
    SmartDashboard.putNumber("Wind Direction", windDirection);
    SmartDashboard.putNumber("WindSpeed", windSpeed);
    SmartDashboard.putNumber("Humidity", humidity);
    SmartDashboard.putNumber("Pressure", pressure);
    SmartDashboard.putBoolean("WeatherCAN", isRunning);
    SmartDashboard.putBoolean("WeatherData", isReceiving);
  }

  public void init() {
    System.out.println("Starting WeatherCAN...");
    this.isReceiving = false;
    try {
      this.can = new CAN(0, 8, 12);
      this.start();
      System.out.println("Is started");
    }
    catch (Exception e) {
      System.out.println(e.toString());
    }
    this.windDirection = 0;
    this.windSpeed = 0;
    this.temperature = 0;
    this.humidity = 0;
    this.pressure = 0;
  }

  public void start() {
    this.isRunning = true;
    byte[] state = {0};
    // state is empty because the api (class + index) should have the data
    can.writePacket(state, 17);
  }

  public void stop() {
    this.isRunning = false;
    byte[] state = {0};
    // state is empty because the api (class + index) should have the data
    can.writePacket(state, 18);
  }

  public boolean isRunning() {
    return this.isRunning;
  }

  public byte getTemperature() {
    return temperature;
  }

  public short getPressure() {
    return pressure;
  }

  public short getWindDirection() {
    return windDirection;
  }

  public float getWindSpeed() {
    return windSpeed;
  }

  public byte getHumidity() {
    return humidity;
  }
}