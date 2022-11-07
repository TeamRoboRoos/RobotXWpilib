// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;

public class NavCAN {
    //#region Classes
    public class TelemetryData {
        public float telem_lat = 0;
        public float telem_lon = 0;
        public float telem_hdg = 0;
        public float telem_spd = 0;
    }

    public class NavData {
        public float wpt_hdg = 0;
        public float wpt_dst = 0;
        public float cur_hdg = 0;
        public float x_spd = 0;
        public float y_spd = 0;
        public float z_spd = 0;
        public int spd_ovr = 0;
        public int wpt_fin = 0;
    }
    //#endregion

    //#region API Classes
    private enum ApiClass {
        TELEMETRY(0 << 4),
        WAYPOINT(1 << 4);

        public final int id;
        ApiClass(int id) {
            this.id = id;
        }
    }

    private enum TelemetryIndex {
        LOCATION(0),
        METADATA(1);

        public final int id;
        TelemetryIndex(int id) {
            this.id = id;
        }
    }

    private enum WaypointIndex {
        NAVIGATION(0),
        OVERRIDES(1);

        public final int id;
        WaypointIndex(int id) {
            this.id = id;
        }
    }
    //#endregion

    private final CAN can_device;
    private final TelemetryData telemetryData = new TelemetryData();
    private final NavData navData = new NavData();

    public NavCAN(int canID) {
        can_device = new CAN(canID);
    }

    public void print_bytes(byte[] bytes) {
        System.out.print("bytes: ");
        for (byte b : bytes) {
            System.out.print(b);
            System.out.print(":");
        }
        System.out.println();
    }

    public void refresh_nav_data() {
        CANData packeta = new CANData();
        ByteBuffer byteBuffera;
        CANData packetb = new CANData();
        ByteBuffer byteBufferb;
        CANData packetc = new CANData();
        ByteBuffer byteBufferc;
        CANData packetd = new CANData();
        ByteBuffer byteBufferd;

        boolean a = can_device.readPacketLatest(ApiClass.TELEMETRY.id | TelemetryIndex.LOCATION.id, packeta);
        byteBuffera = ByteBuffer.wrap(packeta.data);
        byteBuffera.order(ByteOrder.LITTLE_ENDIAN);
        telemetryData.telem_lat = byteBuffera.getInt() / 10e6f;
        telemetryData.telem_lon = byteBuffera.getInt() / 10e6f;

        boolean b = can_device.readPacketLatest(ApiClass.TELEMETRY.id | TelemetryIndex.METADATA.id, packetb);
        byteBufferb = ByteBuffer.wrap(packetb.data);
        byteBufferb.order(ByteOrder.LITTLE_ENDIAN);
        telemetryData.telem_hdg = byteBufferb.getShort() / 1f;
        telemetryData.telem_spd = byteBufferb.getShort() / 1f;

        boolean c = can_device.readPacketLatest(ApiClass.WAYPOINT.id | WaypointIndex.NAVIGATION.id, packetc);
        byteBufferc = ByteBuffer.wrap(packetc.data);
        byteBufferc.order(ByteOrder.LITTLE_ENDIAN);
        navData.wpt_hdg = byteBufferc.getShort() / 1f;
        navData.wpt_dst = byteBufferc.getShort() / 1f;
        navData.cur_hdg = byteBufferc.getShort() / 1f;

        boolean d = can_device.readPacketLatest(ApiClass.WAYPOINT.id | WaypointIndex.OVERRIDES.id, packetd);
        byteBufferd = ByteBuffer.wrap(packetd.data);
        byteBufferd.order(ByteOrder.LITTLE_ENDIAN);
        navData.x_spd = byteBufferd.get() / 1f;
        navData.y_spd = byteBufferd.get() / 1f;
        navData.z_spd = byteBufferd.get() / 1f;
        navData.spd_ovr = byteBufferd.get();
        navData.wpt_fin = byteBufferd.get();
    }

    //#region Getters
    public TelemetryData getTelemetryData() {
        return telemetryData;
    }

    public NavData getNavData() {
        return navData;
    }
    //#endregion
}
