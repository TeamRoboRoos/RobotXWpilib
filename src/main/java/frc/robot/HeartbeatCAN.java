// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;

public class HeartbeatCAN {
    //#region Storage Classes
    public class Msg1_Cur_Loc {
        public float lat = 0;
        public float lon = 0;
    }
    public class Msg2_Misc {
        public int USV_MODE = 1;
        public int USV_STATUS = 1;
        public int GATE_REPORT_ENT = 1;
        public int GATE_REPORT_EXT = 1;
        public int WILDLIFE_NUM = 0;
        public char WILDLIFE_OBJ_1 = 'P';
        public char WILDLIFE_OBJ_3 = 'T';
        public char WILDLIFE_OBJ_2 = 'C';
        public int PATH_SUCCESS = 1;
        public char SCAN_CODE_1 = 'R';
        public char SCAN_CODE_2 = 'G';
        public char SCAN_CODE_3 = 'B';
        public int UAV_UAV_STATUS = 1;
        public int UAV_ITEM_STATUS = 0;
    }
    public class Msg3_Misc {
        public char UAV_OBJ1 = 'R';
        public char UAV_OBJ2 = 'R';
        public int  UAV_SRCH_STATUS = 1;
        public char DOCK_COLOUR = 'R';
        public int DOCK_STATUS = 1;
        public char FLING_COLOUR = 'R';
        public int FLING_STATUS = 1;
    }
    public class Msg4_UAV_OBJ_1 {
        public float lat = 0;
        public float lon = 0;
    }
    public class Msg5_UAV_OBJ_1 {
        public float lat = 0;
        public float lon = 0;
    }
    //#endregion

    //#region API Classes
    private enum ApiClass {
        TELEMETRY(0 << 4),
        METADATA(1 << 4);

        public final int id;
        ApiClass(int id) {
            this.id = id;
        }
    }

    private enum MetadataIndex {
        MISC_1(0),
        MISC_2(1),
        OBJ_1_LOC(2),
        OBJ_2_LOC(3);

        public final int id;
        MetadataIndex(int id) {
            this.id = id;
        }
    }
    //#endregion


    private final CAN can_device;

    public HeartbeatCAN(int canID, int manafacturerID, int typeID) {
        can_device = new CAN(canID, teamID, manafacturerID);
    }

    public void print_bytes(byte[] bytes) {
        System.out.print("bytes: ");
        for (byte b : bytes) {
            System.out.print(b);
            System.out.print(":");
        }
        System.out.println();
    }

    public void transmit_heartbeat() {
        byte[8] 1st = new byte[8];
        ByteBuffer byteBuffer = new ByteBuffer();
    }
}