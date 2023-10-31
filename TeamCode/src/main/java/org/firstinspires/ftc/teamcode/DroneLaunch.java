package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DroneLaunch {
    public enum droneFSM {
        start,
        one,
        two
    }
    droneFSM state = droneFSM.start;


    public DroneLaunch(HWMap hardware) {

    }

    public void loop() {
        while (true) {
            switch (state) {
                case start: //would add conditionals here
                    state = droneFSM.start;
                    break;
                case one:
                    state = droneFSM.start;
                    break;
                case two:
                    state = droneFSM.start;
                    return;
            }
        }

    }
}
