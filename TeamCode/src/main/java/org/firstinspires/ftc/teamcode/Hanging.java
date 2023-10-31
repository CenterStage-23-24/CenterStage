package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hanging {
    public enum hangingFSM {
        start,
        one,
        two
    }
    hangingFSM state = Hanging.hangingFSM.start;

    public Hanging(HWMap hardware) {

    }

    public void loop() {
        while (true) {
            switch (state) {
                case start: //would add conditionals here
                    state = hangingFSM.start;
                    break;
                case one: //state transition should go back to start
                    state = hangingFSM.start;
                    break;
                case two:
                    state = hangingFSM.start;
                    return;
            }
        }

    }
}
