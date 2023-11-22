package org.firstinspires.ftc.teamcode.TeleOp.Tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;
@TeleOp
public class SlidesTunerRun extends LinearOpMode {
    InheritedSlidesTuner inheritedSlidesTuner;
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            HWMap hwMap = new HWMap(hardwareMap);
            inheritedSlidesTuner = new InheritedSlidesTuner(hwMap,telemetry);

        } catch (Exception e) {
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()) {
          inheritedSlidesTuner.loop();

        }


    }
}
