package org.firstinspires.ftc.teamcode.TeleOp.Tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;
@TeleOp
public class ArmTunerRun extends LinearOpMode {
    InheritedArmTuner inheritedArmTuner;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            HWMap hwMap = new HWMap(hardwareMap);
            inheritedArmTuner = new InheritedArmTuner(hwMap,telemetry);



        }catch (Exception e) {
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            inheritedArmTuner.loop();

        }
    }

}
