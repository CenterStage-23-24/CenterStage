package org.firstinspires.ftc.teamcode.TeleOp.Tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;
@TeleOp
public class IMUTunerRun extends LinearOpMode {
    private IMUInheritedTuner imuInheritedTuner;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            HWMap hwMap = new HWMap(hardwareMap);
            ElapsedTime time = new ElapsedTime();
            imuInheritedTuner = new IMUInheritedTuner(hwMap, telemetry, time);
        } catch (Exception e) {
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()) {
            double rightX = imuInheritedTuner.backdropAlignment();
            if(!imuInheritedTuner.robotAtAngle(2))
                imuInheritedTuner.drive(0, 0, rightX, HWMap.readFromIMU());
            else
                imuInheritedTuner.drive(0,0,0,0);

            telemetry.update();

        }
    }
}
