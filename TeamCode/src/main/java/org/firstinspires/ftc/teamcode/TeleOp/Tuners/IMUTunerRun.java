package org.firstinspires.ftc.teamcode.TeleOp.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.IntakeController;

@TeleOp
public class IMUTunerRun extends LinearOpMode {
    private IMUInheritedTuner imuInheritedTuner;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            HWMap hwMap = new HWMap(hardwareMap);
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            imuInheritedTuner = new IMUInheritedTuner(hwMap, telemetry);
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
