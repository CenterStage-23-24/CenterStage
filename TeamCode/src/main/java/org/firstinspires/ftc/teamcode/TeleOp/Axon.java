package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.FtcDashboard;
import org.checkerframework.checker.units.qual.A;

import java.net.CacheRequest;

@Config
@TeleOp
public class Axon extends LinearOpMode {
    private HWMap hwMap;
    private GamepadEx gamePad1;
    private CRServo servo;
    private AnalogInput encoder;
    private AxonClass axon;
    private PIDController pidController;
    public static double p = 0.0009, i = 0, d = 0.0000;
    public static int targetPos = 10;

    @Override
    public void runOpMode() {
        try {
            hwMap = new HWMap(telemetry, hardwareMap);
            gamePad1 = new GamepadEx(gamepad1);

            servo = hwMap.getAxonServoLeft();
            encoder = hwMap.getAxonAnalogLeft();
            pidController = new PIDController(p, i, d);
            axon = new AxonClass(servo, encoder, false, pidController);

        } catch (Exception e) {
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            double measuredPos = axon.getPos();
            //double difference = targetPos - measuredPos;
            //int direction = (int) ((measuredPos-targetPos) / Math.abs(measuredPos-targetPos));
            //int deg = (int) (Math.min(Math.abs(difference), 360 - Math.abs(difference)));
            double error = Math.min(Math.abs(targetPos - measuredPos), 360 - Math.abs(targetPos - measuredPos));

            double calcedpower = axon.setPos(error);

            telemetry.addData("Power: ", calcedpower);
            telemetry.addData("Left Deg.", axon.getPos());
            telemetry.addData("Target Pos", targetPos);
            telemetry.addData("Error: ", error);
            telemetry.update();
        }
    }

    public double normalizeRadiansTau(double angle){
        return (angle + (Math.PI * 2)) % (Math.PI * 2);
    }
}
