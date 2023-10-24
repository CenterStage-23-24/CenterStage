package org.firstinspires.ftc.teamcode.OpenCV;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.teamcode.OpenCV.HWMap;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.lang.Math;

@TeleOp(name = "HorizontalAlignTest")
public class HorizontalAlignTest extends LinearOpMode {
    private HWMap hwMap;
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    private DcMotorEx leftFrontMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx rightBackMotor;
    private DcMotorEx leftBackMotor;
  
    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HWMap(telemetry, hardwareMap);
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        waitForStart();

        drive(0.5);
        while (opModeIsActive()) {
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                tele(tag);
                drive(0.0);
            }
        }
    }

    public void drive(double power){
        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(power);
        leftBackMotor.setPower(power);
    }

    public void tele(AprilTagDetection tag){
        telemetry.addData("-", tag.id);
        telemetry.addData("x", tag.ftcPose.x);
        telemetry.addData("y", tag.ftcPose.y);
        telemetry.addData("z", tag.ftcPose.z);
        telemetry.addData("roll", tag.ftcPose.roll);
        telemetry.addData("pitch", tag.ftcPose.pitch);
        telemetry.addData("yaw", tag.ftcPose.yaw);
        telemetry.addData("-", "outside");
        telemetry.update();
    }

    public void correctWithDistanceSensors() {
        double leftDistance = hwMap.distanceSensorLeft.getDistance(DistanceUnit.INCH);
        double rightDistance = hwMap.distanceSensorRight.getDistance(DistanceUnit.INCH);
        double widthOfRobot = 16; // In inches plz

        double rawCorrectionFactor = Math.atan2(leftDistance - rightDistance, widthOfRobot);
        double correctionFactor = mapValue(rawCorrectionFactor, -Math.PI / 2.0, Math.PI / 2.0, -1.0, 1.0);
        double kp = 1.0; // TODO: Tune this; this is the P in PID

        double y = 1.0; // the y-direction to move in during drop-off
        double x = 0.0; // the x-direction to move in during drop-off
        double rotation = kp * correctionFactor;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation), 1);

        double leftFrontPower = (y + x + rotation) / denominator;
        double leftBackPower = (y - x + rotation) / denominator;
        double rightFrontPower = (y - x - rotation) / denominator;
        double rightBackPower = (y + x - rotation) / denominator;

        leftFrontMotor.setPower(leftFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightBackMotor.setPower(rightBackPower);
    }


    public double mapValue(double input, double inputStart, double inputEnd, double outputStart, double outputEnd) {
        return outputStart + (outputEnd - outputStart) / (inputEnd - inputStart) * (input - inputStart);
    }
}
