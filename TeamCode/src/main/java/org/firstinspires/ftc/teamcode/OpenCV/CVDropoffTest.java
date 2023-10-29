package org.firstinspires.ftc.teamcode.OpenCV;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class CVDropoffTest extends LinearOpMode {
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    private DcMotorEx leftFrontMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx rightBackMotor;
    private DcMotorEx leftBackMotor;
    private HWMap hwMap;
    double range_t;
    double bearing_t;
    double y_t;
    double yaw_t;

    private double avgRange;
    private double avgBearing;
    private double avgY;
    private double avgYaw;
    int trials;
    double real_dist;

    private double avgRangeError;
    private double avgYError;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            hwMap = new HWMap(telemetry, hardwareMap);

            leftFrontMotor = hwMap.leftFrontMotor;
            rightFrontMotor = hwMap.rightFrontMotor;
            rightBackMotor = hwMap.rightBackMotor;
            leftBackMotor = hwMap.leftBackMotor;

            range_t = 20.0;
            bearing_t = 20.0;
            y_t = 20.0;
            yaw_t = 20.0;

            avgRange = 0.0;
            avgBearing = 0.0;
            avgY = 0.0;
            avgYaw = 0.0;
            trials = 0;
            real_dist = 4.75; //Update as needed


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
        } catch (Exception e) {
            telemetry.addLine("Bruh I don't care");
            telemetry.update();
        }
        waitForStart();

        /*
        4 things to test:
        1. CV Drive - driving towards AprilTag using CV nav
            - CV Values: Range, y
        2. Distance Sensor Drive - driving towards AprilTag using DS nav
        3. CV Angular Correction - correcting angle based on CV nav
            - CV Values: Bearing, Yaw
        4. Distance Sensor Angular Correction - correcting angle based on DS nav

         */

        drive(0.5);
        while (trials <= 100) {
            try {
                if (tagProcessor.getDetections().size() > 0) {
                    AprilTagDetection tag = tagProcessor.getDetections().get(0);
                    drive(0.0);
                    tele(tag);
                    calc(tag);
                }
            } catch (Exception e) {
                telemetry.addLine("I don't care");
                telemetry.update();
            }
            /*
            if (tagProcessor.getDetections().get(0).ftcPose.range < range_t) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                drive(0.0);
                tele(tag);
            }
             */
        }
        double sumRange = avgRange;
        double sumY = avgY;
        double sumRangeError = avgRangeError;
        double sumYError = avgYError;
        avgRange = sumRange / trials;
        avgY = sumY / trials;
        avgRangeError = sumRangeError / trials;
        avgYError = sumYError / trials;

        while(opModeIsActive()){
            telemetry.addData("SUM RANGE: ", sumRange);
            telemetry.addData("AVG RANGE: ", avgRange);
            telemetry.addData("SUM Y: ", sumY);
            telemetry.addData("AVG Y: ", avgY);
            telemetry.addData("SUM RANGE ERROR: ", sumRangeError);
            telemetry.addData("AVG RANGE ERROR: ", avgRangeError);
            telemetry.addData("SUM Y ERROR: ", sumYError);
            telemetry.addData("AVG Y ERROR: ", avgYError);
            telemetry.update();
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
        telemetry.addData("range", tag.ftcPose.range);
        telemetry.addData("bearing", tag.ftcPose.bearing);
        telemetry.addData("elevation", tag.ftcPose.elevation);
        telemetry.addData("-", "outside");
        telemetry.addData("trials: ", trials);
        telemetry.addData("Sum of Ranges: ", avgRange);
        telemetry.update();
    }

    public void calc(AprilTagDetection tag){
        double range = tag.ftcPose.range;
        double y = tag.ftcPose.y;
        telemetry.addData("range", range);
        telemetry.addData("y", y);
        telemetry.update();
        avgRange += range;
        avgY += y;
        avgRangeError += range - real_dist;
        avgYError += y - real_dist;
        trials += 1;
    }
}