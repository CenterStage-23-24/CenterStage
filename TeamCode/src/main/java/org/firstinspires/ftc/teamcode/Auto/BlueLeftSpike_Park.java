package org.firstinspires.ftc.teamcode.Auto;

import android.transition.Slide;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Auto.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons.Arm;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Gripper;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Slides;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.TransferController;


@Autonomous
@Config
public class BlueLeftSpike_Park extends LinearOpMode {

    /** Auto Constant Variables: **/
    public static double startX = 12.0; // Start pos X
    public static double startY = 65.0; // Start pos Y
    public static double startHeading = Math.toRadians(270);

    /**Robot Tuning Variables**/
    public static double startXOff = 0.0; // Start pos X offset
    public static double startYOff = 0.0; // Start pos Y offset

    //TODO: Field Tuning Variables:
    public static double autoDelay = 0.0; //Delay before auto starts

    //Declare Mechanisms
    public static SampleMecanumDrive drive;
    public static TransferController transferController;
    public static Arm arm;
    public static Slides slides;
    public static Gripper gripper;

    //Variables for spike mark
    private String propPosition;
    public static double forwardDistance;
    public static double turnAngleSpike;
    public static double turnAnglePark;

    @Override
    public void runOpMode() {

        //Initialize Mechanisms
        HWMap hwMap = new HWMap(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        arm =  new Arm(hwMap, telemetry);
        slides =  new Slides(hwMap, telemetry);
        transferController = new TransferController(arm, slides, telemetry, 0);
        gripper = new Gripper(hwMap);
        gripper.releaseRight();
        gripper.releaseLeft();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Gaytorbytes om nom nom");
            telemetry.update();
            gripper.gripLeft();
            gripper.gripRight();
            sleep(1000);
            while(!transferController.delay()){
                continue;
            }
            while(!transferController.extend("SPIKE")){
                slides.pid(true);
                arm.updatePos();
                transferController.tele();
            }

            telemetry.addData("-", "INIT DONE");
            telemetry.update();
            propPosition = "RIGHT";

        }

        if(propPosition == "CENTER"){
            //forwardDistance = 30;
            turnAngleSpike = 0;
            turnAnglePark = 90-turnAngleSpike;

        } else if(propPosition == "LEFT"){
            //forwardDistance = 25;
            turnAngleSpike = 60;
            turnAnglePark = 90-turnAngleSpike;
        } else{
            //forwardDistance = 25;
            turnAngleSpike = -75;
            turnAnglePark = 90-turnAngleSpike;
        }

        startX += startXOff;
        startY += startYOff;
        drive.setPoseEstimate(new Pose2d(startX, startY, startHeading));
        drive.setExternalHeading(startHeading);
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))
                .forward(27)
                .turn(Math.toRadians(turnAngleSpike))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    gripper.releaseLeft();
                })
                .waitSeconds(1)
                .back(6)
                .turn(Math.toRadians(turnAnglePark))
                .forward(30)
                .build();
        drive.followTrajectorySequenceAsync(trajectory);

        while(opModeIsActive()){
            drive.update();
            slides.pid(true);
            arm.updatePos();
        }
    }
}