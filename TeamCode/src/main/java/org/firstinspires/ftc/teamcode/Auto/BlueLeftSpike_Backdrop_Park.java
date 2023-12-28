package org.firstinspires.ftc.teamcode.Auto;

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
public class BlueLeftSpike_Backdrop_Park extends LinearOpMode {

    /** Auto Constant Variables: **/
    private double startX = 12.0; // Start pos X
    private double startY = 65.0; // Start pos Y
    private double startHeading = Math.toRadians(270);

    /**Robot Tuning Variables**/
    private double startXOff = 0.0; // Start pos X offset
    private double startYOff = 0.0; // Start pos Y offset

    //TODO: Field Tuning Variables:
    private double autoDelay = 0.0; //Delay before auto starts

    //Declare Mechanisms
    private SampleMecanumDrive drive;
    private TransferController transferController;
    private Arm arm;
    private Slides slides;
    private Gripper gripper;

    //Variables for spike mark
    private String propPosition;
    private double forwardDistance;
    private double turnAngleSpike;
    private double turnAnglePark;

    @Override
    public void runOpMode() {

        //Initialize Mechanisms
        HWMap hwMap = new HWMap(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        transferController = new TransferController(arm, slides, telemetry, 0);
        arm =  new Arm(hwMap, telemetry);
        slides =  new Slides(hwMap, telemetry);
        gripper = new Gripper(hwMap);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Gaytorbytes om nom nom");
            telemetry.update();

            propPosition = "RIGHT";
        }

        if(propPosition == "CENTER"){
            forwardDistance = 30;
            turnAngleSpike = 0;
            turnAnglePark = 90-turnAngleSpike;

        } else if(propPosition == "LEFT"){
            forwardDistance = 25;
            turnAngleSpike = 45;
            turnAnglePark = 90-turnAngleSpike;
        } else{
            forwardDistance = 25;
            turnAngleSpike = -45;
            turnAnglePark = 90-turnAngleSpike;
        }

        startX += startXOff;
        startY += startYOff;
        drive.setPoseEstimate(new Pose2d(startX, startY, startHeading));
        drive.setExternalHeading(startHeading);
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))
                .forward(forwardDistance)
                .turn(Math.toRadians(turnAngleSpike))
                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                    //Add gripper drop code
                })
                .waitSeconds(2)
                .turn(Math.toRadians(turnAnglePark))
                .forward(36)
                .build();
        drive.followTrajectorySequenceAsync(trajectory);

        while(opModeIsActive()){
            drive.update();
        }
    }
}
