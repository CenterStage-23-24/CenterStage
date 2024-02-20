package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Auto.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class CVRelocalizer {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private LinearOpMode opMode;
    private SampleMecanumDrive drive;

    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    private final PowerVector ZERO_VECTOR = new PowerVector(0.0, 0.0, 0.0, 0.0);
    private final PowerVector MAX_VECTOR = new PowerVector(1.0, 1.0, 1.0, 1.0);
    private final PowerVector VERTICAL_VECTOR = new PowerVector(1.0, 1.0, 1.0, 1.0);
    private final PowerVector HORIZONTAL_VECTOR = new PowerVector(1.0, -1.0, 1.0, -1.0);
    private final PowerVector TURN_VECTOR = new PowerVector(-1.0, -1.0, 1.0, 1.0);

    private final Pose ZERO_POSE = new Pose(0.0, 0.0, 0.0);

    private Pose initialPose = ZERO_POSE.clone();
    private Pose currentPose = ZERO_POSE.clone();
    private Pose currentErrorPose = ZERO_POSE.clone();

    private int tagId;

    public CVRelocalizer(LinearOpMode opMode, SampleMecanumDrive drive, String webcamName) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
        this.drive = drive;

        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, webcamName), tagProcessor);
    }

    public void init(Pose initialPose) {
        setupExposure();
        initialPose = initialPose.clone();
    }

    private void setupExposure() {
        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!opMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                opMode.sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            opMode.sleep(50);
        }
        exposureControl.setExposure((long)1, TimeUnit.MILLISECONDS);
        opMode.sleep(20);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(255);
        opMode.sleep(20);
    }

    public void setTagId(int newTagId) {
        tagId = newTagId;
    }

    public static class Pose {

        public double heading;
        public double horizontal;
        public double vertical;

        public Pose(double heading, double vertical, double horizontal) {
            this.heading = heading;
            this.vertical = vertical;
            this.horizontal = horizontal;
        }

        public Pose clone() {
            return new Pose(heading, vertical, horizontal);
        }
    }

    public static class MoveDescription {

        public double time;
        public Pose targetPose;
        public double headingWeight;
        public double translationWeight;
        public double correctionSpeed;
        public boolean headingScaling;
        public String description;

        public MoveDescription(double time, Pose targetPose, double headingWeight, double translationWeight, double correctionSpeed, boolean headingScaling, String description) {
            this.time = time;
            this.targetPose = targetPose.clone();
            this.headingWeight = headingWeight;
            this.translationWeight = translationWeight;
            this.correctionSpeed = correctionSpeed;
            this.headingScaling = headingScaling;
            this.description = description;
        }
    }

    public void moveToTarget(MoveDescription moveDescription) {
        double time = moveDescription.time;
        Pose targetPose = moveDescription.targetPose;
        double headingWeight = moveDescription.headingWeight;
        double translationWeight = moveDescription.translationWeight;
        double correctionSpeed = moveDescription.correctionSpeed;
        boolean headingScaling = moveDescription.headingScaling;
        String description = moveDescription.description;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.time() < time) {
            telemetry.addLine("Running " + description);
            telemetry.addData("ID", tagId);

            AprilTagPoseFtc ftcPose = getFtcPose(tagId);

            if (ftcPose == null) {
                telemetry.addLine("April Tag was not detected.");
                telemetry.update();
                setMotorPowersDestructured(ZERO_VECTOR.clone());
                continue;
            }

            double timeScaling = 1.0;
            if (headingScaling) {
                timeScaling = 1.0 - (timer.time() / time);
            }

            PowerVector motorPowers = ZERO_VECTOR
                    .add(headingAlignment(targetPose.heading, ftcPose).scale(headingWeight * timeScaling))
                    .add(verticalAlignment(targetPose.vertical, ftcPose).scale(translationWeight))
                    .add(horizontalAlignment(targetPose.horizontal, ftcPose).scale(translationWeight))
                    .limit(correctionSpeed);

            telemetry.addData("timer.time()", timer.time());
            telemetry.addData("motorPowers", motorPowers);
            printDebugInfo();

            setMotorPowersDestructured(motorPowers);
            telemetry.update();
        }

        initialPose = targetPose.clone();
    }

    private double getHeadingError(double target, AprilTagPoseFtc ftcPose) {
        target = target / 180.0;
        double yaw = ftcPose.yaw / 180.0;

        double headingError = yaw - target;

        currentPose.heading = ftcPose.yaw;
        currentErrorPose.heading = headingError;

        return headingError;
    }

    private PowerVector headingAlignment(double target, AprilTagPoseFtc ftcPose) {
        double headingError = getHeadingError(target, ftcPose);
        return TURN_VECTOR.scale(-headingError).limit(1.0);
    }

    private double getVerticalError(double target, AprilTagPoseFtc ftcPose) {
        // Negative sign added in front because ftcPose.y decreases as robot moves forward
        double verticalError = -(ftcPose.y - target) / Math.abs(target - initialPose.vertical);

        currentPose.vertical = ftcPose.y;
        currentErrorPose.vertical = verticalError;

        return verticalError;
    }

    private PowerVector verticalAlignment(double target, AprilTagPoseFtc ftcPose) {
        double verticalError = getVerticalError(target, ftcPose);
        return VERTICAL_VECTOR.scale(-verticalError).limit(1.0);
    }

    private double getHorizontalError(double target, AprilTagPoseFtc ftcPose) {
        double horizontalError = (ftcPose.x - target) / Math.abs(target - initialPose.horizontal);

        currentPose.horizontal = ftcPose.x;
        currentErrorPose.horizontal = horizontalError;

        return horizontalError;
    }

    private PowerVector horizontalAlignment(double target, AprilTagPoseFtc ftcPose) {
        double horizontalError = getHorizontalError(target, ftcPose);
        return HORIZONTAL_VECTOR.scale(-horizontalError).limit(1.0);
    }

    private AprilTagPoseFtc getFtcPose(int id) {
        AprilTagDetection tag = getTagFromId(id);

        if (tag == null)
            return null;

        return tag.ftcPose;
    }

    private void setMotorPowersDestructured(PowerVector vector)  {
        drive.setMotorPowers(
                vector.getLeftFrontPower(),
                vector.getLeftBackPower(),
                vector.getRightBackPower(),
                vector.getRightFrontPower()
        );
    }

    private void processDebugInfo() {
        AprilTagPoseFtc ftcPose = getFtcPose(tagId);

        if (ftcPose == null) {
            telemetry.addLine("April Tag was not detected.");
            return;
        }

        getHeadingError(currentPose.heading, ftcPose);
        getVerticalError(currentPose.vertical, ftcPose);
        getHorizontalError(currentPose.horizontal, ftcPose);
        printDebugInfo();
    }

    private void printDebugInfo() {
        telemetry.addData("Current Heading:", currentPose.heading);
        telemetry.addData("Current Heading Error:", currentErrorPose.heading);

        telemetry.addData("Current Vertical:", currentPose.vertical);
        telemetry.addData("Current Vertical Error:", currentErrorPose.vertical);

        telemetry.addData("Current Horizontal:", currentPose.horizontal);
        telemetry.addData("Current Horizontal Error:", currentErrorPose.horizontal);
    }

    private AprilTagDetection getTagFromId(int id) {
        for(AprilTagDetection tag : tagProcessor.getDetections()){
            if(tag.id == id){
                return tag;
            }
        }

        return null;
    }
}
