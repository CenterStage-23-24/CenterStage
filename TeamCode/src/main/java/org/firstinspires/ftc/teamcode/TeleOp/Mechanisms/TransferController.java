package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.FSMs.Cycle;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons.Arm;

public class TransferController {
    private static final int MAX_HEIGHT = 50;
    private static final int MIN_HEIGHT = 0;
    private static final int INCREMENT = 10;
    private static final int BUFFER = 20;
    private static final int DELAY_MS = 750;
    private final Arm arm;
    private final Slides slides;
    private final Telemetry telemetry;
    private final ElapsedTime bufferTime = new ElapsedTime();
    private double startTS;
    private RetractState retractState;
    private enum RetractState{
        NOT_STARTED,
        STARTED,
    }

    public TransferController(Arm arm, Slides slides, Telemetry telemetry){
        this.arm = arm;
        this.slides = slides;
        this.telemetry = telemetry;
        retractState = RetractState.NOT_STARTED;
    }

    public boolean extend(){
        telemetry.addData("atPos?", slides.atPos());
        slides.setTargetPos(slides.mmToTicks(MAX_HEIGHT));
        if (slides.atPos()) {
            arm.goToDeposit();
            return true;
        }
        return false;
    }

    public boolean retract(){
        if(retractState == RetractState.NOT_STARTED) {
            startTS = bufferTime.milliseconds();
            retractState = RetractState.STARTED;
        }

        arm.goToIntake();
        boolean armAtPos = arm.axonAtPos(Arm.intakePos, BUFFER);
        if (armAtPos && delay()) {
            slides.setTargetPos(MIN_HEIGHT);
            if(slides.atPos()){
                retractState = RetractState.NOT_STARTED;
                return true;
            }
        }
        return false;
    }

    private boolean delay() {
        double finalTS = bufferTime.milliseconds();
        return (finalTS - startTS) >= DELAY_MS;
    }
}
