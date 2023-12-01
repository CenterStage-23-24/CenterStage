
package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons.Arm;

public class TransferController {
    private int index_increment = 0; //config as needed in CM
    private static final int OFFSET_INCREMENT = 0; //config as needed in CM
    private static final int MAX_SLIDE_HEIGHT = 1000; //convert to CM
    private static final int MIN_SLIDE_HEIGHT = 0; //config as needed in CM
    private int slideIndexPos = MIN_SLIDE_HEIGHT;
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
        slides.setTargetPos(slides.mmToTicks(slideIndexPos));
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
        boolean armAtPos = arm.axonAtPos(arm.getIntakePos(), BUFFER);
        if (armAtPos && delay()) {
            slides.setTargetPos(0);
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

    public void pos_up(){
        int tempPos = slideIndexPos += index_increment;
        if(tempPos < MAX_SLIDE_HEIGHT){
            slideIndexPos = tempPos;
        }
    }

    public void pos_down(){
        int tempPos = slideIndexPos -= index_increment;
        if(tempPos >= MIN_SLIDE_HEIGHT){
            slideIndexPos = tempPos;
        }
    }

    public void offset_up(){
        index_increment += OFFSET_INCREMENT;
    }
    public void offset_down(){
        index_increment -= OFFSET_INCREMENT;
    }
}

