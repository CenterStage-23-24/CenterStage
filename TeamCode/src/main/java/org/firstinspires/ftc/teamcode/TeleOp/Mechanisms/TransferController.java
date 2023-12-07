
package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons.Arm;

public class TransferController {
//ROBOT MEASUREMENT CONSTANTS:
    private static final double index_increment = 6.6; //config as needed in CM
    private static final int OFFSET_INCREMENT = 1; //config as needed in CM
    private static final int MAX_SLIDE_HEIGHT = 64; //convert to CM
    private static final int SAFE_HEIGHT = 24;

/*
TEST BENCH MEASUREMENT CONSTANTS:
    private static final int index_increment = 2; //config as needed in CM
    private static final int OFFSET_INCREMENT = 5; //config as needed in CM
    private static final int MAX_SLIDE_HEIGHT = 17; //convert to CM
    private static final int SAFE_HEIGHT = 2;
*/

    private int min_slide_height = SAFE_HEIGHT; //config as needed in CM
    private double slideIndexPos = min_slide_height;
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
        telemetry.addData("armAtPos:", armAtPos);
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
        double tempPos = slideIndexPos + index_increment;
        if(tempPos < MAX_SLIDE_HEIGHT){
            slideIndexPos = tempPos;
            slides.setTargetPos(slides.mmToTicks(slideIndexPos));
        }
    }

    public void pos_down(){
        double tempPos = slideIndexPos - index_increment;
        if(tempPos >= min_slide_height){
            slideIndexPos = tempPos;
            slides.setTargetPos(slides.mmToTicks(slideIndexPos));
        }
    }

    public void offset_up(){
        int tempMinPos = min_slide_height + OFFSET_INCREMENT;
        double tempIndexPos = slideIndexPos + OFFSET_INCREMENT;
        if(tempMinPos < MAX_SLIDE_HEIGHT && tempIndexPos < MAX_SLIDE_HEIGHT){
            min_slide_height = tempMinPos;
            slideIndexPos = tempIndexPos;
            slides.setTargetPos(slides.mmToTicks(slideIndexPos));
        }
    }
    public void offset_down(){
        int tempMinPos = min_slide_height - OFFSET_INCREMENT;
        double tempIndexPos = slideIndexPos - OFFSET_INCREMENT;
        if(tempMinPos >= SAFE_HEIGHT && tempIndexPos >= SAFE_HEIGHT){
            min_slide_height = tempMinPos;
            slideIndexPos = tempIndexPos;
            slides.setTargetPos(slides.mmToTicks(slideIndexPos));
        }
    }

    public void telem(){
        telemetry.addData("OFFSET: ", OFFSET_INCREMENT);
        telemetry.addData("INDEX INC: ", index_increment);
        telemetry.addData("SLIDE INDEX: ", slideIndexPos);
        telemetry.addData("MIN SLIDE HEIGHT: ", min_slide_height);
    }
}