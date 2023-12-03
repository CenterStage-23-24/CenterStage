
package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons.Arm;

public class TransferController {

    private static final int default_index_inc = 10; //only for telemetry purposes
    private static final int index_increment = default_index_inc; //config as needed in CM
    private static final int OFFSET_INCREMENT = 5; //config as needed in CM
    private static final int MAX_SLIDE_HEIGHT = 64; //convert to CM
    private static final int SAFE_HEIGHT = 24;
    private int min_slide_height = SAFE_HEIGHT; //config as needed in CM

/*
    private static final int default_index_inc = 2; //only for telemetry purposes
    private static final int index_increment = default_index_inc; //config as needed in CM
    private static final int OFFSET_INCREMENT = 5; //config as needed in CM
    private static final int MAX_SLIDE_HEIGHT = 17; //convert to CM
    private static final int SAFE_HEIGHT = 2;
    private int min_slide_height = SAFE_HEIGHT; //config as needed in CM
*/

    private int slideIndexPos = min_slide_height;
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
        //telemetry.addData("SLIDE TARGET POS?", slides.mmToTicks(slideIndexPos));
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
        int tempPos = slideIndexPos + index_increment;
        if(tempPos < MAX_SLIDE_HEIGHT){
            slideIndexPos = tempPos;
            slides.setTargetPos(slides.mmToTicks(slideIndexPos));
        }
    }

    public void pos_down(){
        int tempPos = slideIndexPos - index_increment;
        if(tempPos >= min_slide_height){
            slideIndexPos = tempPos;
            slides.setTargetPos(slides.mmToTicks(slideIndexPos));
        }
    }

    public void offset_up(){
        /*
        slideIndexPos += OFFSET_INCREMENT;
        min_slide_height += OFFSET_INCREMENT;
        slides.setTargetPos(slides.mmToTicks(slideIndexPos));
         */
        int tempPos = min_slide_height + OFFSET_INCREMENT;
        if(tempPos < MAX_HEIGHT){
            min_slide_height = tempPos;
            slideIndexPos += OFFSET_INCREMENT;
            slides.setTargetPos(slides.mmToTicks(slideIndexPos));
        }
    }
    public void offset_down(){
        /*
        slideIndexPos -= OFFSET_INCREMENT;
        min_slide_height -= OFFSET_INCREMENT;
        slides.setTargetPos(slides.mmToTicks(slideIndexPos));
         */
        int tempPos = min_slide_height - OFFSET_INCREMENT;
        if(tempPos >= SAFE_HEIGHT){
            min_slide_height = tempPos;
            slideIndexPos -= OFFSET_INCREMENT;
            slides.setTargetPos(slides.mmToTicks(slideIndexPos));
        }
    }

    public void telem(){
        telemetry.addData("OFFSET: ", OFFSET_INCREMENT);
        telemetry.addData("INDEX INC: ", index_increment);
        telemetry.addData("SLIDE INDEX: ", slideIndexPos);
    }
}

