package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.enums.DispenserPivot;
import org.firstinspires.ftc.teamcode.enums.SlideSetting;
import org.firstinspires.ftc.teamcode.enums.SlideState;

public class Dispenser {
    Gamepad gamepad2;
    Servo dispenseTilt, dispensePivot, dispenseGate;
    Slides slides;

    DispenserPivot curDispenserState = DispenserPivot.CENTER;
    final double tiltMinimum = 0.1012; //Min dispenser tilt servo position before pivoting servo (must not turn into the slides) (used for pivot)

    //Tilt Constants:
    final double TILT_FOR_90 = 0.35;
    final double TILT_FOR_RESET = 0.0;

    //Gate Constants:
    final double GATE_OPEN = 0.45;
    final double GATE_CLOSED = 1.0;

    //Pivot Constants:
    final double PIVOT_CENTER = 0.48;
    final double PIVOT_LEFT = 0.8;
    final double PIVOT_RIGHT = 0.15;

    public Dispenser(HardwareMap hardwareMap, Gamepad gpad2, Slides slides){
        this.gamepad2 = gpad2;
        this.slides = slides;

        dispenseTilt = hardwareMap.get(Servo.class, "DRS11");
        dispensePivot = hardwareMap.get(Servo.class, "DPS12");
        dispenseGate = hardwareMap.get(Servo.class, "DGS13");
    }

    public void actuate(){
        actuateTilt();
        actuateGate();
        actuatePivot();
    }

    void actuateTilt(){
        if(slides.getCurrentExtension() > slides.extendMinimum && slides.curSlideSetting == SlideSetting.EXTEND && slides.curSlideState == SlideState.EXTENDED) {
            dispenseTilt.setPosition(Math.max(TILT_FOR_RESET, ((slides.getCurrentExtension()- slides.rotateTicsDeltaToVertical) / slides.tiltTicsFor90degrees) * TILT_FOR_90));
        } else{
            dispenseTilt.setPosition(TILT_FOR_RESET);
        }
    }

    void actuateGate(){
        if(slides.getCurrentExtension() > slides.extendMinimum && slides.curSlideSetting == SlideSetting.EXTEND && slides.curSlideState == SlideState.EXTENDED) {
            if(gamepad2.left_bumper){
                dispenseGate.setPosition(GATE_OPEN);
            } else {
                dispenseGate.setPosition(GATE_CLOSED);
            }
        } else {
            dispenseGate.setPosition(GATE_CLOSED);
        }
    }

    public void setPivot(){
        if(dispenseTilt.getPosition() > tiltMinimum && slides.getCurrentExtension() > slides.extendMinimum  && slides.curSlideSetting == SlideSetting.EXTEND && slides.curSlideState == SlideState.EXTENDED){
            if(gamepad2.dpad_up){
                curDispenserState = DispenserPivot.CENTER;
            } else if(gamepad2.dpad_left){
                curDispenserState = DispenserPivot.LEFT;
            } else if(gamepad2.dpad_right){
                curDispenserState = DispenserPivot.RIGHT;
            }
        } else {
            dispensePivot.setPosition(PIVOT_CENTER);
            curDispenserState = DispenserPivot.CENTER;
        }
    }

    void actuatePivot(){
        if(curDispenserState == DispenserPivot.CENTER){
            dispensePivot.setPosition(PIVOT_CENTER);
        } else if(curDispenserState == DispenserPivot.LEFT){
            dispensePivot.setPosition(PIVOT_LEFT);
        } else if(curDispenserState == DispenserPivot.RIGHT){
            dispensePivot.setPosition(PIVOT_RIGHT);
        }
    }

    public void setTilt(double position){
        dispenseTilt.setPosition(position);
    }

    public void setGateOpen(boolean open){
        if(open)
            dispenseGate.setPosition(GATE_OPEN);
        else
            dispenseGate.setPosition(GATE_CLOSED);
    }
    public void composeTelemetry(Telemetry telemetry){
        telemetry.addLine("--DISPENSER--");
        telemetry.addData("TILT", dispenseTilt.getPosition());
        telemetry.addData("PIVOT", dispensePivot.getPosition());
        telemetry.addData("GATE", dispenseGate.getPosition());
    }
}
