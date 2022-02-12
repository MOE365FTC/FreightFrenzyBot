package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.enums.SlideState;

public class Intake {
    Gamepad gamepad2;
    DcMotor intake;
    Slides slides;
    Dispenser dispenser;
    public Intake(HardwareMap hardwareMap, Gamepad gpad2, Slides slides, Dispenser dispenser){
        this.gamepad2 = gpad2;
        this.slides = slides;
        this.dispenser = dispenser;

        intake = hardwareMap.dcMotor.get("INM12");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void actuateIntake() {
        if(gamepad2.right_trigger > 0 && slides.curSlideState == SlideState.RETRACTED){
            intake.setPower(gamepad2.right_trigger);
//            if(this.dispenser.hasFreight()){
//                gamepad2.rumble(50);
//            }
        } else if(gamepad2.right_bumper){
            intake.setPower(-1.0);
        } else{
            intake.setPower(0.0);
        }
    }
}
