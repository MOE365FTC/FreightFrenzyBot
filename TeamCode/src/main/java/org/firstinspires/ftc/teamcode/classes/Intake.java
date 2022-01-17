package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.classes.enums.SlideState;

public class Intake {
    Gamepad gamepad2;
    DcMotor intake;
    Slides slides;
    public Intake(HardwareMap hardwareMap, Gamepad gpad2, Slides slides){
        this.gamepad2 = gpad2;
        this.slides = slides;

        intake = hardwareMap.dcMotor.get("INM23");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void actuateIntake() {
        if(gamepad2.right_trigger > 0 && slides.curSlideState == SlideState.RETRACTED){
            intake.setPower(gamepad2.right_trigger);
        } else if(gamepad2.right_bumper){
            intake.setPower(-1.0);
        } else{
            intake.setPower(0.0);
        }
    }
}
