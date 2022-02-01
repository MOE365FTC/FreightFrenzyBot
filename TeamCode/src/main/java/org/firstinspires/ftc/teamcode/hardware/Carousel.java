package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carousel {
    Gamepad gamepad1;
    CRServo spinnerRight, spinnerLeft;

    final double spinPower = 1.0;

    public Carousel(HardwareMap hardwareMap, Gamepad gpad1){
        this.gamepad1 = gpad1;
        spinnerRight = hardwareMap.crservo.get("CRS00");
        spinnerLeft = hardwareMap.crservo.get("CLS10");
    }

    public void actuate() {
        if(gamepad1.dpad_left) {
            spinnerRight.setPower(spinPower);
            spinnerLeft.setPower(spinPower);
        } else if(gamepad1.dpad_right){
            spinnerRight.setPower(-spinPower);
            spinnerLeft.setPower(-spinPower);
        } else{
            spinnerRight.setPower(0.0);
            spinnerLeft.setPower(0.0);
        }
    }
}
