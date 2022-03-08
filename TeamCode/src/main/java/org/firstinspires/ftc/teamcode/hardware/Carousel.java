package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carousel {
    Gamepad gamepad1;
    CRServo spinner;

    double spinPower = 0.25;

    public Carousel(HardwareMap hardwareMap, Gamepad gpad1){
        this.gamepad1 = gpad1;
        spinner = hardwareMap.crservo.get("CSS10");
    }

    public void actuate() {
        if(gamepad1.right_bumper){
            spinPower = 1;
        } else{
            spinPower = 0.25;
        }
        if(gamepad1.dpad_left) {
            spinner.setPower(spinPower);
        } else if(gamepad1.dpad_right){
            spinner.setPower(-spinPower);
        } else{
            spinner.setPower(0.0);
        }
    }

    public void startRed(){
        spinner.setPower(-spinPower);
    }

    public void startBlue(){
        spinner.setPower(spinPower);
    }

    public void stop(){
        spinner.setPower(0.0);
    }
}
