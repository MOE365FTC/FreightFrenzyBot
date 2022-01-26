package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController{

    // basic setup
    protected DcMotor motor;
    protected boolean isBusy = false;
    protected int target;
    protected LinearOpMode opMode;

    // PID stuff
    protected double kp, ki, kd;
    protected int tolerance;
    protected double integralCap = 0;

    public PIDController(DcMotor motor, double kp, double ki, double kd, int tolerance, double integralCap){
        this.motor = motor;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.tolerance = tolerance;
        this.integralCap = 0;
    }

    public void setOpMode(LinearOpMode opMode){
        this.opMode = opMode;
    }

    public void setTarget(int target){
        this.target = target;
    }

    public boolean isBusy(){
        return this.isBusy;
    }

    public void moveAuton(){ // call this to actually move the motor.
        PIDRunnable run = new PIDRunnable(this);
        Thread thread = new Thread(run);
        thread.start();
    }

    public void moveTeleop(){
        double lastError = 0;
        double errorSum = 0;
        double error = this.target - this.motor.getCurrentPosition();
        this.isBusy = true;
        if(Math.abs(error) > this.tolerance){
            error = this.target - this.motor.getCurrentPosition();
            double power = error * this.kp + errorSum * this.ki + (error - lastError) * this.kd;
            this.motor.setPower(power);
            errorSum += error;
            if (Math.abs(errorSum) > this.integralCap){
                errorSum = Math.signum(errorSum) * this.integralCap;
            }
            lastError = error;
        } else {
            this.motor.setPower(0);
        }
    }
}

class PIDRunnable implements Runnable {
    PIDController pid;

    public PIDRunnable(PIDController pid){
        this.pid = pid;
    }

    public void run(){
        double lastError = 0;
        double errorSum = 0;
        double error = pid.target - pid.motor.getCurrentPosition();
        pid.isBusy = true;
        while(pid.opMode.opModeIsActive() && Math.abs(error) > pid.tolerance){
            error = pid.target - pid.motor.getCurrentPosition();
            double power = error * pid.kp + errorSum * pid.ki + (error - lastError) * pid.kd;
            pid.motor.setPower(power);
            errorSum += error;
            if (Math.abs(errorSum) > pid.integralCap){
                errorSum = Math.signum(errorSum) * pid.integralCap;
            }
            lastError = error;
        }
        pid.motor.setPower(0);
        pid.isBusy = false;

    }
}