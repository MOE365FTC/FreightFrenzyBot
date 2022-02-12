package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController{

    // basic setup
    protected DcMotor motor;
    protected boolean isBusy = false;
    protected int target;
    protected int oldTarget = 0;
    protected double lastError = 0;
    protected double errorSum = 0;
    protected LinearOpMode opMode;

    // PID stuff
    protected double kp, ki, kd;
    protected int tolerance;
    protected double integralCap; //If 0 there is no cap

    public PIDController(DcMotor motor, double kp, double ki, double kd, int tolerance, double integralCap){
        this.motor = motor;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.tolerance = tolerance;
        this.integralCap = integralCap;
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
    public double power;
    public double error;
    public void moveTeleop(){
        if(this.target != this.oldTarget){
            this.errorSum = 0;
            this.lastError = 0;
            this.oldTarget = this.target;
        }
        error = this.target - this.motor.getCurrentPosition();
        if(Math.abs(error) > this.tolerance){
            error = this.target - this.motor.getCurrentPosition();
            power = error * this.kp + errorSum * this.ki + (error - lastError) * this.kd;
            errorSum += error;
            if (this.integralCap > 0 && Math.abs(errorSum) > this.integralCap){
                errorSum = Math.signum(errorSum) * this.integralCap;
            }
            lastError = error;
        } else {
            power = 0;
        }
        this.motor.setPower(power);

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
            if (pid.integralCap > 0 && Math.abs(errorSum) > pid.integralCap){
                errorSum = Math.signum(errorSum) * pid.integralCap;
            }
            lastError = error;
        }
        pid.motor.setPower(0);
        pid.isBusy = false;
    }
}