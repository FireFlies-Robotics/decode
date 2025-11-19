package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.PID;

public class Shooter {

    PID pid;
    double kp = 0;
    double ki = 0;
    double kd = 0;


    private LinearOpMode opMode;
    public CRServo tunet;
    public CRServo houd;
    public DcMotor shotingMotor;

    private final double TARGRT_VEL = 10000;

    private double oldShootingPosition;

    double lastTime = 0;






    public Shooter (LinearOpMode opMode) {
        this.opMode = opMode;
   //     tunet = opMode.hardwareMap.get(CRServo.class, "tunet");
        shotingMotor = opMode.hardwareMap.get(DcMotor.class, "shotingMotor");
     //   feywhed = opMode.hardwareMap.get(DcMotor.class, "feywhed");
    }
//    public void
    public void setShotingPower(double power) {
        shotingMotor.setPower(power);
    }
    public void shooterPID(){
        double currentTime = opMode.getRuntime();
        double dt = currentTime - lastTime;

        double velocity = (shotingMotor.getCurrentPosition()/28 - oldShootingPosition) /dt;


        setShotingPower(pid.calculatePIDValue(velocity, TARGRT_VEL));
        oldShootingPosition = shotingMotor.getCurrentPosition()/28;
        lastTime = currentTime;
    }
}
