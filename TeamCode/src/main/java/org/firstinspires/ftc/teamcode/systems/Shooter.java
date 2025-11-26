package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Utils.PID;
import org.firstinspires.ftc.teamcode.systemTeleops.ShooterCheck;
@Config
public class Shooter {

    PID pid;
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;


    private LinearOpMode opMode;
    public CRServo tunet;
    public CRServo houd;
    public DcMotor leftShotingMotor;
    public DcMotor rigtShotingMotor;


    public static double TARGRT_VEL = 5000;

    private double oldShootingPosition;

    double lastTime = 0;




    ShooterCheck shooterCheck;

    public Shooter (LinearOpMode opMode) {

        this.opMode = opMode;
        pid = new PID(kp, ki, kd, opMode);
   //     tunet = opMode.hardwareMap.get(CRServo.class, "tunet");
        leftShotingMotor = opMode.hardwareMap.get(DcMotor.class, "leftShotingMotor");
        rigtShotingMotor = opMode.hardwareMap.get(DcMotor.class, "rightShotingMotor");
        rigtShotingMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //   feywhed = opMode.hardwareMap.get(DcMotor.class, "feywhed");
    }
//    public void
    public void setShotingPower(double power) {
        leftShotingMotor.setPower(power);
        rigtShotingMotor.setPower(power);

    }
    public void shooterPID(){
        pid.setPID(kp,ki, kd);
        double currentTime = opMode.getRuntime();
        double dt = currentTime - lastTime;

        double velocity = (rigtShotingMotor.getCurrentPosition()/28 - oldShootingPosition) /dt;


        setShotingPower(pid.calculatePIDValue(velocity, TARGRT_VEL));
        oldShootingPosition = rigtShotingMotor.getCurrentPosition()/28;
        lastTime = currentTime;
        opMode.telemetry.addData("velocity", velocity);
    }
}
