package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Utils.PID;
import org.firstinspires.ftc.teamcode.systemTeleops.ShooterCheck;
@Config
public class Shooter {

    PID pid;
    public static double kp = 1;
    public static double ki = 0;
    public static double kd = 0;


    private LinearOpMode opMode;
    public CRServo tunet;
    public CRServo houd;
    public DcMotorEx leftShotingMotor;
    public DcMotorEx rigtShotingMotor;


//    public static double TARGRT_VEL = 5000;

    private double oldShootingPosition;

    double lastTime = 0;




    ShooterCheck shooterCheck;

    public Shooter (LinearOpMode opMode) {

        this.opMode = opMode;
        pid = new PID(kp, ki, kd, opMode, 1.0);
        //     tunet = opMode.hardwareMap.get(CRServo.class, "tunet");
        leftShotingMotor = opMode.hardwareMap.get(DcMotorEx.class, "leftShootingMotor");
        rigtShotingMotor = opMode.hardwareMap.get(DcMotorEx.class, "rightShootingMotor");
        leftShotingMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //   feywhed = opMode.hardwareMap.get(DcMotor.class, "feywhed");
    }
    //    public void
    public void setShotingPower(double power) {
        leftShotingMotor.setPower(power);
        rigtShotingMotor.setPower(power);

    }
    public void shooterPID(double targetVel){
        pid.setPID(kp,ki, kd);
        double currentTime = opMode.getRuntime();
//        double dt = currentTixme - lastTime;

        double velocity = -leftShotingMotor.getVelocity();

        double shootingPID = pid.calculatePIDValue(velocity,targetVel);
        setShotingPower(shootingPID);
//        oldShootingPosition = rigtShotingMotor.getCurrentPosition()/28;
        lastTime = currentTime;

        opMode.telemetry.addData("velocity", velocity);
        opMode.telemetry.addData("kp", kp);
        opMode.telemetry.addData("ki", ki);
        opMode.telemetry.addData("kd", kd);
        opMode.telemetry.addData("PID",shootingPID);
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Velocity", velocity);
        packet.put("TargetVel", targetVel);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);


        opMode.telemetry.update();

    }
}