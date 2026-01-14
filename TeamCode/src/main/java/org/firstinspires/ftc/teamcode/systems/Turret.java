package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Utils.PID;
@Config

public class Turret {


    private IMU imu;
    Camera camera;
    Wheels wheels;

    private final double MIN_TURRET_ANGLE = -400; //ToDo find real limits
    private final double MAX_TURRET_ANGLE = 400;


    private LinearOpMode opMode;
    public CRServo rightTurret;
    public CRServo leftTurret;
    public AnalogInput turretAnalog ;
    private double turretOldPos;
    private double trueRotation;
    public static double kp = 0.009;
    public static double ki = 0;
    public static double kd = 0;
//    public static double targetRotation = 180;

    PID pid;
    public Turret(LinearOpMode opMode, IMU imu) {
        pid = new PID(kp, ki, kd, opMode, 1);
        this.imu = imu;
        this.opMode = opMode;
        rightTurret =opMode.hardwareMap.get(CRServo.class, "rightTurret");
        leftTurret = opMode.hardwareMap.get(CRServo.class, "leftTurret");
        turretAnalog = opMode.hardwareMap.get(AnalogInput.class, "turretAnalog");


    }
    public void moveTurret(double power) {
        rightTurret.setPower(power);
        leftTurret.setPower(power);
    }



    public static double getRotationOfInput(AnalogInput input) {
        return (input.getVoltage() / input.getMaxVoltage()) * 360;
    }

    public void updateTurretServoRotation() {
        double currentRotation = getRotationOfInput(turretAnalog);
        double diff = currentRotation - turretOldPos;

        double newRotationEstimate = 165;
        if(Math.abs(diff) > newRotationEstimate){
            //new rotation occur
            if(diff < 0)
                diff += 360; //add rotation
            else
                diff -= 360; //minus rotation

        }
        turretOldPos = currentRotation;
        trueRotation += diff;
        opMode.telemetry.addData("rottion", trueRotation);

    }
    public double turretPID(double targetRotation){
        double pidd = pid.calculatePIDValue(trueRotation+180, targetRotation);
        opMode.telemetry.addData("pid", pidd);

        return pidd;

    }
    public void setTurretFinalPosition(double targetRotation){
        //check if the turret is in limit and fix it
        if (targetRotation > MAX_TURRET_ANGLE){
            leftTurret.setPower(turretPID(targetRotation - 360));
            rightTurret.setPower(turretPID(targetRotation - 360));
        }
        else if (targetRotation < MIN_TURRET_ANGLE){
            leftTurret.setPower(turretPID(targetRotation + 360));
            rightTurret.setPower(turretPID(targetRotation + 360));
        }
        else {
            // if the turret is in limit aim with camera
            turnWithCamera();
        }
    }
    public double calculateTargetRotation() {
        return imu.getRobotYawPitchRollAngles().getYaw()+
//         Math.toDegrees(Math.atan2(opMode.gamepad1.left_stick_y, opMode.gamepad1.left_stick_x)) + 180;
         wheels.getAbsoluteAngle();
    }
    public void turnWithCamera(){
        double erroretion = camera.returnBearing();
        // if there is detection aim with camera
        if (erroretion != -999){
            double poweretion = pid.calculatePIDValue(erroretion, 0);
            leftTurret.setPower(poweretion);
            rightTurret.setPower(poweretion);
        }
        // when no detection aim with imu
        else {
            leftTurret.setPower(turretPID(calculateTargetRotation()));
            rightTurret.setPower(turretPID(calculateTargetRotation()));
        }
    }
    public void setTurretPosition(){
        moveTurret(-turretPID(calculateTargetRotation()));
    }
}



