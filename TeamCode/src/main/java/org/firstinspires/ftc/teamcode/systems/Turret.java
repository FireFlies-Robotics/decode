package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utils.PID;
@Config

public class Turret {

    private IMU imu;
    Camera camera;
    Wheels wheels;

    // Fixed offset you can tune in FTC Dashboard
    public static double SENSOR_ZERO_OFFSET = 146;

    private double turretOldPos = 0;
    private double turretRotation = 0;

    private final double MIN_TURRET_ANGLE = 0;
    private final double MAX_TURRET_ANGLE = 360;

    private LinearOpMode opMode;
    public CRServo rightTurret;
    public CRServo leftTurret;
    public AnalogInput turretAnalog;

    public static double kp = 0.0085;
    public static double ki = 0;
    public static double kd = 0.0003;

    PID pid;
    public static double smallkp = 0.045;
    public static double smallki = 0;
    public static double smallkd = 0.0004;
    PID smallpid;

    public Turret(LinearOpMode opMode, IMU imu, Camera camera) {
        pid = new PID(kp, ki, kd, opMode, 1);
        smallpid = new PID(smallkp, smallki, smallkd, opMode ,1);
        this.camera = camera;

        this.imu = imu;
        this.opMode = opMode;
        rightTurret = opMode.hardwareMap.get(CRServo.class, "rightTurret");
        leftTurret = opMode.hardwareMap.get(CRServo.class, "leftTurret");
        turretAnalog = opMode.hardwareMap.get(AnalogInput.class, "turretAnalog");

    }

    public void init() {

        // Just read the current position with offset applied
        turretOldPos = getRotationOfInput();

        turretRotation = 0; // We say this is zero
//        FtcDashboard.getInstance().startCameraStream();
        opMode.telemetry.addData("Sensor at Init", turretOldPos);
        opMode.telemetry.addLine("Make sure turret is at ZERO position!");
        opMode.telemetry.update();
    }

    // Get sensor reading with optional fixed offset
    public double getRotationOfInput() {
        opMode.telemetry.addData("Sensor Voltage", turretAnalog.getVoltage());
        opMode.telemetry.addData("Max Voltage", turretAnalog.getMaxVoltage());
        opMode.telemetry.addData("Calculated Angle", (turretAnalog.getVoltage() / turretAnalog.getMaxVoltage()));
        return ((turretAnalog.getVoltage() / turretAnalog.getMaxVoltage()) * 360) - SENSOR_ZERO_OFFSET;

    }

    public double getTurretRotation() {
        return turretRotation;
    }

    public void moveTurret(double power) {
        rightTurret.setPower(power);
        leftTurret.setPower(power);
    }

    public void updateTurretServoRotation() {
        double currentRotation = getRotationOfInput();
        double diff = currentRotation - turretOldPos;

        // Handle 360° wraparound
        if (Math.abs(diff) > 180) {
            if (diff < 0)
                diff += 360;
            else
                diff -= 360;
        }

        turretOldPos = currentRotation;
        turretRotation += diff;

        opMode.telemetry.addData("Sensor Reading", currentRotation);
        opMode.telemetry.addData("Turret Rotation", turretRotation);
    }

    public double turretPID(double targetRotation) {
        pid.setPID(kp, ki, kd);

        double pidd = -pid.calculatePIDValue(getRotationOfInput(), targetRotation);
        opMode.telemetry.addData("PID Output", pidd);
        return pidd;
    }

    public void setTurretPosition(double target) {
        moveTurret(turretPID(target));
    }

//    public void setTurretFinalPosition() {
//        if (calculateTargetRotation() > MAX_TURRET_ANGLE) {
//            setTurretPositionWithOffset(-360);
//        }
//        else if (calculateTargetRotation() < MIN_TURRET_ANGLE) {
//            setTurretPositionWithOffset(360);
//        }
//        else {
//            setTurretPosition();
//        }
//    }


    public double calculateTargetRotation() {
        double joystickAngle = Math.toDegrees(Math.atan2(opMode.gamepad1.left_stick_y, opMode.gamepad1.left_stick_x));
        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) ;
        opMode.telemetry.addData("joystickAngle", joystickAngle);
        return (robotHeading +
                joystickAngle + 180);
    }
    double pos =0;
    public void turnWithCamera() {
        pid.setPID(kp, ki, kd);
        smallpid.setPID(smallkp, smallki, smallkd);
        double erroretion = camera.getBearingToTag20();
        opMode.telemetry.addData("bearing", camera.getBearingToTag20());
        opMode.telemetry.addData("angle", getRotationOfInput());
        opMode.telemetry.addData("errotation", erroretion);
//
//
        if (getRotationOfInput() <= -50){
            moveTurret(turretPID(-40));
        }

        else if (getRotationOfInput() >= 50){
            moveTurret(turretPID(40));
        }
        else
        if (erroretion != -999) {

            if (Math.abs(erroretion) > 3){
                double poweretion = pid.calculatePIDValue(erroretion, 0);
                leftTurret.setPower(poweretion);
                rightTurret.setPower(poweretion);
                pos = getRotationOfInput();
            }
            else  if (Math.abs(erroretion) < 3){
                double power = -smallpid.calculatePIDValue(getRotationOfInput(), pos);
                leftTurret.setPower(power);
                rightTurret.setPower(power);
//            else {
////            if (Math.abs(erroretion) > 3){
            }
        }
        else {moveTurret(0);}

//            }

//        }
//        else {
//            moveTurret(turretPID(0));
//        }
//        else {
//            leftTurret.setPower(0);
//            rightTurret.setPower(0);}

//        else {
//            double toZero = -pid.calculatePIDValue(getRotationOfInput(), 0);
//            leftTurret.setPower(toZero);
//            rightTurret.setPower(toZero);
//        }
    }

//
//    public void setTurretPositionWithOffset(double offset) {
//        moveTurret(-turretPID(calculateTargetRotation()+ offset));
////    }
//    public void trunTurretWithCamera(){
//        opMode.telemetry.addData("raw input", getRotationOfInput());
//        opMode.telemetry.addData("turret rotation finale", turretRotation);
//
////        pid.setPID(kp, ki, kd);
////        if (getRotationOfInput() <= 60 && getRotationOfInput()>= -60){
//            turnWithCamera();
////        }
//        else {
//            double toZero = -pid.calculatePIDValue(getRotationOfInput(), 0);
//            leftTurret.setPower(toZero);
//            rightTurret.setPower(toZero);
////        }
}