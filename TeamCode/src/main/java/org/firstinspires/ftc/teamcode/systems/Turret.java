package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
    public static double SENSOR_ZERO_OFFSET = 10;

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

    public Turret(LinearOpMode opMode, IMU imu, Camera camera) {
        pid = new PID(kp, ki, kd, opMode, 1);
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

        opMode.telemetry.addData("Sensor at Init", turretOldPos);
        opMode.telemetry.addLine("Make sure turret is at ZERO position!");
        opMode.telemetry.update();
    }

    // Get sensor reading with optional fixed offset
    private double getRotationOfInput() {
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

        double pidd = pid.calculatePIDValue(turretRotation+180, targetRotation);
        opMode.telemetry.addData("PID Output", pidd);
        return pidd;
    }

    public void setTurretFinalPosition() {
        if (calculateTargetRotation() > MAX_TURRET_ANGLE) {
            setTurretPositionWithOffset(-360);
        }
        else if (calculateTargetRotation() < MIN_TURRET_ANGLE) {
            setTurretPositionWithOffset(360);
        }
        else {
            setTurretPosition();
        }
    }

    public double calculateTargetRotation() {
        double joystickAngle = Math.toDegrees(Math.atan2(opMode.gamepad1.left_stick_y, opMode.gamepad1.left_stick_x));
        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) ;
        opMode.telemetry.addData("joystickAngle", joystickAngle);
        return (robotHeading +
                joystickAngle + 180);
    }

    public void turnWithCamera() {
        pid.setPID(kp, ki, kd);
        double erroretion = camera.returnBearing();
        if (erroretion != -999) {
            double poweretion = -pid.calculatePIDValue(erroretion, 0);
            leftTurret.setPower(poweretion);
            rightTurret.setPower(poweretion);
        }
        else {
            double toZero = -pid.calculatePIDValue(getRotationOfInput(), 0);
            leftTurret.setPower(toZero);
            rightTurret.setPower(toZero);
        }
    }

    public void setTurretPosition() {
        moveTurret(-turretPID(calculateTargetRotation()));
    }

    public void setTurretPositionWithOffset(double offset) {
        moveTurret(-turretPID(calculateTargetRotation()+ offset));
    }
    public void trunTurretWithCamera(){
        opMode.telemetry.addData("raw input", getRotationOfInput());
        opMode.telemetry.addData("turret rotation finale", turretRotation);

        pid.setPID(kp, ki, kd);
        if (getRotationOfInput() <= 60 && getRotationOfInput()>= -60){
            turnWithCamera();
        }
        else {

        }
    }
}