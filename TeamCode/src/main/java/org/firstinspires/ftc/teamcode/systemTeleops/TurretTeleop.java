package org.firstinspires.ftc.teamcode.systemTeleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.systems.Camera;
import org.firstinspires.ftc.teamcode.systems.Turret;

@TeleOp(name = "Turret Control1", group = "TeleOp")
@Config
//@Disabled
public class TurretTeleop extends LinearOpMode {
    Turret turret;

    AnalogInput analogInput;

    IMU imu;
    Camera camera;
    double sensorVoltage;
    public static double position;
    public static double angleToFix = 0;

    int targetTurretAngle = 270;
    @Override
    public void runOpMode() {
        analogInput = hardwareMap.get(AnalogInput.class, "turretAnalog");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();


        // ADD THESE DEBUG LINES BEFORE CREATING TURRET
        telemetry.addData("Sensor Voltage", analogInput.getVoltage());
        telemetry.addData("Max Voltage", analogInput.getMaxVoltage());
        telemetry.addData("Calculated Angle", (analogInput.getVoltage() / analogInput.getMaxVoltage()) * 360);
        telemetry.update();
//        sleep(3000); // Give you time to read it
        camera = new Camera(this);
        turret = new Turret(this, imu, camera);
        turret.init();

        waitForStart();

        while (opModeIsActive()) {
            double stickMagnitude = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

            // ALSO ADD IN LOOP
//            telemetry.addData("LIVE Sensor Voltage", analogInput.getVoltage());
//            telemetry.addData("LIVE Max Voltage", analogInput.getMaxVoltage());

//            turret.updateTurretServoRotation();

//            if (gamepad1.triangle){
            turret.trunTurretWithCamera();
            telemetry.update();
        }
    }
}