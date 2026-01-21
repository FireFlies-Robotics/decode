package org.firstinspires.ftc.teamcode.systemTeleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.systems.Turret;

@TeleOp(name = "Turret Control1", group = "TeleOp")
@Config
//@Disabled
public class TurretTeleop extends LinearOpMode {
    Turret turret;

    AnalogInput analogInput;

    IMU imu;
    double sensorVoltage;
    public static double position;

    int targetTurretAngle = 270;
    @Override
    public void runOpMode() {
        // Initialize the intake system
//        turret.init();


//        telemetry.addLine("Initialized — Ready to start");
//        telemetry.update();
        analogInput = hardwareMap.get(AnalogInput.class, "turretAnalog");

//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
//        imu.resetYaw();
        turret = new Turret(this, imu);

        // Wait for the start button
        waitForStart();

        while (opModeIsActive()) {
//            double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//            double turretTarget = targetTurretAngle - robotHeading;

//            turret.turretPID(turretTarget);

            sensorVoltage = analogInput.getVoltage();
            telemetry.addData( "analog input sensor", sensorVoltage);

//            turret.moveTurret(gamepad1.left_stick_x);
            turret.moveTurret(turret.turretPID(position));
//            turret.moveTurret(gamepad2.right_stick_x);
                // If square (X) button is pressed, run intake

                // If square (X) butt  on is pressed, run intake


//            telemetry.addData("Turret power", turret.rightTurret.getPower());
            turret.updateTurretServoRotation();

            telemetry.update();
        }
    }
}

