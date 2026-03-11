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
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.systems.Camera;
import org.firstinspires.ftc.teamcode.systems.Turret;
import org.firstinspires.ftc.teamcode.systems.TurretPosition;
import org.firstinspires.ftc.teamcode.systems.Wheels;

@TeleOp(name = "Turret Control1", group = "TeleOp")
@Config
//@Disabled
public class TurretTeleop extends LinearOpMode {
    TurretPosition turretPosition;
    Camera camera;
    @Override
    public void runOpMode() {
        camera = new Camera(this);

        turretPosition = new TurretPosition(this, camera);
        // ADD THESE DEBUG LINES BEFORE CREATING TURRET
        waitForStart();

        while (opModeIsActive()) {
////
            if (gamepad1.dpad_down){
                turretPosition.setTurretPosition(0.5);
            }
            if (gamepad1.dpad_left){
                turretPosition.setTurretPosition(0);
            }
            if (gamepad1.dpad_right){
                turretPosition.setTurretPosition(1);
            }
//            turretPosition.calculateTurretPosition(1);
//            if (gamepad1.cross){
//                wheels.driveForwordByPower(-0.5);
//            }
//            else if (gamepad1.triangle)
//                wheels.driveForwordByPower(0.5);
//            }
//            else {wheels.driveForwordByPower(0);}
//            double stickMagnitude = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

            // ALSO ADD IN LOOP
//            telemetry.addData("LIVE Sensor Voltage", analogInput.getVoltage());

            telemetry.update();
        }
    }
}
