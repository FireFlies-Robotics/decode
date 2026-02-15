//package org.firstinspires.ftc.teamcode.systemTeleops;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.IMU;
//
//import org.firstinspires.ftc.teamcode.systems.Turret;
//
//@TeleOp(name = "Turret TeleOp Clean", group = "TeleOp")
//public class TurretTeleOp extends LinearOpMode {
//
//    Turret turret;
//    IMU imu;
//
//    enum Mode { MANUAL, TARGET }
//    Mode mode = Mode.MANUAL;
//
//    @Override
//    public void runOpMode() {
//
//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                        RevHubOrientationOnRobot.UsbFacingDirection.UP
//                )
//        ));
//        imu.resetYaw();
//
//        turret = new Turret(this, imu);
//        turret.init();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            turret.update();
//
//            // mode switch
//            if (gamepad1.cross) mode = Mode.MANUAL;
//            if (gamepad1.triangle) mode = Mode.TARGET;
//
//            if (mode == Mode.MANUAL) {
//                turret.manual(gamepad1.left_stick_x);
//            }
//
//            if (mode == Mode.TARGET) {
//                double target = turret.calculateJoystickTarget();
//                turret.goToAngle(target);
//            }
//
//            telemetry.addData("Mode", mode);
//            turret.telemetry();
//            telemetry.update();
//        }
//    }
//}
