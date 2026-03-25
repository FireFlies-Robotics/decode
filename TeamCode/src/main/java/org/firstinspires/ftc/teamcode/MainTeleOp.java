package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.systems.Camera;
import org.firstinspires.ftc.teamcode.systems.Hood;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Transfer;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;
import org.firstinspires.ftc.teamcode.systems.TurretPosition;
import org.firstinspires.ftc.teamcode.systems.Wheels;

@TeleOp(name = "MainTeleOpBlue", group = "Main ")
@Config
public class MainTeleOp extends LinearOpMode {

    TurretPosition turret;
    AnalogInput analogInput;

    Camera camera;
    Wheels wheels;
    IMU imu;
    Transfer transfer;
    Shooter shooter;
    Intake intake;
    Hood hood;

    public static double offset = 4;

    public static double transferDash = 0.41;

    private double transferPower = 0;

    public static double shootoingPower = 0;

    public static int selectedVelocity = 1245;
    public static int farVelocity = 1500;
    public static int closeVelocity = 1220;

    int targetVelocity = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        imu = hardwareMap.get(IMU.class, "imu");
        intake = new Intake(this);
        transfer = new Transfer(this);
        shooter = new Shooter(this);
        wheels = new Wheels(this, imu, AllianceColor.BLUE);
        camera = new Camera(this);
        turret = new TurretPosition(this, camera);
        hood = new Hood(this);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Speed", "Waiting to start");
        telemetry.update();

        runtime.reset();
        imu.resetYaw();

        boolean shootingOn = false;
        boolean lastpress = false;

        waitForStart();
        runtime.reset();
        shooter.setShotingPower(0);

        while (opModeIsActive()) {
            double startTime = runtime.milliseconds();
            double shooterVelocity = shooter.leftShotingMotor.getVelocity();

            if (gamepad1.options) {
                imu.resetYaw();
            }

            wheels.setMaxSpeed(1 - (gamepad1.left_trigger * 0.7));

            // ================= FIXED TRANSFER + INTAKE LOGIC =================

            // Intake
            if (gamepad1.right_trigger > 0.2) {
                intake.activateIntake(1.0);
                transferPower = transferDash;
            }
            else if (gamepad1.square) {
                intake.activateIntake(-1);
                transferPower = -1;
            }
            else {
                intake.activateIntake(0);
                transferPower = 0;
            }

            // Shooter feeding (override intake)
            if (gamepad1.right_bumper && shooterVelocity >= (targetVelocity - 50)) {
                transferPower = 1;
            }

            // Manual override (highest priority)
            if (gamepad1.triangle) {
                transferPower = 1;
            }
            else if (gamepad1.cross) {
                transferPower = -1;
            }

            // Apply once
            transfer.setTransferPower(transferPower);

            // ================================================================

            if (gamepad1.left_bumper && !lastpress) {
                shootingOn = !shootingOn;
            }
            lastpress = gamepad1.left_bumper;

            if (shootingOn) {
                targetVelocity = selectedVelocity;
            } else {
                targetVelocity = 0;
            }

            if (targetVelocity == 0) {
                shooter.setShotingPower(0);
            } else {
                shooter.shooterPID(targetVelocity);
            }

            if (gamepad1.dpad_up) {
                hood.setPosition(Hood.DOWN);
                selectedVelocity = farVelocity;
            }

            if (gamepad1.dpad_down) {
                hood.setPosition(Hood.UP);
                selectedVelocity = closeVelocity;
            }

            if (shooterVelocity >= (selectedVelocity - 30)) {
                gamepad1.rumble(100);
            }

            double endTime = runtime.milliseconds();

            wheels.driveByJoystickFieldOriented(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x
            );

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("time", runtime.nanoseconds());
            packet.put("shooting velocity", shooterVelocity);
            packet.put("loop time", endTime - startTime);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.addData("loop time", endTime - startTime);

            turret.calculateTurretPosition(offset);

            telemetry.update();
        }
    }
}