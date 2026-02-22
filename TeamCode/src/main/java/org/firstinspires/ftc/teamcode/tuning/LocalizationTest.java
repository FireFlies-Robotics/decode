package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.systems.Camera;
import org.firstinspires.ftc.teamcode.systems.Hood;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Transfer;
import org.firstinspires.ftc.teamcode.systems.Turret;
import org.firstinspires.ftc.teamcode.systems.Wheels;

@Disabled
public class LocalizationTest extends LinearOpMode {

    Turret turret;

    AnalogInput analogInput;

    Camera camera;
    Wheels wheels;
    IMU imu; // Declare class for getting robot angles
    Transfer transfer;
    Shooter shooter;
    Intake intake;

    Hood hood;

    boolean isLeftBumper = false;

    boolean preLeftBumper = false;
    public static double shootoingPower = 0;

    public static int selectedVelocity = 1245;  // hood decides this
    public static int farVelocity = 1580;
    public static int closeVelocity = 1220;
    int targetVelocity = 0;       // shooterPID uses this
    // Time that runs since the program began running
    private ElapsedTime runtime = new ElapsedTime();

    @Override

    public void runOpMode() throws InterruptedException {


        analogInput = hardwareMap.get(AnalogInput.class, "turretAnalog");

        // Runs when init is pressed. Initialize variables and pregame logic here
        imu = hardwareMap.get(IMU.class, "imu");
        intake = new Intake(this);
        transfer = new Transfer(this);
        shooter = new Shooter(this);
        wheels = new Wheels(this, imu, AllianceColor.BLUE);
        camera = new Camera(this);
        turret = new Turret(this, imu, camera);
        turret.init();
        hood =  new Hood(this);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Speed", "Waiting to start");
        telemetry.update();

        runtime.reset();
        imu.resetYaw();
        boolean shootingOn = false;
        boolean lastpress = false;


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                Pose2d pose = drive.localizer.getPose();
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {





                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                0.0
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                Pose2d pose = drive.localizer.getPose();
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
        while (opModeIsActive()) {



            // Reset the robot's default angle with options button
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // Slow the robot down when the left bumber is pressed

            wheels.setMaxSpeed(1 - (gamepad1.left_trigger * 0.7));

            if (gamepad1.right_trigger >0.2) {
                intake.activateIntake(1.0); // full power intake
            } else if (gamepad1.square){
                transfer.setTransferPower(-1);
                intake.activateIntake(-1);
            }else {
                intake.activateIntake(0); // stop when not pressed
            }// --- Shooter toggle on left bumper ---
//
//// Rising edge: button pressed this loop
//            if (lb && !preLeftBumper) {
//                isLeftBumper = !isLeftBumper;
//
//                if (isLeftBumper) {
//                    shooter.setShotingPower(1);
////                    shooter.shooterPID(1500);   // turn on
//                    telemetry.addData("a", 1);
//                } else {
//                    shooter.shooterPID(0);      // turn off
//          h          telemetry.addData("a", 0);
//                }
//            }

// Always update previous state based on the button, not shooter state
            turret.turnWithCamera();
//            turret.moveTurret(gamepad1.right_stick_x);
//            turret.setTurretPosition(position);

            telemetry.addData("raw rotation" ,turret.getRotationOfInput());

            if (gamepad1.right_bumper && shooter.leftShotingMotor.getVelocity() >= (targetVelocity -30)){
                transfer.setTransferPower(1);
            }

            else {transfer.setTransferPower(0);}

            if (gamepad1.left_bumper && !lastpress) {
                shootingOn = !shootingOn;
            }
            lastpress = gamepad1.left_bumper;

            if (shootingOn) {
                targetVelocity = selectedVelocity;
            } else {
                targetVelocity = 0;
            }
            if (targetVelocity == 0){
                shooter.setShotingPower(0);
            }
            else{shooter.shooterPID(targetVelocity);}
//            shooter.setShotingPower(shootoingPower);
            if (gamepad1.dpad_up) {
                hood.setPosition(Hood.DOWN);
                selectedVelocity = farVelocity;
            }

            if (gamepad1.dpad_down) {
                hood.setPosition(Hood.UP);
                selectedVelocity = closeVelocity;
            }
            if (shooter.leftShotingMotor.getVelocity() >= (selectedVelocity - 30)){
                gamepad1.rumble(100);
            }
            if (gamepad1.triangle){transfer.setTransferPower(1);}
            if (gamepad1.cross){transfer.setTransferPower(-1);}

            telemetry.addData("target velocity", targetVelocity);





//            if (gamepad1.right_bumper){
//                intake.transferServo.setPower(1);
//            }
//            else if (gamepad1.left_bumper){
//                intake.transferServo.setPower(-1);
//            }
//            else {
//                intake.transferServo.setPower(0);
//            }
            // Move robot by controller 1
            wheels.driveByJoystickFieldOriented(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("time", runtime.seconds());
            packet.put("shooting velocity",shooter.leftShotingMotor.getVelocity());
            packet.put("up", Hood.UP);
            packet.put("down", Hood.DOWN);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        }
}
