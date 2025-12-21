package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.Hood;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Transfer;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;
import org.firstinspires.ftc.teamcode.systems.Wheels;

@TeleOp(name = "MainTeleOp", group = "Test")
@Config
//Uncomment the line below to disable this op
//@Disabled
public class MainTeleOp extends LinearOpMode {
//    public static double targetVel = 2000;

    // Declare variables you will be using throughout this class here
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
    int targetVelocity = 0;       // shooterPID uses this
    // Time that runs since the program began running
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {


        // Runs when init is pressed. Initialize variables and pregame logic here
        imu = hardwareMap.get(IMU.class, "imu");
        intake = new Intake(this);
        transfer = new Transfer(this);
        shooter = new Shooter(this);
        wheels = new Wheels(this, imu);
        hood =  new Hood(this);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Speed", "Waiting to start");
        telemetry.update();
        runtime.reset();
        imu.resetYaw();
        boolean shootingOn = false;
        boolean lastpress = false;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        shooter.setShotingPower(0);

        runtime.reset();

        // run until the end of the match (driver presses STOP). Logic once game starts here
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
                intake.activateIntake(-4);
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
//                    telemetry.addData("a", 0);
//                }
//            }

// Always update previous state based on the button, not shooter state

            if (gamepad1.right_bumper){
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
                hood.setPosition(Hood.UP);
                selectedVelocity = 1700;
            }

            if (gamepad1.dpad_down) {
                hood.setPosition(Hood.DOWN);
                selectedVelocity = 1230;
            }
            if (shooter.leftShotingMotor.getVelocity() > (selectedVelocity - 10)){
                gamepad1.rumble(100);
            }

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

            // Show data on driver station
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Speed", wheels.getMaxSpeed()*100 + "%");
            telemetry.update();
        }
    }
}