package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.Hood;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Transfer;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Wheels;

@TeleOp(name = "MainTeleOp", group = "Test")
@Config
//Uncomment the line below to disable this op
//@Disabled
public class MainTeleOp extends LinearOpMode {
    public static double targetVel = 2000;

    // Declare variables you will be using throughout this class here
    Wheels wheels;
    IMU imu; // Declare class for getting robot angles
    Transfer transfer;
    Shooter shooter;
    Intake intake;

    Hood hood;

    boolean isLeftBumper = false;



    // Time that runs since the program began running
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        // Runs when init is pressed. Initialize variables and pregame logic here
        imu = hardwareMap.get(IMU.class, "imu");
        intake = new Intake(this);
        transfer = new Transfer(this);
        shooter = new Shooter(this);
        shooter.setShotingPower(0);
        wheels = new Wheels(this, imu);
        hood =  new Hood(this);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Speed", "Waiting to start");
        telemetry.update();

        imu.resetYaw();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP). Logic once game starts here
        while (opModeIsActive()) {


            // Reset the robot's default angle with options button
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // Slow the robot down when the left bumber is pressed

            wheels.setMaxSpeed(1 - Math.pow(gamepad1.left_trigger, 3) *0.3);

            if (gamepad1.right_trigger >0.2) {
                intake.activateIntake(1.0); // full power intake
            } else if (gamepad1.square){
                intake.activateIntake(-1);
            }else {
                intake.activateIntake(0); // stop when not pressed
            }
            if(gamepad1.left_bumper) {
                isLeftBumper = !isLeftBumper;
                if (isLeftBumper){
                    shooter.shooterPID(1500);
                } else {
                    shooter.shooterPID(0);
                }

            }

            if (gamepad1.right_bumper){
                transfer.setTransferPower(1);
            }
            if(gamepad1.dpad_up){
                hood.setPosition(Hood.UP);
            }

            if(gamepad1.dpad_down){
                hood.setPosition(Hood.DOWN);
            }





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

            // Show data on driver station
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Speed", wheels.getMaxSpeed()*100 + "%");
            telemetry.update();
        }
    }
}