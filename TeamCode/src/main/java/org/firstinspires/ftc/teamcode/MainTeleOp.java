package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Wheels;

@TeleOp(name = "MainTeleOp", group = "Test")
//Uncomment the line below to disable this op
//@Disabled
public class MainTeleOp extends LinearOpMode {
    // Declare variables you will be using throughout this class here
    Wheels wheels;
    IMU imu; // Declare class for getting robot angles
    Intake intake;

    // Time that runs since the program began running
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        // Runs when init is pressed. Initialize variables and pregame logic here
        imu = hardwareMap.get(IMU.class, "imu");
        intake = new Intake(this);


        wheels = new Wheels(this, imu);

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
            if (gamepad1.left_bumper) {
                wheels.setMaxSpeed(.3);
            } else {
                wheels.setMaxSpeed(1);
            }
            if (gamepad1.circle) {
                intake.activateIntake(1.0); // full power intake
            } else if (gamepad1.square){
                intake.activateIntake(-1);
            }else {
                intake.activateIntake(0); // stop when not pressed
            }
            if (gamepad1.right_bumper){
                intake.transferServo.setPower(1);
            }
            else if (gamepad1.left_bumper){
                intake.transferServo.setPower(-1);
            }
            else {
                intake.transferServo.setPower(0);
            }


                // Move robot by controller 1
            wheels.driveByJoystickFieldOriented(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            // Show data on driver station
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Speed", wheels.getMaxSpeed()*100 + "%");
            telemetry.update();
        }
    }
}