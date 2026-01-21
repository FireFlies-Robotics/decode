package org.firstinspires.ftc.teamcode.systemTeleops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.systems.Hood;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Transfer;
import org.firstinspires.ftc.teamcode.systems.Wheels;

@Autonomous(name = "close auto")
//@Disabled
public class CloseAutoOp extends LinearOpMode {

    Wheels wheels;
    IMU imu; // Declare class for getting robot angles
    Transfer transfer;
    Shooter shooter;
    Intake intake;
    private ElapsedTime runtime = new ElapsedTime();


    Hood hood;


    @Override
    public void runOpMode() {
        runtime.reset();
        // Initialize the intake system

        // Runs when init is pressed. Initialize variables and pregame logic here
        imu = hardwareMap.get(IMU.class, "imu");
        intake = new Intake(this);
        transfer = new Transfer(this);
        shooter = new Shooter(this);
        wheels = new Wheels(this, imu, AllianceColor.BLUE);
        hood =  new Hood(this);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Speed", "Waiting to start");
        telemetry.update();

        imu.resetYaw();
        shooter.setShotingPower(0);

        runtime.reset();


        waitForStart();

        while (opModeIsActive()) {
            hood.setPosition(Hood.DOWN);


            if(runtime.time() < 6){
                wheels.driveForwordByPower(-.4);
            }
            if (runtime.time() > 6){
                shooter.shooterPID(1300);
                if(shooter.leftShotingMotor.getVelocity() > 1275) {
                    intake.activateIntake(1);
                    transfer.setTransferPower(1);
                }
            }
            if (runtime.time() > 13 && runtime.time() < 16){
                shooter.setShotingPower(0);
//                wheels.driveForwordByPower(-.4);

            }
        }
    }
}
