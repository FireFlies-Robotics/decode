package org.firstinspires.ftc.teamcode.systemTeleops;

import android.provider.ContactsContract;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.systems.Hood;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Transfer;
import org.firstinspires.ftc.teamcode.systems.Turret;
import org.firstinspires.ftc.teamcode.systems.Wheels;

@Autonomous(name = "jotstick test")
//@Disabled
public class JoyTest extends LinearOpMode {

    Wheels wheels;
    IMU imu; // Declare class for getting robot angles
    Transfer transfer;
    Shooter shooter;
    Intake intake;
    Turret turret;
    private ElapsedTime runtime = new ElapsedTime();


    Hood hood;


    @Override
    public void runOpMode() {
        telemetry.addLine("Initialized — Ready to start");
        telemetry.update();
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
        imu.resetYaw();
        turret = new Turret(this, imu);
        Pose2d startPose = new Pose2d(0, 0, 0);
        wheels = new Wheels(this, imu, AllianceColor.BLUE);

        wheels.localizer.setPose(startPose);
        turret.init();
        waitForStart();
        while (opModeIsActive()) {

            telemetry.addData("joystick angle", Math.toDegrees(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)) + 180);
            telemetry.addData("calculated angle", turret.calculateTargetRotation());
            telemetry.addData("imu", imu.getRobotYawPitchRollAngles().getYaw());
//            turret.turretPID(turret.calculateTargetRotation());
//            wheels.updatePose();
//            Pose2d pose = wheels.getEstimatedPose();
//            telemetry.addData("X", pose.position.x);
//            telemetry.addData("Y", pose.position.y);
//            telemetry.addData("Heading", Math.toDegrees(pose.heading.toDouble()));
            turret.updateTurretServoRotation();

            telemetry.update();

//
            turret.setTurretPosition();
//            telemetry.update();
        }
    }
}
