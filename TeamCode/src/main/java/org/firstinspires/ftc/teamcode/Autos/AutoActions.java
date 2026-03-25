package org.firstinspires.ftc.teamcode.Autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.Camera;
import org.firstinspires.ftc.teamcode.systems.Hood;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Transfer;
import org.firstinspires.ftc.teamcode.systems.Turret;
import org.firstinspires.ftc.teamcode.systems.TurretPosition;

public class AutoActions {
    Intake intake;
    Transfer transfer;
    Shooter shooter;
    TurretPosition turret;
    Hood hood;
    Camera camera;
    IMU imu;
    LinearOpMode opMode;
    public AutoActions(Intake intake, Transfer transfer, TurretPosition turret, Shooter shooter, Hood hood, LinearOpMode opMode){
        this.intake = intake;
        this.transfer = transfer;
        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;
        this.opMode = opMode;
    }
    public class IntakeStart implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.intakeMotor.setPower(1);
            transfer.setTransferPower(0.42);
            return false;
        }
    }
    public Action intakeStart() {
        return new IntakeStart();
    }

    public class IntakeEnd implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.intakeMotor.setPower(0);
            transfer.setTransferPower(0);
            return false;
        }
    }
    public Action intakeEnd() {return new IntakeEnd();}


    public class TransferStart implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (shooter.leftShotingMotor.getVelocity()>= 1130){
                transfer.transferMotor.setPower(1);
                intake.activateIntake(1);
            return false;
            }
            else return true;
        }
    }
    public Action transferStart() {
        return new TransferStart();
    }

    public class TransferStartFar implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (shooter.leftShotingMotor.getVelocity()>= 1500){
                transfer.transferMotor.setPower(1);
                intake.activateIntake(1);
                return false;
            }
            else return true;
        }
    }
    public Action transferStartFar() {
        return new TransferStartFar();
    }

    public class TransferEnd implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
                transfer.transferMotor.setPower(0);
                intake.activateIntake(0);
                return false;
        }
    }
    public Action transferEnd() {
        return new TransferEnd();
    }

    public class ShooterStart implements Action{
        private boolean initialized = false;
        private ElapsedTime timer = new ElapsedTime();


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized){
                timer.reset();
                initialized = true;
            }

            shooter.shooterPID(1150);

            double vel = shooter.leftShotingMotor.getVelocity();
            telemetryPacket.put("shooting speed", vel);
            return true;
//                    timer.seconds() < 6;
//            if (shooter.leftShotingMotor.getVelocity()<= 1000){
//                return true;
//            } else
//            return false;
        }
    }
    public Action shooterStart(){return new ShooterStart();}
    public class ShooterStartFar implements Action{
        private boolean initialized = false;
        private ElapsedTime timer = new ElapsedTime();


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized){
                timer.reset();
                initialized = true;
            }

            shooter.shooterPID(1520);

            double vel = shooter.leftShotingMotor.getVelocity();
            telemetryPacket.put("shooting speed", vel);
//            return timer.seconds() < 6;
            return true;
//            if (shooter.leftShotingMotor.getVelocity()<= 1000){
//                return true;
//            } else
//            return false;
        }
    }
    public Action shooterStartFar(){return new ShooterStartFar();}


//
//    public class ShooterSecond implements Action{
//        private boolean initialized = false;
//        private ElapsedTime timer = new ElapsedTime();
//
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if (!initialized){
//                timer.reset();
//                initialized = true;
//            }
//
//            shooter.shooterPID(1160);
//
//            double vel = shooter.leftShotingMotor.getVelocity();
//            telemetryPacket.put("shooting speed", vel);
//            return true;
////                    timer.seconds() < 3;
////            if (shooter.leftShotingMotor.getVelocity()<= 1000){
////                return true;
////            } else
////            return false;
//        }
//    }
//    public Action shooterStartSecond(){return new ShooterSecond();}


    public class ShooterEnd implements Action{
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            shooter.shooterPID(0);

            double vel = shooter.leftShotingMotor.getVelocity();
            telemetryPacket.put("shooting speed", vel);
            return vel > 0;
//            if (shooter.leftShotingMotor.getVelocity()<= 1000){
//                return true;
//            } else
//            return false;
        }
    }
    public Action shooterEnd(){return new ShooterEnd();}

    public class MoveTurretBlueFar implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            turret.calculateTurretPosition(4);
            return true;
        }
    }
    public Action moveTurretblueFar(){return new MoveTurretBlueFar();}



    public class MoveTurretRedFar implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            turret.calculateTurretPosition(0);
            return true;
        }
    }
    public Action moveTurretRedFar(){return new MoveTurretRedFar();}



    public class MoveTurretRedClose implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            turret.calculateTurretPositionRed(-4);
            return true;
        }
    }
    public Action moveTurretRedClose(){return new MoveTurretRedClose();}

    public class MoveTurretBlueClose implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            turret.calculateTurretPosition(4);
            return true;
        }
    }
    public Action moveTurretBlueClose(){return new MoveTurretBlueClose();}
}