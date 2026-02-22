    package org.firstinspires.ftc.teamcode.Autos.BlueClose;

    import com.acmerobotics.dashboard.config.Config;
    import com.acmerobotics.roadrunner.Action;
    import com.acmerobotics.roadrunner.AngularVelConstraint;
    import com.acmerobotics.roadrunner.MinVelConstraint;
    import com.acmerobotics.roadrunner.ParallelAction;
    import com.acmerobotics.roadrunner.SequentialAction;
    import com.acmerobotics.roadrunner.TranslationalVelConstraint;
    import com.acmerobotics.roadrunner.ftc.Actions;
    import com.arcrobotics.ftclib.command.WaitUntilCommand;
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.IMU;

    import org.firstinspires.ftc.onbotjava.handlers.objbuild.WaitForBuild;
    import org.firstinspires.ftc.teamcode.Autos.AutoActions;
    import org.firstinspires.ftc.teamcode.MecanumDrive;
    import org.firstinspires.ftc.teamcode.Autos.Coordinates.BlueCloseCoordianates;
    import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
    import org.firstinspires.ftc.teamcode.systems.Camera;
    import org.firstinspires.ftc.teamcode.systems.Hood;
    import org.firstinspires.ftc.teamcode.systems.Intake;
    import org.firstinspires.ftc.teamcode.systems.Shooter;
    import org.firstinspires.ftc.teamcode.systems.Transfer;
    import org.firstinspires.ftc.teamcode.systems.Turret;
    import org.firstinspires.ftc.teamcode.systems.Wheels;

    import java.util.Arrays;

    @Config
    @Autonomous (name = "BlueClose_3_plus_6 actions", group = "autonomus")

    public class BlueCloseWithActions extends LinearOpMode {
        AutoActions actions;
        Intake intake;
        Transfer transfer;
        Hood hood;
        Shooter shooter;
        Turret turret;
        Camera camera;


        @Override
        public void runOpMode() throws InterruptedException {

            intake = new Intake(this);
            transfer = new Transfer(this);
            shooter = new Shooter(this);
            camera = new Camera(this);
            turret = new Turret(this, null, camera);
            turret.init();
            hood =  new Hood(this);
            MinVelConstraint velCon = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(10),new AngularVelConstraint(10)));


            MecanumDrive drive = new MecanumDrive(hardwareMap, BlueCloseCoordianates.getStart());
            AutoActions actions = new AutoActions(intake, transfer,  turret, shooter, hood, this);
            Action goToShoot_0 = drive.actionBuilder(BlueCloseCoordianates.getStart())
                    .strafeTo(BlueCloseCoordianates.getShooting().position)
                    .build();
            Action waitToShoot0 = drive.actionBuilder(BlueCloseCoordianates.getShooting())
                    .waitSeconds(3)
                    .build();
            Action waitToShoot1 = drive.actionBuilder(BlueCloseCoordianates.getShooting())
                    .waitSeconds(3)
                    .build();
            Action waitToShoot2 = drive.actionBuilder(BlueCloseCoordianates.getShooting())
                    .waitSeconds(3)
                    .build();


            Action goToCollect_1 = drive.actionBuilder(BlueCloseCoordianates.getShooting())
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(BlueCloseCoordianates.getFirstIntakeStart(), BlueCloseCoordianates.getFirstIntakeStart().heading)
                    .splineToLinearHeading(BlueCloseCoordianates.getFirstIntakeEnd(), BlueCloseCoordianates.getFirstIntakeEnd().heading)
                    .build();

            Action goToShoot_1 = drive.actionBuilder(BlueCloseCoordianates.getFirstIntakeEnd())
                    .strafeToLinearHeading(BlueCloseCoordianates.getShooting().position, BlueCloseCoordianates.getShooting().heading)
                    .build();

            Action goToCollect_2 = drive.actionBuilder(BlueCloseCoordianates.getShooting())
                    .splineToLinearHeading(BlueCloseCoordianates.getSecondIntakeStart(), BlueCloseCoordianates.getSecondIntakeStart().heading)
                    .splineToLinearHeading(BlueCloseCoordianates.getSecondIntakeEnd(), BlueCloseCoordianates.getSecondIntakeEnd().heading)
                    .build();

            Action goToShoot_2 = drive.actionBuilder(BlueCloseCoordianates.getSecondIntakeEnd())
                    .setTangent(Math.toRadians(70))
                    .splineToLinearHeading(BlueCloseCoordianates.getShooting(), BlueCloseCoordianates.getShooting().heading)
                    .build();

            waitForStart();
            hood.setPosition(Hood.UP);
            if (isStopRequested()) return;

            Actions.runBlocking(
                    new ParallelAction(actions.moveTurret(),
                            new SequentialAction(
                                    new ParallelAction(
                                            goToShoot_0,
                                            actions.shooterStart(),
                                            actions.transferStart(),
                                            waitToShoot0
                                    ),
                                    actions.shooterEnd(),
                                    actions.transferEnd(),
                            new ParallelAction(
                                    actions.intakeStart(),
                                    goToCollect_1
                            ),
                            goToShoot_1,
                            actions.intakeEnd(),
                            new ParallelAction(

                                    actions.shooterStart(),
                                    actions.transferStart()
                            ),

                            waitToShoot1,
                            actions.shooterEnd(),
                            actions.transferEnd(),
                            actions.intakeStart(),
                            goToCollect_2//,
//                            goToShoot_2,
//                            actions.intakeEnd(),
//                            new ParallelAction(
//                                    actions.shooterStart(),
//                                    actions.transferStart()),
//                            waitToShoot0,
//                            actions.shooterEnd(),
//                            actions.transferEnd()
                            )
                    )
            );

        }
    }
