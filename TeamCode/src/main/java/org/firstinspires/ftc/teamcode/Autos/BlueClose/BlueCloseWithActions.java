    package org.firstinspires.ftc.teamcode.Autos.BlueClose;

    import com.acmerobotics.dashboard.config.Config;
    import com.acmerobotics.roadrunner.Action;
    import com.acmerobotics.roadrunner.AngularVelConstraint;
    import com.acmerobotics.roadrunner.MinVelConstraint;
    import com.acmerobotics.roadrunner.ParallelAction;
    import com.acmerobotics.roadrunner.SequentialAction;
    import com.acmerobotics.roadrunner.TranslationalVelConstraint;
    import com.acmerobotics.roadrunner.ftc.Actions;
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

    import org.firstinspires.ftc.teamcode.Autos.AutoActions;
    import org.firstinspires.ftc.teamcode.Autos.Coordinates.BlueCloseCoordinated;
    import org.firstinspires.ftc.teamcode.MecanumDrive;
    import org.firstinspires.ftc.teamcode.Autos.Coordinates.BlueCloseCoordinated;
    import org.firstinspires.ftc.teamcode.systems.Camera;
    import org.firstinspires.ftc.teamcode.systems.Hood;
    import org.firstinspires.ftc.teamcode.systems.Intake;
    import org.firstinspires.ftc.teamcode.systems.Shooter;
    import org.firstinspires.ftc.teamcode.systems.Transfer;
    import org.firstinspires.ftc.teamcode.systems.Turret;
    import org.firstinspires.ftc.teamcode.systems.TurretPosition;

    import java.util.Arrays;

    @Config
    @Autonomous (name = "BlueClose", group = "autonomus")

    public class BlueCloseWithActions extends LinearOpMode {
        AutoActions actions;
        Intake intake;
        Transfer transfer;
        Hood hood;
        Shooter shooter;
        TurretPosition turret;
        Camera camera;


        @Override
        public void runOpMode() throws InterruptedException {

            intake = new Intake(this);
            transfer = new Transfer(this);
            shooter = new Shooter(this);
            camera = new Camera(this);
            turret = new TurretPosition(this, camera);
            hood =  new Hood(this);
            MinVelConstraint velCon = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(10),new AngularVelConstraint(10)));


            MecanumDrive drive = new MecanumDrive(hardwareMap, BlueCloseCoordinated.getStart());
            AutoActions actions = new AutoActions(intake, transfer,  turret, shooter, hood, this);
            Action goToShoot_0 = drive.actionBuilder(BlueCloseCoordinated.getStart())
                    .strafeTo(BlueCloseCoordinated.getShooting().position)
                    .build();
            Action waitToShoot0 = drive.actionBuilder(BlueCloseCoordinated.getShooting())
                    .waitSeconds(2)
                    .build();
            Action waitToShoot1 = drive.actionBuilder(BlueCloseCoordinated.getShooting())
                    .waitSeconds(3)
                    .build();
            Action waitToShoot2 = drive.actionBuilder(BlueCloseCoordinated.getShooting())
                    .waitSeconds(3)
                    .build();


            Action goToCollect_1 = drive.actionBuilder(BlueCloseCoordinated.getShooting())
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(BlueCloseCoordinated.getFirstIntakeStart(), BlueCloseCoordinated.getFirstIntakeStart().heading)
                    .setTangent(Math.toRadians(270))

                    .splineToLinearHeading(BlueCloseCoordinated.getFirstIntakeEnd(), BlueCloseCoordinated.getFirstIntakeEnd().heading)
                    .build();

            Action goToShoot_1 = drive.actionBuilder(BlueCloseCoordinated.getFirstIntakeEnd())
                    .strafeToLinearHeading(BlueCloseCoordinated.getShooting().position, BlueCloseCoordinated.getShooting().heading)
                    .build();

            Action goToCollect_2 = drive.actionBuilder(BlueCloseCoordinated.getShooting())
                    .splineToLinearHeading(BlueCloseCoordinated.getSecondIntakeStart(), BlueCloseCoordinated.getSecondIntakeStart().heading)
                    .splineToLinearHeading(BlueCloseCoordinated.getSecondIntakeEnd(), BlueCloseCoordinated.getSecondIntakeEnd().heading)
                    .build();

            Action goToShoot_2 = drive.actionBuilder(BlueCloseCoordinated.getSecondIntakeEnd())
                    .setTangent(Math.toRadians(70))
                    .splineToLinearHeading(BlueCloseCoordinated.getLastShooting(), BlueCloseCoordinated.getShooting().heading)
                    .build();

            waitForStart();
            hood.setPosition(Hood.UP);
            if (isStopRequested()) return;

            Actions.runBlocking(
                    new ParallelAction(
                            // Turret and shooter run in parallel throughout
                            actions.moveTurretBlueClose(),
                            actions.shooterStart(),

                            // Main sequence of driving, transferring, and collecting
                            new SequentialAction(
                                    // First shooting cycle
                                    goToShoot_0,
                                    actions.transferStart(),
                                    waitToShoot0, // optional wait to let shooter reach speed
                                    actions.transferEnd(),

                                    // Collect first batch of rings
                                    new ParallelAction(
                                            goToCollect_1,
                                            actions.intakeStart()
                                    ),
                                    actions.intakeEnd(), // stop intake after collection

                                    // Drive back to shooting position
                                    goToShoot_1,
                                    actions.transferStart(),
                                    waitToShoot1,
                                    actions.transferEnd(),

//                                     Collect second batch of rings
                                    new ParallelAction(
                                            goToCollect_2,
                                            actions.intakeStart()
                                    ),
                                    actions.intakeEnd(),

//                                     Final shooting
                                    goToShoot_2,
                                    actions.transferStart(),
                                    waitToShoot2,
                                    actions.transferEnd()
                            )
                    )
            );
        }
    }
