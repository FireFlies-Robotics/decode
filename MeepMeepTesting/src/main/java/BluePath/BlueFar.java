package BluePath;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import MeepMeep.coordinates.BlueCoordinates.BlueFarCoordinatesMeepMeep;

public class BlueFar {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(670);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(BlueFarCoordinatesMeepMeep.getStart())
                        .strafeToLinearHeading(BlueFarCoordinatesMeepMeep.getShooting().position, BlueFarCoordinatesMeepMeep.getShooting().heading)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(BlueFarCoordinatesMeepMeep.getFirstIntakeStart(), BlueFarCoordinatesMeepMeep.getFirstIntakeStart().heading)
                        .strafeToSplineHeading(BlueFarCoordinatesMeepMeep.getFirstIntakeEnd().position, BlueFarCoordinatesMeepMeep.getFirstIntakeEnd().heading)
                .strafeToLinearHeading(BlueFarCoordinatesMeepMeep.getShooting().position, BlueFarCoordinatesMeepMeep.getShooting().heading)
                .setTangent(Math.toRadians(270))

                .splineToLinearHeading(BlueFarCoordinatesMeepMeep.getSecondIntakeStart(), BlueFarCoordinatesMeepMeep.getFirstIntakeStart().heading)
                        .strafeToSplineHeading(BlueFarCoordinatesMeepMeep.getSecondIntakeEnd().position, BlueFarCoordinatesMeepMeep.getSecondIntakeEnd().heading)

                .strafeToLinearHeading(BlueFarCoordinatesMeepMeep.getShooting().position, BlueFarCoordinatesMeepMeep.getShooting().heading)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}