package Red;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import MeepMeep.coordinates.RedCoordinatesMeepMeep.RedCloseCoordinatesMeepMeep;


public class RedClose {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(670);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60,
                        Math.toRadians(180),
                        Math.toRadians(180),
                        15)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(
                                RedCloseCoordinatesMeepMeep.getStart()
                        )
                        .strafeTo(RedCloseCoordinatesMeepMeep.getShooting().position)
                        .setTangent(Math.toRadians(45))
                        .splineToLinearHeading(
                                RedCloseCoordinatesMeepMeep.getFirstIntakeStart(),
                                RedCloseCoordinatesMeepMeep.getFirstIntakeStart().heading
                        )
                        .splineToSplineHeading(
                                RedCloseCoordinatesMeepMeep.getFirstIntakeEnd(),
                                RedCloseCoordinatesMeepMeep.getFirstIntakeEnd().heading
                        )

                        // first intake

                        .strafeToSplineHeading(
                                RedCloseCoordinatesMeepMeep.getShooting().position,
                                RedCloseCoordinatesMeepMeep.getShooting().heading
                        )
                        .waitSeconds(2)

                        // first shoot

                        .splineToLinearHeading(
                                RedCloseCoordinatesMeepMeep.getSecondIntakeStart(),
                                RedCloseCoordinatesMeepMeep.getSecondIntakeStart().heading
                        )
                        .strafeToSplineHeading(
                                RedCloseCoordinatesMeepMeep.getSecondIntakeEnd().position,
                                RedCloseCoordinatesMeepMeep.getSecondIntakeEnd().heading
                        )
                        .strafeToSplineHeading(
                                RedCloseCoordinatesMeepMeep.getShooting().position,
                                RedCloseCoordinatesMeepMeep.getShooting().heading
                        )
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
