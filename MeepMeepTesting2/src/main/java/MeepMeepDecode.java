import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.Pose2d;

public class MeepMeepDecode {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setDimensions(17, 17) // robot size in inches
                .build();

        bot.runAction(
                bot.getDrive().actionBuilder(new Pose2d(0, -60, Math.PI / 2))
                        .strafeTo(new Pose2d(24, -36, Math.PI / 2).position)
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK) // temporary
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}
