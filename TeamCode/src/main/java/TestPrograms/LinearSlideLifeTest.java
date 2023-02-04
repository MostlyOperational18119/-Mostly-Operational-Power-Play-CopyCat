package TestPrograms;

import static org.firstinspires.ftc.teamcode.Variables.Direction;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.DriveMethods;

@Autonomous(name ="LinearSlideTest", group = "A")
@Disabled
public class LinearSlideLifeTest extends DriveMethods {
    public void runOpMode() {
        initMotorsBlue();

        waitForStart();

        for(int i = 0; i < 20; i++) {
            goToHigh();
            sleep(1000);
            goToDown();
            sleep(1000);
        }

        while(opModeIsActive()) {

        }


    }



}
