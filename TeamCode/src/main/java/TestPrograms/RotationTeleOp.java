package TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DriveMethods;

@TeleOp(name="RotationTeleOp",group="A")
@Disabled
public class RotationTeleOp extends DriveMethods {
    @Override
    public void runOpMode() {
        initMotorsBlue();
        calibrateNavXIMU();
        int rotationZ = 0;
        waitForStart();
        while(opModeIsActive()) {
            rotationZ += Math.floor(gamepad1.right_stick_x * 5);
            if(gamepad1.right_bumper) {
                rotationZ = 0;
            }
            if(gamepad1.a) {
                //rotateToHeading(rotationZ,0.2);
            }
            if(gamepad1.b) {
                rotateAngle(rotationZ);
            }
            telemetry.addData("Target Rotation Z: ", rotationZ);
            telemetry.addData("Current Rotation Z: ",getCurrentZ());
            telemetry.update();
            // To make the rotation change at a resonable rate
            sleep(100);

        }
    }
}
