package org.firstinspires.ftc.teamcode;

import android.os.Build;

import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.json.JSONException;
import org.json.JSONObject;

import java.util.HashMap;


import static org.firstinspires.ftc.teamcode.Variables.motorBL;
import static org.firstinspires.ftc.teamcode.Variables.motorBR;
import static org.firstinspires.ftc.teamcode.Variables.motorFL;
import static org.firstinspires.ftc.teamcode.Variables.motorFR;

@TeleOp(name="KeyboardPushBotTeleOp")
public class KeyboardPushBotTeleOp extends DriveMethods {
    private HashMap<String, Boolean> getPressedKeysMap() throws JSONException {
        JSONObject reader = new JSONObject(Variables.curJSONdata);
        HashMap keysMap = new Gson().fromJson(String.valueOf(reader), HashMap.class);
        HashMap<String, Boolean> parsedJson = new HashMap();

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            keysMap.forEach((key,val) -> {
                if(val=="true") {
                    parsedJson.put((String) key, true);
                } else if ((String) val=="false") {
                    parsedJson.put((String) key, false);
                }
            });
            return parsedJson;
        } else {
            return null;
        }
    }

    @Override
    public void runOpMode() {
        initMotorsBlue();
        calibrateNavXIMU();
        double emulatedleftx = 0.0;
        double emulatedlefty = 0.0;
        double rightX = 0.0;
        int speedDiv = 1;

        waitForStart();

        HashMap<String, Boolean> pressedKeys = null;

        while(opModeIsActive()) {
            try {
                pressedKeys = getPressedKeysMap();
            } catch (JSONException e) {
//            throw new RuntimeException(e);
                telemetry.addLine("JSONERROR: " + e.toString());
                pressedKeys = new HashMap<String, Boolean>();
            }
            assert pressedKeys != null;
            if(pressedKeys.containsKey("w") && Boolean.TRUE.equals(pressedKeys.get("w"))) {
                emulatedlefty = 0.5;
            }
            if(pressedKeys.containsKey("a") && Boolean.TRUE.equals(pressedKeys.get("w"))) {
                emulatedleftx = -0.5;
            }
            if(pressedKeys.containsKey("s") && Boolean.TRUE.equals(pressedKeys.get("w"))) {
                emulatedlefty = -0.5;
            }
            if(pressedKeys.containsKey("d") && Boolean.TRUE.equals(pressedKeys.get("w"))) {
                emulatedleftx = 0.5;
            }

            motorFL.setPower((emulatedlefty + emulatedleftx + rightX) / speedDiv);
            motorBL.setPower((emulatedlefty - emulatedleftx + rightX) / speedDiv);
            motorFR.setPower((emulatedlefty - emulatedleftx - rightX) / speedDiv);
            motorBR.setPower((emulatedlefty + emulatedleftx - rightX) / speedDiv);
        }
    }
}
