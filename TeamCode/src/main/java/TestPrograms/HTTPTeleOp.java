package TestPrograms;

import android.os.Build;

import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DriveMethods;
import org.firstinspires.ftc.teamcode.TinyWebServer;
import org.firstinspires.ftc.teamcode.Variables;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.HashMap;
@TeleOp(name="HTTPTeleOp")
@Disabled
public class HTTPTeleOp extends DriveMethods {
    private int port;
    private String ipaddr;
    private Boolean srvRunning;
    private String pressedKeysString = "";
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
        ipaddr = "192.168.43.1";
        port = 6969;

        telemetry.addLine("Init");
        telemetry.update();
        waitForStart();
        TinyWebServer.startServer(ipaddr,port,"/");
        srvRunning = true;
        HashMap<String, Boolean> pressedKeys = null;
        try {
            pressedKeys = getPressedKeysMap();
        } catch (JSONException e) {
//            throw new RuntimeException(e);
        }
        while (opModeIsActive()) {
            if (gamepad1.a && !srvRunning) {
                TinyWebServer.startServer(ipaddr,port,"/");
            }
            if (gamepad1.b && srvRunning) {
                TinyWebServer.stopServer();
            }
            if (srvRunning) {
                telemetry.addLine("Currently active.");
            } else {
                telemetry.addLine("Currently stopped, start me!");
            }
            if(pressedKeys!=null) {
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
                    pressedKeysString = "";
                    pressedKeys.forEach((key,value) -> {
                        if(value) {
                            pressedKeysString += key + " ";
                        }
                        if(pressedKeysString!="") {
                            pressedKeysString = pressedKeysString.substring(0, pressedKeysString.length()) + ".";
                        }
                    });
                }
            }
            telemetry.addLine("Hosting at " + ipaddr + ":" + port + ".");
            telemetry.addLine("Current JSON Data: " + Variables.curJSONdata);
            telemetry.addLine("Pressed Keys: " + pressedKeysString);
            telemetry.update();
        }
        if (srvRunning) {
            TinyWebServer.stopServer();
        }
        srvRunning = false;
    }
}
