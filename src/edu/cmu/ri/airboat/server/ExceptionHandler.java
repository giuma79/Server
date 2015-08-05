package edu.cmu.ri.airboat.server;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.lang.Thread.UncaughtExceptionHandler;
import java.text.SimpleDateFormat;
import java.util.Date;

import android.app.Activity;
import android.app.AlarmManager;
import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbAccessory;
import android.hardware.usb.UsbManager;
import android.os.Environment;
import android.util.Log;

import com.madara.KnowledgeBase;

/**
 * @author jjb
 * Based on code from http://chintanrathod.com/auto-restart-application-after-crash-forceclose-in-android/
 */
public class ExceptionHandler implements UncaughtExceptionHandler {

    private UncaughtExceptionHandler defaultUEH;
    Activity activity;
    Context context;
    int restartMilliseconds;

    public ExceptionHandler(Activity activity, Context context, int restartMilliseconds) {
        this.activity = activity;
        this.context = context;
        this.restartMilliseconds = restartMilliseconds;
    }

    @Override
    public void uncaughtException(Thread thread, Throwable ex) {

        try {

            activity.getIntent().setAction("");

            Intent intent1 = new Intent(activity, AirboatActivity.class);
            Intent intent2 = new Intent(activity, LauncherActivity.class);

            intent1.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP
                    | Intent.FLAG_ACTIVITY_CLEAR_TASK
                    | Intent.FLAG_ACTIVITY_NEW_TASK);
            //intent1.putExtra("EXIT", true);

            intent2.addFlags(//Intent.FLAG_ACTIVITY_CLEAR_TOP
                    Intent.FLAG_ACTIVITY_CLEAR_TASK
                            | Intent.FLAG_ACTIVITY_NEW_TASK
            );
            //intent2.fillIn(activity.getIntent(), Intent.FILL_IN_DATA);
            //UsbAccessory accessory = (UsbAccessory) activity.getIntent().getParcelableExtra(UsbManager.EXTRA_ACCESSORY);
            //IntentFilter filter = new IntentFilter(UsbManager.ACTION_USB_ACCESSORY_ATTACHED);

            PendingIntent pendingIntent1 = PendingIntent.getActivity(context, 0, intent1, intent1.getFlags());
            PendingIntent pendingIntent2 = PendingIntent.getActivity(context, 0, intent2, intent2.getFlags());

            //Following code will restart your application after 5 seconds
            //AlarmManager mgr = (AlarmManager) context.getSystemService(Context.ALARM_SERVICE);
            //mgr.set(AlarmManager.RTC, System.currentTimeMillis() + restartMilliseconds, pendingIntent1);
            //mgr.set(AlarmManager.RTC, System.currentTimeMillis() + restartMilliseconds + 100, pendingIntent2);

            Log.e("jjb", "Unhandled Exception: ", ex);

            //This will finish your activity manually
            activity.finishActivity(0);
            activity.finish();

            //This will stop your application and take out from it.
            //System.exit(2);
            android.os.Process.killProcess(android.os.Process.myPid());
            //super.onDestroy();

        } catch (Exception e){//IOException e) {
            e.printStackTrace();
            //e.printStackTrace();
        }
    }
}