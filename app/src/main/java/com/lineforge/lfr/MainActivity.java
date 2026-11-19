package com.lineforge.lfr;

import android.Manifest;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.ActivityNotFoundException;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.content.res.ColorStateList;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.location.LocationManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.provider.Settings;
import android.text.Editable;
import android.text.InputType;
import android.text.TextWatcher;
import android.view.Gravity;
import android.view.View;
import android.view.ViewGroup;
import android.view.inputmethod.InputMethodManager;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.SeekBar;
import android.widget.Spinner;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.ArrayDeque;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * LineForge LFR: a no-third-party-dependencies Bluetooth Low Energy controller
 * for the ESP32-S3's built-in BLE, using the Nordic UART Service (see BleUartClient).
 * Protocol: two-byte binary commands; compatible with ESP32S3_LFR_BT_NVS.
 * The accompanying firmware extension adds [9,0] status query and [10,1] heartbeat.
 */
public final class MainActivity extends Activity {
    private static final int BG = Color.rgb(10, 16, 32);
    private static final int PANEL = Color.rgb(19, 29, 49);
    private static final int PANEL_ALT = Color.rgb(25, 38, 61);
    private static final int BORDER = Color.rgb(43, 59, 83);
    private static final int TEXT = Color.rgb(240, 246, 255);
    private static final int MUTED = Color.rgb(151, 168, 191);
    private static final int MINT = Color.rgb(54, 226, 198);
    private static final int BLUE = Color.rgb(96, 164, 255);
    private static final int PURPLE = Color.rgb(177, 136, 255);
    private static final int RED = Color.rgb(255, 98, 111);
    private static final int ORANGE = Color.rgb(255, 196, 100);
    private static final int REQ_CONNECT = 151;
    private static final long SCAN_DURATION_MS = 12000;
    private static final String PREFS_NAME = "lineforge_ui";

    private final Handler ui = new Handler(Looper.getMainLooper());
    private final AtomicInteger session = new AtomicInteger();
    private BleUartClient ble;
    private Protocol.Parser parser;
    private volatile boolean connected = false;
    private boolean running = false;
    private boolean startPending = false;
    private int keepAliveTicks = 0;
    private int droppedBytes = 0, crcErrors = 0;
    private long lastLinkWarning = 0;
    private boolean calibrationDialogPending = false;
    private final ArrayDeque<String> recentEvents = new ArrayDeque<>();
    private QtrSensorView sensorView;
    private TextView sensorInfo, calibrationInfo, linkInfo;
    private boolean liveTune = false;
    private boolean calibrationValid = false;
    private boolean queryReceived = false;
    private SharedPreferences preferences;
    private TextView connectionLabel, selectedDevice, heroStatus, eventLog, sendCountLabel;
    private View connectionDot;
    private Button connectButton, startButton, stopButton;
    private Switch liveSwitch;
    private PidControl kp, ki, kd;
    private int sentBytes = 0, receivedBytes = 0;
    private String lastEvent = "Ready to pair";
    private final Runnable heartbeatTask = new Runnable() {
        @Override public void run() {
            if (connected && (running || startPending)) sendBytes(new byte[]{10, 1}, false);
            // The firmware stops telemetry after 4s without a lease refresh.
            if (connected && (++keepAliveTicks % 3) == 0) {
                sendBytes(new byte[]{11, 1}, false);
                if (!queryReceived) sendBytes(new byte[]{9, 0}, false);
            }
            ui.postDelayed(this, 600);
        }
    };

    @Override public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().setStatusBarColor(BG);
        getWindow().setNavigationBarColor(BG);
        preferences = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
        ble = new BleUartClient(this, bleCallback);
        createInterface();
        ui.postDelayed(heartbeatTask, 600);
        addEvent("Power the ESP32-S3, then tap Scan to find it over BLE. No pairing needed.");
    }

    @Override protected void onStop() {
        // Treat the app leaving the foreground as an intentional STOP.
        if ((running || startPending) && connected) {
            running = false;
            startPending = false;
            sendBytes(new byte[]{7, 0}, true);
            updateRunUI();
        }
        super.onStop();
    }

    @Override protected void onDestroy() {
        ui.removeCallbacks(heartbeatTask);
        ble.stopScan();
        disconnectLink();
        super.onDestroy();
    }

    private int dp(float value) {
        return Math.round(getResources().getDisplayMetrics().density * value);
    }

    private GradientDrawable rounded(int color, int radius, int outline) {
        GradientDrawable d = new GradientDrawable();
        d.setColor(color);
        d.setCornerRadius(dp(radius));
        if (outline != 0) d.setStroke(dp(1), outline);
        return d;
    }

    private TextView txt(String value, int sp, int color, boolean heavy) {
        TextView t = new TextView(this);
        t.setText(value);
        t.setTextSize(sp);
        t.setTextColor(color);
        t.setGravity(Gravity.CENTER_VERTICAL);
        if (heavy) t.setTypeface(Typeface.create("sans-serif", Typeface.BOLD));
        return t;
    }

    private LinearLayout col() {
        LinearLayout l = new LinearLayout(this);
        l.setOrientation(LinearLayout.VERTICAL);
        return l;
    }

    private LinearLayout row() {
        LinearLayout l = new LinearLayout(this);
        l.setOrientation(LinearLayout.HORIZONTAL);
        l.setGravity(Gravity.CENTER_VERTICAL);
        return l;
    }

    private LinearLayout.LayoutParams wrap() {
        return new LinearLayout.LayoutParams(ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
    }

    private LinearLayout.LayoutParams full() {
        return new LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT);
    }

    private LinearLayout.LayoutParams weighted() {
        return new LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f);
    }

    private View gap(int height) {
        View v = new View(this);
        v.setLayoutParams(new LinearLayout.LayoutParams(1, dp(height)));
        return v;
    }

    private LinearLayout card() {
        LinearLayout c = col();
        c.setPadding(dp(18), dp(17), dp(18), dp(18));
        c.setBackground(rounded(PANEL, 20, BORDER));
        return c;
    }

    private Button button(String label, int fill, int foreground) {
        Button b = new Button(this);
        b.setText(label);
        b.setAllCaps(false);
        b.setTextSize(14);
        b.setTypeface(Typeface.create("sans-serif-medium", Typeface.BOLD));
        b.setTextColor(foreground);
        b.setMinHeight(0);
        b.setMinimumHeight(0);
        b.setPadding(dp(13), dp(13), dp(13), dp(13));
        b.setBackground(rounded(fill, 14, fill == PANEL_ALT ? BORDER : 0));
        return b;
    }

    private void createInterface() {
        LinearLayout screen = col();
        screen.setBackgroundColor(BG);
        setContentView(screen);

        ScrollView scroll = new ScrollView(this);
        scroll.setClipToPadding(false);
        scroll.setFillViewport(true);
        scroll.setVerticalScrollBarEnabled(false);
        screen.addView(scroll, new LinearLayout.LayoutParams(-1, 0, 1f));
        LinearLayout content = col();
        content.setPadding(dp(18), dp(24), dp(18), dp(20));
        scroll.addView(content);

        LinearLayout heading = row();
        TextView mark = txt("LF", 21, BG, true);
        mark.setGravity(Gravity.CENTER);
        mark.setBackground(rounded(MINT, 15, 0));
        heading.addView(mark, new LinearLayout.LayoutParams(dp(47), dp(47)));
        LinearLayout headingText = col();
        headingText.setPadding(dp(12), 0, 0, 0);
        headingText.addView(txt("LINEFORGE", 22, TEXT, true));
        headingText.addView(txt("ESP32-S3  /  BLUETOOTH PID STUDIO", 10, MUTED, false));
        heading.addView(headingText);
        content.addView(heading);
        content.addView(gap(22));

        LinearLayout deviceCard = card();
        LinearLayout deviceHeader = row();
        connectionDot = new View(this);
        connectionDot.setBackground(rounded(RED, 100, 0));
        deviceHeader.addView(connectionDot, new LinearLayout.LayoutParams(dp(11), dp(11)));
        connectionLabel = txt("OFFLINE", 12, RED, true);
        connectionLabel.setPadding(dp(9), 0, 0, 0);
        deviceHeader.addView(connectionLabel, weighted());
        TextView protocol = txt("BLE  •  NORDIC UART", 10, MUTED, true);
        deviceHeader.addView(protocol);
        deviceCard.addView(deviceHeader);
        deviceCard.addView(gap(13));
        selectedDevice = txt("No ESP32 connected", 16, TEXT, true);
        deviceCard.addView(selectedDevice);
        deviceCard.addView(gap(14));
        connectButton = button("Scan for ESP32 (BLE)", MINT, BG);
        connectButton.setOnClickListener(v -> {
            if (connected || ble.isActive()) {
                disconnectLink();
                addEvent("Disconnected by user.");
            } else ensureBluetoothPermission();
        });
        deviceCard.addView(connectButton, full());
        content.addView(deviceCard, full());
        content.addView(gap(14));

        // Always-visible animated eight-channel QTR sensor monitor.
        LinearLayout sensorCard = card();
        LinearLayout sensorHeading = row();
        sensorHeading.addView(txt("LIVE LINE ARRAY", 13, TEXT, true), weighted());
        sensorHeading.addView(txt("QTR-8A  •  S1–S8", 11, MUTED, false));
        sensorCard.addView(sensorHeading);
        sensorCard.addView(gap(8));
        sensorView = new QtrSensorView(this);
        sensorCard.addView(sensorView, full());
        sensorInfo = txt("Waiting for verified sensor packets…", 11, MUTED, false);
        sensorCard.addView(sensorInfo);
        sensorCard.addView(gap(10));
        calibrationInfo = txt("Saved white/black ranges: awaiting ESP32", 11, MUTED, false);
        sensorCard.addView(calibrationInfo);
        sensorCard.addView(gap(10));
        Button readQtr = button("READ SAVED QTR CALIBRATION", PANEL_ALT, TEXT);
        readQtr.setOnClickListener(v -> requestCalibrationData(true));
        sensorCard.addView(readQtr, full());
        content.addView(sensorCard, full());
        content.addView(gap(18));

        LinearLayout titleRow = row();
        titleRow.addView(txt("PID TUNING", 13, TEXT, true), weighted());
        TextView resolution = txt("0–255  /  10^divider", 11, MUTED, false);
        titleRow.addView(resolution);
        content.addView(titleRow, full());
        content.addView(gap(10));

        kp = new PidControl("KP", "PROPORTIONAL", MINT, 1, 40, 0);
        ki = new PidControl("KI", "INTEGRAL", BLUE, 3, 0, 0);
        kd = new PidControl("KD", "DERIVATIVE", PURPLE, 5, 1, 0);
        content.addView(kp.view, full()); content.addView(gap(11));
        content.addView(ki.view, full()); content.addView(gap(11));
        content.addView(kd.view, full()); content.addView(gap(13));

        LinearLayout autoRow = row();
        LinearLayout autoText = col();
        autoText.addView(txt("Live tuning", 14, TEXT, true));
        autoText.addView(txt("Send changes on slider release", 11, MUTED, false));
        autoRow.addView(autoText, weighted());
        liveSwitch = new Switch(this);
        liveSwitch.setThumbTintList(ColorStateList.valueOf(MINT));
        liveSwitch.setTrackTintList(ColorStateList.valueOf(BORDER));
        liveSwitch.setChecked(preferences.getBoolean("live", false));
        liveTune = liveSwitch.isChecked();
        liveSwitch.setOnCheckedChangeListener((CompoundButton b, boolean checked) -> {
            liveTune = checked;
            preferences.edit().putBoolean("live", checked).apply();
        });
        autoRow.addView(liveSwitch);
        content.addView(autoRow, full());
        content.addView(gap(12));

        LinearLayout uploadButtons = row();
        Button apply = button("APPLY ALL  ↗", MINT, BG);
        apply.setOnClickListener(v -> uploadAll());
        uploadButtons.addView(apply, new LinearLayout.LayoutParams(0, dp(51), 1f));
        View split = new View(this);
        uploadButtons.addView(split, new LinearLayout.LayoutParams(dp(9), 1));
        Button read = button("READ FROM ROBOT", PANEL_ALT, TEXT);
        read.setOnClickListener(v -> requestRobotState());
        uploadButtons.addView(read, new LinearLayout.LayoutParams(0, dp(51), 1f));
        content.addView(uploadButtons);
        content.addView(gap(20));

        LinearLayout calCard = card();
        calCard.addView(txt("SENSOR CALIBRATION", 13, TEXT, true));
        calCard.addView(gap(7));
        calCard.addView(txt("Saved to ESP32 NVS • 8-channel QTR-8A", 12, MUTED, false));
        calCard.addView(gap(14));
        Button recalibrate = button("RECALIBRATE SENSORS", PANEL_ALT, TEXT);
        recalibrate.setOnClickListener(v -> confirmCalibration());
        calCard.addView(recalibrate, full());
        content.addView(calCard, full());
        content.addView(gap(17));

        LinearLayout console = card();
        LinearLayout consoleHead = row();
        consoleHead.addView(txt("LINK CONSOLE", 12, TEXT, true), weighted());
        sendCountLabel = txt("TX 0 B  /  RX 0 B", 10, MUTED, false);
        consoleHead.addView(sendCountLabel);
        console.addView(consoleHead);
        console.addView(gap(12));
        eventLog = txt("Waiting for ESP32…", 12, MUTED, false);
        eventLog.setLineSpacing(dp(5), 1.0f);
        console.addView(eventLog);
        console.addView(gap(8));
        linkInfo = txt("Verified frames: 0  •  CRC errors: 0  •  ignored: 0", 10, MUTED, false);
        console.addView(linkInfo);
        content.addView(console, full());
        content.addView(gap(14));
        TextView foot = txt("Protocol v3  •  CRC8 telemetry  •  BLE NUS", 11, MUTED, false);
        foot.setGravity(Gravity.CENTER);
        content.addView(foot);

        // Sticky run controls, always accessible even while scrolling.
        LinearLayout actionArea = col();
        actionArea.setPadding(dp(18), dp(12), dp(18), dp(14));
        actionArea.setBackground(rounded(PANEL, 18, BORDER));
        heroStatus = txt("MOTORS DISARMED", 11, MUTED, true);
        heroStatus.setGravity(Gravity.CENTER);
        actionArea.addView(heroStatus);
        actionArea.addView(gap(10));
        LinearLayout actionRow = row();
        startButton = button("▶  START RUN", MINT, BG);
        stopButton = button("■  STOP NOW", RED, Color.WHITE);
        actionRow.addView(startButton, new LinearLayout.LayoutParams(0, dp(58), 1f));
        actionRow.addView(new View(this), new LinearLayout.LayoutParams(dp(10), 1));
        actionRow.addView(stopButton, new LinearLayout.LayoutParams(0, dp(58), 1f));
        startButton.setOnClickListener(v -> startRobot());
        stopButton.setOnClickListener(v -> stopRobot());
        actionArea.addView(actionRow);
        screen.addView(actionArea, full());

        kp.restore(); ki.restore(); kd.restore();
        updateRunUI();
    }

    private final class PidControl {
        final String key, description;
        final int color, command;
        final LinearLayout view;
        final TextView largeValue;
        final SeekBar slider;
        final EditText number;
        final Spinner divider;
        boolean binding = false;

        PidControl(String key, String description, int color, int command, int initialNumber, int initialDivider) {
            this.key = key; this.description = description; this.color = color; this.command = command;
            view = card();
            LinearLayout top = row();
            LinearLayout lhs = col();
            lhs.addView(txt(description, 10, MUTED, true));
            lhs.addView(txt(key, 19, color, true));
            top.addView(lhs, weighted());
            largeValue = txt("0.0000", 29, TEXT, true);
            largeValue.setGravity(Gravity.RIGHT | Gravity.CENTER_VERTICAL);
            top.addView(largeValue);
            view.addView(top);
            view.addView(gap(14));
            slider = new SeekBar(MainActivity.this);
            slider.setMax(255);
            slider.setProgressTintList(ColorStateList.valueOf(color));
            slider.setThumbTintList(ColorStateList.valueOf(color));
            slider.setProgressBackgroundTintList(ColorStateList.valueOf(BORDER));
            view.addView(slider, full());
            view.addView(gap(12));
            LinearLayout bottom = row();
            bottom.addView(txt("VALUE", 10, MUTED, true));
            number = new EditText(MainActivity.this);
            number.setTextColor(TEXT);
            number.setTextSize(14);
            number.setSingleLine(true);
            number.setInputType(InputType.TYPE_CLASS_NUMBER);
            number.setSelectAllOnFocus(true);
            number.setGravity(Gravity.CENTER);
            number.setBackground(rounded(PANEL_ALT, 9, BORDER));
            number.setPadding(dp(8), dp(7), dp(8), dp(7));
            LinearLayout.LayoutParams nlp = new LinearLayout.LayoutParams(dp(66), dp(38));
            nlp.setMargins(dp(9), 0, dp(14), 0);
            bottom.addView(number, nlp);
            bottom.addView(txt("DIVIDE BY", 10, MUTED, true));
            divider = new Spinner(MainActivity.this);
            final String[] options = {"1", "10", "100", "1k", "10k"};
            ArrayAdapter<String> adapter = new ArrayAdapter<String>(MainActivity.this,
                    android.R.layout.simple_spinner_item, options) {
                @Override public View getView(int position, View recycled, ViewGroup parent) {
                    TextView t = txt(options[position], 14, TEXT, true);
                    t.setText(options[position] + "  ▾");
                    t.setPadding(dp(10), dp(7), 0, dp(7));
                    return t;
                }
                @Override public View getDropDownView(int position, View recycled, ViewGroup parent) {
                    TextView t = txt(options[position], 15, TEXT, true);
                    t.setBackgroundColor(PANEL_ALT);
                    t.setPadding(dp(18), dp(13), dp(18), dp(13));
                    return t;
                }
            };
            divider.setAdapter(adapter);
            divider.setBackground(rounded(PANEL_ALT, 9, BORDER));
            LinearLayout.LayoutParams dlp = new LinearLayout.LayoutParams(0, dp(38), 1f);
            dlp.setMargins(dp(9), 0, 0, 0);
            bottom.addView(divider, dlp);
            view.addView(bottom);
            slider.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onStartTrackingTouch(SeekBar sb) { }
                @Override public void onProgressChanged(SeekBar sb, int v, boolean fromUser) {
                    if (!fromUser || binding) return;
                    binding = true;
                    number.setText(String.valueOf(v));
                    number.setSelection(number.length());
                    binding = false;
                    updateDisplay();
                }
                @Override public void onStopTrackingTouch(SeekBar sb) {
                    saveLocal();
                    if (liveTune) sendThisGain();
                }
            });
            number.addTextChangedListener(new TextWatcher() {
                @Override public void beforeTextChanged(CharSequence s, int start, int count, int after) { }
                @Override public void onTextChanged(CharSequence s, int start, int before, int count) {
                    if (binding) return;
                    binding = true;
                    slider.setProgress(value());
                    binding = false;
                    updateDisplay();
                }
                @Override public void afterTextChanged(Editable s) { }
            });
            number.setOnFocusChangeListener((v, focused) -> {
                if (!focused) {
                    setData(value(), divisor());
                    saveLocal();
                    if (liveTune) sendThisGain();
                }
            });
            divider.setOnItemSelectedListener(new android.widget.AdapterView.OnItemSelectedListener() {
                @Override public void onNothingSelected(android.widget.AdapterView<?> p) { }
                @Override public void onItemSelected(android.widget.AdapterView<?> p, View v, int position, long id) {
                    updateDisplay();
                    if (binding) return;
                    saveLocal();
                    // Never auto-send divider on initial spinner binding.
                    if (liveTune && divider.getTag() != null) sendThisGain();
                    divider.setTag("bound");
                }
            });
            setData(initialNumber, initialDivider);
        }

        int value() {
            try { return Math.min(255, Math.max(0, Integer.parseInt(number.getText().toString()))); }
            catch (Exception e) { return 0; }
        }
        int divisor() { return Math.min(4, Math.max(0, divider.getSelectedItemPosition())); }
        void setData(int numerator, int divisor) {
            binding = true;
            int v = Math.min(255, Math.max(0, numerator));
            int d = Math.min(4, Math.max(0, divisor));
            number.setText(String.valueOf(v));
            slider.setProgress(v);
            divider.setSelection(d);
            binding = false;
            updateDisplay();
        }
        void updateDisplay() {
            double value = value() / Math.pow(10.0, divisor());
            largeValue.setText(String.format(Locale.US, "%.4f", value));
        }
        void restore() {
            int defNum = command == 1 ? 40 : command == 5 ? 1 : 0;
            setData(preferences.getInt(key + "_v", defNum), preferences.getInt(key + "_d", 0));
        }
        void saveLocal() {
            preferences.edit().putInt(key + "_v", value()).putInt(key + "_d", divisor()).apply();
        }
        void sendThisGain() {
            if (!connected) return;
            sendBytes(new byte[]{(byte)command, (byte)value(), (byte)(command + 1), (byte)divisor()}, true);
            addEvent(key + " updated: " + largeValue.getText());
        }
    }

    private void uploadAll() {
        if (!connected) { showToast("Connect your ESP32 first"); return; }
        blurInputs();
        byte[] values = new byte[]{
                1, (byte)kp.value(), 2, (byte)kp.divisor(),
                3, (byte)ki.value(), 4, (byte)ki.divisor(),
                5, (byte)kd.value(), 6, (byte)kd.divisor()};
        kp.saveLocal(); ki.saveLocal(); kd.saveLocal();
        sendBytes(values, true);
        addEvent("Sent six PID settings (12 bytes). NVS will save after 2 s.");
    }

    private void blurInputs() {
        View focus = getCurrentFocus();
        if (focus != null) {
            focus.clearFocus();
            InputMethodManager input = (InputMethodManager)getSystemService(INPUT_METHOD_SERVICE);
            if (input != null) input.hideSoftInputFromWindow(focus.getWindowToken(), 0);
        }
    }

    private void startRobot() {
        if (!connected) { showToast("Connect ESP32 first"); return; }
        if (queryReceived && !calibrationValid) {
            showToast("Calibrate QTR-8A before starting"); return;
        }
        startPending = true;
        running = false;
        sendBytes(new byte[]{7, 1}, true);
        updateRunUI();
        addEvent("START sent; waiting for verified ESP32 motor status…");
        ui.postDelayed(() -> {
            if (connected && startPending) {
                startPending = false;
                running = false;
                updateRunUI();
                // Do not assume the robot stopped just because its reply was lost.
                // Explicit STOP as well as the firmware heartbeat timeout.
                sendBytes(new byte[]{7, 0}, false);
                addEvent("NO START CONFIRMATION: STOP sent. Check BLE link, power and firmware.");
            }
        }, 2500);
    }

    private void stopRobot() {
        startPending = false;
        running = false;
        updateRunUI();
        if (connected) sendBytes(new byte[]{7, 0}, true);
        addEvent("STOP requested; waiting for ESP32 status.");
    }

    private void updateRunUI() {
        if (heroStatus == null) return;
        heroStatus.setText(startPending ? "START SENT  •  AWAITING ESP32" :
                running ? "RUNNING  •  CONFIRMED BY ESP32" : "MOTORS DISARMED");
        heroStatus.setTextColor(startPending ? ORANGE : running ? MINT : MUTED);
        if (startButton != null) startButton.setAlpha(connected && !running && !startPending ? 1f : 0.55f);
        if (stopButton != null) stopButton.setAlpha(connected ? 1f : 0.7f);
        if (running || startPending) getWindow().addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        else getWindow().clearFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
    }

    private void confirmCalibration() {
        if (!connected) { showToast("Connect the ESP32 first"); return; }
        new AlertDialog.Builder(this)
                .setTitle("Recalibrate QTR-8A?")
                .setMessage("Place the robot on WHITE before proceeding. Once started, leave it still for 2 seconds, then sweep all 8 sensors across the black line for 5 seconds. The motors will stay stopped. Successful values are saved to NVS.")
                .setNegativeButton("Cancel", null)
                .setPositiveButton("Start calibration", (dialog, which) -> {
                    running = false;
                    startPending = false;
                    updateRunUI();
                    sendBytes(new byte[]{8, 1}, true);
                    addEvent("Calibrating: 2 s white + 5 s sweep. Awaiting result…");
                }).show();
    }

    private void requestRobotState() {
        if (!connected) { showToast("Connect the ESP32 first"); return; }
        queryReceived = false;
        sendBytes(new byte[]{9, 0}, true);
        addEvent("Requesting actual PID values from the ESP32…");
        ui.postDelayed(() -> {
            if (connected && !queryReceived)
                addEvent("No verified ESP32 status: check that the firmware feeds NUS RX writes to its command parser and replies via TX notify; also check power and motor-triggered resets.");
        }, 2300);
    }

    private void requestCalibrationData(boolean showDialog) {
        if (!connected) { showToast("Connect ESP32 first"); return; }
        calibrationDialogPending = showDialog;
        sendBytes(new byte[]{12, 0}, true);
        if (showDialog) addEvent("Requesting eight saved white/black calibration pairs…");
    }

    // ------------------------------------------------------------ BLE permissions

    private String[] blePermissions() {
        if (Build.VERSION.SDK_INT >= 31)
            return new String[]{Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT};
        // Android 11 and older only return BLE scan results with location permission.
        return new String[]{Manifest.permission.ACCESS_FINE_LOCATION};
    }

    private boolean hasBlePermissions() {
        for (String p : blePermissions())
            if (checkSelfPermission(p) != PackageManager.PERMISSION_GRANTED) return false;
        return true;
    }

    private void ensureBluetoothPermission() {
        if (!hasBlePermissions()) {
            requestPermissions(blePermissions(), REQ_CONNECT);
            return;
        }
        scanForDevices();
    }

    @Override public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] results) {
        super.onRequestPermissionsResult(requestCode, permissions, results);
        if (requestCode == REQ_CONNECT) {
            boolean granted = results.length > 0;
            for (int r : results) granted &= r == PackageManager.PERMISSION_GRANTED;
            if (granted) scanForDevices();
            else addEvent(Build.VERSION.SDK_INT >= 31
                    ? "Nearby devices permission is required to scan for and connect to the ESP32."
                    : "Location permission is required by Android to scan for BLE devices.");
        }
    }

    private boolean locationServicesOn() {
        LocationManager lm = (LocationManager)getSystemService(LOCATION_SERVICE);
        if (lm == null) return true;
        if (Build.VERSION.SDK_INT >= 28) return lm.isLocationEnabled();
        return lm.isProviderEnabled(LocationManager.GPS_PROVIDER) ||
                lm.isProviderEnabled(LocationManager.NETWORK_PROVIDER);
    }

    // ----------------------------------------------------------- BLE device scan

    private static final class ScanEntry {
        final BluetoothDevice device;
        String name;
        int rssi;
        boolean nus;
        ScanEntry(BluetoothDevice device) { this.device = device; }
        @Override public String toString() {
            return (name == null ? "Unnamed BLE device" : name) + (nus ? "   •  UART service" : "") +
                    "\n" + device.getAddress() + "   " + rssi + " dBm";
        }
    }

    private void scanForDevices() {
        if (!hasBlePermissions()) return;
        BluetoothAdapter adapter = ble.adapter();
        if (adapter == null || !getPackageManager().hasSystemFeature(PackageManager.FEATURE_BLUETOOTH_LE)) {
            showToast("This phone does not support Bluetooth Low Energy"); return;
        }
        if (!adapter.isEnabled()) {
            new AlertDialog.Builder(this).setTitle("Bluetooth is off")
                    .setMessage("Turn on Bluetooth to find the ESP32-S3.")
                    .setNegativeButton("Cancel", null)
                    .setPositiveButton("Turn on", (d, w) -> requestBluetoothOn())
                    .show();
            return;
        }
        if (Build.VERSION.SDK_INT < 31 && !locationServicesOn()) {
            new AlertDialog.Builder(this).setTitle("Location is off")
                    .setMessage("On Android 11 and older, BLE scanning only finds devices while Location is switched on.")
                    .setNegativeButton("Cancel", null)
                    .setPositiveButton("Location settings", (d, w) -> {
                        try { startActivity(new Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS)); }
                        catch (ActivityNotFoundException e) { showToast("Enable Location in Android settings"); }
                    }).show();
            return;
        }

        final List<ScanEntry> entries = new ArrayList<>();
        final ArrayAdapter<ScanEntry> list = new ArrayAdapter<>(this, android.R.layout.select_dialog_item, entries);
        final AlertDialog dialog = new AlertDialog.Builder(this)
                .setTitle("Scanning for ESP32-S3…")
                .setAdapter(list, (d, which) -> {
                    ble.stopScan();
                    if (which >= 0 && which < entries.size()) connectDevice(entries.get(which).device);
                })
                .setNegativeButton("Cancel", null)
                .create();
        final Runnable finishScan = () -> {
            ble.stopScan();
            if (dialog.isShowing())
                dialog.setTitle(entries.isEmpty() ? "No BLE devices found — tap Cancel and retry" : "Select your ESP32-S3");
        };
        dialog.setOnDismissListener(d -> { ui.removeCallbacks(finishScan); ble.stopScan(); });

        boolean started = ble.startScan(new BleUartClient.ScanListener() {
            @Override public void onDevice(BluetoothDevice device, String name, int rssi, boolean advertisesNus) {
                ScanEntry found = null;
                for (ScanEntry e : entries) if (e.device.getAddress().equals(device.getAddress())) { found = e; break; }
                // Hide anonymous beacons; keep anything named or advertising the UART service.
                if (found == null && name == null && !advertisesNus) return;
                boolean resort = found == null || (advertisesNus && !found.nus);
                if (found == null) { found = new ScanEntry(device); entries.add(found); }
                if (name != null) found.name = name;
                found.nus |= advertisesNus;
                found.rssi = rssi;
                if (resort) {
                    // UART-service devices first, then likely ESP32 names, then by signal.
                    Collections.sort(entries, (a, b) -> {
                        if (a.nus != b.nus) return a.nus ? -1 : 1;
                        boolean ae = a.name != null && a.name.toLowerCase(Locale.ROOT).contains("esp");
                        boolean be = b.name != null && b.name.toLowerCase(Locale.ROOT).contains("esp");
                        if (ae != be) return ae ? -1 : 1;
                        return Integer.compare(b.rssi, a.rssi);
                    });
                }
                list.notifyDataSetChanged();
            }
            @Override public void onScanFailed(int errorCode) {
                ui.removeCallbacks(finishScan);
                if (dialog.isShowing()) dialog.setTitle("BLE scan failed (code " + errorCode + ")");
                addEvent("BLE scan failed with code " + errorCode + ". Toggle Bluetooth and retry.");
            }
        });
        if (!started) { showToast("Could not start BLE scan"); return; }
        dialog.show();
        ui.postDelayed(finishScan, SCAN_DURATION_MS);
    }

    @SuppressLint("MissingPermission")
    private void requestBluetoothOn() {
        try { startActivity(new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)); }
        catch (ActivityNotFoundException | SecurityException e) { openBluetoothSettings(); }
    }

    private void openBluetoothSettings() {
        try { startActivity(new Intent(Settings.ACTION_BLUETOOTH_SETTINGS)); }
        catch (ActivityNotFoundException e) { showToast("Open Bluetooth in Android settings"); }
    }

    // --------------------------------------------------------------- BLE session

    private void connectDevice(BluetoothDevice device) {
        if (!hasBlePermissions()) { ensureBluetoothPermission(); return; }
        resetLinkState();
        ble.connect(device); // reports progress through bleCallback
    }

    private void showConnectingUI(String name, String label) {
        connectionLabel.setText(label);
        connectionLabel.setTextColor(ORANGE);
        connectionDot.setBackground(rounded(ORANGE, 100, 0));
        if (name != null) selectedDevice.setText(name);
        connectButton.setEnabled(true);
        connectButton.setText("Cancel");
        updateRunUI();
    }

    private void updateCounters() {
        sendCountLabel.setText("TX " + sentBytes + " B  /  RX " + receivedBytes + " B");
    }

    // All BleUartClient callbacks arrive on the main thread.
    private final BleUartClient.Callback bleCallback = new BleUartClient.Callback() {
        @Override public void onConnecting(String name, int attempt, int maxRetries) {
            showConnectingUI(name, attempt == 0 ? "CONNECTING…" :
                    "RECONNECTING " + attempt + "/" + maxRetries + "…");
        }
        @Override public void onReady(String name, int mtu) {
            int epoch = session.incrementAndGet();
            parser = createParser(epoch);
            connected = true;
            queryReceived = false;
            updateConnectionUI(true, name);
            addEvent("Connected to " + name + " over BLE (Nordic UART, MTU " + mtu + ").");
            sendBytes(new byte[]{11, 1}, true);
            requestRobotState();
            requestCalibrationData(false);
        }
        @Override public void onDisconnected(String name, String reason, boolean willRetry) {
            boolean wasConnected = connected;
            resetLinkState();
            String safety = wasConnected ? " Robot stops via ESP32 heartbeat timeout." : "";
            if (willRetry) {
                showConnectingUI(name, "RECONNECTING…");
                addEvent((wasConnected ? "BLE link lost: " : "Connection attempt failed: ") + reason +
                        ". Retrying automatically…" + safety);
            } else {
                updateConnectionUI(false, null);
                addEvent((wasConnected ? "BLE connection closed: " : "Could not connect: ") + reason + "." + safety);
            }
        }
        @Override public void onData(byte[] data) {
            receivedBytes += data.length;
            updateCounters();
            if (parser != null) parser.feed(data, data.length);
        }
        @Override public void onBytesWritten(int count) {
            sentBytes += count;
            updateCounters();
        }
        @Override public void onError(String message) { addEvent(message); }
    };

    // Stream parser: ignores boot/noise bytes, verifies CRC8 and recovers from
    // frames split across BLE notifications without permanently desynchronizing.
    private Protocol.Parser createParser(int epoch) {
        return new Protocol.Parser(new Protocol.Listener() {
            @Override public void onPacket(int type, byte[] payload) {
                ui.post(() -> {
                    if (session.get() == epoch) receivePacket(type, payload);
                });
            }
            @Override public void onLegacyStatus(byte[] data) {
                ui.post(() -> {
                    if (session.get() == epoch) receiveStateFrame(data, false);
                });
            }
            @Override public void onReply(int code) {
                ui.post(() -> {
                    if (session.get() == epoch) handleReply(code);
                });
            }
            @Override public void onDiscarded(int count, boolean checksumError) {
                if (checksumError) crcErrors++;
                else droppedBytes += count;
                long now = SystemClock.uptimeMillis();
                if (now - lastLinkWarning > 1800) {
                    lastLinkWarning = now;
                    final int c = crcErrors, d = droppedBytes;
                    ui.post(() -> {
                        if (session.get() != epoch) return;
                        linkInfo.setText("CRC errors: " + c + "  •  ignored bytes: " + d);
                        if (!queryReceived && c + d > 4)
                            addEvent("Unverified bytes from ESP32: check the firmware's TX protocol and supply noise.");
                    });
                }
            }
        });
    }

    private void handleReply(int b) {
        if (b == 0x06) return;
        if (b == 0x15) {
            addEvent("ESP32 rejected a command. Check calibration and firmware protocol.");
            if (startPending) {
                startPending = false;
                running = false;
                updateRunUI();
            }
            return;
        }
        if (b == 0xC1) {
            calibrationValid = true;
            addEvent("QTR calibration successful. Saved to NVS.");
            ui.postDelayed(() -> { requestRobotState(); requestCalibrationData(false); }, 350);
            return;
        }
        if (b == 0xC0) {
            calibrationValid = false; running = false; startPending = false;
            updateRunUI(); addEvent("Calibration failed. Motors disabled."); return;
        }
        if (b == 0xC2) { calibrationValid = true; addEvent("Saved QTR calibration loaded."); return; }
        if (b == 0xD1) {
            running = false; startPending = false; updateRunUI();
            addEvent("Safety stop: no valid heartbeat at ESP32."); return;
        }
    }

    private void receivePacket(int type, byte[] payload) {
        if (type == Protocol.STATUS && payload.length == 8) {
            receiveStateFrame(payload, true);
        } else if (type == Protocol.SENSORS && payload.length == 13) {
            int[] samples = new int[8];
            for (int i=0;i<8;i++) samples[i] = payload[i] & 255;
            int pos = (payload[8] & 255) | ((payload[9] & 255) << 8);
            int flags = payload[10] & 255;
            boolean seen = (flags & 1) != 0;
            boolean cal = (flags & 8) != 0;
            sensorView.update(samples, pos, seen, cal);
            int l = (int)payload[11], r = (int)payload[12];
            sensorInfo.setText(cal ? "Calibration running • raw reflectance levels" :
                    seen ? String.format(Locale.US, "Line position %.2f / 7   •   Left %+d%%   Right %+d%%",
                            pos / 1000.0, l, r) :
                            "LINE LOST  •  Motors L " + l + "%  R " + r + "%");
        } else if (type == Protocol.CALIBRATION && payload.length == 32) {
            StringBuilder b = new StringBuilder();
            int minRange = Integer.MAX_VALUE;
            for (int i=0;i<8;i++) {
                int w=(payload[4*i] & 255) | ((payload[4*i+1] & 255)<<8);
                int black=(payload[4*i+2] & 255) | ((payload[4*i+3] & 255)<<8);
                int span=black-w;
                minRange=Math.min(minRange,span);
                b.append(String.format(Locale.US, "S%d   white %4d   black %4d   span %4d\n",i+1,w,black,span));
            }
            calibrationInfo.setText("NVS calibration received • minimum range " + minRange + " ADC counts");
            if (calibrationDialogPending) {
                calibrationDialogPending = false;
                new AlertDialog.Builder(this).setTitle("QTR-8A saved calibration")
                        .setMessage(b.toString()).setPositiveButton("Done",null).show();
            }
        }
    }

    private void receiveStateFrame(byte[] data, boolean framed) {
        if (data.length != 8) return;
        if (!framed) {
            int checksum = 0;
            for (int i = 0; i < 7; ++i) checksum ^= (data[i] & 255);
            if (checksum != (data[7] & 255)) { addEvent("Status checksum error."); return; }
        }
        int kpd=data[1] & 255, kid=data[3] & 255, kdd=data[5] & 255;
        if (kpd>4 || kid>4 || kdd>4) { addEvent("Invalid PID values in status."); return; }
        int flags = data[6] & 255;
        if ((flags & ~7) != 0) return;
        kp.setData(data[0] & 255,kpd);
        ki.setData(data[2] & 255,kid);
        kd.setData(data[4] & 255,kdd);
        kp.saveLocal(); ki.saveLocal(); kd.saveLocal();
        calibrationValid=(flags & 1)!=0;
        boolean wasPending=startPending;
        running=(flags & 2)!=0;
        startPending=false;
        updateRunUI();
        queryReceived=true;
        if (wasPending || !running)
            addEvent("Verified ESP32 status: " + (running ? "MOTORS RUNNING" : "STOPPED") +
                    ", calibration " + (calibrationValid ? "READY" : "MISSING") +
                    ", PID synchronized.");
    }

    /** Sends exact binary bytes (e.g. {7, 1}) to the NUS RX characteristic. */
    private void sendBytes(byte[] payload, boolean reportErrors) {
        if (!connected || !ble.send(payload)) {
            if (reportErrors) showToast("ESP32 is disconnected");
        }
    }

    private void updateConnectionUI(boolean active, String name) {
        connectionLabel.setText(active ? "CONNECTED" : "OFFLINE");
        connectionLabel.setTextColor(active ? MINT : RED);
        connectionDot.setBackground(rounded(active ? MINT : RED, 100, 0));
        if (name != null) selectedDevice.setText(name);
        else if (!active) selectedDevice.setText("No ESP32 connected");
        connectButton.setEnabled(true);
        connectButton.setText(active ? "Disconnect ESP32" : "Scan for ESP32 (BLE)");
        updateRunUI();
    }

    /** Clears per-connection state; stale parser callbacks are ignored via the session counter. */
    private void resetLinkState() {
        session.incrementAndGet();
        parser = null;
        running = false;
        startPending = false;
        queryReceived = false;
        connected = false;
        if (sensorView != null) sensorView.setConnected(false);
    }

    /** User-initiated disconnect (or app shutdown): no automatic reconnection. */
    private void disconnectLink() {
        resetLinkState();
        if (ble != null) ble.disconnect();
        if (connectionLabel != null) updateConnectionUI(false, null);
    }

    private void addEvent(String s) {
        lastEvent = s;
        recentEvents.addFirst(s);
        while (recentEvents.size() > 5) recentEvents.removeLast();
        if (eventLog != null) {
            StringBuilder b = new StringBuilder();
            for (String e : recentEvents) {
                if (b.length() > 0) b.append("\n\n");
                b.append("• ").append(e);
            }
            eventLog.setText(b.toString());
        }
    }

    private void showToast(String s) { Toast.makeText(this, s, Toast.LENGTH_SHORT).show(); }
}
