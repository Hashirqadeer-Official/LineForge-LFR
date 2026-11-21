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
import android.text.TextUtils;
import android.text.TextWatcher;
import android.view.Gravity;
import android.view.View;
import android.view.ViewGroup;
import android.view.inputmethod.EditorInfo;
import android.view.inputmethod.InputMethodManager;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.ProgressBar;
import android.widget.ScrollView;
import android.widget.SeekBar;
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
    // ------------------------------------------------------------------ palette
    private static final int BG = Color.rgb(11, 16, 32);
    private static final int PANEL = Color.rgb(20, 27, 46);
    private static final int PANEL_ALT = Color.rgb(29, 38, 64);
    private static final int BORDER = Color.rgb(38, 49, 76);
    private static final int TEXT = Color.rgb(238, 242, 255);
    private static final int MUTED = Color.rgb(140, 151, 181);
    private static final int MINT = Color.rgb(45, 212, 191);
    private static final int BLUE = Color.rgb(96, 165, 250);
    private static final int PURPLE = Color.rgb(167, 139, 250);
    private static final int RED = Color.rgb(248, 113, 113);
    private static final int ORANGE = Color.rgb(251, 191, 36);
    private static final int GREEN = Color.rgb(52, 211, 153);
    private static final int REQ_CONNECT = 151;
    private static final long SCAN_DURATION_MS = 12000;
    private static final String PREFS_NAME = "lineforge_ui";
    private static final int MAX_EVENTS = 40;
    private static final String[] TAB_NAMES = {"Drive", "Tune", "Calibrate", "Rules", "Log"};
    private static final int[] TAB_ICONS = {TabIcon.DRIVE, TabIcon.TUNE, TabIcon.CALIBRATE, TabIcon.RULES, TabIcon.LOG};
    private static final int TAB_CALIBRATE = 2, TAB_LOG = 4;
    // Option bits shared with the firmware (command 17).
    private static final int RULE_SHARP_TURN = 0x01, RULE_KEEP_OLD_CAL = 0x02, RULE_ROUTE = 0x04,
            RULE_IR_TURN = 0x08;
    // Rule settings shared with the firmware (commands 23/24, frame type 0x54).
    private static final int P_LINE_THRESHOLD = 1, P_SHARP_TIMEOUT = 2, P_SHARP_END = 3,
            P_IR_TRIGGER = 4, P_IR_SPEED = 5, P_IR_TIMEOUT = 6, P_IR_END = 7, P_IR_DEBOUNCE = 8,
            P_INVERT_MODE = 9, P_INVERT_DARK = 10, P_INVERT_CONFIRM = 11, P_GAP_MIN = 12,
            P_BOX_DETECT = 13, P_ARM = 14, P_TURN_TIMEOUT = 15, P_JUNCTION_GAP = 16, PARAM_COUNT = 16;
    /** App unit (%, ms) = firmware value x scale. */
    private static final int[] PARAM_SCALE = {1, 10, 100, 1, 1, 1, 100, 1, 1, 1, 1, 10, 10, 10, 10, 100, 10};
    private static final int[] PARAM_DEFAULT = {0, 50, 15, 0, 0, 35, 15, 1, 0, 0, 6, 10, 5, 25, 15, 30, 30};
    // Route actions shared with the firmware (command 21: action << 4 | seconds).
    private static final int ACT_LEFT = 1, ACT_RIGHT = 2, ACT_STRAIGHT = 3, ACT_UTURN = 4,
            ACT_WAIT_BLUE = 5, ACT_WAIT_GREEN = 6, ACT_FINISH = 7;
    private static final String[] ACTION_NAMES = {"", "Turn left", "Turn right", "Go straight",
            "U-turn", "Wait + blue LED (water)", "Wait + green LED (rescue)", "Finish (stop)"};
    private static final int MAX_ROUTE_STEPS = 32;

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
    private Toggle liveSwitch;
    private PidControl kp, ki, kd;
    private SpeedControl speed;
    private int sentBytes = 0, receivedBytes = 0;
    private String lastEvent = "Ready to pair";

    // Navigation and status widgets
    private final View[] pages = new View[TAB_NAMES.length];
    private final TabIcon[] tabIcons = new TabIcon[TAB_NAMES.length];
    private final TextView[] tabLabels = new TextView[TAB_NAMES.length];
    private LinearLayout statusChip;
    private TextView lastEventLabel, connectHint;
    private TextView calTileValue, pidTileValue, speedTileValue;
    private TextView calBadge;

    // Calibration panel
    private Segmented calModeSeg, calPatternSeg;
    private Stepper calSeconds, calSpeed;
    private LinearLayout autoOptions;
    private TextView calSteps, calProgressText;
    private ProgressBar calProgress;
    private Button calStartButton;
    private boolean calibrating = false;
    // Rules + saved-calibration options
    private Toggle sharpTurnSwitch, keepOldCalSwitch, irTurnSwitch;
    private final Stepper[] paramSteppers = new Stepper[PARAM_COUNT + 1];
    private final Segmented[] paramSegs = new Segmented[PARAM_COUNT + 1];
    private LinearLayout invertOptions;
    private final TextView[] counterViews = new TextView[6];
    private TextView lineModeLabel, irLiveLabel;
    private Stepper pivotSpeedStep;
    private boolean bindingRules = false;
    private boolean restorePending = false;
    // Competition route
    private Toggle routeSwitch;
    private Segmented pathSeg;
    private Stepper waitReservoirStep, waitBuildingStep, turnDelayStep;
    private LinearLayout routeListView;
    private TextView routeSummary;
    private final ArrayList<int[]> routeSteps = new ArrayList<>();   // {action, seconds}
    private final ArrayList<String> routeNotes = new ArrayList<>();
    private int robotRouteLen = -1;
    private boolean resetWarningShown = false;
    private long calFailedAtMs = 0;
    private boolean calManual = true;
    private long calStartedMs = 0, calExpectedMs = 0;

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
        getWindow().setNavigationBarColor(PANEL);
        preferences = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
        ble = new BleUartClient(this, bleCallback);
        createInterface();
        ui.postDelayed(heartbeatTask, 600);
        addEvent("Switch on the robot, then tap Connect. No pairing needed.");
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
        ui.removeCallbacks(calTicker);
        ble.stopScan();
        disconnectLink();
        super.onDestroy();
    }

    // ------------------------------------------------------------ view helpers

    private interface IntListener { void on(int value); }

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

    /** Same color at reduced opacity, for soft tinted backgrounds. */
    private static int tint(int color, int alpha) {
        return Color.argb(alpha, Color.red(color), Color.green(color), Color.blue(color));
    }

    private TextView txt(String value, int sp, int color, boolean heavy) {
        TextView t = new TextView(this);
        t.setText(value);
        t.setTextSize(sp);
        t.setTextColor(color);
        t.setGravity(Gravity.CENTER_VERTICAL);
        if (heavy) t.setTypeface(Typeface.create("sans-serif-medium", Typeface.BOLD));
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

    private View hgap(int width) {
        View v = new View(this);
        v.setLayoutParams(new LinearLayout.LayoutParams(dp(width), 1));
        return v;
    }

    private LinearLayout card() {
        LinearLayout c = col();
        c.setPadding(dp(18), dp(16), dp(18), dp(18));
        c.setBackground(rounded(PANEL, 20, BORDER));
        return c;
    }

    /** Card title with an optional muted subtitle on the right. */
    private LinearLayout cardHeader(String title, String note) {
        LinearLayout h = row();
        h.addView(txt(title, 15, TEXT, true), weighted());
        if (note != null) h.addView(txt(note, 11, MUTED, false));
        return h;
    }

    private Button button(String label, int fill, int foreground) {
        Button b = new Button(this);
        b.setText(label);
        b.setAllCaps(false);
        b.setTextSize(15);
        b.setTypeface(Typeface.create("sans-serif-medium", Typeface.BOLD));
        b.setTextColor(foreground);
        b.setMinHeight(0);
        b.setMinimumHeight(0);
        b.setStateListAnimator(null);
        b.setPadding(dp(14), dp(12), dp(14), dp(12));
        b.setBackground(rounded(fill, 14, fill == PANEL_ALT ? BORDER : 0));
        return b;
    }

    private TextView help(String s) {
        TextView t = txt(s, 12, MUTED, false);
        t.setLineSpacing(dp(2), 1f);
        return t;
    }

    private ScrollView page(LinearLayout content) {
        ScrollView scroll = new ScrollView(this);
        scroll.setFillViewport(true);
        scroll.setVerticalScrollBarEnabled(false);
        content.setPadding(dp(16), dp(6), dp(16), dp(24));
        scroll.addView(content);
        return scroll;
    }

    // ------------------------------------------------------------ main layout

    private void createInterface() {
        LinearLayout screen = col();
        screen.setBackgroundColor(BG);
        setContentView(screen);

        screen.addView(buildTopBar(), full());

        FrameLayout pager = new FrameLayout(this);
        screen.addView(pager, new LinearLayout.LayoutParams(-1, 0, 1f));
        pages[0] = page(buildDrivePage());
        pages[1] = page(buildTunePage());
        pages[2] = page(buildCalibratePage());
        pages[3] = page(buildRulesPage());
        pages[4] = page(buildLogPage());
        for (View p : pages) pager.addView(p, new FrameLayout.LayoutParams(-1, -1));

        screen.addView(buildRunBar(), full());
        screen.addView(buildTabBar(), full());

        kp.restore(); ki.restore(); kd.restore();
        updateConnectionUI(false, null);
        showTab(Math.min(TAB_NAMES.length - 1, Math.max(0, preferences.getInt("tab", 0))));
    }

    private View buildTopBar() {
        LinearLayout bar = row();
        bar.setPadding(dp(16), dp(14), dp(16), dp(10));
        TextView mark = txt("LF", 17, BG, true);
        mark.setGravity(Gravity.CENTER);
        mark.setBackground(rounded(MINT, 12, 0));
        bar.addView(mark, new LinearLayout.LayoutParams(dp(40), dp(40)));
        LinearLayout title = col();
        title.setPadding(dp(11), 0, dp(8), 0);
        title.addView(txt("LineForge", 19, TEXT, true));
        title.addView(txt("Line follower control", 11, MUTED, false));
        bar.addView(title, weighted());

        statusChip = row();
        statusChip.setPadding(dp(11), dp(7), dp(13), dp(7));
        connectionDot = new View(this);
        statusChip.addView(connectionDot, new LinearLayout.LayoutParams(dp(8), dp(8)));
        connectionLabel = txt("Offline", 12, RED, true);
        connectionLabel.setPadding(dp(7), 0, 0, 0);
        statusChip.addView(connectionLabel);
        statusChip.setOnClickListener(v -> showTab(0));
        bar.addView(statusChip);
        return bar;
    }

    private View buildTabBar() {
        LinearLayout wrap = col();
        View line = new View(this);
        line.setBackgroundColor(BORDER);
        wrap.addView(line, new LinearLayout.LayoutParams(-1, dp(1)));
        LinearLayout bar = row();
        bar.setBackgroundColor(PANEL);
        bar.setPadding(0, dp(6), 0, dp(8));
        for (int i = 0; i < TAB_NAMES.length; i++) {
            final int index = i;
            LinearLayout item = col();
            item.setGravity(Gravity.CENTER);
            item.setPadding(0, dp(4), 0, dp(2));
            tabIcons[i] = new TabIcon(this, TAB_ICONS[i], MUTED);
            item.addView(tabIcons[i], new LinearLayout.LayoutParams(dp(24), dp(24)));
            tabLabels[i] = txt(TAB_NAMES[i], 11, MUTED, true);
            tabLabels[i].setGravity(Gravity.CENTER);
            tabLabels[i].setPadding(0, dp(3), 0, 0);
            item.addView(tabLabels[i]);
            item.setOnClickListener(v -> showTab(index));
            bar.addView(item, weighted());
        }
        wrap.addView(bar, full());
        return wrap;
    }

    private void showTab(int index) {
        blurInputs();
        for (int i = 0; i < TAB_NAMES.length; i++) {
            boolean on = i == index;
            pages[i].setVisibility(on ? View.VISIBLE : View.GONE);
            tabIcons[i].setColor(on ? MINT : MUTED);
            tabLabels[i].setTextColor(on ? MINT : MUTED);
        }
        preferences.edit().putInt("tab", index).apply();
    }

    private View buildRunBar() {
        LinearLayout area = col();
        area.setPadding(dp(16), dp(10), dp(16), dp(12));
        area.setBackgroundColor(BG);
        heroStatus = txt("Motors off", 12, MUTED, true);
        heroStatus.setGravity(Gravity.CENTER);
        area.addView(heroStatus, full());
        area.addView(gap(8));
        LinearLayout actions = row();
        startButton = button("▶  Start", MINT, BG);
        stopButton = button("■  Stop", RED, Color.WHITE);
        startButton.setTextSize(17);
        stopButton.setTextSize(17);
        actions.addView(startButton, new LinearLayout.LayoutParams(0, dp(56), 1f));
        actions.addView(hgap(10));
        actions.addView(stopButton, new LinearLayout.LayoutParams(0, dp(56), 1f));
        startButton.setOnClickListener(v -> startRobot());
        stopButton.setOnClickListener(v -> stopRobot());
        area.addView(actions, full());
        return area;
    }

    // ------------------------------------------------------------- Drive page

    private LinearLayout buildDrivePage() {
        LinearLayout content = col();

        LinearLayout conn = card();
        LinearLayout head = row();
        LinearLayout names = col();
        selectedDevice = txt("No robot connected", 17, TEXT, true);
        names.addView(selectedDevice);
        connectHint = help("Switch on the robot, then tap Connect.");
        names.addView(connectHint);
        head.addView(names, weighted());
        conn.addView(head);
        conn.addView(gap(14));
        connectButton = button("Connect to robot", MINT, BG);
        connectButton.setOnClickListener(v -> {
            if (connected || ble.isActive()) {
                disconnectLink();
                addEvent("Disconnected by user.");
            } else ensureBluetoothPermission();
        });
        conn.addView(connectButton, new LinearLayout.LayoutParams(-1, dp(50)));
        conn.addView(gap(12));
        lastEventLabel = txt("", 12, MUTED, false);
        lastEventLabel.setMaxLines(2);
        lastEventLabel.setEllipsize(TextUtils.TruncateAt.END);
        lastEventLabel.setOnClickListener(v -> showTab(TAB_LOG));
        conn.addView(lastEventLabel, full());
        content.addView(conn, full());
        content.addView(gap(12));

        // Quick status tiles
        LinearLayout tiles = row();
        calTileValue = addTile(tiles, "Calibration", v -> showTab(TAB_CALIBRATE));
        tiles.addView(hgap(8));
        pidTileValue = addTile(tiles, "PID", v -> showTab(1));
        tiles.addView(hgap(8));
        speedTileValue = addTile(tiles, "Speed", null);
        content.addView(tiles, full());
        content.addView(gap(12));

        LinearLayout sensorCard = card();
        sensorCard.addView(cardHeader("Live line sensors", "QTR-8A  •  S1–S8"));
        sensorCard.addView(gap(8));
        sensorView = new QtrSensorView(this);
        sensorCard.addView(sensorView, full());
        sensorInfo = help("Waiting for sensor data…");
        sensorCard.addView(sensorInfo);
        content.addView(sensorCard, full());
        content.addView(gap(12));

        content.addView(buildCountersCard(), full());
        content.addView(gap(12));

        speed = new SpeedControl();
        content.addView(speed.view, full());
        return content;
    }

    private TextView addTile(LinearLayout parent, String label, View.OnClickListener click) {
        LinearLayout tile = col();
        tile.setPadding(dp(12), dp(11), dp(12), dp(12));
        tile.setBackground(rounded(PANEL, 16, BORDER));
        tile.addView(txt(label, 11, MUTED, true));
        TextView value = txt("—", 15, TEXT, true);
        value.setSingleLine(true);
        value.setEllipsize(TextUtils.TruncateAt.END);
        value.setPadding(0, dp(4), 0, 0);
        tile.addView(value);
        if (click != null) tile.setOnClickListener(click);
        parent.addView(tile, weighted());
        return value;
    }

    // -------------------------------------------------------------- Tune page

    private LinearLayout buildTunePage() {
        LinearLayout content = col();
        LinearLayout intro = card();
        intro.addView(cardHeader("PID tuning", "gain = value ÷ divider"));
        intro.addView(gap(6));
        intro.addView(help("Adjust each gain with − / +, the slider, or by typing a number. "
                + "Tap Send to robot when ready, or turn on Live tuning to send every change."));
        intro.addView(gap(12));
        LinearLayout live = row();
        LinearLayout liveText = col();
        liveText.addView(txt("Live tuning", 14, TEXT, true));
        liveText.addView(txt("Send changes to the robot instantly", 11, MUTED, false));
        live.addView(liveText, weighted());
        liveTune = preferences.getBoolean("live", false);
        liveSwitch = new Toggle(liveTune, on -> {
            liveTune = on;
            preferences.edit().putBoolean("live", on).apply();
        });
        live.addView(liveSwitch.view);
        intro.addView(live, full());
        content.addView(intro, full());
        content.addView(gap(12));

        kp = new PidControl("KP", "Proportional", "Steers back toward the line. Higher = sharper corrections.", MINT, 1, 40, 0);
        ki = new PidControl("KI", "Integral", "Removes slow drift to one side. Usually small or 0.", BLUE, 3, 0, 0);
        kd = new PidControl("KD", "Derivative", "Damps wobbling. Raise it if the robot oscillates.", PURPLE, 5, 1, 0);
        content.addView(kp.view, full()); content.addView(gap(12));
        content.addView(ki.view, full()); content.addView(gap(12));
        content.addView(kd.view, full()); content.addView(gap(14));

        LinearLayout buttons = row();
        Button apply = button("Send to robot", MINT, BG);
        apply.setOnClickListener(v -> uploadAll());
        buttons.addView(apply, new LinearLayout.LayoutParams(0, dp(50), 1f));
        buttons.addView(hgap(10));
        Button read = button("Read from robot", PANEL_ALT, TEXT);
        read.setOnClickListener(v -> requestRobotState());
        buttons.addView(read, new LinearLayout.LayoutParams(0, dp(50), 1f));
        content.addView(buttons, full());
        return content;
    }

    // --------------------------------------------------------- Calibrate page

    private LinearLayout buildCalibratePage() {
        LinearLayout content = col();

        LinearLayout status = card();
        LinearLayout head = row();
        head.addView(txt("Sensor calibration", 15, TEXT, true), weighted());
        calBadge = txt("Unknown", 11, MUTED, true);
        calBadge.setPadding(dp(10), dp(4), dp(10), dp(4));
        head.addView(calBadge);
        status.addView(head);
        status.addView(gap(6));
        calibrationInfo = help("Saved white/black values: waiting for robot.");
        status.addView(calibrationInfo);
        status.addView(gap(12));
        Button view = button("View saved values", PANEL_ALT, TEXT);
        view.setOnClickListener(v -> requestCalibrationData(true));
        status.addView(view, new LinearLayout.LayoutParams(-1, dp(46)));
        status.addView(gap(10));
        Button restore = button("Use saved calibration", PANEL_ALT, TEXT);
        restore.setOnClickListener(v -> restoreSavedCalibration());
        status.addView(restore, new LinearLayout.LayoutParams(-1, dp(46)));
        status.addView(gap(14));
        keepOldCalSwitch = optionSwitch("Keep previous calibration if a new one fails",
                "The robot keeps driving on the calibration saved in its memory",
                (preferences.getInt("rule_flags", RULE_KEEP_OLD_CAL) & RULE_KEEP_OLD_CAL) != 0, status);
        content.addView(status, full());
        content.addView(gap(12));

        LinearLayout method = card();
        method.addView(cardHeader("Method", null));
        method.addView(gap(10));
        calModeSeg = new Segmented(new String[]{"Manual", "Automatic"},
                preferences.getInt("cal_mode", 0), i -> {
                    preferences.edit().putInt("cal_mode", i).apply();
                    updateCalibrationPanel();
                });
        method.addView(calModeSeg.view, full());

        autoOptions = col();
        autoOptions.addView(gap(16));
        autoOptions.addView(txt("Movement", 13, TEXT, true));
        autoOptions.addView(gap(8));
        calPatternSeg = new Segmented(new String[]{"Spin in circles", "Wiggle left–right"},
                preferences.getInt("cal_pattern", 0), i -> {
                    preferences.edit().putInt("cal_pattern", i).apply();
                    updateCalibrationPanel();
                });
        autoOptions.addView(calPatternSeg.view, full());
        autoOptions.addView(gap(16));
        calSeconds = new Stepper("Duration", "How long the robot moves", 1, 10,
                preferences.getInt("cal_seconds", 3), "s", BLUE, false);
        calSeconds.onCommit = v -> { preferences.edit().putInt("cal_seconds", v).apply(); updateCalibrationPanel(); };
        autoOptions.addView(calSeconds.view, full());
        autoOptions.addView(gap(14));
        calSpeed = new Stepper("Turn speed", "Motor power while calibrating", 10, 100,
                preferences.getInt("cal_speed", 30), "%", PURPLE, false);
        calSpeed.onCommit = v -> preferences.edit().putInt("cal_speed", v).apply();
        autoOptions.addView(calSpeed.view, full());
        method.addView(autoOptions, full());
        content.addView(method, full());
        content.addView(gap(12));

        LinearLayout steps = card();
        steps.addView(cardHeader("How to do it", null));
        steps.addView(gap(8));
        calSteps = txt("", 13, TEXT, false);
        calSteps.setLineSpacing(dp(4), 1f);
        steps.addView(calSteps, full());
        steps.addView(gap(14));
        calProgress = new ProgressBar(this, null, android.R.attr.progressBarStyleHorizontal);
        calProgress.setMax(1000);
        calProgress.setProgressTintList(ColorStateList.valueOf(MINT));
        calProgress.setProgressBackgroundTintList(ColorStateList.valueOf(BORDER));
        calProgress.setVisibility(View.GONE);
        steps.addView(calProgress, new LinearLayout.LayoutParams(-1, dp(10)));
        calProgressText = txt("", 13, MINT, true);
        calProgressText.setVisibility(View.GONE);
        calProgressText.setPadding(0, dp(8), 0, dp(4));
        steps.addView(calProgressText, full());
        steps.addView(gap(6));
        calStartButton = button("Start calibration", MINT, BG);
        calStartButton.setOnClickListener(v -> startCalibration());
        steps.addView(calStartButton, new LinearLayout.LayoutParams(-1, dp(52)));
        content.addView(steps, full());

        updateCalibrationPanel();
        return content;
    }

    private void updateCalibrationPanel() {
        if (calModeSeg == null || calSteps == null) return;
        boolean manual = calModeSeg.selected() == 0;
        autoOptions.setVisibility(manual ? View.GONE : View.VISIBLE);
        int secs = calSeconds.get();
        if (manual) {
            calSteps.setText("1.  Place the sensors over WHITE, away from the line.\n"
                    + "2.  Tap Start and keep the robot still for 2 seconds.\n"
                    + "3.  Then slide it by hand across the black line for 5 seconds, "
                    + "so every sensor crosses the line 3–4 times.\n"
                    + "Motors stay off the whole time.");
        } else if (calPatternSeg.selected() == 0) {
            calSteps.setText("1.  Put the robot ON the black line, wheels on the floor.\n"
                    + "2.  Tap Start. It spins in place for " + secs + " s so every sensor passes over the line.\n"
                    + "3.  Keep hands clear. Tap Stop to cancel at any time.\n"
                    + "Tip: give it enough time for at least one full turn.");
        } else {
            calSteps.setText("1.  Center the robot ON the black line, wheels on the floor.\n"
                    + "2.  Tap Start. It rocks left and right for " + secs + " s, sweeping all sensors across the line.\n"
                    + "3.  Keep hands clear. Tap Stop to cancel at any time.\n"
                    + "Tip: if outer sensors fail, raise the turn speed.");
        }
    }

    private void startCalibration() {
        if (!connected) { showToast("Connect to the robot first"); return; }
        if (calibrating) return;
        if (running || startPending) { showToast("Stop the robot first"); return; }
        blurInputs();
        if (calModeSeg.selected() == 0) {
            beginCalibration(new byte[]{8, 1}, 7000, true, "manual");
            return;
        }
        final int secs = calSeconds.get(), power = calSpeed.get();
        final boolean spin = calPatternSeg.selected() == 0;
        new AlertDialog.Builder(this)
                .setTitle(spin ? "The robot will spin" : "The robot will wiggle")
                .setMessage("The motors will run for " + secs + " s at " + power + "% power. "
                        + "Place the robot on the line with its wheels on the floor and keep hands clear.\n\n"
                        + "Tap Stop to cancel at any time.")
                .setNegativeButton("Cancel", null)
                .setPositiveButton("Start", (d, w) -> beginCalibration(
                        new byte[]{15, (byte)secs, 16, (byte)power, 8, (byte)(spin ? 2 : 3)},
                        secs * 1000L, false, spin ? "auto spin" : "auto wiggle"))
                .show();
    }

    private void beginCalibration(byte[] command, long expectedMs, boolean manual, String label) {
        running = false;
        startPending = false;
        sendBytes(command, true);
        calibrating = true;
        calManual = manual;
        calStartedMs = SystemClock.uptimeMillis();
        calExpectedMs = expectedMs;
        calProgress.setProgress(0);
        calProgress.setVisibility(View.VISIBLE);
        calProgressText.setVisibility(View.VISIBLE);
        calProgressText.setTextColor(ORANGE);
        calStartButton.setEnabled(false);
        calStartButton.setAlpha(0.5f);
        addEvent("Calibration started (" + label + ").");
        updateRunUI();
        ui.removeCallbacks(calTicker);
        ui.post(calTicker);
    }

    private final Runnable calTicker = new Runnable() {
        @Override public void run() {
            if (!calibrating) return;
            long t = SystemClock.uptimeMillis() - calStartedMs;
            calProgress.setProgress((int)Math.min(1000, t * 1000 / Math.max(1, calExpectedMs)));
            long left = Math.max(0, (calExpectedMs - t + 999) / 1000);
            String text;
            if (t >= calExpectedMs) text = "Finishing… waiting for the robot";
            else if (calManual && t < 2000) text = "Keep still on WHITE…  " + left + " s";
            else if (calManual) text = "Now slide across the line…  " + left + " s";
            else text = "Robot is moving…  " + left + " s";
            calProgressText.setText(text);
            if (t > calExpectedMs + 6000) {
                finishCalibration(false, "No result from the robot. Check that the new firmware is uploaded.");
                return;
            }
            ui.postDelayed(this, 100);
        }
    };

    private void finishCalibration(boolean ok, String message) {
        if (!calibrating) return;
        calibrating = false;
        ui.removeCallbacks(calTicker);
        calProgress.setProgress(ok ? 1000 : 0);
        calProgressText.setVisibility(View.VISIBLE);
        calProgressText.setTextColor(ok ? GREEN : RED);
        calProgressText.setText(message);
        calStartButton.setEnabled(true);
        calStartButton.setAlpha(1f);
        updateRunUI();
    }

    // ------------------------------------------------------------- Rules page

    private interface BoolListener { void on(boolean value); }

    /** Clear two-state control: [ OFF | ON ], the active side is filled (red / green). */
    private final class Toggle {
        final LinearLayout view = row();
        private final TextView off, on;
        private final BoolListener listener;
        private boolean checked;

        Toggle(boolean initial, BoolListener listener) {
            this.listener = listener;
            view.setBackground(rounded(BG, 12, BORDER));
            view.setPadding(dp(3), dp(3), dp(3), dp(3));
            off = txt("OFF", 12, MUTED, true);
            on = txt("ON", 12, MUTED, true);
            off.setGravity(Gravity.CENTER);
            on.setGravity(Gravity.CENTER);
            off.setOnClickListener(v -> userSet(false));
            on.setOnClickListener(v -> userSet(true));
            view.addView(off, new LinearLayout.LayoutParams(dp(50), dp(34)));
            view.addView(on, new LinearLayout.LayoutParams(dp(50), dp(34)));
            setChecked(initial);
        }

        private void userSet(boolean value) {
            if (value == checked) return;
            setChecked(value);
            if (listener != null) listener.on(value);
        }

        /** Programmatic update: the listener is not called. */
        void setChecked(boolean value) {
            checked = value;
            on.setBackground(value ? rounded(GREEN, 10, 0) : null);
            on.setTextColor(value ? BG : MUTED);
            off.setBackground(value ? null : rounded(tint(RED, 70), 10, RED));
            off.setTextColor(value ? MUTED : TEXT);
        }

        boolean isChecked() { return checked; }
    }

    /** Title/subtitle row with an ON/OFF toggle; changes are saved and sent to the robot. */
    private Toggle optionSwitch(String title, String subtitle, boolean checked, LinearLayout parent) {
        LinearLayout r = row();
        LinearLayout names = col();
        names.addView(txt(title, 15, TEXT, true));
        names.addView(txt(subtitle, 11, MUTED, false));
        r.addView(names, weighted());
        Toggle t = new Toggle(checked, value -> {
            if (bindingRules) return;
            saveRuleOptions();
            sendRuleOptions(true);
        });
        r.addView(t.view);
        parent.addView(r, full());
        return t;
    }

    /** Tappable "▸ title" header that shows/hides the returned section. */
    private LinearLayout collapsible(LinearLayout parent, String title) {
        TextView head = txt("▸  " + title, 13, MINT, true);
        head.setPadding(0, dp(8), 0, dp(8));
        LinearLayout body = col();
        body.setVisibility(View.GONE);
        head.setOnClickListener(v -> {
            boolean show = body.getVisibility() != View.VISIBLE;
            body.setVisibility(show ? View.VISIBLE : View.GONE);
            head.setText((show ? "▾  " : "▸  ") + title);
        });
        parent.addView(head, full());
        parent.addView(body, full());
        return body;
    }

    private TextView label(String s) {
        TextView t = txt(s, 13, TEXT, true);
        t.setPadding(0, dp(14), 0, dp(6));
        return t;
    }

    // ------------------------------------------------------- rule settings

    private int paramValue(int id) { return preferences.getInt("p" + id, PARAM_DEFAULT[id]); }

    /** Number setting; minFw/maxFw are firmware units, shown as firmware x scale. */
    private Stepper paramStepper(int id, String title, String hint, int minFw, int maxFw, String unit, int color) {
        final int scale = PARAM_SCALE[id];
        Stepper st = new Stepper(title, hint, minFw * scale, maxFw * scale, paramValue(id) * scale, unit, color, false);
        st.stepSize = scale;
        st.onCommit = v -> setParam(id, Math.round(v / (float)scale));
        paramSteppers[id] = st;
        return st;
    }

    private Segmented paramSeg(int id, String[] labels) {
        Segmented sg = new Segmented(labels, paramValue(id), i -> setParam(id, i));
        paramSegs[id] = sg;
        return sg;
    }

    private void setParam(int id, int fwValue) {
        preferences.edit().putInt("p" + id, fwValue).apply();
        if (paramSteppers[id] != null) paramSteppers[id].set(fwValue * PARAM_SCALE[id]);
        if (connected) sendBytes(new byte[]{23, (byte)id, 24, (byte)fwValue}, false);
        if (id == P_INVERT_MODE) updateInvertUi();
    }

    private void sendAllParams() {
        if (!connected) return;
        byte[] b = new byte[PARAM_COUNT * 4];
        for (int id = 1; id <= PARAM_COUNT; id++) {
            int k = (id - 1) * 4;
            b[k] = 23; b[k + 1] = (byte)id; b[k + 2] = 24; b[k + 3] = (byte)paramValue(id);
        }
        sendBytes(b, false);
    }

    /** The robot's saved settings are the source of truth once it reports them. */
    private void applyParamsFromRobot(byte[] payload) {
        SharedPreferences.Editor e = preferences.edit();
        for (int i = 0; i < payload.length && i < PARAM_COUNT; i++) {
            int id = i + 1, v = payload[i] & 255;
            e.putInt("p" + id, v);
            if (paramSteppers[id] != null) paramSteppers[id].set(v * PARAM_SCALE[id]);
            if (paramSegs[id] != null) paramSegs[id].select(v, false);
        }
        e.apply();
        updateInvertUi();
    }

    private void updateInvertUi() {
        if (invertOptions != null) invertOptions.setVisibility(paramValue(P_INVERT_MODE) == 1 ? View.VISIBLE : View.GONE);
    }

    // --------------------------------------------------------------- counters

    private LinearLayout buildCountersCard() {
        LinearLayout c = card();
        LinearLayout head = row();
        head.addView(txt("Counters", 15, TEXT, true), weighted());
        TextView reset = txt("Reset", 13, MINT, true);
        reset.setPadding(dp(10), dp(4), 0, dp(4));
        reset.setOnClickListener(v -> {
            if (!connected) { showToast("Connect to the robot first"); return; }
            sendBytes(new byte[]{27, 0}, true);
            addEvent("Counters reset.");
        });
        head.addView(reset);
        c.addView(head, full());
        c.addView(help("Counted since START. Future rules can act on these numbers."));
        c.addView(gap(10));
        String[] names = {"IR left", "IR right", "Junctions", "Left turns", "Right turns", "Gaps"};
        for (int r = 0; r < 2; r++) {
            LinearLayout line = row();
            for (int k = 0; k < 3; k++) {
                if (k > 0) line.addView(hgap(8));
                counterViews[r * 3 + k] = addTile(line, names[r * 3 + k], null);
                counterViews[r * 3 + k].setText("0");
            }
            c.addView(line, full());
            if (r == 0) c.addView(gap(8));
        }
        c.addView(gap(10));
        lineModeLabel = help("Line: waiting for robot…");
        c.addView(lineModeLabel, full());
        LinearLayout adv = collapsible(c, "Counting settings");
        adv.addView(paramStepper(P_GAP_MIN, "Gap", "Line lost at least this long counts as a gap", 1, 100, "ms", BLUE).view, full());
        adv.addView(gap(12));
        adv.addView(paramStepper(P_JUNCTION_GAP, "Junction spacing", "Ignore a second junction sooner than this", 5, 100, "ms", BLUE).view, full());
        return c;
    }

    private LinearLayout buildRulesPage() {
        LinearLayout content = col();
        LinearLayout intro = card();
        intro.addView(cardHeader("Driving rules", "saved on the robot"));
        intro.addView(gap(6));
        intro.addView(help("Tap ON or OFF to switch a rule; its settings are right below it. "
                + "Changes are sent to the robot immediately and kept after power-off."));
        content.addView(intro, full());
        content.addView(gap(12));
        int flags = preferences.getInt("rule_flags", RULE_KEEP_OLD_CAL);

        // Line sensors: threshold + inverted line (white line on black)
        LinearLayout line = card();
        line.addView(cardHeader("Line sensors", "used by every rule"));
        line.addView(gap(12));
        line.addView(paramStepper(P_LINE_THRESHOLD, "Black threshold",
                "How dark a sensor must read to count as the line", 10, 100, "%", MINT).view, full());
        line.addView(label("Line color"));
        line.addView(paramSeg(P_INVERT_MODE, new String[]{"Black line", "Auto", "White line"}).view, full());
        line.addView(gap(6));
        line.addView(help("Auto switches by itself when part of the track is a white line on a black floor "
                + "(most sensors dark, the line light), and back again afterwards."));
        invertOptions = col();
        invertOptions.addView(gap(12));
        invertOptions.addView(paramStepper(P_INVERT_DARK, "Dark sensors to switch",
                "How many of the 8 sensors must be dark", 5, 8, "", PURPLE).view, full());
        invertOptions.addView(gap(12));
        invertOptions.addView(paramStepper(P_INVERT_CONFIRM, "Confirm time",
                "The pattern must last this long (a junction is shorter)", 2, 50, "ms", PURPLE).view, full());
        line.addView(invertOptions, full());
        updateInvertUi();
        content.addView(line, full());
        content.addView(gap(12));

        // Sharp turn recovery (QTR outer sensors)
        LinearLayout sharp = card();
        sharpTurnSwitch = optionSwitch("Sharp turn recovery", "QTR outer sensors • 90° turns at speed",
                (flags & RULE_SHARP_TURN) != 0, sharp);
        sharp.addView(gap(8));
        sharp.addView(help("S1 (leftmost) sees the line, then the robot loses it → spin left in place "
                + "(right motor forward, left backward) until the line is found, then PID continues. S8 mirrors it."));
        sharp.addView(gap(12));
        pivotSpeedStep = new Stepper("Spin speed", "Motor power while turning in place", 10, 100,
                preferences.getInt("pivot_speed", 35), "%", MINT, true);
        pivotSpeedStep.onCommit = v -> { saveRuleOptions(); sendRuleOptions(true); };
        sharp.addView(pivotSpeedStep.view, full());
        LinearLayout sharpAdv = collapsible(sharp, "More settings");
        sharpAdv.addView(label("Stop spinning when"));
        sharpAdv.addView(paramSeg(P_SHARP_END, new String[]{"Outer sensor sees line", "Center sees line"}).view, full());
        sharpAdv.addView(gap(12));
        sharpAdv.addView(paramStepper(P_SHARP_TIMEOUT, "Give up after",
                "Back to normal PID if the line isn't found", 1, 50, "ms", ORANGE).view, full());
        content.addView(sharp, full());
        content.addView(gap(12));

        // IR side sensors
        LinearLayout ir = card();
        irTurnSwitch = optionSwitch("IR turn rule", "Left / right IR sensors (GPIO 14 / 4)",
                (flags & RULE_IR_TURN) != 0, ir);
        ir.addView(gap(8));
        irLiveLabel = txt("IR-L  ○      IR-R  ○", 13, MUTED, true);
        ir.addView(irLiveLabel, full());
        ir.addView(gap(6));
        ir.addView(help("IR-L sees the line → spin left (right motor forward, left backward) until the line is "
                + "found, then PID continues. IR-R mirrors it. Detections are always counted on the Drive tab."));
        ir.addView(label("Start turning"));
        ir.addView(paramSeg(P_IR_TRIGGER, new String[]{"When line is lost", "Immediately"}).view, full());
        ir.addView(gap(12));
        ir.addView(paramStepper(P_IR_SPEED, "Spin speed", "Motor power while turning in place",
                10, 100, "%", MINT).view, full());
        LinearLayout irAdv = collapsible(ir, "More settings");
        irAdv.addView(label("Stop spinning when"));
        irAdv.addView(paramSeg(P_IR_END, new String[]{"Outer sensor sees line", "Center sees line"}).view, full());
        irAdv.addView(gap(12));
        irAdv.addView(paramStepper(P_IR_TIMEOUT, "Give up after",
                "Back to normal PID if the line isn't found", 1, 50, "ms", ORANGE).view, full());
        irAdv.addView(gap(12));
        irAdv.addView(paramStepper(P_IR_DEBOUNCE, "Repeat filter",
                "0 = record every detection, even very short ones. Raise only if one line counts twice.",
                0, 255, "ms", PURPLE).view, full());
        content.addView(ir, full());
        content.addView(gap(12));

        content.addView(buildRouteCard(flags), full());
        return content;
    }

    // ------------------------------------------------------ Competition route

    private LinearLayout buildRouteCard(int flags) {
        LinearLayout c = card();
        routeSwitch = optionSwitch("Competition route", "Fire Rescue: follow the programmed steps",
                (flags & RULE_ROUTE) != 0, c);
        c.addView(gap(10));
        c.addView(help("At every junction or box the robot carries out the next step below. "
                + "Plain 90° corners (only one way to go) are turned automatically and don't use a step. "
                + "Start the robot inside the Start box. While route mode is on, sharp turn recovery is not used."));
        c.addView(gap(16));

        c.addView(txt("Quick setup from the map", 14, TEXT, true));
        c.addView(gap(4));
        c.addView(help("After the reservoir, pick the turn that is NOT blocked:"));
        c.addView(gap(8));
        pathSeg = new Segmented(new String[]{"1st right turn", "2nd right turn"},
                preferences.getInt("route_path", 0), i -> preferences.edit().putInt("route_path", i).apply());
        c.addView(pathSeg.view, full());
        c.addView(gap(14));
        waitReservoirStep = new Stepper("Wait at water reservoir", "Blue LED turns on", 1, 15,
                preferences.getInt("wait_reservoir", 3), "s", BLUE, false);
        waitReservoirStep.onCommit = v -> preferences.edit().putInt("wait_reservoir", v).apply();
        c.addView(waitReservoirStep.view, full());
        c.addView(gap(12));
        waitBuildingStep = new Stepper("Wait at building on fire", "Green LED turns on", 1, 15,
                preferences.getInt("wait_building", 5), "s", GREEN, false);
        waitBuildingStep.onCommit = v -> preferences.edit().putInt("wait_building", v).apply();
        c.addView(waitBuildingStep.view, full());
        c.addView(gap(12));
        Button create = button("Create route from map", PANEL_ALT, TEXT);
        create.setOnClickListener(v -> {
            blurInputs();
            buildMapRoute(pathSeg.selected() == 1, waitReservoirStep.get(), waitBuildingStep.get());
            saveRouteLocal();
            renderRoute();
            showToast("Route created: " + routeSteps.size() + " steps. Check it, then send it.");
        });
        c.addView(create, new LinearLayout.LayoutParams(-1, dp(48)));
        c.addView(gap(18));

        LinearLayout head = row();
        head.addView(txt("Route steps", 14, TEXT, true), weighted());
        routeSummary = txt("", 11, MUTED, false);
        head.addView(routeSummary);
        c.addView(head, full());
        c.addView(gap(4));
        c.addView(help("Tap a step to change it, long-press to delete it."));
        c.addView(gap(8));
        routeListView = col();
        c.addView(routeListView, full());
        c.addView(gap(10));
        LinearLayout edit = row();
        Button add = button("Add step", PANEL_ALT, TEXT);
        add.setOnClickListener(v -> editRouteStep(-1));
        edit.addView(add, new LinearLayout.LayoutParams(0, dp(46), 1f));
        edit.addView(hgap(10));
        Button clear = button("Clear all", PANEL_ALT, TEXT);
        clear.setOnClickListener(v -> new AlertDialog.Builder(this)
                .setTitle("Clear all route steps?")
                .setNegativeButton("Cancel", null)
                .setPositiveButton("Clear", (d, w) -> {
                    routeSteps.clear(); routeNotes.clear(); saveRouteLocal(); renderRoute();
                }).show());
        edit.addView(clear, new LinearLayout.LayoutParams(0, dp(46), 1f));
        c.addView(edit, full());
        c.addView(gap(16));

        LinearLayout routeAdv = collapsible(c, "Advanced route settings");
        routeAdv.addView(paramStepper(P_BOX_DETECT, "Box detection",
                "Whole sensor array dark this long = inside a box", 5, 100, "ms", BLUE).view, full());
        routeAdv.addView(gap(12));
        routeAdv.addView(paramStepper(P_ARM, "Settle time",
                "Straight, centered line needed before the next junction", 5, 100, "ms", BLUE).view, full());
        routeAdv.addView(gap(12));
        routeAdv.addView(paramStepper(P_TURN_TIMEOUT, "Turn gives up after",
                "Stop the run if a turn never finds the line", 5, 100, "ms", ORANGE).view, full());
        c.addView(gap(10));
        turnDelayStep = new Stepper("Turn delay", "Drive forward before turning, so the wheels reach the junction",
                0, 500, preferences.getInt("turn_delay", 100), "ms", PURPLE, false);
        turnDelayStep.onCommit = v -> preferences.edit().putInt("turn_delay", v).apply();
        c.addView(turnDelayStep.view, full());
        c.addView(gap(16));
        Button send = button("Send route to robot", MINT, BG);
        send.setOnClickListener(v -> sendRoute());
        c.addView(send, new LinearLayout.LayoutParams(-1, dp(50)));

        loadRouteLocal();
        renderRoute();
        return c;
    }

    private void addStep(int action, int seconds, String note) {
        routeSteps.add(new int[]{action, seconds});
        routeNotes.add(note);
    }

    /**
     * Fire Rescue map (Figure 1): start in the Start box heading right.
     * Outbound: reservoir (left branch) -> main line -> down the unblocked link ->
     * left along the lower grid -> curve around the fire -> up the right side ->
     * building. Return: the same path back (rulebook), mirrored.
     */
    private void buildMapRoute(boolean secondRight, int waitReservoir, int waitBuilding) {
        routeSteps.clear();
        routeNotes.clear();
        addStep(ACT_LEFT, 0, "Water reservoir branch (on the left)");
        addStep(ACT_WAIT_BLUE, waitReservoir, "Reservoir box: wait, blue LED on");
        addStep(ACT_UTURN, 0, "Turn around, back down to the main line");
        addStep(ACT_LEFT, 0, "Main line: turn toward the building");
        if (!secondRight) {
            addStep(ACT_RIGHT, 0, "1st right turn: down the 1st link");
            addStep(ACT_RIGHT, 0, "Lower grid: turn right toward the curve");
        } else {
            addStep(ACT_STRAIGHT, 0, "Pass the 1st link (blocked)");
            addStep(ACT_RIGHT, 0, "2nd right turn: down the 2nd link");
            addStep(ACT_RIGHT, 0, "Lower grid: turn right toward the curve");
            addStep(ACT_STRAIGHT, 0, "Cross the 1st link");
        }
        addStep(ACT_STRAIGHT, 0, "Last grid junction: straight onto the curve");
        addStep(ACT_LEFT, 0, "Right side: branch to the building (on the left)");
        addStep(ACT_WAIT_GREEN, waitBuilding, "Building on fire: wait, green LED on");
        addStep(ACT_UTURN, 0, "Turn around, same path back");
        addStep(ACT_RIGHT, 0, "Right side: turn down toward the curve");
        addStep(ACT_STRAIGHT, 0, "First grid junction after the curve: straight");
        if (!secondRight) {
            addStep(ACT_LEFT, 0, "1st link: turn up");
            addStep(ACT_LEFT, 0, "Main line: turn toward Start");
        } else {
            addStep(ACT_STRAIGHT, 0, "Cross the 1st link");
            addStep(ACT_LEFT, 0, "2nd link: turn up");
            addStep(ACT_LEFT, 0, "Main line: turn toward Start");
            addStep(ACT_STRAIGHT, 0, "Pass the 1st link");
        }
        addStep(ACT_STRAIGHT, 0, "Pass the reservoir branch");
        addStep(ACT_FINISH, 0, "Start/End box: finish");
    }

    private static String actionLabel(int action, int seconds) {
        String name = action >= 1 && action <= 7 ? ACTION_NAMES[action] : "?";
        if (action == ACT_WAIT_BLUE || action == ACT_WAIT_GREEN)
            name = "Wait " + seconds + " s + " + (action == ACT_WAIT_BLUE ? "blue LED" : "green LED");
        return name;
    }

    private void renderRoute() {
        if (routeListView == null) return;
        routeListView.removeAllViews();
        if (routeSteps.isEmpty()) {
            routeListView.addView(help("No steps yet. Tap Create route from map, or Add step."));
        }
        for (int i = 0; i < routeSteps.size(); i++) {
            final int index = i;
            int[] st = routeSteps.get(i);
            LinearLayout item = row();
            item.setPadding(dp(10), dp(8), dp(10), dp(8));
            item.setBackground(rounded(BG, 12, BORDER));
            TextView num = txt(String.valueOf(i + 1), 12, BG, true);
            num.setGravity(Gravity.CENTER);
            int color = st[0] == ACT_WAIT_BLUE ? BLUE : st[0] == ACT_WAIT_GREEN ? GREEN
                    : st[0] == ACT_FINISH ? RED : st[0] == ACT_UTURN ? ORANGE : MINT;
            num.setBackground(rounded(color, 100, 0));
            item.addView(num, new LinearLayout.LayoutParams(dp(26), dp(26)));
            LinearLayout texts = col();
            texts.setPadding(dp(10), 0, 0, 0);
            texts.addView(txt(actionLabel(st[0], st[1]), 14, TEXT, true));
            String note = routeNotes.get(i);
            if (!note.isEmpty()) texts.addView(txt(note, 11, MUTED, false));
            item.addView(texts, weighted());
            item.setOnClickListener(v -> editRouteStep(index));
            item.setOnLongClickListener(v -> {
                new AlertDialog.Builder(this).setTitle("Delete step " + (index + 1) + "?")
                        .setNegativeButton("Cancel", null)
                        .setPositiveButton("Delete", (d, w) -> {
                            routeSteps.remove(index); routeNotes.remove(index);
                            saveRouteLocal(); renderRoute();
                        }).show();
                return true;
            });
            LinearLayout.LayoutParams lp = full();
            lp.setMargins(0, 0, 0, dp(6));
            routeListView.addView(item, lp);
        }
        updateRouteSummary();
    }

    private void updateRouteSummary() {
        if (routeSummary == null) return;
        String text = routeSteps.size() + " steps";
        if (robotRouteLen >= 0) {
            text += robotRouteLen == routeSteps.size() ? "  •  on robot" : "  •  robot has " + robotRouteLen;
        }
        routeSummary.setText(text);
        routeSummary.setTextColor(robotRouteLen >= 0 && robotRouteLen != routeSteps.size() ? ORANGE : MUTED);
    }

    /** index -1 adds a new step at the end. */
    private void editRouteStep(int index) {
        if (index < 0 && routeSteps.size() >= MAX_ROUTE_STEPS) {
            showToast("A route can have at most " + MAX_ROUTE_STEPS + " steps");
            return;
        }
        String[] items = new String[7];
        System.arraycopy(ACTION_NAMES, 1, items, 0, 7);
        new AlertDialog.Builder(this)
                .setTitle(index < 0 ? "Add step" : "Step " + (index + 1))
                .setItems(items, (d, which) -> {
                    int action = which + 1;
                    if (action == ACT_WAIT_BLUE || action == ACT_WAIT_GREEN) askWaitSeconds(index, action);
                    else applyRouteStep(index, action, 0);
                })
                .setNegativeButton("Cancel", null)
                .show();
    }

    private void askWaitSeconds(int index, int action) {
        EditText input = new EditText(this);
        input.setInputType(InputType.TYPE_CLASS_NUMBER);
        int current = index >= 0 ? routeSteps.get(index)[1] : (action == ACT_WAIT_BLUE ? 3 : 5);
        input.setText(String.valueOf(Math.max(1, current)));
        input.setSelectAllOnFocus(true);
        input.setGravity(Gravity.CENTER);
        new AlertDialog.Builder(this)
                .setTitle("Wait time (1–15 seconds)")
                .setView(input)
                .setNegativeButton("Cancel", null)
                .setPositiveButton("OK", (d, w) -> {
                    int secs;
                    try { secs = Integer.parseInt(input.getText().toString()); }
                    catch (NumberFormatException e) { secs = 3; }
                    applyRouteStep(index, action, Math.min(15, Math.max(1, secs)));
                }).show();
    }

    private void applyRouteStep(int index, int action, int seconds) {
        if (index < 0) {
            addStep(action, seconds, "");
        } else {
            boolean sameAction = routeSteps.get(index)[0] == action;
            routeSteps.set(index, new int[]{action, seconds});
            if (!sameAction) routeNotes.set(index, "");
        }
        saveRouteLocal();
        renderRoute();
    }

    private void saveRouteLocal() {
        try {
            org.json.JSONArray arr = new org.json.JSONArray();
            for (int i = 0; i < routeSteps.size(); i++) {
                org.json.JSONObject o = new org.json.JSONObject();
                o.put("a", routeSteps.get(i)[0]);
                o.put("s", routeSteps.get(i)[1]);
                o.put("n", routeNotes.get(i));
                arr.put(o);
            }
            preferences.edit().putString("route", arr.toString()).apply();
        } catch (org.json.JSONException ignored) { }
        updateRouteSummary();
    }

    private void loadRouteLocal() {
        routeSteps.clear();
        routeNotes.clear();
        try {
            org.json.JSONArray arr = new org.json.JSONArray(preferences.getString("route", "[]"));
            for (int i = 0; i < arr.length() && i < MAX_ROUTE_STEPS; i++) {
                org.json.JSONObject o = arr.getJSONObject(i);
                int a = o.optInt("a", ACT_STRAIGHT);
                if (a < 1 || a > 7) continue;
                addStep(a, Math.min(15, Math.max(0, o.optInt("s", 0))), o.optString("n", ""));
            }
        } catch (org.json.JSONException ignored) { }
    }

    private void sendRoute() {
        if (!connected) { showToast("Connect to the robot first"); return; }
        if (running || startPending) { showToast("Stop the robot first"); return; }
        blurInputs();
        int n = routeSteps.size();
        byte[] bytes = new byte[6 + 2 * n + 2];
        int k = 0;
        bytes[k++] = 17; bytes[k++] = (byte)ruleFlags();
        bytes[k++] = 18; bytes[k++] = (byte)pivotSpeedStep.get();
        bytes[k++] = 20; bytes[k++] = (byte)n;
        for (int[] st : routeSteps) {
            bytes[k++] = 21;
            bytes[k++] = (byte)((st[0] << 4) | (st[1] & 0x0F));
        }
        bytes[k++] = 22; bytes[k] = (byte)(turnDelayStep.get() / 10);
        sendBytes(bytes, true);
        saveRuleOptions();
        addEvent("Route sent: " + n + " steps, turn delay " + (turnDelayStep.get() / 10 * 10) + " ms. "
                + (routeSwitch.isChecked() ? "Route mode is ON." : "Turn on Competition route to use it."));
        showToast("Route sent to robot");
        // Read the status back so the step count shows what the robot stored.
        ui.postDelayed(() -> { if (connected) sendBytes(new byte[]{9, 0}, false); }, 600);
    }

    private int ruleFlags() {
        int f = 0;
        if (sharpTurnSwitch != null && sharpTurnSwitch.isChecked()) f |= RULE_SHARP_TURN;
        if (keepOldCalSwitch != null && keepOldCalSwitch.isChecked()) f |= RULE_KEEP_OLD_CAL;
        if (routeSwitch != null && routeSwitch.isChecked()) f |= RULE_ROUTE;
        if (irTurnSwitch != null && irTurnSwitch.isChecked()) f |= RULE_IR_TURN;
        return f;
    }

    private void saveRuleOptions() {
        preferences.edit().putInt("rule_flags", ruleFlags())
                .putInt("pivot_speed", pivotSpeedStep.get()).apply();
    }

    private void sendRuleOptions(boolean log) {
        if (!connected) return;
        sendBytes(new byte[]{17, (byte)ruleFlags(), 18, (byte)pivotSpeedStep.get()}, true);
        if (log) addEvent("Rules sent: sharp turn " + (sharpTurnSwitch.isChecked() ? "ON" : "OFF")
                + " (spin " + pivotSpeedStep.get() + "%), keep previous calibration "
                + (keepOldCalSwitch.isChecked() ? "ON" : "OFF") + ", IR turn "
                + (irTurnSwitch != null && irTurnSwitch.isChecked() ? "ON" : "OFF") + ", route mode "
                + (routeSwitch != null && routeSwitch.isChecked() ? "ON" : "OFF") + ".");
    }

    /** Robot's saved options are the source of truth once it reports them. */
    private void applyRuleOptionsFromRobot(int flags, int pivot) {
        bindingRules = true;
        sharpTurnSwitch.setChecked((flags & RULE_SHARP_TURN) != 0);
        keepOldCalSwitch.setChecked((flags & RULE_KEEP_OLD_CAL) != 0);
        if (routeSwitch != null) routeSwitch.setChecked((flags & RULE_ROUTE) != 0);
        if (irTurnSwitch != null) irTurnSwitch.setChecked((flags & RULE_IR_TURN) != 0);
        bindingRules = false;
        if (pivot >= 10 && pivot <= 100) pivotSpeedStep.set(pivot);
        saveRuleOptions();
    }

    private void restoreSavedCalibration() {
        if (!connected) { showToast("Connect to the robot first"); return; }
        if (calibrating) { showToast("Wait for calibration to finish"); return; }
        if (running || startPending) { showToast("Stop the robot first"); return; }
        restorePending = true;
        sendBytes(new byte[]{19, 0}, true);
        addEvent("Asking the robot to use its saved calibration…");
        ui.postDelayed(() -> restorePending = false, 3000);
    }

    // --------------------------------------------------------------- Log page

    private LinearLayout buildLogPage() {
        LinearLayout content = col();
        LinearLayout activity = card();
        LinearLayout head = row();
        head.addView(txt("Activity", 15, TEXT, true), weighted());
        TextView clear = txt("Clear", 13, MINT, true);
        clear.setPadding(dp(10), dp(4), 0, dp(4));
        clear.setOnClickListener(v -> { recentEvents.clear(); addEvent("Log cleared."); });
        head.addView(clear);
        activity.addView(head);
        activity.addView(gap(10));
        eventLog = txt("", 12, TEXT, false);
        eventLog.setLineSpacing(dp(4), 1f);
        eventLog.setTextIsSelectable(true);
        activity.addView(eventLog, full());
        content.addView(activity, full());
        content.addView(gap(12));

        LinearLayout stats = card();
        stats.addView(cardHeader("Bluetooth link", "BLE  •  Nordic UART"));
        stats.addView(gap(8));
        sendCountLabel = help("TX 0 B  /  RX 0 B");
        stats.addView(sendCountLabel);
        linkInfo = help("CRC errors: 0  •  ignored bytes: 0");
        stats.addView(linkInfo);
        content.addView(stats, full());
        content.addView(gap(14));
        TextView foot = txt("LineForge  •  Protocol v3  •  CRC8 telemetry", 11, MUTED, false);
        foot.setGravity(Gravity.CENTER);
        content.addView(foot, full());
        return content;
    }

    // -------------------------------------------------------- reusable inputs

    /** Row of mutually exclusive options (like a segmented button). */
    private final class Segmented {
        final LinearLayout view = row();
        private final TextView[] items;
        private final IntListener listener;
        private int selected;

        Segmented(String[] labels, int initial, IntListener listener) {
            this.listener = listener;
            view.setBackground(rounded(PANEL_ALT, 12, BORDER));
            view.setPadding(dp(3), dp(3), dp(3), dp(3));
            items = new TextView[labels.length];
            for (int i = 0; i < labels.length; i++) {
                final int index = i;
                TextView t = txt(labels[i], 13, MUTED, true);
                t.setGravity(Gravity.CENTER);
                t.setSingleLine(true);
                t.setOnClickListener(v -> select(index, true));
                items[i] = t;
                view.addView(t, new LinearLayout.LayoutParams(0, dp(38), 1f));
            }
            select(Math.min(labels.length - 1, Math.max(0, initial)), false);
        }

        void select(int index, boolean notify) {
            boolean changed = index != selected;
            selected = index;
            for (int i = 0; i < items.length; i++) {
                boolean on = i == index;
                items[i].setBackground(on ? rounded(tint(MINT, 46), 10, MINT) : null);
                items[i].setTextColor(on ? TEXT : MUTED);
            }
            if (notify && changed && listener != null) listener.on(index);
        }

        int selected() { return selected; }
    }

    /**
     * Precise number input: [−] [typed number] [+] with an optional slider.
     * Tap −/+ for steps of 1, long-press for steps of 10.
     */
    private final class Stepper {
        final LinearLayout view = col();
        final EditText field;
        final SeekBar slider;
        final int min, max;
        IntListener onChange;   // every user change (live display / constraints)
        IntListener onCommit;   // the user finished changing the value
        private int value;
        int stepSize = 1;        // − / + change by this (long-press: x10)
        private boolean binding = false;
        private final Runnable commitTask = () -> { if (onCommit != null) onCommit.on(value); };

        Stepper(String title, String hint, int min, int max, int initial, String unit, int color, boolean withSlider) {
            this.min = min;
            this.max = max;
            if (title != null) {
                LinearLayout header = row();
                LinearLayout names = col();
                names.addView(txt(title, 13, TEXT, true));
                if (hint != null) names.addView(txt(hint, 11, MUTED, false));
                header.addView(names, weighted());
                header.addView(txt(min + "–" + max + (unit.isEmpty() ? "" : " " + unit), 11, MUTED, false));
                view.addView(header, full());
                view.addView(gap(8));
            }
            LinearLayout controls = row();
            controls.addView(stepButton("−", -1, -10), new LinearLayout.LayoutParams(dp(48), dp(46)));
            field = new EditText(MainActivity.this);
            field.setTextColor(TEXT);
            field.setTextSize(18);
            field.setTypeface(Typeface.create("sans-serif-medium", Typeface.BOLD));
            field.setSingleLine(true);
            field.setInputType(InputType.TYPE_CLASS_NUMBER);
            field.setImeOptions(EditorInfo.IME_ACTION_DONE);
            field.setSelectAllOnFocus(true);
            field.setGravity(Gravity.CENTER);
            field.setBackground(rounded(BG, 12, BORDER));
            field.setPadding(dp(6), 0, dp(6), 0);
            LinearLayout.LayoutParams flp = new LinearLayout.LayoutParams(0, dp(46), 1f);
            flp.setMargins(dp(8), 0, dp(8), 0);
            controls.addView(field, flp);
            controls.addView(stepButton("+", 1, 10), new LinearLayout.LayoutParams(dp(48), dp(46)));
            if (!unit.isEmpty()) {
                TextView u = txt(unit, 14, MUTED, true);
                u.setPadding(dp(10), 0, 0, 0);
                controls.addView(u);
            }
            view.addView(controls, full());

            if (withSlider) {
                slider = new SeekBar(MainActivity.this);
                slider.setMax(max - min);
                slider.setProgressTintList(ColorStateList.valueOf(color));
                slider.setThumbTintList(ColorStateList.valueOf(color));
                slider.setProgressBackgroundTintList(ColorStateList.valueOf(BORDER));
                slider.setPadding(dp(12), dp(14), dp(12), dp(4));
                slider.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                    @Override public void onStartTrackingTouch(SeekBar sb) { blurInputs(); }
                    @Override public void onProgressChanged(SeekBar sb, int p, boolean fromUser) {
                        if (fromUser && !binding) userChange(min + p, -1);
                    }
                    @Override public void onStopTrackingTouch(SeekBar sb) { commitNow(); }
                });
                view.addView(slider, full());
            } else {
                slider = null;
            }

            field.addTextChangedListener(new TextWatcher() {
                @Override public void beforeTextChanged(CharSequence s, int a, int b, int c) { }
                @Override public void onTextChanged(CharSequence s, int a, int b, int c) {
                    if (binding) return;
                    try {
                        int v = Integer.parseInt(s.toString());
                        if (v >= Stepper.this.min && v <= Stepper.this.max) {
                            value = v;
                            syncSlider();
                            if (onChange != null) onChange.on(value);
                        }
                    } catch (NumberFormatException ignored) { }
                }
                @Override public void afterTextChanged(Editable s) { }
            });
            field.setOnEditorActionListener((v, action, event) -> {
                if (action == EditorInfo.IME_ACTION_DONE) { blurInputs(); return true; }
                return false;
            });
            field.setOnFocusChangeListener((v, focused) -> {
                if (!focused) {
                    int typed;
                    try { typed = Integer.parseInt(field.getText().toString()); }
                    catch (NumberFormatException e) { typed = value; }
                    userChange(typed, -1);
                    apply(value); // show the clamped number
                    commitNow();
                }
            });
            apply(initial);
        }

        private TextView stepButton(String label, int step, int bigStep) {
            TextView b = txt(label, 22, TEXT, true);
            b.setGravity(Gravity.CENTER);
            b.setBackground(rounded(PANEL_ALT, 12, BORDER));
            b.setOnClickListener(v -> { blurInputs(); userChange(value + step * stepSize, 450); });
            b.setOnLongClickListener(v -> { blurInputs(); userChange(value + bigStep * stepSize, 450); return true; });
            return b;
        }

        int get() { return value; }

        /** Programmatic update: no listeners fire. */
        void set(int v) { apply(v); }

        private void apply(int v) {
            value = Math.min(max, Math.max(min, v));
            binding = true;
            String s = String.valueOf(value);
            if (!field.getText().toString().equals(s)) field.setText(s);
            syncSlider();
            binding = false;
        }

        private void syncSlider() {
            if (slider == null) return;
            boolean was = binding;
            binding = true;
            slider.setProgress(value - min);
            binding = was;
        }

        /** User-driven change; commitDelayMs &lt; 0 means "wait for an explicit commit". */
        private void userChange(int v, int commitDelayMs) {
            int before = value;
            apply(v);
            if (value != before && onChange != null) onChange.on(value);
            ui.removeCallbacks(commitTask);
            if (commitDelayMs >= 0) ui.postDelayed(commitTask, commitDelayMs);
        }

        private void commitNow() {
            ui.removeCallbacks(commitTask);
            commitTask.run();
        }
    }

    // ------------------------------------------------------------ PID + speed

    private final class PidControl {
        final String key;
        final int command;
        final LinearLayout view;
        final TextView largeValue;
        final Stepper number;
        final Segmented divider;

        PidControl(String key, String name, String description, int color, int command,
                   int initialNumber, int initialDivider) {
            this.key = key;
            this.command = command;
            view = card();
            LinearLayout top = row();
            LinearLayout lhs = col();
            lhs.addView(txt(key, 20, color, true));
            lhs.addView(txt(name, 12, MUTED, true));
            top.addView(lhs, weighted());
            largeValue = txt("0.0000", 28, TEXT, true);
            largeValue.setGravity(Gravity.END | Gravity.CENTER_VERTICAL);
            top.addView(largeValue);
            view.addView(top, full());
            view.addView(gap(4));
            view.addView(help(description));
            view.addView(gap(14));

            number = new Stepper(null, null, 0, 255, initialNumber, "", color, true);
            number.onChange = v -> updateDisplay();
            number.onCommit = v -> { saveLocal(); if (liveTune) sendThisGain(); };
            view.addView(number.view, full());
            view.addView(gap(12));
            view.addView(txt("Divide by", 12, MUTED, true));
            view.addView(gap(6));
            divider = new Segmented(new String[]{"÷1", "÷10", "÷100", "÷1k", "÷10k"}, initialDivider, i -> {
                updateDisplay();
                saveLocal();
                if (liveTune) sendThisGain();
            });
            view.addView(divider.view, full());
            updateDisplay();
        }

        int value() { return number.get(); }
        int divisor() { return Math.min(4, Math.max(0, divider.selected())); }

        void setData(int numerator, int divisor) {
            number.set(numerator);
            divider.select(Math.min(4, Math.max(0, divisor)), false);
            updateDisplay();
        }

        double gain() { return value() / Math.pow(10.0, divisor()); }

        void updateDisplay() {
            largeValue.setText(String.format(Locale.US, "%.4f", gain()));
            refreshStatusTiles();
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
            addEvent(key + " sent: " + largeValue.getText());
        }
    }

    /** Motor speed (% of full PWM): command 13 = base speed, 14 = max speed. */
    private final class SpeedControl {
        final LinearLayout view = card();
        final Stepper baseStep, maxStep;

        SpeedControl() {
            view.addView(cardHeader("Motor speed", "% of full power"));
            view.addView(gap(12));
            baseStep = new Stepper("Base speed", "Cruising speed on the line", 0, 100,
                    preferences.getInt("speed_base", 25), "%", ORANGE, true);
            maxStep = new Stepper("Max speed", "Fastest any wheel may turn", 0, 100,
                    preferences.getInt("speed_max", 30), "%", RED, true);
            // Base speed can never be above max speed.
            baseStep.onChange = v -> { if (v > maxStep.get()) maxStep.set(v); refreshStatusTiles(); };
            maxStep.onChange = v -> { if (v < baseStep.get()) baseStep.set(v); refreshStatusTiles(); };
            baseStep.onCommit = v -> { saveLocal(); send(true); };
            maxStep.onCommit = v -> { saveLocal(); send(true); };
            view.addView(baseStep.view, full());
            view.addView(gap(16));
            view.addView(maxStep.view, full());
            view.addView(gap(10));
            view.addView(help("Changes are sent to the robot right away. Start gives a short kick at max speed."));
        }

        int base() { return baseStep.get(); }
        int max() { return maxStep.get(); }

        void setData(int base, int max) {
            maxStep.set(max);
            baseStep.set(Math.min(maxStep.get(), base));
            refreshStatusTiles();
        }

        void saveLocal() {
            preferences.edit().putInt("speed_base", base()).putInt("speed_max", max()).apply();
        }

        void send(boolean log) {
            if (!connected) return;
            sendBytes(new byte[]{13, (byte)base(), 14, (byte)max()}, true);
            if (log) addEvent("Speed sent: base " + base() + "%, max " + max() + "%.");
        }
    }

    private void refreshStatusTiles() {
        if (calTileValue == null) return;
        String calText;
        int calColor;
        if (calibrating) { calText = "Calibrating"; calColor = ORANGE; }
        else if (!connected) { calText = "—"; calColor = MUTED; }
        else if (calibrationValid) { calText = "Ready"; calColor = GREEN; }
        else if (queryReceived) { calText = "Needed"; calColor = RED; }
        else { calText = "Checking…"; calColor = MUTED; }
        calTileValue.setText(calText);
        calTileValue.setTextColor(calColor);
        if (calBadge != null) {
            calBadge.setText(calText.equals("—") ? "Offline" : calText);
            calBadge.setTextColor(calColor);
            calBadge.setBackground(rounded(tint(calColor, 40), 20, 0));
        }
        if (kp != null && ki != null && kd != null) {
            pidTileValue.setText(String.format(Locale.US, "%s · %s · %s",
                    shortNumber(kp.gain()), shortNumber(ki.gain()), shortNumber(kd.gain())));
        }
        if (speed != null) speedTileValue.setText(speed.base() + " / " + speed.max() + "%");
    }

    private static String shortNumber(double v) {
        if (v == Math.rint(v)) return String.valueOf((long)v);
        String s = String.format(Locale.US, "%.4f", v);
        while (s.endsWith("0")) s = s.substring(0, s.length() - 1);
        return s;
    }

    // --------------------------------------------------------------- actions

    /** PID, speed and rule switches exactly as shown on screen. */
    private byte[] settingsBytes() {
        return new byte[]{
                1, (byte)kp.value(), 2, (byte)kp.divisor(),
                3, (byte)ki.value(), 4, (byte)ki.divisor(),
                5, (byte)kd.value(), 6, (byte)kd.divisor(),
                13, (byte)speed.base(), 14, (byte)speed.max(),
                17, (byte)ruleFlags(), 18, (byte)pivotSpeedStep.get()};
    }

    private void uploadAll() {
        if (!connected) { showToast("Connect to the robot first"); return; }
        blurInputs();
        kp.saveLocal(); ki.saveLocal(); kd.saveLocal(); speed.saveLocal(); saveRuleOptions();
        sendBytes(settingsBytes(), true);
        sendAllParams();
        addEvent("Sent PID, speed and rule settings. The robot saves them after 2 s.");
        showToast("Sent to robot");
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
        if (!connected) { showToast("Connect to the robot first"); return; }
        if (calibrating) { showToast("Wait for calibration to finish"); return; }
        if (queryReceived && !calibrationValid) {
            showToast("Calibrate the sensors first");
            showTab(TAB_CALIBRATE);
            return;
        }
        blurInputs();
        // Send what is on screen first, so the robot never starts with old values
        // (e.g. PID changed while Live tuning was off).
        kp.saveLocal(); ki.saveLocal(); kd.saveLocal(); speed.saveLocal();
        sendBytes(settingsBytes(), false);
        startPending = true;
        running = false;
        sendBytes(new byte[]{7, 1}, true);
        updateRunUI();
        addEvent("START sent; waiting for the robot to confirm…");
        ui.postDelayed(() -> {
            if (connected && startPending) {
                startPending = false;
                running = false;
                updateRunUI();
                // Do not assume the robot stopped just because its reply was lost.
                // Explicit STOP as well as the firmware heartbeat timeout.
                sendBytes(new byte[]{7, 0}, false);
                addEvent("No START confirmation: STOP sent. Check the BLE link, power and firmware.");
            }
        }, 2500);
    }

    private void stopRobot() {
        startPending = false;
        running = false;
        updateRunUI();
        if (connected) sendBytes(new byte[]{7, 0}, true);
        addEvent(calibrating ? "STOP sent: cancelling calibration…" : "STOP sent.");
    }

    private void updateRunUI() {
        if (heroStatus == null) return;
        String status;
        int color;
        if (!connected) { status = "Not connected"; color = MUTED; }
        else if (calibrating) { status = "Calibrating sensors…"; color = ORANGE; }
        else if (startPending) { status = "Starting… waiting for robot"; color = ORANGE; }
        else if (running) { status = "●  Running"; color = GREEN; }
        else { status = "Motors off  •  ready"; color = MUTED; }
        heroStatus.setText(status);
        heroStatus.setTextColor(color);
        if (startButton != null) startButton.setAlpha(connected && !running && !startPending && !calibrating ? 1f : 0.45f);
        if (stopButton != null) stopButton.setAlpha(connected ? 1f : 0.6f);
        if (running || startPending || calibrating) getWindow().addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        else getWindow().clearFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        refreshStatusTiles();
    }

    private void requestRobotState() {
        if (!connected) { showToast("Connect to the robot first"); return; }
        queryReceived = false;
        sendBytes(new byte[]{9, 0}, true);
        addEvent("Reading settings from the robot…");
        ui.postDelayed(() -> {
            if (connected && !queryReceived)
                addEvent("No reply from the robot. Make sure the LineForge BLE firmware is uploaded; also check power.");
        }, 2300);
    }

    private void requestCalibrationData(boolean showDialog) {
        if (!connected) { showToast("Connect to the robot first"); return; }
        calibrationDialogPending = showDialog;
        sendBytes(new byte[]{12, 0}, true);
        if (showDialog) addEvent("Reading saved calibration values…");
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
        setStatusChip(label, ORANGE);
        if (name != null) selectedDevice.setText(name);
        connectHint.setText("Keep the robot switched on and close to the phone.");
        connectButton.setEnabled(true);
        connectButton.setText("Cancel");
        connectButton.setTextColor(TEXT);
        connectButton.setBackground(rounded(PANEL_ALT, 14, BORDER));
        updateRunUI();
    }

    private void setStatusChip(String label, int color) {
        connectionLabel.setText(label);
        connectionLabel.setTextColor(color);
        connectionDot.setBackground(rounded(color, 100, 0));
        statusChip.setBackground(rounded(tint(color, 34), 20, tint(color, 90)));
    }

    private void updateCounters() {
        sendCountLabel.setText("TX " + sentBytes + " B  /  RX " + receivedBytes + " B");
    }

    // All BleUartClient callbacks arrive on the main thread.
    private final BleUartClient.Callback bleCallback = new BleUartClient.Callback() {
        @Override public void onConnecting(String name, int attempt, int maxRetries) {
            showConnectingUI(name, attempt == 0 ? "Connecting…" :
                    "Reconnecting " + attempt + "/" + maxRetries);
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
            sendBytes(new byte[]{26, 0}, false);   // read rule settings
        }
        @Override public void onDisconnected(String name, String reason, boolean willRetry) {
            boolean wasConnected = connected;
            resetLinkState();
            String safety = wasConnected ? " Robot stops via ESP32 heartbeat timeout." : "";
            if (willRetry) {
                showConnectingUI(name, "Reconnecting…");
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
        if (b == 0x15 && restorePending) {
            restorePending = false;
            addEvent("The robot has no saved calibration to use. Run a new calibration.");
            showToast("No saved calibration on the robot");
            return;
        }
        if (b == 0x15) {
            addEvent("The robot rejected a command. Check calibration and firmware version.");
            if (startPending) {
                startPending = false;
                running = false;
                updateRunUI();
            }
            if (calibrating && SystemClock.uptimeMillis() - calStartedMs < 1500)
                finishCalibration(false, "The robot refused to calibrate. Upload the latest firmware and try again.");
            return;
        }
        if (b == 0xC1) {
            calibrationValid = true;
            addEvent("Calibration successful. Saved on the robot.");
            finishCalibration(true, "✓  Calibration successful, saved on the robot");
            updateRunUI();
            ui.postDelayed(() -> { requestRobotState(); requestCalibrationData(false); }, 350);
            return;
        }
        if (b == 0xC0) {
            calibrationValid = false; running = false; startPending = false;
            calFailedAtMs = SystemClock.uptimeMillis();
            finishCalibration(false, "✗  Calibration failed: some sensors never saw the line. Try again.");
            updateRunUI(); addEvent("Calibration failed. Motors disabled until calibrated."); return;
        }
        if (b == 0xC2) {
            calibrationValid = true;
            // Sent at boot, or when a calibration is cancelled with STOP.
            if (calibrating) {
                finishCalibration(false, "Calibration cancelled. The previous calibration is kept.");
                addEvent("Calibration cancelled; previous calibration kept.");
            } else if (restorePending) {
                restorePending = false;
                addEvent("Robot is now using its saved calibration.");
                showToast("Saved calibration in use");
                calProgressText.setVisibility(View.VISIBLE);
                calProgressText.setTextColor(GREEN);
                calProgressText.setText("✓  Using the saved calibration");
            } else if (SystemClock.uptimeMillis() - calFailedAtMs < 3000) {
                addEvent("New calibration failed; the robot keeps using its previous calibration.");
                calProgressText.setVisibility(View.VISIBLE);
                calProgressText.setTextColor(ORANGE);
                calProgressText.setText("New calibration failed, so the previous saved calibration is still in use.");
            } else addEvent("Saved calibration loaded.");
            updateRunUI();
            return;
        }
        if (b == 0xE1) {
            running = false; startPending = false; updateRunUI();
            addEvent("Route complete: the robot reached the finish.");
            showToast("Route complete");
            return;
        }
        if (b == 0xE0) {
            running = false; startPending = false; updateRunUI();
            addEvent("Route stopped: a turn did not find the line. Check the steps and the turn delay.");
            return;
        }
        if (b == 0xD1) {
            running = false; startPending = false; updateRunUI();
            addEvent("Safety stop: no valid heartbeat at ESP32."); return;
        }
    }

    private void receivePacket(int type, byte[] payload) {
        if (type == Protocol.STATUS && payload.length >= 8) {
            receiveStateFrame(payload, true);
        } else if (type == Protocol.SENSORS && payload.length >= 13) {
            int[] samples = new int[8];
            for (int i=0;i<8;i++) samples[i] = payload[i] & 255;
            int pos = (payload[8] & 255) | ((payload[9] & 255) << 8);
            int flags = payload[10] & 255;
            boolean seen = (flags & 1) != 0;
            boolean cal = (flags & 8) != 0;
            boolean pivot = (flags & 16) != 0;
            sensorView.update(samples, pos, seen, cal);
            int l = (int)payload[11], r = (int)payload[12];
            sensorInfo.setText(cal ? "Calibration running • raw reflectance levels" :
                    pivot ? "Sharp turn recovery: spinning to find the line" :
                    seen ? String.format(Locale.US, "Line position %.2f / 7   •   Left %+d%%   Right %+d%%",
                            pos / 1000.0, l, r) :
                            "LINE LOST  •  Motors L " + l + "%  R " + r + "%");
            if (payload.length >= 16 && routeSwitch != null && routeSwitch.isChecked() && (running || startPending)) {
                int done = payload[13] & 255, total = payload[14] & 255;
                sensorInfo.setText("Route step " + done + " of " + total + "   •   " + sensorInfo.getText());
            }
        } else if (type == Protocol.PARAMS) {
            applyParamsFromRobot(payload);
        } else if (type == Protocol.COUNTERS && payload.length >= 10) {
            int[] v = {(payload[0] & 255) | ((payload[1] & 255) << 8),
                    (payload[2] & 255) | ((payload[3] & 255) << 8),
                    payload[4] & 255, payload[5] & 255, payload[6] & 255, payload[7] & 255};
            for (int i = 0; i < 6; i++) counterViews[i].setText(String.valueOf(v[i]));
            int f = payload[8] & 255;
            boolean inverted = (f & 4) != 0;
            lineModeLabel.setText(inverted ? "Line: WHITE line on black floor (inverted)"
                    : "Line: black line on light floor");
            lineModeLabel.setTextColor(inverted ? ORANGE : MUTED);
            boolean l = (f & 1) != 0, r = (f & 2) != 0;
            irLiveLabel.setText("IR-L  " + (l ? "●  sees line" : "○") + "      IR-R  " + (r ? "●  sees line" : "○"));
            irLiveLabel.setTextColor(l || r ? GREEN : MUTED);
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
        if (data.length < 8) return;
        if (!framed) {
            int checksum = 0;
            for (int i = 0; i < 7; ++i) checksum ^= (data[i] & 255);
            if (checksum != (data[7] & 255)) { addEvent("Status checksum error."); return; }
        }
        int kpd=data[1] & 255, kid=data[3] & 255, kdd=data[5] & 255;
        if (kpd>4 || kid>4 || kdd>4) { addEvent("Invalid PID values in status."); return; }
        int flags = data[6] & 255;
        if ((flags & ~7) != 0) return;
        if (framed && !resetWarningShown && (data[7] & 255) == 9) {
            resetWarningShown = true;
            addEvent("⚠ The robot restarted because its battery voltage dropped (brownout), "
                    + "usually when the motors start. Charge the battery, or power the ESP32 "
                    + "separately from the motors / add a large capacitor.");
            showToast("Robot restarted: battery voltage dropped");
        }
        boolean differs = kp.value() != (data[0] & 255) || kp.divisor() != kpd
                || ki.value() != (data[2] & 255) || ki.divisor() != kid
                || kd.value() != (data[4] & 255) || kd.divisor() != kdd
                || (framed && data.length >= 10 && speed != null
                    && (speed.base() != (data[8] & 255) || speed.max() != (data[9] & 255)));
        if (differs && queryReceived)
            addEvent("The robot reported different PID/speed values than the app showed "
                    + "(it may have restarted). Showing the robot's values now.");
        kp.setData(data[0] & 255,kpd);
        ki.setData(data[2] & 255,kid);
        kd.setData(data[4] & 255,kdd);
        kp.saveLocal(); ki.saveLocal(); kd.saveLocal();
        // Framed status from the BLE firmware also carries base/max speed %.
        if (framed && data.length >= 10 && speed != null) {
            int base = data[8] & 255, max = data[9] & 255;
            if (base <= 100 && max <= 100) { speed.setData(base, max); speed.saveLocal(); }
        }
        if (framed && data.length >= 12 && sharpTurnSwitch != null)
            applyRuleOptionsFromRobot(data[10] & 255, data[11] & 255);
        if (framed && data.length >= 14 && turnDelayStep != null) {
            robotRouteLen = data[12] & 255;
            int delay10 = data[13] & 255;
            if (delay10 <= 50) {
                turnDelayStep.set(delay10 * 10);
                preferences.edit().putInt("turn_delay", delay10 * 10).apply();
            }
            updateRouteSummary();
        }
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
        setStatusChip(active ? "Connected" : "Offline", active ? GREEN : RED);
        if (name != null) selectedDevice.setText(name);
        else if (!active) selectedDevice.setText("No robot connected");
        connectHint.setText(active ? "Connected over Bluetooth Low Energy."
                : "Switch on the robot, then tap Connect.");
        connectButton.setEnabled(true);
        connectButton.setText(active ? "Disconnect" : "Connect to robot");
        connectButton.setTextColor(active ? TEXT : BG);
        connectButton.setBackground(active ? rounded(PANEL_ALT, 14, BORDER) : rounded(MINT, 14, 0));
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
        if (calibrating) finishCalibration(false, "Connection lost during calibration. Reconnect and check the result.");
        resetWarningShown = false;
    }

    /** User-initiated disconnect (or app shutdown): no automatic reconnection. */
    private void disconnectLink() {
        resetLinkState();
        if (ble != null) ble.disconnect();
        if (connectionLabel != null) updateConnectionUI(false, null);
    }

    private void addEvent(String s) {
        lastEvent = s;
        String time = new java.text.SimpleDateFormat("HH:mm:ss", Locale.US).format(new java.util.Date());
        recentEvents.addFirst(time + "   " + s);
        while (recentEvents.size() > MAX_EVENTS) recentEvents.removeLast();
        if (lastEventLabel != null) lastEventLabel.setText(s);
        if (eventLog != null) {
            StringBuilder b = new StringBuilder();
            for (String e : recentEvents) {
                if (b.length() > 0) b.append("\n\n");
                b.append(e);
            }
            eventLog.setText(b.toString());
        }
    }

    private void showToast(String s) { Toast.makeText(this, s, Toast.LENGTH_SHORT).show(); }
}
