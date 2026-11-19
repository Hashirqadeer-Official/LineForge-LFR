package com.lineforge.lfr;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.BluetoothStatusCodes;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanRecord;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.Context;
import android.os.Build;
import android.os.Handler;
import android.os.Looper;
import android.os.ParcelUuid;

import java.util.ArrayDeque;
import java.util.List;
import java.util.UUID;

/**
 * BLE transport for the ESP32-S3 Nordic UART Service (NUS).
 *
 * Phone -> ESP32: writes to the RX characteristic (6E400002).
 * ESP32 -> Phone: notifications from the TX characteristic (6E400003).
 *
 * All public methods must be called on the main thread and every callback is
 * delivered on the main thread. GATT operations are strictly serialized:
 * Android allows only one outstanding GATT operation per connection.
 */
@SuppressLint("MissingPermission") // Callers check BLUETOOTH_SCAN / BLUETOOTH_CONNECT first.
final class BleUartClient {
    static final UUID NUS_SERVICE = UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    static final UUID NUS_RX = UUID.fromString("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
    static final UUID NUS_TX = UUID.fromString("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
    private static final UUID CCCD = UUID.fromString("00002902-0000-1000-8000-00805F9B34FB");

    /** Large enough that a 38-byte calibration frame fits in one notification. */
    private static final int REQUESTED_MTU = 247;
    private static final int DEFAULT_MTU = 23;
    private static final int MAX_QUEUED_WRITES = 32;
    private static final long CONNECT_TIMEOUT_MS = 15000;
    private static final long WRITE_TIMEOUT_MS = 1500;
    private static final long BUSY_RETRY_MS = 25;
    private static final int MAX_BUSY_RETRIES = 20;
    /** Backoff used after an established link drops unexpectedly. */
    private static final long[] RECONNECT_DELAYS_MS = {1000, 2000, 4000};
    /** One quick retry for the first connection (Android's GATT error 133 is often transient). */
    private static final int INITIAL_CONNECT_RETRIES = 1;

    interface Callback {
        /** attempt 0 = first connection attempt; otherwise a retry out of maxRetries. */
        void onConnecting(String name, int attempt, int maxRetries);
        /** Services discovered, notifications on, writes allowed. */
        void onReady(String name, int mtu);
        /** Link is down. If willRetry, onConnecting follows automatically after a delay. */
        void onDisconnected(String name, String reason, boolean willRetry);
        void onData(byte[] data);
        void onBytesWritten(int count);
        void onError(String message);
    }

    interface ScanListener {
        void onDevice(BluetoothDevice device, String name, int rssi, boolean advertisesNus);
        void onScanFailed(int errorCode);
    }

    private final Context context;
    private final Callback callback;
    private final Handler main = new Handler(Looper.getMainLooper());
    private final ArrayDeque<byte[]> queue = new ArrayDeque<>();

    private BluetoothDevice device;
    private String deviceName;
    private BluetoothGatt gatt;
    private BluetoothGattCharacteristic rx;
    private int writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT;
    private int mtu = DEFAULT_MTU;
    /** Incremented whenever a GATT instance is opened or closed; stale callbacks are ignored. */
    private int generation = 0;
    private boolean ready = false;
    private boolean discoveryStarted = false;
    private boolean userClosed = true;
    private int retriesLeft = 0, retryBudget = 0, attempt = 0;

    private boolean writeInFlight = false;
    private int inFlightBytes = 0;
    private int writeToken = 0;
    private int busyRetries = 0;

    private BluetoothLeScanner scanner;
    private ScanCallback scanCallback;

    BleUartClient(Context context, Callback callback) {
        this.context = context.getApplicationContext();
        this.callback = callback;
    }

    BluetoothAdapter adapter() {
        BluetoothManager m = (BluetoothManager)context.getSystemService(Context.BLUETOOTH_SERVICE);
        return m == null ? null : m.getAdapter();
    }

    boolean isReady() { return ready; }

    /** True while connecting, connected, or waiting to reconnect. */
    boolean isActive() { return !userClosed && device != null; }

    // ------------------------------------------------------------------ scanning

    boolean startScan(ScanListener listener) {
        stopScan();
        BluetoothAdapter a = adapter();
        if (a == null || !a.isEnabled()) return false;
        BluetoothLeScanner s = a.getBluetoothLeScanner();
        if (s == null) return false;
        ScanCallback cb = new ScanCallback() {
            @Override public void onScanResult(int callbackType, ScanResult result) { report(this, listener, result); }
            @Override public void onBatchScanResults(List<ScanResult> results) {
                for (ScanResult r : results) report(this, listener, r);
            }
            @Override public void onScanFailed(int errorCode) {
                main.post(() -> { if (scanCallback == this) listener.onScanFailed(errorCode); });
            }
        };
        ScanSettings settings = new ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build();
        try {
            // No hardware filter: some firmwares put the 128-bit NUS UUID in the scan
            // response or omit it entirely, so filtering would hide the ESP32.
            s.startScan(null, settings, cb);
        } catch (RuntimeException e) {
            return false;
        }
        scanner = s;
        scanCallback = cb;
        return true;
    }

    private void report(ScanCallback owner, ScanListener listener, ScanResult r) {
        BluetoothDevice d = r.getDevice();
        if (d == null) return;
        ScanRecord rec = r.getScanRecord();
        boolean nus = false;
        String name = null;
        if (rec != null) {
            List<ParcelUuid> uuids = rec.getServiceUuids();
            nus = uuids != null && uuids.contains(new ParcelUuid(NUS_SERVICE));
            name = rec.getDeviceName();
        }
        if (name == null) {
            try { name = d.getName(); } catch (SecurityException ignored) { }
        }
        final String finalName = name;
        final boolean finalNus = nus;
        main.post(() -> {
            if (scanCallback == owner) listener.onDevice(d, finalName, r.getRssi(), finalNus);
        });
    }

    void stopScan() {
        BluetoothLeScanner s = scanner;
        ScanCallback cb = scanCallback;
        scanner = null;
        scanCallback = null;
        if (s != null && cb != null) {
            // Throws IllegalStateException if Bluetooth was switched off meanwhile.
            try { s.stopScan(cb); } catch (RuntimeException ignored) { }
        }
    }

    // ---------------------------------------------------------------- connection

    void connect(BluetoothDevice target) {
        stopScan();
        closeGatt();
        userClosed = false;
        device = target;
        String n = null;
        try { n = target.getName(); } catch (SecurityException ignored) { }
        deviceName = n == null ? target.getAddress() : n;
        attempt = 0;
        retriesLeft = retryBudget = INITIAL_CONNECT_RETRIES;
        openGatt();
    }

    /** User-initiated disconnect: no automatic reconnection afterwards. */
    void disconnect() {
        userClosed = true;
        closeGatt();
        device = null;
    }

    private void openGatt() {
        generation++;
        final int gen = generation;
        resetLinkFields();
        callback.onConnecting(deviceName, attempt, retryBudget);
        BluetoothAdapter a = adapter();
        if (a == null || !a.isEnabled()) {
            retriesLeft = 0;
            fail(gen, "Bluetooth is turned off");
            return;
        }
        BluetoothGatt g;
        try {
            g = device.connectGatt(context, false, gattCallback(gen), BluetoothDevice.TRANSPORT_LE);
        } catch (RuntimeException e) {
            g = null;
        }
        if (g == null) {
            fail(gen, "Android refused to open a GATT connection");
            return;
        }
        gatt = g;
        main.postDelayed(() -> {
            if (gen == generation && !ready) fail(gen, "connection timed out");
        }, CONNECT_TIMEOUT_MS);
    }

    private void resetLinkFields() {
        ready = false;
        discoveryStarted = false;
        rx = null;
        mtu = DEFAULT_MTU;
        queue.clear();
        writeInFlight = false;
        writeToken++;
        busyRetries = 0;
    }

    private void closeGatt() {
        generation++;
        resetLinkFields();
        BluetoothGatt g = gatt;
        gatt = null;
        if (g != null) {
            try { g.disconnect(); } catch (RuntimeException ignored) { }
            try { g.close(); } catch (RuntimeException ignored) { }
        }
    }

    /** Tears down the current GATT and either schedules a reconnect or reports final failure. */
    private void fail(int gen, String reason) {
        if (gen != generation) return;
        closeGatt();
        if (!userClosed && device != null && retriesLeft > 0) {
            long delay = RECONNECT_DELAYS_MS[Math.min(attempt, RECONNECT_DELAYS_MS.length - 1)];
            retriesLeft--;
            attempt++;
            final int scheduled = generation;
            callback.onDisconnected(deviceName, reason, true);
            main.postDelayed(() -> {
                if (!userClosed && device != null && generation == scheduled && gatt == null) openGatt();
            }, delay);
        } else {
            device = null;
            userClosed = true;
            callback.onDisconnected(deviceName, reason, false);
        }
    }

    private BluetoothGattCallback gattCallback(final int gen) {
        // GATT callbacks arrive on a binder thread; hop to main for all state changes.
        return new BluetoothGattCallback() {
            @Override public void onConnectionStateChange(BluetoothGatt g, int status, int newState) {
                main.post(() -> handleConnectionState(gen, g, status, newState));
            }
            @Override public void onMtuChanged(BluetoothGatt g, int newMtu, int status) {
                main.post(() -> handleMtu(gen, g, newMtu, status));
            }
            @Override public void onServicesDiscovered(BluetoothGatt g, int status) {
                main.post(() -> handleServices(gen, g, status));
            }
            @Override public void onDescriptorWrite(BluetoothGatt g, BluetoothGattDescriptor d, int status) {
                main.post(() -> handleDescriptorWrite(gen, d, status));
            }
            @Override public void onCharacteristicWrite(BluetoothGatt g, BluetoothGattCharacteristic c, int status) {
                main.post(() -> handleWriteDone(gen, status));
            }
            // Android 13+ delivers the value directly; its default implementation would
            // call the deprecated overload below, so overriding both never double-delivers.
            @Override public void onCharacteristicChanged(BluetoothGatt g, BluetoothGattCharacteristic c, byte[] value) {
                deliver(gen, c, value);
            }
            @SuppressWarnings("deprecation")
            @Override public void onCharacteristicChanged(BluetoothGatt g, BluetoothGattCharacteristic c) {
                deliver(gen, c, c.getValue());
            }
        };
    }

    private void deliver(int gen, BluetoothGattCharacteristic c, byte[] value) {
        if (value == null || value.length == 0 || !NUS_TX.equals(c.getUuid())) return;
        final byte[] copy = value.clone(); // the characteristic buffer is reused by the stack
        main.post(() -> { if (gen == generation && ready) callback.onData(copy); });
    }

    private void handleConnectionState(int gen, BluetoothGatt g, int status, int newState) {
        if (gen != generation) return;
        if (status == BluetoothGatt.GATT_SUCCESS && newState == BluetoothProfile.STATE_CONNECTED) {
            // Ask for a bigger MTU first; discovery starts in onMtuChanged (or now, if refused).
            if (!g.requestMtu(REQUESTED_MTU)) startDiscovery(gen, g);
            return;
        }
        if (newState == BluetoothProfile.STATE_DISCONNECTED || status != BluetoothGatt.GATT_SUCCESS) {
            fail(gen, describeStatus(status));
        }
    }

    private void handleMtu(int gen, BluetoothGatt g, int newMtu, int status) {
        if (gen != generation) return;
        if (status == BluetoothGatt.GATT_SUCCESS) mtu = newMtu;
        startDiscovery(gen, g);
    }

    private void startDiscovery(int gen, BluetoothGatt g) {
        if (discoveryStarted) return; // the peer may renegotiate MTU later
        discoveryStarted = true;
        if (!g.discoverServices()) fail(gen, "service discovery could not start");
    }

    private void handleServices(int gen, BluetoothGatt g, int status) {
        if (gen != generation || ready) return;
        if (status != BluetoothGatt.GATT_SUCCESS) { fail(gen, "service discovery failed (" + status + ")"); return; }
        BluetoothGattService service = g.getService(NUS_SERVICE);
        BluetoothGattCharacteristic rxChar = service == null ? null : service.getCharacteristic(NUS_RX);
        BluetoothGattCharacteristic txChar = service == null ? null : service.getCharacteristic(NUS_TX);
        if (rxChar == null || txChar == null) {
            retriesLeft = 0; // wrong device or firmware: retrying will not help
            fail(gen, service == null ? "Nordic UART service not found on this device"
                    : "NUS RX/TX characteristic missing");
            return;
        }
        int rxProps = rxChar.getProperties();
        if ((rxProps & BluetoothGattCharacteristic.PROPERTY_WRITE) != 0) {
            writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT; // acknowledged, ordered
        } else if ((rxProps & BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE) != 0) {
            writeType = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE;
        } else {
            retriesLeft = 0;
            fail(gen, "NUS RX characteristic is not writable");
            return;
        }
        rx = rxChar;

        int txProps = txChar.getProperties();
        boolean notify = (txProps & BluetoothGattCharacteristic.PROPERTY_NOTIFY) != 0;
        boolean indicate = (txProps & BluetoothGattCharacteristic.PROPERTY_INDICATE) != 0;
        if ((!notify && !indicate) || !g.setCharacteristicNotification(txChar, true)) {
            retriesLeft = 0;
            fail(gen, "NUS TX characteristic does not support notifications");
            return;
        }
        BluetoothGattDescriptor cccd = txChar.getDescriptor(CCCD);
        if (cccd == null) {
            callback.onError("ESP32 TX has no CCCD descriptor; replies may not arrive.");
            becomeReady(g);
            return;
        }
        byte[] enable = notify ? BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                : BluetoothGattDescriptor.ENABLE_INDICATION_VALUE;
        if (!writeDescriptor(g, cccd, enable)) fail(gen, "could not enable TX notifications");
    }

    @SuppressWarnings("deprecation")
    private static boolean writeDescriptor(BluetoothGatt g, BluetoothGattDescriptor d, byte[] value) {
        if (Build.VERSION.SDK_INT >= 33) {
            return g.writeDescriptor(d, value) == BluetoothStatusCodes.SUCCESS;
        }
        d.setValue(value);
        return g.writeDescriptor(d);
    }

    private void handleDescriptorWrite(int gen, BluetoothGattDescriptor d, int status) {
        if (gen != generation || ready || !CCCD.equals(d.getUuid())) return;
        if (status != BluetoothGatt.GATT_SUCCESS) { fail(gen, "enabling TX notifications failed (" + status + ")"); return; }
        BluetoothGatt g = gatt;
        if (g != null) becomeReady(g);
    }

    private void becomeReady(BluetoothGatt g) {
        ready = true;
        attempt = 0;
        retriesLeft = retryBudget = RECONNECT_DELAYS_MS.length; // budget for later link drops
        g.requestConnectionPriority(BluetoothGatt.CONNECTION_PRIORITY_HIGH); // lower latency for STOP
        callback.onReady(deviceName, mtu);
        pump();
    }

    // -------------------------------------------------------------------- writes

    /**
     * Queues raw bytes for the RX characteristic. Writes larger than the ATT payload
     * are split on even offsets so a [command, value] pair is never separated.
     */
    boolean send(byte[] packet) {
        if (!ready || gatt == null || rx == null || packet.length == 0) return false;
        queue.addAll(Protocol.splitForWrite(packet, mtu - 3));
        boolean overflow = false;
        while (queue.size() > MAX_QUEUED_WRITES) { queue.removeFirst(); overflow = true; }
        if (overflow) callback.onError("BLE link congested: oldest queued commands dropped.");
        pump();
        return true;
    }

    @SuppressWarnings("deprecation")
    private void pump() {
        if (writeInFlight || !ready || queue.isEmpty() || gatt == null || rx == null) return;
        byte[] next = queue.peekFirst();
        boolean accepted;
        try {
            if (Build.VERSION.SDK_INT >= 33) {
                accepted = gatt.writeCharacteristic(rx, next, writeType) == BluetoothStatusCodes.SUCCESS;
            } else {
                rx.setWriteType(writeType);
                rx.setValue(next);
                accepted = gatt.writeCharacteristic(rx);
            }
        } catch (RuntimeException e) {
            accepted = false;
        }
        if (!accepted) {
            // The stack is briefly busy (e.g. a connection-parameter update); retry shortly.
            if (++busyRetries <= MAX_BUSY_RETRIES) {
                final int token = writeToken;
                main.postDelayed(() -> { if (token == writeToken) pump(); }, BUSY_RETRY_MS);
            } else {
                busyRetries = 0;
                queue.removeFirst();
                callback.onError("BLE write rejected by Android; command dropped.");
                pump();
            }
            return;
        }
        busyRetries = 0;
        queue.removeFirst();
        writeInFlight = true;
        inFlightBytes = next.length;
        final int token = ++writeToken;
        // Safety net in case a stack never reports completion (seen with write-without-response).
        main.postDelayed(() -> {
            if (token == writeToken && writeInFlight) {
                writeInFlight = false;
                callback.onBytesWritten(inFlightBytes);
                pump();
            }
        }, WRITE_TIMEOUT_MS);
    }

    private void handleWriteDone(int gen, int status) {
        if (gen != generation || !writeInFlight) return;
        writeInFlight = false;
        writeToken++;
        if (status == BluetoothGatt.GATT_SUCCESS) callback.onBytesWritten(inFlightBytes);
        else callback.onError("BLE write failed (GATT status " + status + ").");
        pump();
    }

    private static String describeStatus(int status) {
        switch (status) {
            case BluetoothGatt.GATT_SUCCESS: return "disconnected";
            case 8: return "link supervision timeout (out of range or ESP32 reset)";
            case 19: return "ESP32 closed the connection";
            case 22: return "connection terminated by phone";
            case 62: return "connection failed to establish";
            case 133: return "GATT error 133 (device busy or out of range)";
            default: return "GATT status " + status;
        }
    }
}
