/*
 * Copyright 2026 Rob Meades
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* This written by Google Gemini from my prompts. */

// ######################################################################
// This Android application works with ESP32 app BLE_INTERFACE_VERSION 1,
// see the ESP-IDF application in file spider_launcher.c for details.
// ######################################################################

package com.example.spider_launcher

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.*
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.content.Context
import android.content.pm.PackageManager
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.text.BasicTextField
import androidx.compose.foundation.verticalScroll
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalFocusManager
import androidx.compose.ui.unit.dp
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.focus.onFocusChanged
import androidx.core.content.ContextCompat
import kotlinx.coroutines.*
import java.util.*

class MainActivity : ComponentActivity() {

    private val bluetoothAdapter: BluetoothAdapter? by lazy {
        val bluetoothManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothManager.adapter
    }

    // Mutable states exposed to UI (initialized to null / blank until read from device)
    var connectionStatus = mutableStateOf("Disconnected")
    var deviceStateName = mutableStateOf("UNKNOWN")
    var currentHeightMm = mutableStateOf<String?>(null)

    // Config values (Null until read from connected device)
    var autoEnable = mutableStateOf<Boolean?>(null)
    var emergencyStop = mutableStateOf<Boolean?>(null)
    var autoPeriodSec = mutableStateOf<String?>(null)
    var heightMaxMm = mutableStateOf<String?>(null)
    var speedMmSec = mutableStateOf<String?>(null)
    var skitteringTrack = mutableStateOf<String?>(null)
    var jumpingTrack = mutableStateOf<String?>(null)
    var danglingTrack = mutableStateOf<String?>(null)
    var resettingTrack = mutableStateOf<String?>(null)
    var randomEnable = mutableStateOf<Boolean?>(null)
    var muteEnable = mutableStateOf<Boolean?>(null)

    private var bluetoothGatt: BluetoothGatt? = null
    private val TARGET_DEVICE_NAME = "spider_launcher"

    private var pollingJob: Job? = null
    private val scope = CoroutineScope(Dispatchers.Main + SupervisorJob())

    // UUID mappings from firmware source
    private val SERVICE_UUID = UUID.fromString("0000FFE0-0000-1000-8000-00805F9B34FB")
    private val CHAR_UUIDS = mapOf(
        "LAUNCH_NOW" to UUID.fromString("0000FFE1-0000-1000-8000-00805F9B34FB"),
        "UP" to UUID.fromString("0000FFE2-0000-1000-8000-00805F9B34FB"),
        "DOWN" to UUID.fromString("0000FFE3-0000-1000-8000-00805F9B34FB"),
        "THIS_IS_GROUND_LEVEL" to UUID.fromString("0000FFE4-0000-1000-8000-00805F9B34FB"),
        "THIS_IS_HEIGHT_MAX" to UUID.fromString("0000FFE5-0000-1000-8000-00805F9B34FB"),
        "RESET_TO_DEFAULTS" to UUID.fromString("0000FFE6-0000-1000-8000-00805F9B34FB"),
        "EMERGENCY_STOP" to UUID.fromString("0000FFE7-0000-1000-8000-00805F9B34FB"),
        "PLAY_TRACK" to UUID.fromString("0000FFE8-0000-1000-8000-00805F9B34FB"),
        "AUTO" to UUID.fromString("0000FFE9-0000-1000-8000-00805F9B34FB"),
        "AUTO_PERIOD_SEC" to UUID.fromString("0000FFEA-0000-1000-8000-00805F9B34FB"),
        "HEIGHT_MAX_MM" to UUID.fromString("0000FFEB-0000-1000-8000-00805F9B34FB"),
        "SPEED_MM_PER_SECOND" to UUID.fromString("0000FFEC-0000-1000-8000-00805F9B34FB"),
        "SKITTERING_TRACK" to UUID.fromString("0000FFED-0000-1000-8000-00805F9B34FB"),
        "JUMPING_TRACK" to UUID.fromString("0000FFEE-0000-1000-8000-00805F9B34FB"),
        "DANGLING_TRACK" to UUID.fromString("0000FFEF-0000-1000-8000-00805F9B34FB"),
        "RELAXING_TRACK" to UUID.fromString("0000FFF0-0000-1000-8000-00805F9B34FB"),
        "RANDOM" to UUID.fromString("0000FFF1-0000-1000-8000-00805F9B34FB"),
        "MUTE" to UUID.fromString("0000FFF2-0000-1000-8000-00805F9B34FB"),
        "STATE" to UUID.fromString("0000FFF3-0000-1000-8000-00805F9B34FB"),
        "HEIGHT_CURRENT_MM" to UUID.fromString("0000FFF4-0000-1000-8000-00805F9B34FB")
    )

    // Unified list controlling the exact order characteristics are read/updated
    private val ALL_CHARACTERISTIC_KEYS = listOf(
        "EMERGENCY_STOP",
        "STATE",
        "HEIGHT_CURRENT_MM",
        "AUTO",
        "RANDOM",
        "MUTE",
        "SPEED_MM_PER_SECOND",
        "AUTO_PERIOD_SEC",
        "HEIGHT_MAX_MM",
        "SKITTERING_TRACK",
        "JUMPING_TRACK",
        "DANGLING_TRACK",
        "RELAXING_TRACK"
    )

    private val launcherStates = listOf(
        "NULL", "HEIGHT_UNKNOWN", "READY", "STEP_UP", "STEP_DOWN",
        "RUN", "RUNNING_SKITTERING", "RUNNING_JUMPING", "RUNNING_DANGLING",
        "RUNNING_RESETTING", "HALT"
    )

    // Queue for sequential GATT reads
    private val readQueue = LinkedList<UUID>()
    private var isReading = false

    private val requestPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        if (permissions.all { it.value }) {
            startBleScan()
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            MaterialTheme {
                Surface(modifier = Modifier.fillMaxSize(), color = MaterialTheme.colorScheme.background) {
                    MainScreen(
                        connectionStatus = connectionStatus.value,
                        deviceState = deviceStateName.value,
                        currentHeight = currentHeightMm.value,
                        emergencyStop = emergencyStop.value,
                        autoEnable = autoEnable.value,
                        autoPeriodSec = autoPeriodSec.value,
                        heightMaxMm = heightMaxMm.value,
                        speedMmSec = speedMmSec.value,
                        skitteringTrack = skitteringTrack.value,
                        jumpingTrack = jumpingTrack.value,
                        danglingTrack = danglingTrack.value,
                        resettingTrack = resettingTrack.value,
                        randomEnable = randomEnable.value,
                        muteEnable = muteEnable.value,
                        onSendCommand = { name -> writeCommand(name) },
                        onSendValueCommand = { name, value -> writeValueCommand(name, value) }
                    )
                }
            }
        }
        checkPermissionsAndScan()
    }

    override fun onDestroy() {
        super.onDestroy()
        pollingJob?.cancel()
        try {
            bluetoothGatt?.disconnect()
            bluetoothGatt?.close()
        } catch (_: SecurityException) {}
        bluetoothGatt = null
    }

    private fun checkPermissionsAndScan() {
        val requiredPermissions = if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.BLUETOOTH_CONNECT
            )
        } else {
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION
            )
        }

        if (requiredPermissions.all { ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED }) {
            startBleScan()
        } else {
            requestPermissionLauncher.launch(requiredPermissions)
        }
    }

    @SuppressLint("MissingPermission")
    private fun startBleScan() {
        val scanner = bluetoothAdapter?.bluetoothLeScanner
        if (scanner == null) {
            connectionStatus.value = "Bluetooth Le Scanner unavailable"
            return
        }

        connectionStatus.value = "Scanning for $TARGET_DEVICE_NAME..."
        val scanCallback = object : ScanCallback() {
            override fun onScanResult(callbackType: Int, result: ScanResult?) {
                val device = result?.device
                val name = device?.name ?: result?.scanRecord?.deviceName
                if (name == TARGET_DEVICE_NAME) {
                    scanner.stopScan(this)
                    connectionStatus.value = "Connecting..."
                    device?.connectGatt(applicationContext, false, gattCallback)
                }
            }
        }
        try {
            scanner.startScan(scanCallback)
        } catch (e: SecurityException) {
            connectionStatus.value = "Permission denied for scan"
        }
    }

    @SuppressLint("PrivateApi")
    private fun refreshGattCache(gatt: BluetoothGatt?): Boolean {
        return try {
            val refreshMethod = gatt?.javaClass?.getMethod("refresh")
            refreshMethod?.invoke(gatt) as? Boolean ?: false
        } catch (e: Exception) {
            false
        }
    }

    private val gattCallback = object : BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                bluetoothGatt = gatt
                connectionStatus.value = "Connected"
                refreshGattCache(gatt)
                gatt.discoverServices()
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                connectionStatus.value = "Disconnected (Re-scanning...)"
                pollingJob?.cancel()
                try {
                    gatt.close()
                } catch (_: SecurityException) {}
                bluetoothGatt = null
                readQueue.clear()
                isReading = false

                android.os.Handler(mainLooper).postDelayed({
                    startBleScan()
                }, 1000)
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                // Populate read queue using the unified list
                readQueue.clear()
                ALL_CHARACTERISTIC_KEYS.forEach { key ->
                    CHAR_UUIDS[key]?.let { readQueue.add(it) }
                }
                processNextRead()

                // Start periodic polling loop for STATE and conditional height updates
                startPeriodicPolling()
            }
        }

        override fun onCharacteristicRead(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            value: ByteArray,
            status: Int
        ) {
            isReading = false
            if (status == BluetoothGatt.GATT_SUCCESS) {
                val intVal = parseLittleEndianInt(value)
                val uuid = characteristic.uuid

                when (uuid) {
                    CHAR_UUIDS["EMERGENCY_STOP"] -> emergencyStop.value = intVal != 0
                    CHAR_UUIDS["STATE"] -> {
                        if (intVal in launcherStates.indices) {
                            deviceStateName.value = launcherStates[intVal]
                        }
                    }
                    CHAR_UUIDS["HEIGHT_CURRENT_MM"] -> {
                        currentHeightMm.value = if (intVal == -1) "Unknown" else "$intVal mm"
                    }
                    CHAR_UUIDS["AUTO"] -> autoEnable.value = intVal != 0
                    CHAR_UUIDS["AUTO_PERIOD_SEC"] -> autoPeriodSec.value = intVal.toString()
                    CHAR_UUIDS["HEIGHT_MAX_MM"] -> heightMaxMm.value = intVal.toString()
                    CHAR_UUIDS["SPEED_MM_PER_SECOND"] -> speedMmSec.value = intVal.toString()
                    CHAR_UUIDS["SKITTERING_TRACK"] -> skitteringTrack.value = intVal.toString()
                    CHAR_UUIDS["JUMPING_TRACK"] -> jumpingTrack.value = intVal.toString()
                    CHAR_UUIDS["DANGLING_TRACK"] -> danglingTrack.value = intVal.toString()
                    CHAR_UUIDS["RELAXING_TRACK"] -> resettingTrack.value = intVal.toString()
                    CHAR_UUIDS["RANDOM"] -> randomEnable.value = intVal != 0
                    CHAR_UUIDS["MUTE"] -> muteEnable.value = intVal != 0
                }
            }
            processNextRead()
        }
    }

    private fun startPeriodicPolling() {
        pollingJob?.cancel()
        pollingJob = scope.launch {
            while (isActive && bluetoothGatt != null) {
                // Re-read current state every 3 seconds
                queueRead(CHAR_UUIDS["STATE"])

                // Re-read current height every 3 seconds if state is RUNNING_*
                if (deviceStateName.value.startsWith("RUNNING_")) {
                    queueRead(CHAR_UUIDS["HEIGHT_CURRENT_MM"])
                }

                delay(3000)
            }
        }
    }

    private fun queueRead(uuid: UUID?) {
        uuid?.let {
            if (!readQueue.contains(it)) {
                readQueue.add(it)
            }
            processNextRead()
        }
    }

    private fun queueAllSettingsRead() {
        // Uses the exact same unified order list
        ALL_CHARACTERISTIC_KEYS.forEach { key ->
            CHAR_UUIDS[key]?.let { readQueue.add(it) }
        }
        processNextRead()
    }

    @SuppressLint("MissingPermission")
    private fun processNextRead() {
        if (isReading || readQueue.isEmpty()) return
        val gatt = bluetoothGatt ?: return
        val nextUuid = readQueue.poll() ?: return
        val service = gatt.getService(SERVICE_UUID)
        val characteristic = service?.getCharacteristic(nextUuid)

        if (characteristic != null) {
            isReading = true
            val success = gatt.readCharacteristic(characteristic)
            if (!success) {
                isReading = false
                processNextRead()
            }
        } else {
            processNextRead()
        }
    }

    private fun parseLittleEndianInt(bytes: ByteArray): Int {
        if (bytes.isEmpty()) return 0
        return when (bytes.size) {
            1 -> bytes[0].toInt() and 0xFF
            2 -> ((bytes[1].toInt() and 0xFF) shl 8) or (bytes[0].toInt() and 0xFF)
            4 -> ((bytes[3].toInt() and 0xFF) shl 24) or
                    ((bytes[2].toInt() and 0xFF) shl 16) or
                    ((bytes[1].toInt() and 0xFF) shl 8) or
                    (bytes[0].toInt() and 0xFF)
            else -> bytes[0].toInt()
        }
    }

    // Interrupt ongoing bulk reads to prioritize immediate actions
    private fun interruptAndPrepareForAction() {
        val wasReadingOrQueued = isReading || readQueue.isNotEmpty()
        readQueue.clear()
        isReading = false

        if (wasReadingOrQueued) {
            // Restart settings sync later if we interrupted an ongoing batch read
            android.os.Handler(mainLooper).postDelayed({
                if (readQueue.isEmpty()) {
                    queueAllSettingsRead()
                }
            }, 3000)
        }
    }

    @SuppressLint("MissingPermission")
    private fun writeCommand(charKey: String) {
        val gatt = bluetoothGatt ?: return
        val service = gatt.getService(SERVICE_UUID) ?: return
        val uuid = CHAR_UUIDS[charKey] ?: return
        val characteristic = service.getCharacteristic(uuid) ?: return

        interruptAndPrepareForAction()

        characteristic.value = byteArrayOf()
        characteristic.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
        gatt.writeCharacteristic(characteristic)

        // Post-command trigger actions
        if (charKey == "UP" || charKey == "DOWN" || charKey == "THIS_IS_GROUND_LEVEL") {
            android.os.Handler(mainLooper).postDelayed({
                queueRead(CHAR_UUIDS["HEIGHT_CURRENT_MM"])
            }, 500)
        } else if (charKey == "RESET_TO_DEFAULTS") {
            android.os.Handler(mainLooper).postDelayed({
                queueAllSettingsRead()
            }, 1000)
        }
    }

    @SuppressLint("MissingPermission")
    private fun writeValueCommand(charKey: String, intValue: Int) {
        val gatt = bluetoothGatt ?: return
        val service = gatt.getService(SERVICE_UUID) ?: return
        val uuid = CHAR_UUIDS[charKey] ?: return
        val characteristic = service.getCharacteristic(uuid) ?: return

        interruptAndPrepareForAction()

        val byteArray = byteArrayOf(
            (intValue and 0xFF).toByte(),
            ((intValue ushr 8) and 0xFF).toByte(),
            ((intValue ushr 16) and 0xFF).toByte(),
            ((intValue ushr 24) and 0xFF).toByte()
        )
        characteristic.value = byteArray
        characteristic.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
        gatt.writeCharacteristic(characteristic)

        // Trigger a re-read of this specific setting after a short delay
        android.os.Handler(mainLooper).postDelayed({
            queueRead(uuid)
        }, 500)
    }
}

@Composable
fun HoldingActionButton(
    text: String,
    isConnected: Boolean,
    containerColor: Color = MaterialTheme.colorScheme.primary,
    contentColor: Color = MaterialTheme.colorScheme.onPrimary,
    modifier: Modifier = Modifier,
    onActionStart: () -> Unit,
    onActionStop: () -> Unit = {}
) {
    val coroutineScope = rememberCoroutineScope()
    var job by remember { mutableStateOf<Job?>(null) }

    // Use primary with standard 12% alpha when disabled to match standard Material Buttons
    val backgroundColor = if (isConnected) containerColor else containerColor.copy(alpha = 0.12f)
    val textColor = if (isConnected) contentColor else MaterialTheme.colorScheme.onSurface.copy(alpha = 0.38f)

    Box(
        contentAlignment = Alignment.Center,
        modifier = modifier
            .height(40.dp)
            .background(
                color = backgroundColor,
                shape = MaterialTheme.shapes.extraLarge
            )
            .pointerInput(isConnected) {
                if (!isConnected) return@pointerInput
                detectTapGestures(
                    onPress = {
                        onActionStart()
                        job = coroutineScope.launch {
                            while (isActive) {
                                delay(150)
                                onActionStart()
                            }
                        }
                        try {
                            awaitRelease()
                        } finally {
                            job?.cancel()
                            job = null
                            onActionStop()
                        }
                    }
                )
            }
            .padding(horizontal = 12.dp)
    ) {
        Text(
            text = text,
            style = MaterialTheme.typography.labelLarge,
            color = textColor
        )
    }
}

@Composable
fun SectionDivider(title: String) {
    Box(
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 4.dp),
        contentAlignment = Alignment.Center
    ) {
        HorizontalDivider(
            modifier = Modifier.fillMaxWidth(),
            thickness = 1.dp,
            color = MaterialTheme.colorScheme.outlineVariant
        )
        Surface(
            color = MaterialTheme.colorScheme.background, // Matches screen background to mask the line behind the text
            modifier = Modifier.padding(horizontal = 8.dp)
        ) {
            Text(
                text = title,
                style = MaterialTheme.typography.titleSmall,
                color = MaterialTheme.colorScheme.primary,
                modifier = Modifier.padding(horizontal = 4.dp)
            )
        }
    }
}

@Composable
fun SwitchParamRow(
    label: String,
    value: Boolean?,
    isConnected: Boolean,
    onSave: (Int) -> Unit,
    confirmAndExecute: (String, String, () -> Unit) -> Unit
) {
    if (value != null) {
        var fieldVal by remember(value) { mutableStateOf(value) }
        Row(
            verticalAlignment = Alignment.CenterVertically,
            modifier = Modifier
                .fillMaxWidth()
                .heightIn(min = 20.dp)
                .padding(vertical = 0.dp)
        ) {
            Text(
                text = label,
                style = MaterialTheme.typography.bodyMedium,
                color = if (isConnected) MaterialTheme.colorScheme.onSurface else MaterialTheme.colorScheme.onSurface.copy(alpha = 0.38f)
            )
            Spacer(modifier = Modifier.weight(1f))
            Switch(
                checked = fieldVal,
                enabled = isConnected,
                onCheckedChange = { newVal ->
                    confirmAndExecute("Toggle $label", "Are you sure you want to change $label?") {
                        fieldVal = newVal
                        onSave(if (newVal) 1 else 0)
                    }
                },
                modifier = Modifier
                    .height(36.dp)
                    .padding(4.dp)
            )
        }
    } else {
        Text(
            "$label: Connecting / Reading...",
            style = MaterialTheme.typography.bodySmall,
            color = Color.Gray,
            modifier = Modifier.padding(vertical = 0.dp)
        )
    }
}

@Composable
fun CompactParamRow(
    label: String,
    value: String?,
    isConnected: Boolean,
    onSave: (Int) -> Unit,
    confirmAndExecute: (String, String, () -> Unit) -> Unit
) {
    var textValue by remember(value) { mutableStateOf(value ?: "") }
    val focusManager = LocalFocusManager.current

    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 4.dp),
        horizontalArrangement = Arrangement.spacedBy(8.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(
            text = label,
            modifier = Modifier.weight(1f),
            style = MaterialTheme.typography.bodyMedium,
            color = if (isConnected) MaterialTheme.colorScheme.onSurface else MaterialTheme.colorScheme.onSurface.copy(alpha = 0.38f)
        )

        BasicTextField(
            value = textValue,
            onValueChange = { textValue = it },
            enabled = isConnected,
            singleLine = true,
            textStyle = MaterialTheme.typography.bodyMedium.copy(
                color = if (isConnected) MaterialTheme.colorScheme.onSurface else MaterialTheme.colorScheme.onSurface.copy(alpha = 0.38f)
            ),
            modifier = Modifier
                .width(70.dp)
                .background(MaterialTheme.colorScheme.surfaceVariant, MaterialTheme.shapes.small)
                .padding(horizontal = 8.dp, vertical = 6.dp)
                .onFocusChanged { focusState ->
                    // Revert to original value if focus is lost and changes weren't saved
                    if (!focusState.isFocused) {
                        textValue = value ?: ""
                    }
                }
        )

        Button(
            onClick = {
                textValue.toIntOrNull()?.let { intVal ->
                    confirmAndExecute("Update $label", "Are you sure you want to update $label to $intVal?") {
                        focusManager.clearFocus()
                        onSave(intVal)
                    }
                }
            },
            enabled = isConnected && textValue.toIntOrNull() != null && value != null,
            contentPadding = PaddingValues(horizontal = 12.dp, vertical = 4.dp),
            modifier = Modifier.height(36.dp)
        ) {
            Text("Save", style = MaterialTheme.typography.labelMedium)
        }
    }
}

@Composable
fun MainScreen(
    connectionStatus: String,
    deviceState: String,
    currentHeight: String?,
    emergencyStop: Boolean?,
    autoEnable: Boolean?,
    autoPeriodSec: String?,
    heightMaxMm: String?,
    speedMmSec: String?,
    skitteringTrack: String?,
    jumpingTrack: String?,
    danglingTrack: String?,
    resettingTrack: String?,
    randomEnable: Boolean?,
    muteEnable: Boolean?,
    onSendCommand: (String) -> Unit,
    onSendValueCommand: (String, Int) -> Unit
) {
    val scrollState = rememberScrollState()
    val focusManager = LocalFocusManager.current
    val isConnected = (connectionStatus == "Connected")
    val isEStopActive = emergencyStop == true
    val areMotorsEnabled = isConnected && !isEStopActive
    var showDialog by remember { mutableStateOf(false) }
    var dialogTitle by remember { mutableStateOf("") }
    var dialogMessage by remember { mutableStateOf("") }
    var pendingAction by remember { mutableStateOf<(() -> Unit)?>(null) }

    fun confirmAndExecute(title: String, message: String, action: () -> Unit) {
        dialogTitle = title
        dialogMessage = message
        pendingAction = action
        showDialog = true
    }

    if (showDialog) {
        AlertDialog(
            onDismissRequest = { showDialog = false },
            title = { Text(dialogTitle) },
            text = { Text(dialogMessage) },
            confirmButton = {
                TextButton(onClick = {
                    showDialog = false
                    pendingAction?.invoke()
                }) {
                    Text("Confirm")
                }
            },
            dismissButton = {
                TextButton(onClick = { showDialog = false }) {
                    Text("Cancel")
                }
            }
        )
    }

    Column(
        modifier = Modifier
            .fillMaxSize()
            .pointerInput(Unit) {
                detectTapGestures(onTap = {
                    focusManager.clearFocus() // Clears focus when tapping anywhere on the background
                })
            }
            .padding(8.dp)
            .statusBarsPadding()
            .verticalScroll(scrollState),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text("Spider Launcher Control", style = MaterialTheme.typography.headlineMedium)
        Spacer(modifier = Modifier.height(8.dp))

        // Connection & Status Card
        Card(
            modifier = Modifier.fillMaxWidth(),
            colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant)
        ) {
            Column(modifier = Modifier.padding(4.dp)) {
                Text("Connection: $connectionStatus", style = MaterialTheme.typography.bodyLarge)
                Text("Device State: $deviceState", style = MaterialTheme.typography.titleMedium, color = Color.Red)
                Text("Current Height: ${currentHeight ?: "--"}", style = MaterialTheme.typography.bodyMedium)
                if (isEStopActive) {
                    Text("EMERGENCY STOP ENGAGED", style = MaterialTheme.typography.titleMedium, color = MaterialTheme.colorScheme.error)
                }
            }
        }

        Spacer(modifier = Modifier.height(16.dp))

        // Launch and E-Stop Row
        Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceEvenly) {
            Button(
                onClick = { onSendCommand("LAUNCH_NOW") },
                enabled = areMotorsEnabled,
                colors = ButtonDefaults.buttonColors(
                    containerColor = MaterialTheme.colorScheme.primary,
                    contentColor = MaterialTheme.colorScheme.onPrimary,
                    disabledContainerColor = MaterialTheme.colorScheme.primary.copy(alpha = 0.12f),
                    disabledContentColor = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.38f)
                ),
                elevation = ButtonDefaults.buttonElevation(defaultElevation = 4.dp),
                modifier = Modifier.weight(1f).padding(horizontal = 4.dp)
            ) {
                Text("Launch")
            }

            // Emergency Stop Button: Uses primary color when disengaged so it matches Launch/Steps, turns bright red only when active
            val eStopContainerColor = if (isEStopActive) MaterialTheme.colorScheme.error else MaterialTheme.colorScheme.primary
            val eStopContentColor = if (isEStopActive) MaterialTheme.colorScheme.onError else MaterialTheme.colorScheme.onPrimary

            Button(
                onClick = {
                    if (isEStopActive) {
                        confirmAndExecute(
                            "Clear E-Stop",
                            "Are you sure you want to clear the Emergency Stop?"
                        ) {
                            onSendValueCommand("EMERGENCY_STOP", 0)
                        }
                    } else {
                        onSendValueCommand("EMERGENCY_STOP", 1)
                    }
                },
                enabled = isConnected, // Only disabled if disconnected entirely, stays active even if e-stop is on so you can clear it
                colors = ButtonDefaults.buttonColors(
                    containerColor = eStopContainerColor,
                    contentColor = eStopContentColor,
                    disabledContainerColor = MaterialTheme.colorScheme.primary.copy(alpha = 0.12f),
                    disabledContentColor = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.38f)
                ),
                elevation = ButtonDefaults.buttonElevation(defaultElevation = if (isEStopActive) 8.dp else 4.dp),
                modifier = Modifier.weight(1f).padding(horizontal = 4.dp)
            ) {
                Text(if (isEStopActive) "E-STOP ACTIVE" else "EMERGENCY STOP", style = MaterialTheme.typography.labelLarge)
            }
        }

        // Step up / Step down row
        Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceEvenly) {
            HoldingActionButton(
                text = "Step up",
                isConnected = areMotorsEnabled,
                containerColor = MaterialTheme.colorScheme.primary,
                contentColor = MaterialTheme.colorScheme.onPrimary,
                modifier = Modifier.weight(1f).padding(horizontal = 4.dp),
                onActionStart = { onSendCommand("UP") }
            )
            HoldingActionButton(
                text = "Step down",
                isConnected = areMotorsEnabled,
                containerColor = MaterialTheme.colorScheme.primary,
                contentColor = MaterialTheme.colorScheme.onPrimary,
                modifier = Modifier.weight(1f).padding(horizontal = 4.dp),
                onActionStart = { onSendCommand("DOWN") }
            )
        }

        // Guarded Position / Reset Controls
        SectionDivider("Calibration")
        Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceEvenly) {
            Button(
                onClick = {
                    confirmAndExecute("Set Ground Level", "Are you sure you want to set the current position as ground level?") {
                        onSendCommand("THIS_IS_GROUND_LEVEL")
                    }
                },
                enabled = isConnected
            ) { Text("This is ground level") }

            Button(
                onClick = {
                    confirmAndExecute("Set Max Height", "Are you sure you want to record the current height as the new maximum height?") {
                        onSendCommand("THIS_IS_HEIGHT_MAX")
                    }
                },
                enabled = isConnected
            ) { Text("This is max height") }
        }

        // Settings & Configurations
        SectionDivider("Settings")
        Button(
            onClick = {
                confirmAndExecute("Reset to Defaults", "Are you sure you want to reset all device settings and NVRAM to defaults?") {
                    onSendCommand("RESET_TO_DEFAULTS")
                }
            },
            enabled = isConnected,
            colors = ButtonDefaults.buttonColors(containerColor = MaterialTheme.colorScheme.error)
        ) {
            Text("Reset to Defaults")
        }

        Spacer(modifier = Modifier.height(4.dp))

        // Unified Switch Rows with tight padding
        SwitchParamRow(
            label = "Auto Mode",
            value = autoEnable,
            isConnected = isConnected,
            onSave = { onSendValueCommand("AUTO", it) },
            confirmAndExecute = ::confirmAndExecute
        )
        SwitchParamRow(
            label = "Random Enable",
            value = randomEnable,
            isConnected = isConnected,
            onSave = { onSendValueCommand("RANDOM", it) },
            confirmAndExecute = ::confirmAndExecute
        )
        SwitchParamRow(
            label = "Mute Enable",
            value = muteEnable,
            isConnected = isConnected,
            onSave = { onSendValueCommand("MUTE", it) },
            confirmAndExecute = ::confirmAndExecute
        )

        CompactParamRow(
            label = "Speed (mm/s)",
            value = speedMmSec,
            isConnected = isConnected,
            onSave = { onSendValueCommand("SPEED_MM_PER_SECOND", it) },
            confirmAndExecute = ::confirmAndExecute
        )
        CompactParamRow(
            label = "Auto Period (sec)",
            value = autoPeriodSec,
            isConnected = isConnected,
            onSave = { onSendValueCommand("AUTO_PERIOD_SEC", it) },
            confirmAndExecute = ::confirmAndExecute
        )
        CompactParamRow(
            label = "Max Height (mm)",
            value = heightMaxMm,
            isConnected = isConnected,
            onSave = { onSendValueCommand("HEIGHT_MAX_MM", it) },
            confirmAndExecute = ::confirmAndExecute
        )
        CompactParamRow(
            label = "Skittering Track",
            value = skitteringTrack,
            isConnected = isConnected,
            onSave = { onSendValueCommand("SKITTERING_TRACK", it) },
            confirmAndExecute = ::confirmAndExecute
        )
        CompactParamRow(
            label = "Jumping Track",
            value = jumpingTrack,
            isConnected = isConnected,
            onSave = { onSendValueCommand("JUMPING_TRACK", it) },
            confirmAndExecute = ::confirmAndExecute
        )
        CompactParamRow(
            label = "Dangling Track",
            value = danglingTrack,
            isConnected = isConnected,
            onSave = { onSendValueCommand("DANGLING_TRACK", it) },
            confirmAndExecute = ::confirmAndExecute
        )
        CompactParamRow(
            label = "Resetting Track",
            value = resettingTrack,
            isConnected = isConnected,
            onSave = { onSendValueCommand("RESETTING_TRACK", it) },
            confirmAndExecute = ::confirmAndExecute
        )

        Spacer(modifier = Modifier.height(24.dp))
    }
}