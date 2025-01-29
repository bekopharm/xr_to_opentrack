<?php

declare(ticks=1);

/**
 * Run with `php xr_to_opentrack_poc.php`
 * 
 * Written with php 8.2 - YMMV
 * 
 * This is a proof-of-concept reading IMU data from xr_driver
 * e.g. for Viture Pro glasses on Linux PC. This data is
 * provided via IPC shared memory by Breezy Desktop and
 * forwarded via UDP to OpenTrack (or directly to a game that
 * features head tracking via UDP).
 * 
 * This snippet is released under CC0-1.0
 *             Creative Commons Zero v1.0 Universal
 *                  "No Rights Reserved"
 * @see https://creativecommons.org/public-domain/cc0/
 */

/**
 * OpenTrack UDP port (INPUT source for UDP over network)
 */
$port = 4242;
$ip = "127.0.0.1";

// make sure to clean up on termination
function sig_handler($sig)
{
    if (isset($GLOBALS["fp"])) {
        echo PHP_EOL . "Got SIG $sig - closing shop." . PHP_EOL;
        fclose($GLOBALS["fp"]);
    }
    exit;
}

pcntl_signal(SIGINT,  "sig_handler");
pcntl_signal(SIGTERM, "sig_handler");
pcntl_signal(SIGHUP,  "sig_handler");

// created by the xr_driver for shared memory access
$file = '/tmp/shader_runtime_';
$file .= 'imu_quat_data';
// Seems like PHP ftok is different from C++ ftok
// Breezy uses proj_id (int)0, php requires that as ASCII for ftok => use chr() to get ASCII code
// If this doesn't work: See `ipcs -m` for available shared memory. 
// You're looking for one with about 64 bytes.
$id = ftok($file, chr("0"));

if ($id === -1) {
    echo "[DEBUG] Could not find key for shared memory access on $file from xrDriver - are the glasses connected?" . PHP_EOL;
    die(-1);
}

echo "[DEBUG] ipc_key, got key " . dechex($id) . " for path $file" . PHP_EOL;

# open shared memory block read-only
$memory = shmop_open($id, 'a', 0, 0);

if ($memory === false) {
    echo "[DEBUG] ipc_key, could not open shared memory key " . dechex($id) . PHP_EOL;
    die(-1);
}

$size = shmop_size($memory);
echo "[DEBUG] ipc_key, opened shared memory segment with key " . dechex($id) . " with " . $size . " bytes" . PHP_EOL;

// I won't pretend I understand Euler. This is ~~magic~~math that
// translates quaternion from IMU to pitch/roll/jaw
function quaternion_to_euler($q)
{
    $euler = (object) [
        'pitch' => 0,
        'roll' => 0,
        'jaw' => 0,
    ];

    // roll (x-axis)
    $sinr_cosp = 2 * ($q['w'] * $q['x'] + $q['y'] * $q['z']);
    $cosr_cosp = 1 - 2 * ($q['x'] * $q['x'] + $q['y'] * $q['y']);
    $euler->roll = atan2($sinr_cosp, $cosr_cosp) * (180.0 / M_PI);

    // pitch (y-axis rotation)
    $sinp = ($q['w'] * $q['y'] + $q['z'] * $q['x']);
    $euler->pitch = asin($sinp) * (180.0 / M_PI);

    // yaw (z-axis rotation)
    $siny_cosp = 2 * ($q['w'] * $q['z'] + $q['x'] * $q['y']);
    $cosy_cosp = 1 - 2 * ($q['y'] * $q['y'] + $q['z'] * $q['z']);
    $euler->yaw = atan2($siny_cosp, $cosy_cosp) * (180.0 / M_PI);

    return $euler;
}

$fp = pfsockopen("udp://$ip", $port, $errno, $errstr);
if (!$fp || $errno) {
    echo "[DEBUG] Could not start UDP sender: $errno - $errstr" . PHP_EOL;
    die(-1);
} else {
    echo PHP_EOL . "⚠ Starting UDP write to udp://$ip:$port ⚠" . PHP_EOL;
    echo "⚠ Set your OpenTrack input to 'UDP over network' and hit START ⚠" . PHP_EOL;
    echo "⚠ START also calibrates the input so hold still! ⚠" . PHP_EOL . PHP_EOL;
    echo "Not getting data? Check manually with netcat: `nc -ul -p 4242 | xxd`" . PHP_EOL . PHP_EOL;
}

$framenumber = 0;
while ($memory) {
    $input  = shmop_read($memory, 0, $size);
    // https://www.php.net/manual/en/function.pack.php
    $format = [
        'fx',
        'fy',
        'fz',
        'fw',
        'fstage_1_quat_x',
        'fstage_1_quat_y',
        'fstage_1_quat_z',
        'fstage_1_quat_w',
        'fstage_2_quat_x',
        'fstage_2_quat_y',
        'fstage_2_quat_z',
        'fstage_2_quat_w',
        'ftimestamp_ms',
        'fstage_1_ts',
        'fstage_2_ts',

    ];

    $imu_data = unpack(implode("/", $format), $input);

    // Debug:
    /*
    $imu_x = number_format($imu_data["x"], 3);
    $imu_y = number_format($imu_data["y"], 3);
    $imu_z = number_format($imu_data["z"], 3);
    $imu_w = number_format($imu_data["w"], 3);

    echo " x: $imu_x y: $imu_y z: $imu_z w: $imu_w" . PHP_EOL;
    */

    $packed = "";
    $euler = quaternion_to_euler($imu_data);

    // TODO: quaternion probably need some maths applied as well - who knows?
    $mapping = [
        'x' => $imu_data['x'] * 10,
        'y' => $imu_data['y'] * 10,
        'z' => $imu_data['z'] * 10,
        'yaw' => $euler->yaw,
        'pitch' => $euler->pitch,
        'roll' => $euler->roll
    ];

    // what's supposed to happen: OpenTrack expects UDP data in the
    // format of 6 doubles (x,y,z,yaw,pitch,roll) followed by a long
    // for a (unspecified) framenumber (probably for order which is not
    // guaranteed with UDP)
    foreach ($mapping as $key => $value) {
        $packed = $packed . pack('d', $value);
    }

    $packed = $packed . pack('L', $framenumber);
    $framenumber++;

    // This should make for 52 bytes
    if (!empty($packed)) {
        $byte = @fwrite($fp, $packed, 52);
        # echo "Written bytes: $byte" . PHP_EOL;
    }

    // reading twice a second should be enough for smooth tracking
    $sleep = sleep(0.5);
}
