# Flashing the Pixhawk 6X firmware

How to build and flash the pinned PX4 firmware onto the X500's Pixhawk 6X with `scripts/firmware.py`, and how to change the pin. Written from the bench flash of 2026-11-04 (v1.17.0 to v1.15.4). Reference for the options is `python3 scripts/firmware.py --help`; the short version is the README's "Firmware" section.

## When to flash

Flash when `scripts/firmware.py --check` (or the `firmware` row of `host_setup.py --guide`) says the board's PX4 version does not match `firmware/versions.json`. A mismatch is not cosmetic: `px4_msgs` describes one PX4 version, and a board running another publishes different topics or none that the driver reads (on the v1.17.0 board `flight.armed` and `flight.mode` stayed empty). Either reflash the board, or re-pin `px4_msgs`; the check does not choose for you.

Flashing replaces the application only. The bootloader is not touched, and the tool never writes parameters.

## Prerequisites

- **You are at the vehicle, props off, battery disconnected.** USB power alone runs the flight controller; that says nothing about the motors. A flash is never unattended.
- QGroundControl closed. No `mavros_node`, `MicroXRCEAgent` or `px4_control_node` running (stop the driver in console first). The tool checks all three and the USB port.
- The board is the only `3185:0035` device on the host and is in application mode.
- A built artifact (below) whose sha256 matches the one recorded at build time.
- `fuser` (psmisc) or `lsof`, `pyserial`, and for the version read `pymavlink`: `bash scripts/deps/px4.sh` installs them.

## 1. What is pinned

`firmware/versions.json` is the single pairing file. It records PX4-Autopilot `v1.15.4` (commit `99c40407ff`), `px4_msgs` branch `release/1.15` at `a1045ec`, the board target `px4_fmu-v6x_default` (board_id 53), USB `3185:0035`, the toolchain image, and the 7 PX4 submodules the board needs. `firmware/PX4-Autopilot` is the shallow submodule at that commit. Fetch it and the messages package:

```bash
git submodule update --init --depth 1 firmware/PX4-Autopilot src/px4_msgs
```

Do not hand-edit the firmware tag without changing `px4_msgs` to match; the `firmware` row compares the two.

## 2. Back up the parameters first

```bash
python3 scripts/firmware.py --backup-params ~/px4_before.params --diff /path/to/cpsl_px4_v15_4_params.params
```

This is read-only. It starts its own temporary MAVROS on the USB port (so the port must be free), writes a QGC-format file, and stops it again. It refuses to overwrite a file. `--diff` prints how many parameters are identical, differ, or exist on only one side. `--diff-params A B` compares any two files.

The flash command refuses without a backup made by this tool, from this board, no more than an hour old (`--backup-max-age`).

## 3. Build

```bash
python3 scripts/firmware.py --build
```

This only **prints** the host commands; run them yourself, in order: create `firmware/out/` (gitignored), `git submodule update --init --depth 1` for the 7 listed submodules, fetch the one NuttX release tag (`nuttx-12.12.0`), the Docker build in `px4io/px4-dev-nuttx-focal:2022-08-12`, and `firmware.py --record`, which stores the artifact's sha256 beside it. The tag step matters: a depth-1 NuttX has no tags, and PX4's version-header script fails without one (`IndexError` in `px_update_git_header.py`).

Expect about 5 minutes on a first run (image pull about 70 s, submodules about 70 s, build about 90 s), so run it detached. Results of the 2026-11-04 build: `firmware/out/px4_fmu-v6x_default/px4_fmu-v6x_default.px4`, 1,823,112 bytes, sha256 `871616fb...6e32`. PX4's own build also initialises a few nested submodules (uavcan dsdl, nlohmann_json, heatshrink, pymavlink); that is expected.

Do not install PX4's toolchain on the host: `Tools/setup/ubuntu.sh` uses sudo and is the user's call. See PX4's [building PX4, NuttX section](https://docs.px4.io/v1.15/en/dev_setup/building_px4.html#nuttx-pixhawk-based-boards).

## 4. Flash

Dry run first. It runs every guard and flashes nothing:

```bash
python3 scripts/firmware.py --flash --dry-run --backup ~/px4_before.params --confirm-serial <serial>
python3 scripts/firmware.py --flash --apply   --backup ~/px4_before.params --confirm-serial <serial>
```

`<serial>` is the board's USB serial from `--check` (`0` on this board). `--apply` runs the pinned `Tools/px_uploader.py`, which reboots the board into its bootloader itself. **Do not unplug it.**

The guards (each refusal names its guard):

| Guard | Passes when |
|---|---|
| `identity` | exactly one `3185:0035` device, in application mode (the bootloader has the same VID:PID; told apart by product string `PX4 BL FMU v6X.x`) |
| `serial_confirm` | the typed serial equals the board's |
| `backup` | from `--backup-params`, under an hour old, unchanged, this board |
| `driver_stopped` | no `MicroXRCEAgent`, `mavros_node`, `px4_control_node` or QGroundControl |
| `artifact` | sha256 equals the `--record` value, board_id 53, built from the pinned commit |
| `port_free` | `fuser`/`lsof` (else a scan of `/proc/*/fd`) find the port unused |
| `uploader` | `px_uploader.py` and pyserial are present |

ModemManager is reported as a warning, not a refusal.

On the bench the real run took about 30 s: bootloader found (`usb-Auterion_PX4_BL_FMU_v6X.x_0-if00`), erase, program and verify to 100%, "Rebooting", and the application port was back within 5 s. The image used 99.48% of the board's flash.

## 5. Verify afterwards

```bash
python3 scripts/firmware.py --check
python3 scripts/firmware.py --backup-params ~/px4_after.params --diff ~/px4_before.params
```

`--check` should report the current version equal to the pin (on the bench: `flight_sw_version 0x010f04ff`, v1.15.4, git hash `99c40407ff`), and the `firmware` row of `host_setup.py --guide` should be PASS. The post-flash diff shows what the version change did to the parameters. On the bench, 1079 of the board's parameters were identical, 15 changed (for example `MC_ROLL_P` 4 to 6.5, `MAV_PROTO_VER` 2 to 0), and the rest were names that exist in only one version. Calibration IDs and `SYS_AUTOSTART` survived. **Re-applying the lab parameter file is done by you in QGroundControl**; the tool never does it.

## Console GUI versus the host command

The PX4 setup page in console has a **Firmware** section that runs this same tool as jobs: the check rows, a **Back up params** button (with the diff summary), the build commands shown to copy (console never builds), and a **Flash** button. Flash stays disabled until the check rows pass, a backup from this console session exists, the driver is stopped, and you tick that you are present with props off and type the board target (`px4_fmu-v6x_default`). Console then passes the serial itself. It adds no firmware logic and no authentication beyond what its other admin routes use. See console's PX4 tutorial for the page.

Only the host command was used for the real flash. Flashing from the console container is untested: the container needs the artifact and `firmware/PX4-Autopilot` in its worktree and `psmisc`/`pymavlink` from `scripts/deps/px4.sh`.

## Another version or custom firmware

(Untested; the bench only flashed the pinned version.)

1. Point the submodule at the new tag or your fork's commit: `git -C firmware/PX4-Autopilot fetch --depth 1 origin tag <tag>` and check it out, then commit the new gitlink.
2. Update `firmware/versions.json`: `firmware.tag` and `.commit`, and the matching `px4_msgs` branch and commit (the message set must describe the same PX4 version). If NuttX differs, update `nuttx.tag` and the `submodules` list; the toolchain image may need to change too.
3. Re-pin `src/px4_msgs`, rebuild the driver, then repeat sections 2 to 5.

A different board needs a new target, board_id and VID:PID in the pairing file, and is not tested here.

## Recovery

- **Upload never finds the bootloader:** the application must reboot into the bootloader and re-enumerate. Check that nothing else holds the port and that ModemManager is not probing it (`systemctl status ModemManager`; `host_setup.py --usb` shows the rule and its undo). Stop it and re-run `--apply`.
- **Interrupted flash:** this flash does not touch the bootloader (not exercised on the bench). Re-plug USB and re-run the flash; `px_uploader.py` waits for the bootloader. If the board enumerates as `PX4 BL FMU v6X.x`, the tool's `identity` guard will refuse it; use PX4's [firmware flashing page](https://docs.px4.io/v1.15/en/config/firmware.html) (QGroundControl) to load the `.px4` file.
- **Parameters look wrong afterwards:** you still have `~/px4_before.params`; re-apply the lab file in QGroundControl.
