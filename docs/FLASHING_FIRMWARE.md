# Flashing the Pixhawk 6X firmware

How to build and flash the pinned PX4 firmware onto the X500's Pixhawk 6X with `scripts/firmware.py`. Written from the bench flash of 2026-11-04 (v1.17.0 to v1.15.4). Option reference: `python3 scripts/firmware.py --help`; short version: the "Firmware" section of this repo's [README](../README.md).

**Run every command from the root of this repository (the CPSL_ROS2_PX4 checkout; in console that is `components/CPSL_ROS2_PX4`).** `scripts/` paths are relative to it.

## Do this

1. `--check`; flash only if the version differs from the pin (1).
2. Build the artifact with Docker (3).
3. Meet the prerequisites below.
4. Back up parameters (2), then at once dry-run and flash (4).
5. Verify (5).
6. **Do not fly, and keep props off, until the lab parameter file is re-applied in QGroundControl and checked.** PX4 itself changed defaults between versions, so parameters differ after a version bump; the tool never writes parameters.

## Prerequisites

- Docker available to your user (only for the build); no sudo is needed for anything here. The one sudo item is ModemManager (see Recovery): the tool only prints that fix and you run it.
- **You are at the vehicle, props off, battery disconnected, the board powered from USB only.** A flash is never unattended.
- QGroundControl closed for the whole backup and flash; open it again only afterwards, to re-apply parameters.
- No `mavros_node`, `MicroXRCEAgent` or `px4_control_node` running. In console, press **Stop** in the Driver block of the PX4 setup page; from a shell, check with `ps`.
- The board is the only `3185:0035` device on the host and is in application mode.
- `fuser` (psmisc) or `lsof`, `pyserial`, and for the version read `pymavlink`: `bash scripts/deps/px4.sh` installs them.

## 1. When to flash, and what is pinned

```bash
python3 scripts/firmware.py --check
```

Flash when the reported board version differs from the pin. A mismatch is not cosmetic: `px4_msgs` describes one PX4 version, and a board running another publishes different topics or none that the driver reads (on the v1.17.0 board `flight.armed` and `flight.mode` stayed empty). The same comparison appears as the `firmware` row of `python3 scripts/host_setup.py --iface <your-iface> --guide` (`host_setup.py` is this repo's read-only host check: `--guide` is the guided checklist, `--usb` checks the USB link and ModemManager). Reflash, or re-pin `px4_msgs`; the check does not choose.

`firmware/versions.json` is the single pairing file. It records PX4-Autopilot `v1.15.4` (commit `99c40407ff`), `px4_msgs` branch `release/1.15` at `a1045ec`, the build target `px4_fmu-v6x_default` (`board_id` 53 is the number the bootloader checks the image against), USB `3185:0035`, the Docker image PX4 builds in, and the 7 PX4 submodules the board needs. `firmware/PX4-Autopilot` is the shallow submodule at that commit. Flashing replaces the application only; the bootloader is not touched.

```bash
git submodule update --init --depth 1 firmware/PX4-Autopilot src/px4_msgs
```

Do not hand-edit the firmware tag without changing `px4_msgs` to match.

## 2. Back up the parameters (right before flashing)

Do this just before section 4: the flash accepts only a backup under an hour old.

```bash
python3 scripts/firmware.py --backup-params ~/px4_before.params --diff /path/to/cpsl_px4_v15_4_params.params
```

This is read-only. It starts its own temporary MAVROS on the USB port (so the port must be free), writes a QGC-format file, and stops it again. It refuses to overwrite a file. `--diff` counts parameters identical, different, or on one side only; `--diff-params A B` compares any two files.

## 3. Build

```bash
python3 scripts/firmware.py --build
```

This only **prints** the host commands; run them yourself, in order: create `firmware/out/` (gitignored), `git submodule update --init --depth 1` for the 7 listed submodules, fetch the one NuttX release tag (`nuttx-12.12.0`), the Docker build in `px4io/px4-dev-nuttx-focal:2022-08-12`, and `firmware.py --record`, which stores the artifact's sha256 beside it. The tag step matters: a depth-1 NuttX has no tags and PX4's version-header script fails without one.

Expect about 5 minutes on a first run. Run it so it survives a closed terminal (for example `systemd-run --user` or `tmux`) and watch its log. The artifact is `firmware/out/px4_fmu-v6x_default/px4_fmu-v6x_default.px4`. If the artifact is missing or the `artifact` guard reports a sha256 mismatch, the file is not the one `--record` stored: do not flash it. Rebuild and run `--record` again.

Do not install PX4's toolchain on the host: `Tools/setup/ubuntu.sh` uses sudo and is the user's call. See PX4's [building PX4, NuttX section](https://docs.px4.io/v1.15/en/dev_setup/building_px4.html#nuttx-pixhawk-based-boards).

## 4. Flash

Dry run first. It runs every guard and flashes nothing:

```bash
python3 scripts/firmware.py --flash --dry-run --backup ~/px4_before.params --confirm-serial <serial>
python3 scripts/firmware.py --flash --apply   --backup ~/px4_before.params --confirm-serial <serial>
```

`<serial>` is the value of the `serial` shown in the `board` row of `--check` (`0` on this board). `--apply` runs the pinned `Tools/px_uploader.py`, which reboots the board into its bootloader itself. **Do not unplug it** until the uploader prints "Rebooting" and exits (about 30 s) and `--check` sees the board again.

The guards (each refusal names its guard):

| Guard | Passes when |
|---|---|
| `identity` | exactly one `3185:0035` device, in application mode (bootloader mode shares the VID:PID and is told apart by its product string) |
| `serial_confirm` | the typed serial equals the board's |
| `backup` | from `--backup-params`, under an hour old (`--backup-max-age`), unchanged, this board; the flash refuses without one made by this tool |
| `driver_stopped` | no `MicroXRCEAgent`, `mavros_node`, `px4_control_node` or QGroundControl |
| `artifact` | sha256 equals the `--record` value, board_id 53, built from the pinned commit |
| `port_free` | `fuser`/`lsof` (else a scan of `/proc/*/fd`) find the port unused |
| `uploader` | `px_uploader.py` and pyserial are present |

ModemManager is reported as a warning, not a refusal.

## 5. Verify afterwards

```bash
python3 scripts/firmware.py --check
python3 scripts/firmware.py --backup-params ~/px4_after.params --diff ~/px4_before.params
```

`--check` should report the current version equal to the pin (`board runs v1.15.4 ..., the pinned v1.15.4`; compare the version number, not the git hash, which `--check` showed as `000000ff0704c499` on the bench and looks mis-decoded; a one-off pymavlink read gave `0x010f04ff`), and the `firmware` row of `host_setup.py --guide` should be PASS. On the bench, calibration IDs and `SYS_AUTOSTART` survived. **Re-applying the lab parameter file is done by you in QGroundControl**; the tool never does it.

## Console GUI versus the host command

The PX4 setup page in console has a **Firmware** section that runs this same tool as jobs (console never builds; it shows the build commands to copy). **Flash** stays disabled until the check rows pass, a backup from this console session exists, the driver is stopped, and you tick that you are present with props off and type the board target (`px4_fmu-v6x_default`). Console then passes the serial itself. It adds no authentication beyond what its other admin routes use.

The console GUI flash has not been run on the bench, so treat it as untested rather than as forbidden; the host command is the proven path. Flashing from the console container is also untested: the container needs the artifact and `firmware/PX4-Autopilot` in its worktree and `psmisc`/`pymavlink` from `scripts/deps/px4.sh`.

## Recovery

Recovery from an interrupted flash is **unproven on the bench**; nothing below was exercised.

- **Upload never finds the bootloader:** the application must reboot into the bootloader and re-enumerate, and ModemManager can interfere (it did not on the bench). Check nothing else holds the port, then `python3 scripts/host_setup.py --usb`: it reports ModemManager and `--usb --apply` prints a udev-rule fix and its undo (`--undo`). It prints only; you run the printed commands with sudo yourself. Then re-run `--apply`.
- **Flash interrupted (unplugged, power loss):** this flash does not rewrite the bootloader, so the board should still enter it. Re-plug USB and try `--apply` again. But the tool's `identity` guard refuses a board already in bootloader mode (`PX4 BL FMU v6X.x`), so this may fail. Fallback, using PX4's [firmware flashing page](https://docs.px4.io/v1.15/en/config/firmware.html) and QGroundControl (path as of QGC 4.x; confirm):
  1. Unplug the board and close nothing else on the port.
  2. Open QGroundControl, go to Vehicle Setup > Firmware.
  3. Plug the board in when asked.
  4. Choose "Custom firmware file" and select `firmware/out/px4_fmu-v6x_default/px4_fmu-v6x_default.px4`.
  5. Wait for "Upgrade complete", then close QGroundControl before any further `firmware.py` use.
