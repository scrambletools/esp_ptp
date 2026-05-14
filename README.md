# PTP / gPTP implementation for ESP-IDF

This component provides a Precision Time Protocol daemon for ESP-IDF
with support for both the standard IEEE 1588 PTP profile and the
IEEE 802.1AS gPTP profile (the Audio Video Bridging time-sync profile).

It was originally ported from the
[NuttX PTP daemon](https://github.com/apache/nuttx-apps/tree/master/netutils/ptpd)
to ESP-IDF as an example by Ondrej Kosta at Espressif. Since then it has
been enhanced by Scramble Tools to support gPTP, multi-port operation,
software-disciplined clocks for chips without IEEE 1588 hardware, and
out-of-band time transport (e.g. 802.11 beacon Vendor IE for
AVB-over-Wi-Fi). It is maintained by Scramble and registered at the
ESP Component Registry (<https://components.espressif.com>).

## Terminology

This component uses **BTC** (best timetransmitter clock) in place of
the older "grandmaster clock" / "GM" terminology from IEEE 1588 and
802.1AS, and **BTCA** in place of "BMCA" (best master clock
algorithm). The wire format and algorithm are unchanged — only the
identifiers and prose differ. The renaming applies to both code
identifiers (e.g. `btc_id`, `btc_priority1`) and human-readable
comments / docs.

## Profiles

- **Standard PTP** (IEEE 1588) — UDP-based client/server, default profile.
- **gPTP** (IEEE 802.1AS) — peer-delay (Pdelay_Req/Resp) and Sync over
  L2 raw frames at ethertype 0x88f7. Required for AVB.

Profile is selected by `CONFIG_NETUTILS_PTPD_GPTP_PROFILE` and can be
changed at runtime via `ptpd_set_profile()`.

## Mediums and per-port topology

Each PTP port has three orthogonal knobs:

- **Medium** — `ethernet`, `wifi`, or `wifi_cp` (Wi-Fi reached over
  ESP-Hosted on a coprocessor).
- **Type** — `primary` (single-port endpoint), `failover` (hot standby),
  or `bridged` (participates in an L2 bridge with the other port).
- **Wi-Fi role** — `ap` or `sta`, only meaningful when medium is
  `wifi`/`wifi_cp`. AP transmits beacons (FollowUpInfo carrier) and
  responds to FTM; STA consumes beacons and initiates FTM.

`CONFIG_ESP_PTP_NUM_PORTS` selects 1- or 2-port operation. With 2 ports
the daemon runs full Best-Master-Clock per port (PRIMARY+FAILOVER) or
participates as a time-aware bridge (BRIDGED+BRIDGED).

## Time transport

- **gPTP on Ethernet** — standard 802.1AS Pdelay_Req/Resp + Sync on an
  L2TAP-bound socket. The default and most precise path.
- **Beacon Vendor IE on Wi-Fi** — for wireless endpoints that cannot
  carry 802.1AS frames natively, the AP publishes the IEEE 802.1AS-2020
  §12.7 `FollowUpInformation` TLV in its 802.11 Beacon Vendor IE
  (Path C of the AVB-over-Wi-Fi plan; byte-identical to the FTM-frame
  Vendor IE so a future carrier swap stays wire-compatible). Publish is
  internal to this component (`ptp_beacon_ie.c`, auto-fires on
  `WIFI_EVENT_AP_START`) when at least one port is `wifi_cp` + AP role.
- **External peer-delay** — applications can push out-of-band peer-delay
  measurements (e.g. from 802.11 FTM) via `ptpd_inject_peer_delay()` on
  ports configured with `peer_delay_source = ftm_external`.

## Clock backends

- **Hardware (ESP32-P4 EMAC IEEE 1588)** — `ptpd_now()` reads
  `CLOCK_PTP_SYSTEM` backed by the EMAC timestamp engine.
- **Software (`ptp_clock_sw.c`)** — for chips without IEEE 1588
  hardware (e.g. ESP32-C6). Disciplines a software clock backed by
  `esp_timer_get_time()` with `(offset_ns, rate_ppb)` tunables.
  `ptpd_now()` routes through this backend automatically when
  `ptp_clock_sw_init()` has been called.

## Public API

| Function | Purpose |
| --- | --- |
| `ptpd_start(iface)` | Start daemon on a single Ethernet port (legacy convenience). |
| `ptpd_start_port(port, iface, medium, peer_delay_source)` | Multi-port start — configures medium and peer-delay source per port. |
| `ptpd_now(ts)` | Read PTP-disciplined system time (HW or SW backend). |
| `ptpd_inject_peer_delay(port, ns)` | Push externally-measured peer-delay (FTM). |
| `ptpd_inject_sync(port, fup_info, len)` | Push out-of-band Sync/FollowUp (beacon-IE consumer side). |
| `ptpd_register_sync_egress_cb(port, cb, ctx)` | Register a callback to receive marshalled `FollowUpInformation` for AP-side carriage onto the wire. |
| `ptpd_set_profile(pid, profile)` | Switch between standard PTP and gPTP. |
| `ptpd_status(pid, *status)` | Query daemon status (clock identity, offset, master, etc.). |
| `ptpd_stop(pid)` | Stop the daemon. |

## Configuration

Per-port topology and profile selection are exposed under *PTP Daemon
Configuration* in menuconfig. Companion components such as `esp_avb`
derive their port configuration from these settings — `esp_ptp` is the
single source of truth for the per-port medium/type/wifi_role taxonomy.

## Tested with

Targets:

- ESP32-P4 — full hardware-timestamped gPTP on the EMAC
- ESP32-C6 — software-clock gPTP, both as a Wi-Fi STA endpoint and as
  the Wi-Fi coprocessor for a P4 host (over ESP-Hosted SDIO)

Boards:

- Waveshare ESP32-P4-ETH
- Waveshare ESP32-P4-WiFi6-PoE-ETH (P4 + onboard C6 over SDIO)
- ESP32-C6 dev board

Network peers:

- MOTU AVB switch (gPTP BTC)
- Apple Mac Mini M1 / M4
- Sonnet Thunderbolt AVB Adapter
