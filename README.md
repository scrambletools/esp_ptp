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
AVB-over-Wi-Fi). It is maintained by Scramble.

The source lives at <https://github.com/scrambletools/esp_ptp>.
The component is published in the ESP Component Registry
(<https://components.espressif.com>) under the **scrambletools**
namespace, so depend on it as:

```yaml
dependencies:
  scrambletools/esp_ptp: "*"
```

Old github.com/scrambletoolsllc/... links redirect to the
scrambletools repos.

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

Profile is selected by `CONFIG_ESP_PTP_GPTP_PROFILE` and can be
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
- **Beacon Vendor IE on Wi-Fi** — for `wifi_ftm` ports operating as
  AP, the daemon publishes the IEEE 802.1AS-2020 §12.7
  `FollowUpInformation` TLV in the 802.11 Beacon Vendor IE
  (byte-identical to the FTM-frame Vendor IE so a future carrier
  swap stays wire-compatible). Publish is internal to this component
  (`ptp_beacon_ie.c`, auto-fires on `WIFI_EVENT_AP_START`) when at
  least one port is `wifi_ftm` + AP-mode reached via a coprocessor.
- **External peer-delay** — applications can push out-of-band
  peer-delay measurements (e.g. from 802.11 FTM) via
  `ptpd_inject_peer_delay()` on `wifi_ftm` ports operating as STA.

## Clock backends

- **Hardware (ESP32-P4 EMAC IEEE 1588)** — `ptpd_now()` reads
  `CLOCK_PTP_SYSTEM` backed by the EMAC timestamp engine.
- **Software (`ptp_clock_sw.c`)** — for chips without IEEE 1588
  hardware (e.g. ESP32-C6). Disciplines an affine software clock with
  `(offset_ns, rate_ppb)` tunables over a free-running local counter:
  SYSTIMER on the C6 (62.5 ns a tick), or with
  `CONFIG_ESP_PTP_LOCAL_CLOCK_GPTIMER` a general purpose timer at
  40 MHz (25 ns a tick, inside the 40 ns of 802.1AS-2020 B.1.2).
  `ptpd_now()` routes through this backend automatically when
  `ptp_clock_sw_init()` has been called.

## Public API

| Function | Purpose |
| --- | --- |
| `ptpd_start(iface)` | Start daemon on a single eth_hwts port (legacy convenience). |
| `ptpd_start_port(port, iface, medium)` | Multi-port start. `medium` ∈ {`eth_hwts`, `wifi_ftm`}. Peer-delay mechanism is derived from medium + the port's `wifi_mode` (set via Kconfig). |
| `ptpd_now(ts)` | Read PTP-disciplined system time (HW or SW backend). |
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

- Waveshare ESP32-P4-ETH — wired endpoints (with an ES8389 audio hat)
- Waveshare ESP32-P4-WIFI6-POE-ETH — Ethernet ↔ Wi-Fi bridge (P4 host +
  onboard C6 over SDIO)
- Waveshare ESP32-C6-LCD-1.69 — Wi-Fi STA endpoint

Network peers:

- MOTU AVB Switch (gPTP BTC)
- PreSonus AVB switch (gPTP BTC)
- Linux linuxptp `ptp4l` (gPTP profile, Intel I210/I226) and a Linux
  gPTP bridge
- Apple Mac mini M1 / M4
- Sonnet Thunderbolt AVB Adapter

## IEEE 802.1AS test results

No conformance claim is made. These are bench measurements against
IEEE Std 802.1AS-2020 Annex B (time-synchronization performance) and a
clause-by-clause review of the Wi-Fi media-dependent layer (clause 12).
The Annex B figures quoted are B.1.2 (LocalClock granularity no
coarser than 40 ns) and B.3 (any two PTP Instances within seven hops
synchronized to within 1 µs peak-to-peak).

### How it was measured

- **Network.** A MOTU AVB Switch is the gPTP BTC (clock identity
  00:01:f2:ff:fe:ff:3b:14). The boards under test each sit on one of its
  ports, one hop from the BTC, so any two of them are two hops apart.
  The endpoints report a link delay of about 440 to 470 ns to the
  switch.
- **Pulses.** Each board runs the `clock_scope_probe` component from
  scrambletools/ESP-AVB-Bridge (`CONFIG_CLOCK_SCOPE_PROBE`). Once a second
  it arms a general purpose timer alarm for the next gPTP second, and
  the alarm raises a GPIO through the ETM event matrix, so no software
  runs in the edge path. The P4 maps the timer to the EMAC PTP clock,
  the C6 to its software clock. It also logs its own estimate of each
  edge's placement error against that clock. Pins: GPIO45 on the
  ESP32-P4-WIFI6-POE-ETH (its bank powered from LDO4), GPIO33 on the
  ESP32-P4-ETH (a header pin outside the audio hat), GPIO18 on the
  ESP32-C6-LCD-1.69.
- **Oscilloscope.** A Rigol DHO804 (four channels, 10x probes,
  1 V/div), triggered on the bridge's pulse, takes one single-shot
  acquisition a second at 2 µs/div (20 ns a sample, ±10 µs window).
  Each edge is timed at 1.65 V by linear interpolation between
  samples, and the other channels are compared to the trigger channel.
- **Correction.** Each scope second is matched to the boards' logged
  pulse records by gPTP second. Subtracting each board's own
  placement error leaves the clock-to-clock difference; both raw and
  corrected figures are given. MTIE and TDEV are computed from the
  once-a-second corrected differences, unfiltered (B.4 specifies a
  reference endpoint filter that was not applied).

### Wired (Ethernet, behind a MOTU AVB Switch)

One hour, 2026-11-18 04:07–05:07 UTC, 3,444 matched seconds. Bridge:
Waveshare ESP32-P4-WIFI6-POE-ETH; endpoints: two Waveshare ESP32-P4-ETH
(esp_ptp 1.15.0). Endpoint A received a live AVB audio stream
throughout, and endpoint B ran a two-way AVB audio session for about
3.5 minutes of the hour with no change in spread.

| Pair (two hops) | Median | Std dev | Peak-to-peak | Within ±1 µs |
| --- | --- | --- | --- | --- |
| Endpoint A − bridge | −39 ns | 46 ns | 292 ns | 3444 / 3444 |
| Endpoint B − bridge | −32 ns | 46 ns | 309 ns | 3444 / 3444 |
| Endpoint B − endpoint A | +8 ns | 48 ns | 312 ns | 3444 / 3444 |

- Raw edges, before correction: std dev 117–122 ns, peak-to-peak
  722–800 ns, still all within ±1 µs. Each board's pulse placement
  alone varies by 84–93 ns (std dev), which is most of the raw spread.
- MTIE over 1 / 10 / 100 / 1000 s: 214–251 / 242–276 / 279–307 /
  292–312 ns. TDEV over 1 / 10 / 100 s: 46–49 / 14–15 / 4.3–4.6 ns. It
  falls like white noise; no wander shows within the hour.
- B.3 is met for these two-hop pairs. Seven hops were not built.

### LocalClock granularity (B.1.2)

| Board | Local clock | Tick | B.1.2 |
| --- | --- | --- | --- |
| ESP32-P4 (EMAC PTP clock, IDF default) | EMAC 1588 counter | 40 ns | at the limit |
| ESP32-C6, default | SYSTIMER | 62.5 ns | not met |
| ESP32-C6, `CONFIG_ESP_PTP_LOCAL_CLOCK_GPTIMER` | GPTimer at 40 MHz | 25 ns | met |

On a Waveshare ESP32-C6-LCD-1.69 (2026-11-18), the scope probe's own
pulse placement error against the local clock fell from 25.3 ns to
10.2 ns std dev with the 40 MHz clock (39 % to 94 % of pulses within
±13 ns).

### Wi-Fi (one hop, bridge to station)

Waveshare ESP32-P4-WIFI6-POE-ETH bridge (onboard C6 as the AP and FTM
responder) to a Waveshare ESP32-C6-LCD-1.69 station, with FTM time
transfer and Espressif's pre-release PTP-compatible FTM libraries
(2026-11-10 drop), channel 6, 20 MHz:

- 2026-11-11, 600 s: 1.72 µs peak-to-peak (1.48 µs without the two
  seconds hit by a lost initial FTM frame), 95 % within −373 to
  +686 ns, std dev 270 ns. One Wi-Fi hop alone exceeds the seven-hop
  1 µs of B.3, so B.3 is not met over Wi-Fi.
- 2026-11-01 (the 11-01 drop): 599 / 600 seconds within 1 µs, range
  −552 to +1045 ns.
- Without FTM time transfer (beacon element only), a station behind this
  bridge does not hold lock: its offset wanders about ±2.5 ms. The
  bridge's element carries no BTC-vs-TSF rate, and its gPTP and TSF
  markers are taken on two chips without a capture bracket. Behind a
  Linux access point that publishes a measured rate, the station's own
  servo error on the beacon path stays about 10–17 µs (self-reported,
  not scope-measured).

### Protocol review and open items

- Clause 10/11 items used on these paths (Signaling TLVs, message
  interval requests, receipt timeouts, peer-delay asCapable, path trace)
  were reviewed against the final 802.1AS-2020 and Cor 1-2021 text
  (2026-09-25) and are conformant. The review is in
  scrambletools/ESP-AVB-Bridge `docs/ftm-final-standard-verification.md`.
- Clause 12 (802.11 links): Table 12-2/12-3 requests, grants and the
  §12.7 Follow_Up element need the Espressif PTP-compatible FTM API,
  which is not in a released ESP-IDF yet. Per-association PortSync/MD
  state machines (12.1.3) are partial.
- Not measured: B.1.3 jitter and wander, B.2.4 rate ratio error, and
  B.4 MTIE/jitter through the reference endpoint filter.
