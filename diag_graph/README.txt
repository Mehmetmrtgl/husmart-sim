Copy of Autoware's autoware_launch/config/system/diagnostics (universe-cuda image, 2026-04-13).
Change: timeout: 3.0 added to ADAPI's 1 Hz diagnostic units (localization/state, routing/state, mrm_request/delegate),
as in Autoware's CARLA configuration. The default 1.0 s was occasionally exceeded with the AWSIM sim clock; the unit went
STALE, triggered an MRM (emergency stop) and the vehicle could not start in autonomous mode.
run_autoware_beytepe.sh (and the old run_e2e_sim.sh) mount this folder read-only over the original inside the container.
Change (2026-11-08): hysteresis: 0.5 added to the /autoware/localization/scan_matching_status (NDT) unit in localization.yaml.
NDT warnings shorter than 0.5 s do not trigger an MRM; any longer NDT warning/error still does.
Previous version: ~/yedek/2026-11-08_ndt_uyari/localization.yaml.oncesi
