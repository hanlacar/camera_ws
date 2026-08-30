"""Synthetic section-state-machine and safety-priority acceptance tests."""

from dataclasses import replace
import ast
import json
from pathlib import Path

import pytest

from camera_navigation.mission_decision_core import (
    MissionDecisionConfig, MissionDecisionMachine, MissionInputs,
    ROUTE_MODE_SECTIONS, map_route_section)
from camera_navigation.camera_mission_decision_node import (
    vehicle_steering_from_bev)


def sample(now=0.0, section="NORMAL", **kwargs):
    values = dict(now=now, stamp=now+100.0, section=section,
                  input_fresh=True, planner_valid=True,
                  planner_state="VALID", planner_drive=2.0,
                  required_steering_deg=0.0, stop_line_tf_valid=True)
    values.update(kwargs)
    if values.get("uphill_detected") and "imu_pitch_deg" not in kwargs:
        values["imu_pitch_deg"] = 15.0
    return MissionInputs(**values)


def test_slope_requires_uphill_and_does_not_cross_on_detection_loss():
    machine = MissionDecisionMachine()
    result = machine.update(sample(section="SLOPE",
                                   stop_line_distances_m=(2.0, 4.0)))
    assert result.state == "APPROACH_FIRST_LINE" and not result.override_active
    machine.update(sample(.1, "SLOPE", uphill_detected=True,
                          stop_line_detected=True,
                          stop_line_distances_m=(.1, 2.1)))
    lost = machine.update(sample(.2, "SLOPE", uphill_detected=True,
                                 stop_line_distances_m=()))
    assert not machine.first_line_crossed
    assert not lost.override_active


def enter_slope_stop(machine, at=0.0):
    feedback = dict(encoder_valid=True, speed_valid=True,
                    current_speed_mps=.2, distance_valid=True)
    machine.update(sample(at, "SLOPE", stop_line_detected=True,
                          stop_line_distances_m=(1.0,), distance_m=0.0,
                          **feedback))
    machine.update(sample(at+.1, "SLOPE", stop_line_detected=True,
                          stop_line_distances_m=(.1,), distance_m=.05,
                          **feedback))
    crossed = machine.update(sample(
        at+.2, "SLOPE", stop_line_distances_m=(-.05,), distance_m=.1,
        **feedback))
    assert machine.first_line_crossed
    assert not crossed.override_active
    machine.update(sample(at+.3, "SLOPE", uphill_detected=True,
                          imu_valid=True, imu_pitch_deg=15.0,
                          distance_m=.2, **feedback))
    machine.update(sample(at+.56, "SLOPE", uphill_detected=True,
                          imu_valid=True, imu_pitch_deg=15.0,
                          distance_m=.3, **feedback))
    machine.update(sample(at+.6, "SLOPE", uphill_detected=True,
                          imu_valid=True, imu_pitch_deg=15.0,
                          stop_line_distances_m=(2.0,), distance_m=.7,
                          **feedback))
    return machine.update(sample(
        at+.7, "SLOPE", uphill_detected=True, imu_valid=True,
        imu_pitch_deg=15.0, stop_line_distances_m=(1.9,), distance_m=.8,
        **feedback))


def test_slope_stops_only_after_stationary_then_holds_three_seconds():
    machine = MissionDecisionMachine()
    started = enter_slope_stop(machine)
    assert started.state == "APPROACH_SECOND_LINE"
    assert not started.override_active
    slow = machine.update(sample(
        .8, "SLOPE", uphill_detected=True, imu_valid=True,
        stop_line_distances_m=(1.4,), encoder_valid=True, speed_valid=True,
        current_speed_mps=.2, distance_valid=True, distance_m=.9))
    assert slow.state == "DECELERATE_SECOND_LINE" and slow.drive_override == 1.0
    moving = machine.update(sample(
        .9, "SLOPE", uphill_detected=True, imu_valid=True,
        stop_line_distances_m=(.6,), encoder_valid=True, speed_valid=True,
        current_speed_mps=.2, distance_valid=True, distance_m=1.0))
    assert moving.state == "STOP_SECOND_LINE" and moving.drive_override == 0.0
    machine.update(sample(1.0, "SLOPE", uphill_detected=True, imu_valid=True,
        stop_line_distances_m=(.59,), encoder_valid=True, speed_valid=True,
        current_speed_mps=0.0, distance_valid=True, distance_m=1.0))
    before = machine.update(sample(
        1.41, "SLOPE", uphill_detected=True, imu_valid=True,
        stop_line_distances_m=(.58,), encoder_valid=True, speed_valid=True,
        current_speed_mps=0.0, distance_valid=True, distance_m=1.0))
    assert before.state == "HOLD_3_SECONDS"
    assert before.override_active and not machine.slope_stop_completed
    complete = machine.update(sample(
        4.42, "SLOPE", uphill_detected=True, imu_valid=True,
        stop_line_distances_m=(.57,), encoder_valid=True, speed_valid=True,
        current_speed_mps=0.0, distance_valid=True, distance_m=1.0))
    assert complete.state == "SLOPE_COMPLETE"
    assert not complete.override_active and machine.slope_stop_completed


def test_slope_one_shot_and_section_exit_reset():
    machine = MissionDecisionMachine()
    machine.update(sample(3.9, "SLOPE"))
    machine.slope_stop_completed = True
    again = machine.update(sample(4.0, "SLOPE"))
    assert not again.override_active and machine.slope_stop_completed
    machine.update(sample(4.1, "NORMAL"))
    assert not machine.first_line_crossed and not machine.slope_stop_completed


def test_slope_lines_need_not_be_simultaneously_visible():
    machine = MissionDecisionMachine(); between = enter_slope_stop(machine)
    assert machine.line_spacing_m == pytest.approx(2.6)
    assert between.state == "APPROACH_SECOND_LINE"


def test_slope_requires_positive_fifteen_degree_pitch_and_latches_arm():
    machine = MissionDecisionMachine()
    machine.update(sample(-.1, "SLOPE"))
    machine.first_line_crossed = True
    below = machine.update(sample(section="SLOPE", uphill_detected=True,
                                  imu_valid=True, imu_pitch_deg=14.9))
    assert below.state == "FIRST_LINE_CROSSED" and not machine.slope_armed
    machine.update(sample(.1, "SLOPE", uphill_detected=True,
                          imu_valid=True, imu_pitch_deg=15.0))
    machine.update(sample(.36, "SLOPE", uphill_detected=True,
                          imu_valid=True, imu_pitch_deg=15.0))
    assert machine.slope_armed


def test_numeric_route_mode_mapping_and_same_section_mode_reset():
    assert ROUTE_MODE_SECTIONS == {1: "NORMAL", 2: "SLOPE", 3: "NORMAL",
        4: "NORMAL", 5: "NORMAL", 6: "NORMAL", 7: "NORMAL",
        8: "LEFT_SIGNAL_MONITOR", 9: "ACCELERATION", 10: "NORMAL",
        11: "EXIT_SIGNAL"}
    assert map_route_section("INTERSECTION") == (None, "INTERSECTION")
    machine = MissionDecisionMachine()
    machine.update(sample(section="9", sign_detected=True))
    assert machine.acceleration_armed
    machine.update(sample(.1, section="1"))
    assert not machine.acceleration_armed


@pytest.mark.parametrize("mode", (4, 6))
def test_numeric_intersection_modes_do_not_use_legacy_signal_override(mode):
    result = MissionDecisionMachine().update(sample(
        section=str(mode), traffic_light="R", stop_line_distances_m=(.5,)))
    assert result.section == "NORMAL" and not result.override_active


def test_second_line_feedback_stale_or_lost_timeout_fails_closed():
    stale_machine = MissionDecisionMachine(); enter_slope_stop(stale_machine)
    stale = stale_machine.update(sample(
        .8, "SLOPE", uphill_detected=True, imu_valid=True,
        stop_line_distances_m=(.6,), distance_valid=False, speed_valid=False))
    assert stale.override_active and stale.drive_override == 0.0
    assert stale.failure_reason == "SLOPE_FEEDBACK_STALE"

    lost_machine = MissionDecisionMachine(); enter_slope_stop(lost_machine)
    lost = lost_machine.update(sample(
        1.21, "SLOPE", uphill_detected=True, imu_valid=True,
        stop_line_distances_m=(), distance_valid=True, distance_m=1.0,
        speed_valid=True, current_speed_mps=.1))
    assert lost.override_active and lost.drive_override == 0.0
    assert not lost_machine.slope_stop_completed


def test_mode_8_only_latches_green_left_and_never_overrides():
    machine = MissionDecisionMachine()
    result = machine.update(sample(section="8", traffic_aspect="GREEN_DOWN",
        traffic_aspect_confidence=.9, traffic_aspect_fresh=True,
        traffic_aspect_sample_id=1))
    assert not result.override_active and not machine.left_signal_confirmed
    result = machine.update(sample(.1, section="8", traffic_aspect="GREEN_LEFT",
        traffic_aspect_confidence=.9, traffic_aspect_fresh=True,
        traffic_aspect_sample_id=2))
    assert not result.override_active and machine.left_signal_confirmed


def test_mode_11_green_down_confirmation_and_release_latch():
    machine = MissionDecisionMachine()
    for index, aspect in enumerate(("GREEN_CIRCLE", "GREEN_DOWN", "GREEN_DOWN"), 1):
        result = machine.update(sample(index*.1, section="11",
            traffic_aspect=aspect, traffic_aspect_confidence=.9,
            traffic_aspect_fresh=True, traffic_aspect_sample_id=index,
            rgb_green_down_verified=aspect == "GREEN_DOWN"))
        assert result.override_active and result.drive_override == 0.0
    released = machine.update(sample(.4, section="11",
        traffic_aspect="GREEN_DOWN", traffic_aspect_confidence=.9,
        traffic_aspect_fresh=True, traffic_aspect_sample_id=4,
        rgb_green_down_verified=True))
    assert released.state == "EXIT_RELEASED" and not released.override_active
    lost = machine.update(sample(.5, section="11", traffic_aspect="UNKNOWN",
        traffic_aspect_fresh=False, traffic_aspect_sample_id=5))
    assert not lost.override_active
    reentered = machine.update(sample(.6, section="1"))
    assert not machine.exit_release_latched


def test_mode_11_never_releases_from_generic_yolo_g_or_unverified_down():
    machine = MissionDecisionMachine()
    for index in range(1, 5):
        generic = machine.update(sample(
            index*.1, section="11", traffic_aspect="UNKNOWN",
            traffic_aspect_confidence=.95, traffic_aspect_fresh=True,
            traffic_aspect_sample_id=index,
            rgb_green_down_verified=False))
        assert generic.override_active and generic.drive_override == 0.0
    for index in range(5, 9):
        conflict = machine.update(sample(
            index*.1, section="11", traffic_aspect="GREEN_DOWN",
            traffic_aspect_confidence=.95, traffic_aspect_fresh=True,
            traffic_aspect_sample_id=index,
            rgb_green_down_verified=False))
        assert conflict.override_active and conflict.drive_override == 0.0
    assert not machine.exit_release_latched


def test_acceleration_uses_manifest_class_not_guessed_alias():
    root = Path(__file__).parents[2]
    manifest = (root / "camera_yolo_inference" / "config" /
                "class_manifest.yaml").read_text()
    source = (Path(__file__).parents[1] / "camera_navigation" /
              "camera_mission_perception_node.py").read_text()
    assert "traffic20" in manifest
    assert 'name == "traffic20"' in source
    assert "speed_20_sign" not in source


def test_required_diagnostics_are_present_and_json_finite():
    diagnostics = MissionDecisionMachine().update(sample()).diagnostics
    required = {"route_mode", "mapped_section", "stop_line_phase",
        "first_line_crossed", "uphill_confirmed_after_first_line",
        "second_line_tracked", "second_line_distance_m",
        "first_to_second_travel_m", "vehicle_stationary",
        "stationary_duration_sec", "slope_hold_elapsed_sec",
        "slope_hold_remaining_sec", "traffic_aspect",
        "traffic_aspect_confidence", "traffic_aspect_age_sec",
        "left_signal_confirmed", "exit_signal_state",
        "exit_release_latched", "acceleration_sign_latched",
        "acceleration_straight_confirmed"}
    assert required <= diagnostics.keys()
    json.dumps(diagnostics, allow_nan=False)


@pytest.mark.parametrize("light,distance,active,drive", [
    ("R", 3.0, False, 0.0), ("R", 1.0, True, 0.0),
    ("G", 3.0, False, 0.0), ("G", 1.0, False, 0.0),
    ("UNKNOWN", 3.0, False, 0.0), ("UNKNOWN", 1.0, True, 0.0),
])
def test_intersection_before_line_table(light, distance, active, drive):
    machine = MissionDecisionMachine()
    result = machine.update(sample(
        section="INTERSECTION", traffic_light=light,
        stop_line_detected=True, stop_line_distances_m=(distance,)))
    assert result.override_active is active
    assert result.drive_override == drive


def test_intersection_after_line_never_abruptly_stops_for_red_or_unknown():
    machine = MissionDecisionMachine()
    machine.update(sample(section="INTERSECTION", traffic_light="G",
                          stop_line_distances_m=(.1,)))
    crossed = machine.update(sample(.1, "INTERSECTION", traffic_light="R",
                                    stop_line_distances_m=(-.01,)))
    assert machine.intersection_line_crossed
    assert not crossed.override_active
    unknown = machine.update(sample(.2, "INTERSECTION",
                                    traffic_light="UNKNOWN",
                                    stop_line_distances_m=()))
    assert not unknown.override_active


def test_intersection_detection_loss_does_not_fabricate_crossing():
    machine = MissionDecisionMachine()
    machine.update(sample(section="INTERSECTION", traffic_light="R",
                          stop_line_distances_m=(.5,)))
    result = machine.update(sample(.1, "INTERSECTION", traffic_light="R",
                                   stop_line_distances_m=()))
    assert not machine.intersection_line_crossed
    assert result.override_active and result.drive_override == 0.0


def test_green_detection_loss_keeps_planner_and_negative_nearest_crosses():
    machine = MissionDecisionMachine()
    green = machine.update(sample(section="INTERSECTION", traffic_light="G",
                                  stop_line_distances_m=()))
    assert not green.override_active
    crossed = machine.update(sample(.1, "INTERSECTION", traffic_light="R",
                                    stop_line_distances_m=(-.02, 2.0)))
    assert machine.intersection_line_crossed and not crossed.override_active


def braking_machine(deceleration=1.0):
    return MissionDecisionMachine(MissionDecisionConfig(
        calibrated_deceleration_mps2=deceleration,
        total_control_latency_sec=.2,
        stop_distance_safety_margin_m=.1,
        intersection_target_stop_margin_m=1.0,
        intersection_detection_range_m=2.0,
        minimum_line_clearance_m=.2))


def red_input(distance, speed, now=0.0, **kwargs):
    values = dict(traffic_light="R", stop_line_detected=True,
                  stop_line_distances_m=(distance,), encoder_valid=True,
                  encoder_count=100, speed_valid=True,
                  current_speed_mps=speed, distance_valid=True,
                  distance_m=10.0, traffic_sample_id=int(now*100)+1)
    values.update(kwargs)
    return sample(now, "INTERSECTION", **values)


def test_red_at_two_metres_can_stop_at_one_metre_target():
    machine = braking_machine()
    result = machine.update(red_input(2.0, .5))
    assert result.state == "LATE_RED_CAN_STOP_AT_TARGET"
    assert result.drive_override == 1.0
    assert result.diagnostics["available_to_target_m"] == pytest.approx(1.0)
    assert result.diagnostics["required_stop_m"] == pytest.approx(.325)
    assert result.diagnostics["can_stop_at_target"]
    decelerating = machine.update(red_input(1.5, .4, .1, distance_m=10.25))
    assert decelerating.state == "RED_DECELERATE"
    assert decelerating.drive_override == 1.0
    target = machine.update(red_input(1.0, .2, .2, distance_m=10.5))
    assert target.state == "RED_STOPPED"
    assert target.drive_override == 0.0 and machine.red_stop_latched


def test_late_red_misses_target_but_stops_before_line():
    machine = braking_machine()
    result = machine.update(red_input(1.2, .5))
    assert result.state == "LATE_RED_STOP_BEFORE_LINE"
    assert result.drive_override == 0.0 and machine.red_stop_latched
    assert not result.diagnostics["can_stop_at_target"]
    assert result.diagnostics["can_stop_before_line"]


def test_very_late_red_commits_and_never_stops_inside_intersection():
    machine = braking_machine()
    commit = machine.update(red_input(.3, 1.0))
    assert commit.state == "LATE_RED_COMMIT_TO_CROSS"
    assert commit.drive_override == 1.0 and machine.commit_to_cross_latched
    red = machine.update(red_input(.2, 1.0, .1, distance_m=10.1))
    assert red.drive_override == 1.0
    unknown = machine.update(sample(
        .2, "INTERSECTION", traffic_light="UNKNOWN",
        stop_line_distances_m=(.1,), encoder_valid=True, speed_valid=True,
        current_speed_mps=1.0, distance_valid=True, distance_m=10.2,
        traffic_sample_id=3))
    assert unknown.drive_override == 1.0
    crossed = machine.update(sample(
        .3, "INTERSECTION", traffic_light="R", stop_line_distances_m=(-.01,),
        encoder_valid=True, speed_valid=True, current_speed_mps=1.0,
        distance_valid=True, distance_m=10.31, traffic_sample_id=4))
    assert crossed.state == "LINE_CROSSED" and not crossed.override_active
    after = machine.update(sample(
        .4, "INTERSECTION", traffic_light="R", stop_line_distances_m=(),
        encoder_valid=True, speed_valid=True, current_speed_mps=1.0,
        distance_valid=True, distance_m=10.4, traffic_sample_id=5))
    assert after.state == "INTERSECTION_COMPLETE"
    assert not after.override_active


def test_commit_can_latch_crossing_from_cumulative_distance_after_mask_loss():
    machine = braking_machine()
    machine.update(red_input(.3, 1.0, distance_m=5.0))
    crossed = machine.update(sample(
        .2, "INTERSECTION", traffic_light="UNKNOWN",
        stop_line_distances_m=(), stop_line_tf_valid=False,
        encoder_valid=True, speed_valid=True, current_speed_mps=1.0,
        distance_valid=True, distance_m=5.31, traffic_sample_id=2))
    assert crossed.state == "LINE_CROSSED"
    assert machine.line_crossed_latched and not crossed.override_active


@pytest.mark.parametrize("missing", [
    "encoder", "speed", "speed_stale", "deceleration", "depth_tf"])
def test_late_red_unknown_feasibility_stops_and_never_commits(missing):
    machine = braking_machine(0.0 if missing == "deceleration" else 1.0)
    kwargs = {}
    if missing == "encoder": kwargs["encoder_valid"] = False
    if missing in ("speed", "speed_stale"):
        kwargs["speed_valid"] = False
    if missing == "depth_tf": kwargs["stop_line_tf_valid"] = False
    result = machine.update(red_input(1.5, .5, **kwargs))
    assert result.override_active and result.drive_override == 0.0
    assert result.failure_reason == "STOP_FEASIBILITY_UNKNOWN"
    assert not machine.commit_to_cross_latched


def test_red_stopped_holds_r_and_unknown_then_requires_three_new_green_samples():
    machine = braking_machine()
    stopped = machine.update(red_input(1.0, 0.0, traffic_sample_id=1))
    assert stopped.state == "RED_STOPPED" and machine.red_stop_latched
    for index, light in enumerate(("R", "UNKNOWN", "G", "G"), start=2):
        held = machine.update(sample(
            index*.1, "INTERSECTION", traffic_light=light,
            stop_line_distances_m=(), stop_line_tf_valid=False,
            traffic_sample_id=index))
        assert held.override_active and held.drive_override == 0.0
    released = machine.update(sample(
        .6, "INTERSECTION", traffic_light="G", stop_line_distances_m=(),
        stop_line_tf_valid=False, traffic_sample_id=6))
    assert released.state == "GREEN_PROCEED" and not released.override_active
    assert machine.green_release_latched


def test_intersection_stale_input_is_safety_stop():
    machine = MissionDecisionMachine()
    result = machine.update(sample(section="INTERSECTION", input_fresh=False,
                                   traffic_light="G",
                                   stop_line_distances_m=(3.0,)))
    assert result.safety_blocked and result.drive_override == 0.0


def test_red_left_permission_matches_green_only_for_stable_left_turn():
    left = MissionDecisionMachine()
    left.update(sample(0.0, "INTERSECTION", traffic_light="LEFT",
                       required_steering_deg=-3.0,
                       stop_line_distances_m=(1.0,)))
    permitted = left.update(sample(
        .26, "INTERSECTION", traffic_light="LEFT",
        required_steering_deg=-3.0, stop_line_distances_m=(1.0,)))
    assert permitted.state == "GREEN_PROCEED"
    assert not permitted.override_active
    assert permitted.diagnostics["intersection_permission_source"] == "RED_LEFT"

    straight = MissionDecisionMachine()
    blocked = straight.update(sample(
        0.0, "INTERSECTION", traffic_light="LEFT",
        required_steering_deg=0.0, stop_line_distances_m=(1.0,)))
    assert blocked.override_active and blocked.drive_override == 0.0
    assert not blocked.diagnostics["intersection_proceed_permitted"]


def test_acceleration_requires_sign_and_stable_straight():
    machine = MissionDecisionMachine()
    one_frame_false = machine.update(sample(section="ACCELERATION",
                                            sign_detected=False))
    assert one_frame_false.state == "ACCEL_IDLE"
    armed = machine.update(sample(.1, "ACCELERATION", sign_detected=True))
    assert machine.acceleration_armed and armed.drive_override != 3.0
    short = machine.update(sample(.59, "ACCELERATION", sign_detected=True))
    assert short.drive_override != 3.0
    fast = machine.update(sample(.61, "ACCELERATION", sign_detected=True))
    assert fast.state == "ACCEL_HIGH_SPEED" and fast.drive_override == 3.0


def accelerate(machine):
    machine.update(sample(0.0, "ACCELERATION", sign_detected=True,
                          required_steering_deg=0.0))
    return machine.update(sample(.51, "ACCELERATION", sign_detected=True,
                                 required_steering_deg=0.0))


def test_acceleration_small_noise_turn_then_post_turn_sequence():
    machine = MissionDecisionMachine()
    assert accelerate(machine).drive_override == 3.0
    noise = machine.update(sample(.6, "ACCELERATION", sign_detected=True,
                                  required_steering_deg=1.5))
    assert noise.drive_override == 3.0
    machine.update(sample(.7, "ACCELERATION", sign_detected=True,
                          required_steering_deg=3.0))
    turn = machine.update(sample(.96, "ACCELERATION", sign_detected=True,
                                 required_steering_deg=3.0))
    assert turn.drive_override == 1.0
    assert turn.diagnostics["turn_direction"] == "RIGHT"
    zero = machine.update(sample(1.0, "ACCELERATION", sign_detected=True,
                                 required_steering_deg=0.0))
    assert zero.drive_override == 1.0
    cruise = machine.update(sample(1.51, "ACCELERATION", sign_detected=True,
                                   required_steering_deg=0.0))
    assert cruise.drive_override == 2.0
    never_fast = machine.update(sample(
        2.0, "ACCELERATION", sign_detected=True,
        required_steering_deg=0.0))
    assert never_fast.drive_override == 2.0


def test_left_turn_is_reported_and_uses_slow_stage():
    machine = MissionDecisionMachine(); accelerate(machine)
    machine.update(sample(.6, "ACCELERATION", sign_detected=True,
                          required_steering_deg=-3.0))
    result = machine.update(sample(.86, "ACCELERATION", sign_detected=True,
                                   required_steering_deg=-3.0))
    assert result.drive_override == 1.0
    assert result.diagnostics["turn_direction"] == "LEFT"


def test_drive_and_steering_control_contracts_fail_closed():
    for invalid in (-2.0, .5, 4.0):
        result = MissionDecisionMachine().update(sample(planner_drive=invalid))
        assert result.failure_reason == "DRIVE_STAGE_INVALID"
    reverse = MissionDecisionMachine().update(sample(
        section="ACCELERATION", planner_drive=-1.0))
    assert reverse.failure_reason == "REVERSE_FORBIDDEN"
    over = MissionDecisionMachine().update(sample(
        section="ACCELERATION", required_steering_deg=27.01))
    assert over.failure_reason == "STEERING_RANGE_INVALID"


def test_bev_internal_sign_is_converted_once_to_vehicle_contract():
    assert vehicle_steering_from_bev(3.0) == -3.0  # LEFT
    assert vehicle_steering_from_bev(-4.0) == 4.0  # RIGHT


def test_forward_mission_overrides_never_generate_reverse_stage():
    results = [
        MissionDecisionMachine().update(sample(section="SLOPE")),
        MissionDecisionMachine().update(sample(
            section="INTERSECTION", traffic_light="UNKNOWN",
            stop_line_distances_m=(1.5,))),
        accelerate(MissionDecisionMachine()),
    ]
    assert all(result.drive_override in (0.0, 1.0, 2.0, 3.0)
               for result in results)
    assert all(result.drive_override != -1.0 for result in results)


def test_planner_invalid_has_priority_over_fast_mission_request():
    machine = MissionDecisionMachine(); accelerate(machine)
    result = machine.update(sample(
        .6, "ACCELERATION", sign_detected=True,
        planner_valid=False, planner_state="INVALID", planner_drive=2.0))
    assert result.safety_blocked
    assert result.override_active and result.drive_override == 0.0
    assert result.effective_drive == 0.0
    assert result.diagnostics["mission_override_requested"]
    assert result.diagnostics["mission_drive_requested"] == 3.0


def test_timestamp_rewind_and_explicit_reset_clear_latches():
    machine = MissionDecisionMachine(); accelerate(machine)
    rewind = replace(sample(.7, "ACCELERATION", sign_detected=True), stamp=1.0)
    result = machine.update(rewind)
    assert result.safety_blocked and result.failure_reason == "TIMESTAMP_REWIND"
    assert not machine.acceleration_armed
    machine.reset()
    assert machine.section == "NORMAL" and machine.state == "NORMAL_IDLE"


def test_input_timeout_resets_acceleration_latches_and_stays_stopped():
    machine = MissionDecisionMachine(); accelerate(machine)
    timeout = machine.update(sample(.6, "ACCELERATION", input_fresh=False,
                                    sign_detected=True))
    assert timeout.safety_blocked and timeout.drive_override == 0.0
    assert not machine.acceleration_armed and not machine.high_speed_completed


def test_ros_decision_node_has_no_control_or_mcu_publisher():
    source = Path(__file__).parents[1] / "camera_navigation" / \
        "camera_mission_decision_node.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    publisher_topics = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == "create_publisher" and len(node.args) >= 2
                and isinstance(node.args[1], ast.Constant)):
            publisher_topics.append(node.args[1].value)
    assert publisher_topics
    assert not any(topic in ("/camera_drive", "/camera_wheel") or
                   str(topic).startswith("/mcu") for topic in publisher_topics)


def test_validation_defaults_do_not_change_planner_contract():
    root = Path(__file__).parents[1]
    text = "\n".join(path.read_text(encoding="utf-8") for path in (
        root / "config" / "mission_decision.yaml",
        root / "launch" / "camera_mission_decision.launch.py",
        root / "launch" / "camera_mission_validation.launch.py"))
    assert "planner_variant" not in text
    assert "line_track_mode" not in text
    assert "road_boundary_fallback" not in text
    assert "debug_overlay_enabled: false" in text
    config = (root / "config" / "mission_decision.yaml").read_text(
        encoding="utf-8")
    assert "calibrated_deceleration_mps2: 0.0" in config
    assert "mode_topic: /mcu/current_mode" in config
    assert "section_topic: /camera/mission/section" in config
    assert "traffic_state_topic: /camera/traffic_light_fused/state" in config
    assert "traffic_aspect_topic: /camera/traffic_light_fused/aspect" in config
    assert "/camera/traffic_light_rgb/" not in config
    for contract in ("encoder_topic: /mcu/encoder",
                     "speed_topic: /mcu/speed_mps",
                     "distance_topic: /mcu/distance_m"):
        assert contract in config


def test_camera_workspace_consumes_mcu_motion_without_recomputing_it():
    workspace = Path(__file__).parents[2]
    pixel = (Path(__file__).parents[1] / "camera_navigation" /
             "camera_pixel_controller_node.py").read_text(encoding="utf-8")
    pursuit = (workspace / "race_control" / "race_control" /
               "pure_pursuit_node.py").read_text(encoding="utf-8")
    legacy = (workspace / "race_vehicle_interface" /
              "race_vehicle_interface" /
              "arduino_serial_bridge_node.py").read_text(encoding="utf-8")
    launch = (workspace / "race_control" / "launch" /
              "autonomy_stack.launch.py").read_text(encoding="utf-8")
    assert '"distance_topic": "/mcu/distance_m"' in pixel
    assert "Odometry" not in pixel and "on_odom" not in pixel
    assert '"speed_feedback_topic": "/mcu/speed_mps"' in pursuit
    forbidden = ("encoder_delta_to_distance_m", "encoder_delta_to_speed_mps",
                 "rpm_to_speed_mps")
    assert not any(name in pixel+pursuit+legacy for name in forbidden)
    assert "arduino_serial_bridge_node" not in launch
    assert "vehicle_interface_node" not in launch
