from medirun_server.robot_health import RobotHealth


def ready_robot(now=0.0):
    health = RobotHealth("bot")
    health.nav2_available = True
    health.update_pose(0.0, 0.0, 0.0, now)
    health.update_battery(80.0, now)
    health.update_payload(True, now)
    return health


def test_ready_when_all_conditions_met():
    assert ready_robot().readiness_text(1.0) == "READY"


def test_offline_after_heartbeat_timeout():
    health = ready_robot(0.0)
    assert "ROBOT_OFFLINE" in health.blockers(10.0)
    # emergency_dispatcher 는 "READY" 포함 여부만 보므로 차단 문구에 READY 가 들어가면 안 된다
    assert "READY" not in health.readiness_text(10.0)


def test_battery_ratio_and_levels():
    health = ready_robot()
    health.update_battery(0.25, 0.0)  # REP 비율(0~1) 입력
    assert health.battery_percent == 25.0
    assert "LOW_BATTERY" in health.blockers(0.0)
    assert health.battery_level() == "LOW"
    health.update_battery(15.0, 0.0)
    assert health.battery_level() == "RETURN"
    health.update_battery(5.0, 0.0)
    assert health.battery_level() == "CRITICAL"


def test_payload_missing_and_estop_block():
    health = ready_robot()
    health.update_payload(False, 0.0)
    health.update_safety("ESTOP", 0.0)
    assert {"PAYLOAD_MISSING", "EMERGENCY_STOP"} <= set(health.blockers(0.0))


def test_unknown_values_do_not_block_except_localization_and_nav2():
    health = RobotHealth("bot")
    health.touch(0.0)
    assert health.blockers(0.0) == ["NAV2_UNAVAILABLE", "LOCALIZATION_UNKNOWN"]
