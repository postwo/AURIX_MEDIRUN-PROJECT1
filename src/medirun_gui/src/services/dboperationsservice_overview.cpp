#include "dboperationsoverviewservice.h"
#include "dbconnection.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSqlError>
#include <QSqlQuery>
#include <QVariant>

namespace {

int percentFromCounts(int connected, int total)
{
    if (total <= 0) {
        return 0;
    }
    return qRound(100.0 * connected / total);
}

QString healthKindForPercent(int percent)
{
    if (percent >= 95) {
        return QStringLiteral("green");
    }
    if (percent >= 80) {
        return QStringLiteral("amber");
    }
    return QStringLiteral("red");
}

// 진행 중인 미션이 있으면 미션 상태를, 없으면 최근 robot_session 상태를 표시한다.
void statusLabelAndKind(const QString &missionStatus, const QString &sessionStatus, QString &label, QString &kind)
{
    if (!missionStatus.isEmpty()) {
        if (missionStatus == QStringLiteral("queued")) {
            label = QStringLiteral("● 대기열");
            kind = QStringLiteral("muted");
        } else if (missionStatus == QStringLiteral("replanning")) {
            label = QStringLiteral("● 재계획");
            kind = QStringLiteral("amber");
        } else if (missionStatus == QStringLiteral("dispensing")) {
            label = QStringLiteral("● 반출 중");
            kind = QStringLiteral("cyan");
        } else if (missionStatus == QStringLiteral("arrived")) {
            label = QStringLiteral("● 도착");
            kind = QStringLiteral("cyan");
        } else {
            // dispatched, navigating
            label = QStringLiteral("● 출동 중");
            kind = QStringLiteral("cyan");
        }
        return;
    }

    if (sessionStatus == QStringLiteral("running")) {
        label = QStringLiteral("● 운행 중");
        kind = QStringLiteral("cyan");
    } else if (sessionStatus == QStringLiteral("paused")) {
        label = QStringLiteral("● 일시정지");
        kind = QStringLiteral("amber");
    } else if (sessionStatus == QStringLiteral("failed") || sessionStatus == QStringLiteral("aborted")) {
        label = QStringLiteral("● 오류");
        kind = QStringLiteral("red");
    } else {
        label = QStringLiteral("● 대기");
        kind = QStringLiteral("muted");
    }
}

void severityLabelAndKind(const QString &severity, QString &label, QString &kind)
{
    if (severity == QStringLiteral("critical")) {
        label = QStringLiteral("CRITICAL");
        kind = QStringLiteral("red");
    } else if (severity == QStringLiteral("error")) {
        label = QStringLiteral("ERROR");
        kind = QStringLiteral("red");
    } else if (severity == QStringLiteral("warn")) {
        label = QStringLiteral("WARN");
        kind = QStringLiteral("amber");
    } else {
        label = QStringLiteral("INFO");
        kind = QStringLiteral("cyan");
    }
}

} // namespace


bool DbOperationsOverviewService::fetchSummary(OperationsSummaryMetrics &metrics, QString &errorMessage)
{
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery robotQuery(QStringLiteral(
        "SELECT "
        "  (SELECT COUNT(*) FROM robot WHERE deregistered_at IS NULL) AS registered_robots, "
        "  (SELECT COUNT(*) FROM ( "
        "     SELECT DISTINCT ON (robot_id) robot_id, status "
        "     FROM connection_status_log "
        "     WHERE target = 'ros_bridge' "
        "     ORDER BY robot_id, changed_at DESC, id DESC "
        "   ) latest "
        "   JOIN robot r ON r.id = latest.robot_id AND r.deregistered_at IS NULL "
        "   WHERE latest.status = 'connected' "
        "  ) AS connected_robots"));
    if (!robotQuery.isActive() || !robotQuery.next()) {
        errorMessage = QStringLiteral("DB 조회 실패: %1").arg(robotQuery.lastError().text());
        return false;
    }
    metrics.registeredRobots = robotQuery.value(QStringLiteral("registered_robots")).toInt();
    metrics.connectedRobots = robotQuery.value(QStringLiteral("connected_robots")).toInt();

    QSqlQuery missionQuery(QStringLiteral(
        "SELECT "
        "  COUNT(*) FILTER (WHERE status NOT IN ('completed', 'cancelled', 'failed')) AS active_missions, "
        "  COUNT(*) FILTER (WHERE status NOT IN ('completed', 'cancelled', 'failed') AND is_emergency) AS active_emergency_missions, "
        "  COUNT(*) FILTER (WHERE created_at::date = CURRENT_DATE) AS today_calls, "
        "  COUNT(*) FILTER (WHERE created_at::date = CURRENT_DATE AND is_emergency) AS today_emergency_calls "
        "FROM mission"));
    if (!missionQuery.isActive() || !missionQuery.next()) {
        errorMessage = QStringLiteral("DB 조회 실패: %1").arg(missionQuery.lastError().text());
        return false;
    }
    metrics.activeMissions = missionQuery.value(QStringLiteral("active_missions")).toInt();
    metrics.activeEmergencyMissions = missionQuery.value(QStringLiteral("active_emergency_missions")).toInt();
    metrics.todayCalls = missionQuery.value(QStringLiteral("today_calls")).toInt();
    metrics.todayEmergencyCalls = missionQuery.value(QStringLiteral("today_emergency_calls")).toInt();

    QSqlQuery eventQuery(QStringLiteral(
        "SELECT COUNT(*) FROM system_event "
        "WHERE severity IN ('warn', 'error', 'critical') AND created_at >= NOW() - INTERVAL '24 hours'"));
    if (!eventQuery.isActive() || !eventQuery.next()) {
        errorMessage = QStringLiteral("DB 조회 실패: %1").arg(eventQuery.lastError().text());
        return false;
    }
    metrics.safetyEventsLast24h = eventQuery.value(0).toInt();

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::fetchRobotStatus(QVector<RobotStatusRow> &rows, QString &errorMessage)
{
    rows.clear();

    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery query(QStringLiteral(
        "SELECT "
        "  r.name AS robot_name, "
        "  sess.status AS session_status, "
        "  dr.battery_percent AS battery_percent, "
        "  cb.zone_name AS zone_name, "
        "  m.status AS mission_status "
        "FROM robot r "
        "LEFT JOIN LATERAL ( "
        "  SELECT status FROM robot_session "
        "  WHERE robot_session.robot_id = r.id "
        "  ORDER BY started_at DESC LIMIT 1 "
        ") sess ON true "
        "LEFT JOIN LATERAL ( "
        "  SELECT id FROM driving_session ds "
        "  WHERE ds.robot_id = r.id "
        "  ORDER BY ds.started_at DESC LIMIT 1 "
        ") latest_ds ON true "
        "LEFT JOIN LATERAL ( "
        "  SELECT battery_percent FROM driving_record "
        "  WHERE driving_record.driving_session_id = latest_ds.id "
        "  ORDER BY recorded_at DESC LIMIT 1 "
        ") dr ON true "
        "LEFT JOIN LATERAL ( "
        "  SELECT status, call_button_id FROM mission "
        "  WHERE mission.robot_id = r.id AND mission.status NOT IN ('completed', 'cancelled', 'failed') "
        "  ORDER BY created_at DESC LIMIT 1 "
        ") m ON true "
        "LEFT JOIN call_button cb ON cb.id = m.call_button_id "
        "WHERE r.deregistered_at IS NULL "
        "ORDER BY r.name"));
    if (!query.isActive()) {
        errorMessage = QStringLiteral("DB 조회 실패: %1").arg(query.lastError().text());
        return false;
    }

    while (query.next()) {
        RobotStatusRow row;
        row.robotName = query.value(QStringLiteral("robot_name")).toString();

        const QString missionStatus = query.value(QStringLiteral("mission_status")).toString();
        const QString sessionStatus = query.value(QStringLiteral("session_status")).toString();
        statusLabelAndKind(missionStatus, sessionStatus, row.statusLabel, row.statusKind);

        const QVariant battery = query.value(QStringLiteral("battery_percent"));
        row.batteryText = battery.isNull() ? QStringLiteral("-") : QStringLiteral("%1%").arg(qRound(battery.toDouble()));

        const QString zoneName = query.value(QStringLiteral("zone_name")).toString();
        row.routeText = zoneName.isEmpty() ? QStringLiteral("-") : QStringLiteral("목적지: %1").arg(zoneName);

        rows.push_back(row);
    }

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::fetchSystemHealth(QVector<SystemHealthRow> &rows, QString &errorMessage)
{
    rows.clear();

    if (!openDbConnection(errorMessage)) {
        return false;
    }

    // Nav2 연결/카메라 연결은 여기서 다루지 않는다 — connection_status_log의
    // target enum(ros_bridge/database/serial/camera/unity)에 "nav2"가 없고,
    // 아무 코드도 target='database'/'camera' 로그를 쓰지 않아 DB로는 항상
    // 0%가 나온다. 그 둘은 페이지가 NavigationService/CameraFeedProvider의
    // 실시간 시그널로 직접 채운다 (OperationsOverviewPage::applyNav2Connection
    // /applyCameraConnection).
    // 등록해제된 로봇의 예전 connection_status_log 행은 robot_id가 남아있어도
    // 더 이상 "현재 연결 상태"로 셀 이유가 없으므로, 지금 등록돼 있는(활성)
    // 로봇으로만 분모를 제한한다 (fetchSummary의 connected_robots 계산과 동일 조건).
    QSqlQuery connQuery(QStringLiteral(
        "SELECT "
        "  COUNT(*) FILTER (WHERE latest.status = 'connected') AS ros_connected, "
        "  COUNT(*) AS ros_total "
        "FROM ( "
        "  SELECT DISTINCT ON (robot_id) robot_id, status "
        "  FROM connection_status_log "
        "  WHERE target = 'ros_bridge' "
        "  ORDER BY robot_id, changed_at DESC, id DESC "
        ") latest "
        "JOIN robot r ON r.id = latest.robot_id AND r.deregistered_at IS NULL"));
    if (!connQuery.isActive() || !connQuery.next()) {
        errorMessage = QStringLiteral("DB 조회 실패: %1").arg(connQuery.lastError().text());
        return false;
    }

    const int rosPercent = percentFromCounts(
        connQuery.value(QStringLiteral("ros_connected")).toInt(), connQuery.value(QStringLiteral("ros_total")).toInt());

    rows.push_back({QStringLiteral("ROS2 통신"), rosPercent, healthKindForPercent(rosPercent)});
    // 이 지점까지 왔다는 것 자체가 방금 DB 쿼리가 성공했다는 뜻이라 100으로 둔다.
    rows.push_back({QStringLiteral("DB 동기화"), 100, QStringLiteral("green")});

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::fetchRecentEvents(int limit, QVector<OperationsEventRow> &rows, QString &errorMessage)
{
    rows.clear();

    if (!openDbConnection(errorMessage)) {
        return false;
    }

    // system_event만으로는 ROS 연결 끊김/재연결 같은 연결 상태 변화가 안 보인다
    // (TelemetryLogRecorder가 그건 connection_status_log에만 기록함) — 실시간
    // 관제 이벤트 스트림이 실제로 "지금 무슨 일이 있었는지" 다 보여주려면 두
    // 테이블을 시간순으로 합쳐야 한다.
    QSqlQuery query;
    query.prepare(QStringLiteral(
        "SELECT created_at, severity, message, robot_name FROM ("
        "  SELECT se.created_at AS created_at, se.severity AS severity, se.message AS message, "
        "         r.name AS robot_name "
        "  FROM system_event se "
        "  LEFT JOIN robot_session rs ON rs.id = se.session_id "
        "  LEFT JOIN robot r ON r.id = rs.robot_id "
        "  UNION ALL "
        "  SELECT csl.changed_at AS created_at, "
        "         CASE csl.status "
        "           WHEN 'connected' THEN 'info' "
        "           WHEN 'reconnecting' THEN 'warn' "
        "           ELSE 'error' "
        "         END AS severity, "
        "         (CASE csl.target "
        "            WHEN 'ros_bridge' THEN 'ROS2' "
        "            WHEN 'database' THEN 'DB' "
        "            WHEN 'serial' THEN '아두이노' "
        "            WHEN 'camera' THEN '카메라' "
        "            WHEN 'unity' THEN 'Unity' "
        "            ELSE csl.target "
        "          END || ' ' || "
        "          CASE csl.status "
        "            WHEN 'connected' THEN '연결됨' "
        "            WHEN 'disconnected' THEN '연결 끊김' "
        "            WHEN 'reconnecting' THEN '재연결 중' "
        "            ELSE '연결 오류' "
        "          END || "
        "          COALESCE(' · ' || NULLIF(csl.detail, ''), '')) AS message, "
        "         r.name AS robot_name "
        "  FROM connection_status_log csl "
        "  LEFT JOIN robot r ON r.id = csl.robot_id "
        "  UNION ALL "
        // mission_cancel/mission_return_home/mission_retry/mission_accept는
        // MissionStorageService가 system_event에도 같이 남기므로(위 첫 번째
        // SELECT), 여기서 또 넣으면 같은 이벤트가 두 번 뜬다 — 제외한다.
        "  SELECT ual.created_at AS created_at, 'info' AS severity, "
        "         (CASE ual.action_type "
        "            WHEN 'recording_start' THEN '녹화 시작' "
        "            WHEN 'recording_stop' THEN '녹화 종료' "
        "            WHEN 'slam_start' THEN 'SLAM 매핑 시작' "
        "            WHEN 'slam_stop' THEN 'SLAM 매핑 종료' "
        "            WHEN 'map_save' THEN '지도 저장' "
        "            WHEN 'map_load' THEN '지도 불러오기' "
        "            WHEN 'waypoint_set_create' THEN '웨이포인트 세트 생성' "
        "            WHEN 'waypoint_set_delete' THEN '웨이포인트 세트 삭제' "
        "            WHEN 'waypoint_set_run' THEN '웨이포인트 경로 주행' "
        "            WHEN 'mode_change' THEN '모드 변경' "
        "            ELSE ual.action_type "
        "          END || COALESCE(' · ' || NULLIF(ual.issued_by, ''), '')) AS message, "
        "         r.name AS robot_name "
        "  FROM user_action_log ual "
        "  LEFT JOIN robot r ON r.id = ual.robot_id "
        "  WHERE ual.action_type NOT IN ('mission_cancel', 'mission_return_home', 'mission_retry', 'mission_accept') "
        "  UNION ALL "
        "  SELECT f.created_at AS created_at, 'info' AS severity, "
        "         (CASE f.file_category "
        "            WHEN 'snapshot' THEN '스냅샷 저장' "
        "            WHEN 'video' THEN '녹화 영상 저장' "
        "            ELSE COALESCE(f.file_category, '파일 저장') "
        "          END || ' · ' || f.original_filename "
        "          || COALESCE(' · ' || NULLIF(f.uploaded_by, ''), '')) AS message, "
        "         NULL AS robot_name "
        "  FROM files f "
        ") combined_events "
        // DB에는 계속 다 쌓이지만(삭제 안 함), 화면에는 미션 관제 "오늘 처리
        // 이력"과 동일하게 최근 24시간 안의 것만 보여준다.
        "WHERE created_at >= NOW() - INTERVAL '24 hours' "
        "ORDER BY created_at DESC "
        "LIMIT :limit"));
    query.bindValue(QStringLiteral(":limit"), limit);
    if (!query.exec()) {
        errorMessage = QStringLiteral("DB 조회 실패: %1").arg(query.lastError().text());
        return false;
    }

    while (query.next()) {
        OperationsEventRow row;
        row.occurredAt = dbTimestampToLocal(query.value(QStringLiteral("created_at")));
        severityLabelAndKind(query.value(QStringLiteral("severity")).toString(), row.severityLabel, row.kind);

        const QString robotName = query.value(QStringLiteral("robot_name")).toString();
        const QString message = query.value(QStringLiteral("message")).toString();
        row.message = robotName.isEmpty() ? message : QStringLiteral("%1 · %2").arg(message, robotName);

        rows.push_back(row);
    }

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::fetchSystemEvents(
    const QDate &startDate, const QDate &endDate, const QString &keyword, QVector<OperationsEventRow> &rows,
    QString &errorMessage)
{
    rows.clear();

    if (!openDbConnection(errorMessage)) {
        return false;
    }

    // endDate까지 "포함"해서 보여줘야 하므로 다음 날 00:00을 배타적 상한으로 쓴다.
    const QDateTime start(startDate.startOfDay());
    const QDateTime end(endDate.addDays(1).startOfDay());
    const QString trimmedKeyword = keyword.trimmed();

    // 이름 있는 바인딩(:keyword)을 같은 문장 안에서 두 번 이상 쓰면 QPSQL
    // 드라이버가 각 자리마다 다른 파라미터로 취급해서 하나만 실제로 바인딩된
    // 값이 들어가고 나머지는 비어버리는 문제가 있었다(검색어를 쳐도 목록이
    // 하나도 안 바뀌던 원인) — 자리마다 이름을 따로 줘서 피한다. LIKE 패턴의
    // '%'도 SQL에서 문자열 연결(||)하지 않고 C++에서 미리 붙여서 바인딩한다.
    QString sql = QStringLiteral(
        "SELECT created_at, severity, message, event_type "
        "FROM system_event "
        "WHERE created_at >= :start AND created_at < :end");
    if (!trimmedKeyword.isEmpty()) {
        sql += QStringLiteral(" AND (message ILIKE :keywordMessage OR event_type ILIKE :keywordEventType)");
    }
    sql += QStringLiteral(" ORDER BY created_at DESC");

    QSqlQuery query;
    query.prepare(sql);
    query.bindValue(QStringLiteral(":start"), start);
    query.bindValue(QStringLiteral(":end"), end);
    if (!trimmedKeyword.isEmpty()) {
        // QString::arg()는 리터럴 '%'와 플레이스홀더 "%1"이 섞이면 파싱이
        // 헷갈릴 수 있어(예: "%%%1%%") 그냥 이어붙인다.
        const QString likePattern = QLatin1Char('%') + trimmedKeyword + QLatin1Char('%');
        query.bindValue(QStringLiteral(":keywordMessage"), likePattern);
        query.bindValue(QStringLiteral(":keywordEventType"), likePattern);
    }
    if (!query.exec()) {
        errorMessage = QStringLiteral("DB 조회 실패: %1").arg(query.lastError().text());
        return false;
    }

    while (query.next()) {
        OperationsEventRow row;
        row.occurredAt = dbTimestampToLocal(query.value(QStringLiteral("created_at")));
        severityLabelAndKind(query.value(QStringLiteral("severity")).toString(), row.severityLabel, row.kind);
        row.message = QStringLiteral("%1 · %2").arg(
            query.value(QStringLiteral("event_type")).toString(), query.value(QStringLiteral("message")).toString());
        rows.push_back(row);
    }

    errorMessage.clear();
    return true;
}
