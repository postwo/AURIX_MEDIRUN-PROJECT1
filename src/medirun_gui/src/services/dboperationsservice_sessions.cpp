#include "dboperationsoverviewservice.h"
#include "dbconnection.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSqlError>
#include <QSqlQuery>
#include <QVariant>


bool DbOperationsOverviewService::startRobotSession(
    int robotId,
    const QString &mode,
    const QString &sessionName,
    const QVector<NavGoalPoint> &goals,
    bool hasStartPose,
    const NavGoalPoint &startPose,
    int &sessionId,
    QString &errorMessage)
{
    sessionId = -1;

    if (!openDbConnection(errorMessage)) {
        return false;
    }

    // 이전 주행이 정상 종료(finishRobotSession)되지 못하고 남아있을 수 있으므로
    // 같은 로봇의 미완료 세션/목표를 실패 처리해 정리한 뒤 새 세션을 시작한다.
    QSqlQuery staleGoalQuery;
    staleGoalQuery.prepare(QStringLiteral(
        "UPDATE nav_goal_log SET status = 'failed', finished_at = NOW() "
        "WHERE robot_id = :robotId AND status IN ('queued', 'accepted', 'navigating')"));
    staleGoalQuery.bindValue(QStringLiteral(":robotId"), robotId);
    if (!staleGoalQuery.exec()) {
        errorMessage = QStringLiteral("이전 주행 목표 정리 실패: %1").arg(staleGoalQuery.lastError().text());
        return false;
    }

    QSqlQuery staleSessionQuery;
    staleSessionQuery.prepare(QStringLiteral(
        "UPDATE robot_session SET status = 'failed', ended_at = NOW() "
        "WHERE robot_id = :robotId AND status IN ('planned', 'running', 'paused')"));
    staleSessionQuery.bindValue(QStringLiteral(":robotId"), robotId);
    if (!staleSessionQuery.exec()) {
        errorMessage = QStringLiteral("이전 주행 세션 정리 실패: %1").arg(staleSessionQuery.lastError().text());
        return false;
    }

    QSqlQuery sessionQuery;
    sessionQuery.prepare(QStringLiteral(
        "INSERT INTO robot_session (robot_id, session_name, mode, status) "
        "VALUES (:robotId, :sessionName, :mode, 'running') RETURNING id"));
    sessionQuery.bindValue(QStringLiteral(":robotId"), robotId);
    sessionQuery.bindValue(QStringLiteral(":sessionName"), sessionName);
    sessionQuery.bindValue(QStringLiteral(":mode"), mode);
    if (!sessionQuery.exec() || !sessionQuery.next()) {
        errorMessage = QStringLiteral("주행 세션 생성 실패: %1").arg(sessionQuery.lastError().text());
        return false;
    }
    const int newSessionId = sessionQuery.value(0).toInt();

    for (int i = 0; i < goals.size(); ++i) {
        const NavGoalPoint &goal = goals.at(i);
        // 첫 웨이포인트의 시작 위치는 마지막으로 알려진 AMCL 위치, 두 번째부터는
        // 직전 웨이포인트의 목적지를 시작 위치로 근사한다.
        const bool hasStart = i == 0 ? hasStartPose : true;
        const NavGoalPoint &start = i == 0 ? startPose : goals.at(i - 1);

        QSqlQuery goalQuery;
        goalQuery.prepare(QStringLiteral(
            "INSERT INTO nav_goal_log "
            "(robot_id, session_id, waypoint_seq_no, start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw, status) "
            "VALUES (:robotId, :sessionId, :seqNo, :startX, :startY, :startYaw, :goalX, :goalY, :goalYaw, 'navigating')"));
        goalQuery.bindValue(QStringLiteral(":robotId"), robotId);
        goalQuery.bindValue(QStringLiteral(":sessionId"), newSessionId);
        goalQuery.bindValue(QStringLiteral(":seqNo"), i);
        goalQuery.bindValue(QStringLiteral(":startX"), hasStart ? QVariant(start.x) : QVariant());
        goalQuery.bindValue(QStringLiteral(":startY"), hasStart ? QVariant(start.y) : QVariant());
        goalQuery.bindValue(QStringLiteral(":startYaw"), hasStart ? QVariant(start.yaw) : QVariant());
        goalQuery.bindValue(QStringLiteral(":goalX"), goal.x);
        goalQuery.bindValue(QStringLiteral(":goalY"), goal.y);
        goalQuery.bindValue(QStringLiteral(":goalYaw"), goal.yaw);
        if (!goalQuery.exec()) {
            errorMessage = QStringLiteral("주행 목표 기록 실패: %1").arg(goalQuery.lastError().text());
            return false;
        }
    }

    sessionId = newSessionId;
    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::markGoalReached(int sessionId, int reachedIndex, QString &errorMessage)
{
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "UPDATE nav_goal_log SET status = 'arrived', finished_at = NOW() "
        "WHERE session_id = :sessionId AND waypoint_seq_no = :seqNo AND status = 'navigating'"));
    query.bindValue(QStringLiteral(":sessionId"), sessionId);
    query.bindValue(QStringLiteral(":seqNo"), reachedIndex);
    if (!query.exec()) {
        errorMessage = QStringLiteral("웨이포인트 도달 기록 실패: %1").arg(query.lastError().text());
        return false;
    }

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::finishRobotSession(int sessionId, bool success, bool cancelled, QString &errorMessage)
{
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    // success/cancelled 어느 쪽도 아니면 실패(failed)로 본다.
    const QString goalStatus = success ? QStringLiteral("arrived") : cancelled ? QStringLiteral("cancelled") : QStringLiteral("failed");
    const QString sessionStatus = success ? QStringLiteral("completed") : cancelled ? QStringLiteral("aborted") : QStringLiteral("failed");

    QSqlQuery goalQuery;
    goalQuery.prepare(QStringLiteral(
        "UPDATE nav_goal_log SET status = :status, finished_at = NOW() "
        "WHERE session_id = :sessionId AND status NOT IN ('arrived', 'failed', 'cancelled')"));
    goalQuery.bindValue(QStringLiteral(":status"), goalStatus);
    goalQuery.bindValue(QStringLiteral(":sessionId"), sessionId);
    if (!goalQuery.exec()) {
        errorMessage = QStringLiteral("주행 목표 마감 실패: %1").arg(goalQuery.lastError().text());
        return false;
    }

    QSqlQuery sessionQuery;
    sessionQuery.prepare(QStringLiteral(
        "UPDATE robot_session SET status = :status, ended_at = NOW() WHERE id = :sessionId"));
    sessionQuery.bindValue(QStringLiteral(":status"), sessionStatus);
    sessionQuery.bindValue(QStringLiteral(":sessionId"), sessionId);
    if (!sessionQuery.exec()) {
        errorMessage = QStringLiteral("주행 세션 마감 실패: %1").arg(sessionQuery.lastError().text());
        return false;
    }

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::startDrivingSession(int robotId, int &drivingSessionId, int &robotSessionId, QString &errorMessage)
{
    drivingSessionId = -1;
    robotSessionId = -1;

    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery sessionLookup;
    sessionLookup.prepare(QStringLiteral(
        "SELECT id FROM robot_session WHERE robot_id = :robotId AND status = 'running' "
        "ORDER BY started_at DESC LIMIT 1"));
    sessionLookup.bindValue(QStringLiteral(":robotId"), robotId);
    if (!sessionLookup.exec()) {
        errorMessage = QStringLiteral("진행 중인 세션 조회 실패: %1").arg(sessionLookup.lastError().text());
        return false;
    }

    int sessionId = -1;
    if (sessionLookup.next()) {
        sessionId = sessionLookup.value(0).toInt();
    } else {
        // 진행 중인 주행 세션이 없으면(로봇이 유휴 상태) 녹화 전용 수동 세션을
        // 새로 만든다 — 미션 주행 중이 아니어도 녹화만 따로 시작할 수 있어야 함.
        QSqlQuery createSession;
        createSession.prepare(QStringLiteral(
            "INSERT INTO robot_session (robot_id, session_name, mode, status) "
            "VALUES (:robotId, '실시간 관제 녹화', 'manual', 'running') RETURNING id"));
        createSession.bindValue(QStringLiteral(":robotId"), robotId);
        if (!createSession.exec() || !createSession.next()) {
            errorMessage = QStringLiteral("녹화용 세션 생성 실패: %1").arg(createSession.lastError().text());
            return false;
        }
        sessionId = createSession.value(0).toInt();
    }

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "INSERT INTO driving_session (robot_id, session_id) VALUES (:robotId, :sessionId) RETURNING id"));
    query.bindValue(QStringLiteral(":robotId"), robotId);
    query.bindValue(QStringLiteral(":sessionId"), sessionId);
    if (!query.exec() || !query.next()) {
        errorMessage = QStringLiteral("주행 녹화 세션 생성 실패: %1").arg(query.lastError().text());
        return false;
    }

    drivingSessionId = query.value(0).toInt();
    robotSessionId = sessionId;
    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::appendDrivingRecord(
    int drivingSessionId, double x, double y, double yaw, double linearVel, double angularVel,
    double batteryPercent, QString &errorMessage)
{
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "INSERT INTO driving_record "
        "(driving_session_id, recorded_at, pos_x, pos_y, yaw, linear_vel, angular_vel, battery_percent) "
        "VALUES (:drivingSessionId, NOW(), :x, :y, :yaw, :linearVel, :angularVel, :batteryPercent)"));
    query.bindValue(QStringLiteral(":drivingSessionId"), drivingSessionId);
    query.bindValue(QStringLiteral(":x"), x);
    query.bindValue(QStringLiteral(":y"), y);
    query.bindValue(QStringLiteral(":yaw"), yaw);
    query.bindValue(QStringLiteral(":linearVel"), linearVel);
    query.bindValue(QStringLiteral(":angularVel"), angularVel);
    query.bindValue(QStringLiteral(":batteryPercent"), batteryPercent);
    if (!query.exec()) {
        errorMessage = QStringLiteral("주행 기록 저장 실패: %1").arg(query.lastError().text());
        return false;
    }

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::finishDrivingSession(int drivingSessionId, QString &errorMessage)
{
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "UPDATE driving_session SET ended_at = NOW() WHERE id = :drivingSessionId"));
    query.bindValue(QStringLiteral(":drivingSessionId"), drivingSessionId);
    if (!query.exec()) {
        errorMessage = QStringLiteral("주행 녹화 세션 마감 실패: %1").arg(query.lastError().text());
        return false;
    }

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::startCameraSession(
    int robotSessionId, int drivingSessionId, const QString &cameraName, const QString &topicName,
    int &cameraSessionId, QString &errorMessage)
{
    cameraSessionId = -1;
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "INSERT INTO camera_session (session_id, driving_session_id, camera_name, topic_name, stream_status) "
        "VALUES (:sessionId, :drivingSessionId, :cameraName, :topicName, 'recording') RETURNING id"));
    query.bindValue(QStringLiteral(":sessionId"), robotSessionId);
    query.bindValue(QStringLiteral(":drivingSessionId"), drivingSessionId);
    query.bindValue(QStringLiteral(":cameraName"), cameraName);
    query.bindValue(QStringLiteral(":topicName"), topicName);
    if (!query.exec() || !query.next()) {
        errorMessage = QStringLiteral("카메라 세션 생성 실패: %1").arg(query.lastError().text());
        return false;
    }

    cameraSessionId = query.value(0).toInt();
    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::recordCameraRecording(
    int robotSessionId, int cameraSessionId, int durationSec, int width, int height, double fps,
    const QString &codec, int &cameraRecordingId, QString &errorMessage)
{
    cameraRecordingId = -1;
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "INSERT INTO camera_recording "
        "(session_id, camera_session_id, duration_sec, resolution_width, resolution_height, fps, codec) "
        "VALUES (:sessionId, :cameraSessionId, :durationSec, :width, :height, :fps, :codec) RETURNING id"));
    query.bindValue(QStringLiteral(":sessionId"), robotSessionId);
    query.bindValue(QStringLiteral(":cameraSessionId"), cameraSessionId);
    query.bindValue(QStringLiteral(":durationSec"), durationSec);
    query.bindValue(QStringLiteral(":width"), width);
    query.bindValue(QStringLiteral(":height"), height);
    query.bindValue(QStringLiteral(":fps"), fps);
    query.bindValue(QStringLiteral(":codec"), codec);
    if (!query.exec() || !query.next()) {
        errorMessage = QStringLiteral("카메라 녹화 기록 실패: %1").arg(query.lastError().text());
        return false;
    }

    cameraRecordingId = query.value(0).toInt();
    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::recordFile(
    const QString &entityType, const QString &entityId, const QString &fileCategory, const QString &storagePath,
    const QString &originalFilename, const QString &mimeType, const QString &uploadedBy,
    QString &errorMessage)
{
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "INSERT INTO files "
        "(entity_type, entity_id, file_category, storage_backend, storage_path, original_filename, mime_type, uploaded_by) "
        "VALUES (:entityType, :entityId, :fileCategory, 'local', :storagePath, :originalFilename, :mimeType, :uploadedBy)"));
    query.bindValue(QStringLiteral(":entityType"), entityType);
    query.bindValue(QStringLiteral(":entityId"), entityId);
    query.bindValue(QStringLiteral(":fileCategory"), fileCategory);
    query.bindValue(QStringLiteral(":storagePath"), storagePath);
    query.bindValue(QStringLiteral(":originalFilename"), originalFilename);
    query.bindValue(QStringLiteral(":mimeType"), mimeType);
    query.bindValue(QStringLiteral(":uploadedBy"), uploadedBy.isEmpty() ? QVariant() : QVariant(uploadedBy));
    if (!query.exec()) {
        errorMessage = QStringLiteral("파일 기록 실패: %1").arg(query.lastError().text());
        return false;
    }

    errorMessage.clear();
    return true;
}
