#include "dboperationsoverviewservice.h"
#include "dbconnection.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSqlError>
#include <QSqlQuery>
#include <QVariant>


bool DbOperationsOverviewService::recordConnectionStatus(
    int robotId, const QString &target, const QString &status, const QString &detail, QString &errorMessage)
{
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "INSERT INTO connection_status_log (robot_id, target, status, detail) "
        "VALUES (:robotId, :target, :status, :detail)"));
    query.bindValue(QStringLiteral(":robotId"), robotId);
    query.bindValue(QStringLiteral(":target"), target);
    query.bindValue(QStringLiteral(":status"), status);
    query.bindValue(QStringLiteral(":detail"), detail.isEmpty() ? QVariant() : QVariant(detail));
    if (!query.exec()) {
        errorMessage = QStringLiteral("연결 상태 기록 실패: %1").arg(query.lastError().text());
        return false;
    }

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::recordSystemEvent(
    const QString &eventType, const QString &severity, const QString &message, QString &errorMessage)
{
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "INSERT INTO system_event (event_type, severity, message) VALUES (:eventType, :severity, :message)"));
    query.bindValue(QStringLiteral(":eventType"), eventType);
    query.bindValue(QStringLiteral(":severity"), severity);
    query.bindValue(QStringLiteral(":message"), message);
    if (!query.exec()) {
        errorMessage = QStringLiteral("이벤트 기록 실패: %1").arg(query.lastError().text());
        return false;
    }

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::recordControlCommand(
    int robotId, const QString &commandSource, const QString &commandType, bool emergencyStopActive,
    const QString &issuedBy, QString &errorMessage)
{
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    // control_command_log.session_id가 NOT NULL이라, 진행 중(status='running')인
    // robot_session이 없으면 감사 기록 전용 수동 세션을 하나 만들어 붙인다.
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
        QSqlQuery createSession;
        createSession.prepare(QStringLiteral(
            "INSERT INTO robot_session (robot_id, session_name, mode, status) "
            "VALUES (:robotId, '실시간 관제', 'manual', 'running') RETURNING id"));
        createSession.bindValue(QStringLiteral(":robotId"), robotId);
        if (!createSession.exec() || !createSession.next()) {
            errorMessage = QStringLiteral("제어 명령용 세션 생성 실패: %1").arg(createSession.lastError().text());
            return false;
        }
        sessionId = createSession.value(0).toInt();
    }

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "INSERT INTO control_command_log "
        "(session_id, command_source, authority_state, command_type, issued_by, emergency_stop_active) "
        "VALUES (:sessionId, :commandSource, 'granted', :commandType, :issuedBy, :emergencyStopActive)"));
    query.bindValue(QStringLiteral(":sessionId"), sessionId);
    query.bindValue(QStringLiteral(":commandSource"), commandSource);
    query.bindValue(QStringLiteral(":commandType"), commandType);
    query.bindValue(QStringLiteral(":issuedBy"), issuedBy.isEmpty() ? QVariant() : QVariant(issuedBy));
    query.bindValue(QStringLiteral(":emergencyStopActive"), emergencyStopActive);
    if (!query.exec()) {
        errorMessage = QStringLiteral("제어 명령 기록 실패: %1").arg(query.lastError().text());
        return false;
    }

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::recordUserAction(
    int robotId,
    int sessionId,
    int userId,
    const QString &actionType,
    const QString &payloadJson,
    const QString &issuedBy,
    QString &errorMessage)
{
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "INSERT INTO user_action_log (robot_id, session_id, user_id, action_type, payload, issued_by, occurred_at) "
        "VALUES (:robotId, :sessionId, :userId, :actionType, :payload::jsonb, :issuedBy, NOW())"));
    query.bindValue(QStringLiteral(":robotId"), robotId);
    query.bindValue(QStringLiteral(":sessionId"), sessionId > 0 ? QVariant(sessionId) : QVariant());
    query.bindValue(QStringLiteral(":userId"), userId > 0 ? QVariant(userId) : QVariant());
    query.bindValue(QStringLiteral(":actionType"), actionType);
    query.bindValue(QStringLiteral(":payload"), payloadJson.isEmpty() ? QVariant() : QVariant(payloadJson));
    query.bindValue(QStringLiteral(":issuedBy"), issuedBy.isEmpty() ? QVariant() : QVariant(issuedBy));
    if (!query.exec()) {
        errorMessage = QStringLiteral("사용자 조작 기록 실패: %1").arg(query.lastError().text());
        return false;
    }

    errorMessage.clear();
    return true;
}
