#include "dboperationsoverviewservice.h"
#include "dbconnection.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSqlError>
#include <QSqlQuery>
#include <QVariant>


bool DbOperationsOverviewService::loadActiveCallButtons(QVector<CallButtonRow> &rows, QString &errorMessage)
{
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    rows.clear();
    QSqlQuery query;
    query.prepare(QStringLiteral(
        "SELECT id, code, zone_name, pos_x, pos_y, yaw FROM call_button "
        "WHERE deregistered_at IS NULL AND pos_x IS NOT NULL AND pos_y IS NOT NULL"));
    if (!query.exec()) {
        errorMessage = QStringLiteral("호출 버튼 목록 조회 실패: %1").arg(query.lastError().text());
        return false;
    }
    while (query.next()) {
        CallButtonRow row;
        row.id = query.value(0).toInt();
        row.code = query.value(1).toString();
        row.zoneName = query.value(2).toString();
        row.x = query.value(3).toDouble();
        row.y = query.value(4).toDouble();
        row.yaw = query.value(5).isNull() ? 0.0 : query.value(5).toDouble();
        rows.append(row);
    }

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::upsertCallButton(
    const QString &code, const QString &zoneName, double x, double y, double yaw, int &callButtonDbId,
    QString &errorMessage)
{
    callButtonDbId = 0;
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "INSERT INTO call_button (code, zone_name, pos_x, pos_y, yaw) "
        "VALUES (:code, :zoneName, :x, :y, :yaw) "
        "ON CONFLICT (code) DO UPDATE SET "
        "zone_name = EXCLUDED.zone_name, pos_x = EXCLUDED.pos_x, pos_y = EXCLUDED.pos_y, "
        "yaw = EXCLUDED.yaw, deregistered_at = NULL "
        "RETURNING id"));
    query.bindValue(QStringLiteral(":code"), code);
    query.bindValue(QStringLiteral(":zoneName"), zoneName.isEmpty() ? code : zoneName);
    query.bindValue(QStringLiteral(":x"), x);
    query.bindValue(QStringLiteral(":y"), y);
    query.bindValue(QStringLiteral(":yaw"), yaw);
    if (!query.exec() || !query.next()) {
        errorMessage = QStringLiteral("호출 버튼 등록 실패: %1").arg(query.lastError().text());
        return false;
    }
    callButtonDbId = query.value(0).toInt();

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::deregisterCallButton(const QString &code, QString &errorMessage)
{
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "UPDATE call_button SET deregistered_at = NOW() "
        "WHERE code = :code AND deregistered_at IS NULL"));
    query.bindValue(QStringLiteral(":code"), code);
    if (!query.exec()) {
        errorMessage = QStringLiteral("호출 버튼 등록 해제 실패: %1").arg(query.lastError().text());
        return false;
    }

    errorMessage.clear();
    return true;
}
