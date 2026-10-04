#include "dboperationsoverviewservice.h"
#include "dbconnection.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSqlError>
#include <QSqlQuery>
#include <QVariant>

namespace {

// 웨이포인트 관리 페이지에서 쓰는 이름(YAML 저장 형식과 동일하게)
QString waypointKindToJson(WaypointMarkerKind kind)
{
    switch (kind) {
    case WaypointMarkerKind::CallZone:
        return QStringLiteral("call_zone");
    case WaypointMarkerKind::Home:
        return QStringLiteral("home");
    case WaypointMarkerKind::Waypoint:
        break;
    }
    return QStringLiteral("waypoint");
}

WaypointMarkerKind waypointKindFromJson(const QString &text)
{
    if (text == QStringLiteral("call_zone")) {
        return WaypointMarkerKind::CallZone;
    }
    if (text == QStringLiteral("home")) {
        return WaypointMarkerKind::Home;
    }
    return WaypointMarkerKind::Waypoint;
}

QJsonObject waypointLibraryToJson(const QString &frameId, const WaypointMap &waypoints, const TrajectoryMap &trajectories)
{
    QJsonObject waypointsJson;
    for (auto it = waypoints.cbegin(); it != waypoints.cend(); ++it) {
        const WaypointDef &def = it.value();
        QJsonObject entry;
        entry.insert(QStringLiteral("x"), def.x);
        entry.insert(QStringLiteral("y"), def.y);
        entry.insert(QStringLiteral("yaw"), def.yaw);
        entry.insert(QStringLiteral("kind"), waypointKindToJson(def.kind));
        entry.insert(QStringLiteral("label"), def.label.isEmpty() ? it.key() : def.label);
        waypointsJson.insert(it.key(), entry);
    }

    QJsonObject trajectoriesJson;
    for (auto it = trajectories.cbegin(); it != trajectories.cend(); ++it) {
        QJsonArray names;
        for (const QString &name : it.value()) {
            names.append(name);
        }
        trajectoriesJson.insert(it.key(), names);
    }

    QJsonObject root;
    root.insert(QStringLiteral("frame_id"), frameId);
    root.insert(QStringLiteral("waypoints"), waypointsJson);
    root.insert(QStringLiteral("trajectories"), trajectoriesJson);
    return root;
}

void waypointLibraryFromJson(
    const QJsonObject &root, QString &frameId, WaypointMap &waypoints, TrajectoryMap &trajectories)
{
    frameId = root.value(QStringLiteral("frame_id")).toString(QStringLiteral("map"));

    const QJsonObject waypointsJson = root.value(QStringLiteral("waypoints")).toObject();
    for (auto it = waypointsJson.constBegin(); it != waypointsJson.constEnd(); ++it) {
        const QJsonObject entry = it.value().toObject();
        WaypointDef def;
        def.name = it.key();
        def.x = entry.value(QStringLiteral("x")).toDouble();
        def.y = entry.value(QStringLiteral("y")).toDouble();
        def.yaw = entry.value(QStringLiteral("yaw")).toDouble();
        def.kind = waypointKindFromJson(entry.value(QStringLiteral("kind")).toString());
        def.label = entry.value(QStringLiteral("label")).toString(it.key());
        waypoints.insert(def.name, def);
    }

    const QJsonObject trajectoriesJson = root.value(QStringLiteral("trajectories")).toObject();
    for (auto it = trajectoriesJson.constBegin(); it != trajectoriesJson.constEnd(); ++it) {
        QStringList names;
        for (const QJsonValue &value : it.value().toArray()) {
            names.append(value.toString());
        }
        trajectories.insert(it.key(), names);
    }
}

} // namespace


bool DbOperationsOverviewService::loadWaypointLibraryByName(
    int robotId, const QString &name, QString &frameId, WaypointMap &waypoints, TrajectoryMap &trajectories,
    QString &errorMessage)
{
    // 호출자는 항상 name="home"만 넘긴다 — type='home'으로 명시적으로 걸러
    // waypoint/trajectory 행과 절대 안 섞이게 한다.
    frameId = QStringLiteral("map");
    waypoints.clear();
    trajectories.clear();

    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "SELECT id, waypoints FROM waypoint_set "
        "WHERE robot_id = :robotId AND type = 'home' AND name = :name AND deregistered_at IS NULL"));
    query.bindValue(QStringLiteral(":robotId"), robotId);
    query.bindValue(QStringLiteral(":name"), name);
    if (!query.exec()) {
        errorMessage = QStringLiteral("웨이포인트 라이브러리 조회 실패: %1").arg(query.lastError().text());
        return false;
    }

    if (!query.next()) {
        errorMessage = QStringLiteral("'%1' 웨이포인트 라이브러리를 찾을 수 없습니다.").arg(name);
        return false;
    }

    const int rowId = query.value(0).toInt();
    const QJsonDocument doc = QJsonDocument::fromJson(query.value(1).toString().toUtf8());
    if (!doc.isObject()) {
        errorMessage = QStringLiteral("저장된 웨이포인트 라이브러리 형식이 올바르지 않습니다.");
        return false;
    }

    waypointLibraryFromJson(doc.object(), frameId, waypoints, trajectories);
    // 홈은 이 행 하나뿐이라 파싱된 모든 항목에 이 행의 PK를 그대로 붙인다 —
    // trajectory 스텝을 이름 대신 이 id로 저장할 때 쓴다.
    for (auto it = waypoints.begin(); it != waypoints.end(); ++it) {
        it.value().dbId = rowId;
    }
    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::saveWaypointLibrary(
    int robotId, const QString &name, const QString &frameId, const WaypointMap &waypoints,
    const TrajectoryMap &trajectories, QString &errorMessage)
{
    // 호출자는 항상 name="home"만 넘긴다 — type='home' 행 하나로 upsert한다.
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    const QJsonObject root = waypointLibraryToJson(frameId, waypoints, trajectories);
    const QString json = QString::fromUtf8(QJsonDocument(root).toJson(QJsonDocument::Compact));

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "INSERT INTO waypoint_set (robot_id, type, name, waypoints) "
        "VALUES (:robotId, 'home', :name, :waypoints::jsonb) "
        "ON CONFLICT (robot_id, type, name) DO UPDATE SET "
        "waypoints = EXCLUDED.waypoints, deregistered_at = NULL"));
    query.bindValue(QStringLiteral(":robotId"), robotId);
    query.bindValue(QStringLiteral(":name"), name);
    query.bindValue(QStringLiteral(":waypoints"), json);
    if (!query.exec()) {
        errorMessage = QStringLiteral("웨이포인트 라이브러리 저장 실패: %1").arg(query.lastError().text());
        return false;
    }

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::deregisterHomeEntry(int robotId, QString &errorMessage)
{
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "UPDATE waypoint_set SET deregistered_at = NOW() "
        "WHERE robot_id = :robotId AND type = 'home' AND name = 'home' AND deregistered_at IS NULL"));
    query.bindValue(QStringLiteral(":robotId"), robotId);
    if (!query.exec()) {
        errorMessage = QStringLiteral("홈 등록 해제 실패: %1").arg(query.lastError().text());
        return false;
    }

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::loadActiveWaypointEntries(int robotId, QVector<WaypointDef> &rows, QString &errorMessage)
{
    rows.clear();
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "SELECT id, name, waypoints FROM waypoint_set "
        "WHERE robot_id = :robotId AND type = 'waypoint' AND deregistered_at IS NULL"));
    query.bindValue(QStringLiteral(":robotId"), robotId);
    if (!query.exec()) {
        errorMessage = QStringLiteral("경유점 목록 조회 실패: %1").arg(query.lastError().text());
        return false;
    }

    while (query.next()) {
        const int rowId = query.value(0).toInt();
        const QString code = query.value(1).toString();
        const QJsonObject entry = QJsonDocument::fromJson(query.value(2).toString().toUtf8()).object();
        // type='waypoint' 기본값 백필로 예전 "전체 번들" 저장 방식(레거시,
        // {frame_id, waypoints, trajectories} 형태) 행이 섞여 들어올 수 있다 —
        // 새 개별 지점 포맷({x,y,yaw,label})이 아니면 건너뛴다.
        if (entry.contains(QStringLiteral("frame_id"))) {
            continue;
        }
        WaypointDef def;
        def.name = code;
        def.label = entry.value(QStringLiteral("label")).toString(code);
        def.x = entry.value(QStringLiteral("x")).toDouble();
        def.y = entry.value(QStringLiteral("y")).toDouble();
        def.yaw = entry.value(QStringLiteral("yaw")).toDouble();
        def.kind = WaypointMarkerKind::Waypoint;
        def.dbId = rowId;
        rows.append(def);
    }

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::upsertWaypointEntry(
    int robotId, const QString &code, const QString &label, double x, double y, double yaw, int &waypointDbId,
    QString &errorMessage)
{
    waypointDbId = 0;
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QJsonObject entry;
    entry.insert(QStringLiteral("x"), x);
    entry.insert(QStringLiteral("y"), y);
    entry.insert(QStringLiteral("yaw"), yaw);
    entry.insert(QStringLiteral("label"), label.isEmpty() ? code : label);
    const QString json = QString::fromUtf8(QJsonDocument(entry).toJson(QJsonDocument::Compact));

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "INSERT INTO waypoint_set (robot_id, type, name, waypoints) "
        "VALUES (:robotId, 'waypoint', :code, :entry::jsonb) "
        "ON CONFLICT (robot_id, type, name) DO UPDATE SET "
        "waypoints = EXCLUDED.waypoints, deregistered_at = NULL "
        "RETURNING id"));
    query.bindValue(QStringLiteral(":robotId"), robotId);
    query.bindValue(QStringLiteral(":code"), code);
    query.bindValue(QStringLiteral(":entry"), json);
    if (!query.exec() || !query.next()) {
        errorMessage = QStringLiteral("경유점 등록 실패: %1").arg(query.lastError().text());
        return false;
    }
    waypointDbId = query.value(0).toInt();

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::deregisterWaypointEntry(int robotId, const QString &code, QString &errorMessage)
{
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "UPDATE waypoint_set SET deregistered_at = NOW() "
        "WHERE robot_id = :robotId AND type = 'waypoint' AND name = :code AND deregistered_at IS NULL"));
    query.bindValue(QStringLiteral(":robotId"), robotId);
    query.bindValue(QStringLiteral(":code"), code);
    if (!query.exec()) {
        errorMessage = QStringLiteral("경유점 등록 해제 실패: %1").arg(query.lastError().text());
        return false;
    }

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::loadActiveTrajectoryEntries(int robotId, QVector<TrajectoryRow> &rows, QString &errorMessage)
{
    rows.clear();
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "SELECT name, waypoints FROM waypoint_set "
        "WHERE robot_id = :robotId AND type = 'trajectory' AND deregistered_at IS NULL"));
    query.bindValue(QStringLiteral(":robotId"), robotId);
    if (!query.exec()) {
        errorMessage = QStringLiteral("경로 목록 조회 실패: %1").arg(query.lastError().text());
        return false;
    }

    while (query.next()) {
        TrajectoryRow row;
        row.name = query.value(0).toString();
        // 저장 형식은 {"table":"call_button"|"waypoint","id":N} 객체 배열이다
        // (code/name 문자열은 call_button과 waypoint_set 두 테이블에서 각자
        // 독립적으로 매겨져 서로 겹칠 수 있어, 어느 테이블인지 명시해야만
        // id로 정확히 구분된다). 호출자에게는 지금까지와 동일하게 code
        // 문자열 목록으로 되돌려준다 — 여기서 table+id를 실제 code/name으로
        // 역조회한다.
        const QJsonArray steps = QJsonDocument::fromJson(query.value(1).toString().toUtf8()).array();
        for (const QJsonValue &value : steps) {
            const QJsonObject step = value.toObject();
            const QString table = step.value(QStringLiteral("table")).toString();
            const int stepId = step.value(QStringLiteral("id")).toInt();

            QString code;
            if (table == QStringLiteral("call_button")) {
                QSqlQuery lookup;
                lookup.prepare(QStringLiteral("SELECT code FROM call_button WHERE id = :id"));
                lookup.bindValue(QStringLiteral(":id"), stepId);
                if (lookup.exec() && lookup.next()) {
                    code = lookup.value(0).toString();
                }
            } else if (table == QStringLiteral("waypoint")) {
                QSqlQuery lookup;
                lookup.prepare(QStringLiteral("SELECT type, name FROM waypoint_set WHERE id = :id"));
                lookup.bindValue(QStringLiteral(":id"), stepId);
                if (lookup.exec() && lookup.next()) {
                    // 홈 행의 name 컬럼은 "home"(소문자) 고정값이라, 웨이포인트
                    // 관리 페이지가 실제로 쓰는 "HOME"(대문자) 코드로 되돌려준다.
                    const QString rowType = lookup.value(0).toString();
                    code = (rowType == QStringLiteral("home")) ? QStringLiteral("HOME") : lookup.value(1).toString();
                }
            }
            // 참조 대상이 이후 삭제/등록해제됐으면 code가 안 채워진다 —
            // 그 스텝은 조용히 건너뛴다(에러 아님).
            if (!code.isEmpty()) {
                row.waypointCodes.append(code);
            }
        }
        rows.append(row);
    }

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::upsertTrajectoryEntry(
    int robotId, const QString &name, const QVector<TrajectoryStep> &steps, QString &errorMessage)
{
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    // 스텝은 호출자(웨이포인트관리 페이지)가 이미 테이블+id로 넘겨준다 —
    // 그 웨이포인트/콜존을 화면에 추가했던 시점에 실제 upsert로 받아온 PK를
    // 그대로 쓰는 것이라, 여기서 이름으로 다시 찾을 필요도 없고 대소문자·
    // 중복 등으로 어긋날 여지도 없다.
    QJsonArray stepArray;
    for (const TrajectoryStep &step : steps) {
        QJsonObject obj;
        obj.insert(QStringLiteral("table"), step.table);
        obj.insert(QStringLiteral("id"), step.id);
        stepArray.append(obj);
    }

    const QString json = QString::fromUtf8(QJsonDocument(stepArray).toJson(QJsonDocument::Compact));

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "INSERT INTO waypoint_set (robot_id, type, name, waypoints) "
        "VALUES (:robotId, 'trajectory', :name, :steps::jsonb) "
        "ON CONFLICT (robot_id, type, name) DO UPDATE SET "
        "waypoints = EXCLUDED.waypoints, deregistered_at = NULL"));
    query.bindValue(QStringLiteral(":robotId"), robotId);
    query.bindValue(QStringLiteral(":name"), name);
    query.bindValue(QStringLiteral(":steps"), json);
    if (!query.exec()) {
        errorMessage = QStringLiteral("경로 등록 실패: %1").arg(query.lastError().text());
        return false;
    }

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::deregisterTrajectoryEntry(int robotId, const QString &name, QString &errorMessage)
{
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "UPDATE waypoint_set SET deregistered_at = NOW() "
        "WHERE robot_id = :robotId AND type = 'trajectory' AND name = :name AND deregistered_at IS NULL"));
    query.bindValue(QStringLiteral(":robotId"), robotId);
    query.bindValue(QStringLiteral(":name"), name);
    if (!query.exec()) {
        errorMessage = QStringLiteral("경로 등록 해제 실패: %1").arg(query.lastError().text());
        return false;
    }

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::findTrajectoriesReferencing(
    int robotId, const QString &table, int dbId, QStringList &trajectoryNames, QString &errorMessage)
{
    trajectoryNames.clear();
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    // jsonb 배열 포함 연산자(@>)로 waypoints 배열 안에 {"table":...,"id":...}
    // 객체가 있는 행만 찾는다.
    QJsonObject step;
    step.insert(QStringLiteral("table"), table);
    step.insert(QStringLiteral("id"), dbId);
    QJsonArray containment;
    containment.append(step);
    const QString containmentJson = QString::fromUtf8(QJsonDocument(containment).toJson(QJsonDocument::Compact));

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "SELECT name FROM waypoint_set WHERE robot_id = :robotId AND type = 'trajectory' "
        "AND deregistered_at IS NULL AND waypoints @> :step::jsonb"));
    query.bindValue(QStringLiteral(":robotId"), robotId);
    query.bindValue(QStringLiteral(":step"), containmentJson);
    if (!query.exec()) {
        errorMessage = QStringLiteral("경로 참조 확인 실패: %1").arg(query.lastError().text());
        return false;
    }
    while (query.next()) {
        trajectoryNames.append(query.value(0).toString());
    }

    errorMessage.clear();
    return true;
}


bool DbOperationsOverviewService::loadTrajectoryPosesForButton(
    int robotId, const QString &buttonCode, QString &trajectoryName, QVector<NavGoalPoint> &poses, QString &errorMessage)
{
    poses.clear();
    trajectoryName.clear();
    if (!openDbConnection(errorMessage)) {
        return false;
    }

    QSqlQuery buttonQuery;
    buttonQuery.prepare(QStringLiteral(
        "SELECT id FROM call_button WHERE code = :code AND deregistered_at IS NULL"));
    buttonQuery.bindValue(QStringLiteral(":code"), buttonCode);
    if (!buttonQuery.exec()) {
        errorMessage = QStringLiteral("콜버튼 조회 실패: %1").arg(buttonQuery.lastError().text());
        return false;
    }
    if (!buttonQuery.next()) {
        // 등록 안 됐거나 해제된 버튼 코드 — 에러 아님, 호출자가 폴백한다.
        errorMessage.clear();
        return true;
    }
    const int callButtonId = buttonQuery.value(0).toInt();

    QStringList trajectoryNames;
    if (!findTrajectoriesReferencing(robotId, QStringLiteral("call_button"), callButtonId, trajectoryNames, errorMessage)) {
        return false;
    }
    if (trajectoryNames.isEmpty()) {
        // 이 버튼에 연결된 trajectory가 없다 — 에러 아님, 호출자가 기존 단일 pose
        // 주행으로 폴백한다.
        errorMessage.clear();
        return true;
    }
    if (trajectoryNames.size() > 1) {
        qWarning() << "call_button code" << buttonCode << "가 여러 trajectory에 연결되어 있어 첫 번째만 사용:"
                   << trajectoryNames;
    }
    trajectoryName = trajectoryNames.first();

    QSqlQuery query;
    query.prepare(QStringLiteral(
        "SELECT waypoints FROM waypoint_set WHERE robot_id = :robotId AND type = 'trajectory' "
        "AND name = :name AND deregistered_at IS NULL"));
    query.bindValue(QStringLiteral(":robotId"), robotId);
    query.bindValue(QStringLiteral(":name"), trajectoryName);
    if (!query.exec() || !query.next()) {
        errorMessage = QStringLiteral("경로 조회 실패: %1").arg(query.lastError().text());
        return false;
    }

    // loadActiveTrajectoryEntries와 동일한 스텝 형식({"table":"call_button"|"waypoint","id":N})을
    // 디코딩하되, code 문자열이 아니라 x/y/yaw를 직접 resolve한다.
    const QJsonArray steps = QJsonDocument::fromJson(query.value(0).toString().toUtf8()).array();
    for (const QJsonValue &value : steps) {
        const QJsonObject step = value.toObject();
        const QString table = step.value(QStringLiteral("table")).toString();
        const int stepId = step.value(QStringLiteral("id")).toInt();

        NavGoalPoint point;
        bool resolved = false;
        if (table == QStringLiteral("call_button")) {
            QSqlQuery lookup;
            lookup.prepare(QStringLiteral(
                "SELECT pos_x, pos_y, yaw FROM call_button WHERE id = :id AND pos_x IS NOT NULL AND pos_y IS NOT NULL"));
            lookup.bindValue(QStringLiteral(":id"), stepId);
            if (lookup.exec() && lookup.next()) {
                point.x = lookup.value(0).toDouble();
                point.y = lookup.value(1).toDouble();
                point.yaw = lookup.value(2).toDouble();
                resolved = true;
            }
        } else if (table == QStringLiteral("waypoint")) {
            QSqlQuery lookup;
            lookup.prepare(QStringLiteral("SELECT waypoints FROM waypoint_set WHERE id = :id"));
            lookup.bindValue(QStringLiteral(":id"), stepId);
            if (lookup.exec() && lookup.next()) {
                const QJsonObject pose = QJsonDocument::fromJson(lookup.value(0).toString().toUtf8()).object();
                point.x = pose.value(QStringLiteral("x")).toDouble();
                point.y = pose.value(QStringLiteral("y")).toDouble();
                point.yaw = pose.value(QStringLiteral("yaw")).toDouble();
                resolved = true;
            }
        }
        // 참조 대상이 이후 삭제/등록해제됐으면 조용히 건너뛴다(loadActiveTrajectoryEntries와 동일 패턴).
        if (resolved) {
            poses.append(point);
        }
    }

    errorMessage.clear();
    return true;
}
