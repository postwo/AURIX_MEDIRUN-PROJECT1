#include "missioncontroller.h"

#include "../ros/navigationservice.h"
#include "missionrosinterface.h"
#include "missionstorageservice.h"
#include "operationsoverviewservice.h"
#include "robotservice.h"
#include "waypointrepository.h"

#include <QDateTime>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSettings>
#include <QTimer>
#include <QTime>
#include <QUuid>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <QtMath>

#include "missioncontroller_helpers.h"

void MissionController::confirmStartNextQueue(const QString &queueId)
{
    if (m_snapshot.state != MissionState::Idle) {
        return;
    }
    const int index = queueIndex(queueId);
    if (index < 0 || m_queueItems.at(index).status != MissionQueueStatus::Waiting) {
        return;
    }
    log(QStringLiteral("QUEUE"), QStringLiteral("operator confirmed next queue_id=%1").arg(queueId));
    startMissionForQueue(queueId, false);
}

void MissionController::declineNextQueue()
{
    log(QStringLiteral("QUEUE"), QStringLiteral("operator declined next queue start, staying stopped"));
    addHistory(QStringLiteral("QUEUE_START_DECLINED"), QString(), QString(), QString(), QString(),
               QStringLiteral("DECLINED"), QStringLiteral("다음 호출 주행을 보류하고 현재 위치에서 대기"));
    emit historyChanged();
}

void MissionController::markQueueStatus(const QString &queueId, MissionQueueStatus status)
{
    const int index = queueIndex(queueId);
    if (index >= 0) {
        m_queueItems[index].status = status;
    }
}

int MissionController::queueIndex(const QString &queueId) const
{
    for (int i = 0; i < m_queueItems.size(); ++i) {
        if (m_queueItems.at(i).queueId == queueId) {
            return i;
        }
    }
    return -1;
}

MissionQueueItem *MissionController::activeQueueItem()
{
    const int index = queueIndex(m_snapshot.queueId);
    if (index < 0) {
        return nullptr;
    }
    return &m_queueItems[index];
}

const MissionQueueItem *MissionController::activeQueueItem() const
{
    const int index = queueIndex(m_snapshot.queueId);
    if (index < 0) {
        return nullptr;
    }
    return &m_queueItems[index];
}

void MissionController::setMissionState(MissionState state)
{
    if (m_snapshot.state == state) {
        return;
    }
    log(QStringLiteral("MISSION"),
        QStringLiteral("state: %1 -> %2").arg(missionStateText(m_snapshot.state), missionStateText(state)));
    m_snapshot.state = state;
    emit missionChanged();
}

void MissionController::saveQueue()
{
    m_storageService->saveQueue(m_queueItems);
    if (MissionQueueItem *item = activeQueueItem()) {
        m_snapshot.missionId = item->missionId;
        m_snapshot.queueId = item->queueId;
    }
}

void MissionController::addHistory(const QString &eventType,
                                   const QString &waypointId,
                                   const QString &waypointName,
                                   const QString &missionId,
                                   const QString &queueId,
                                   const QString &result,
                                   const QString &detailMessage)
{
    MissionHistoryEntry entry;
    entry.historyId = historyId();
    entry.occurredAt = QDateTime::currentDateTime();
    entry.eventType = eventType;
    entry.waypointId = waypointId;
    entry.waypointName = waypointName;
    entry.missionId = missionId;
    entry.queueId = queueId;
    entry.result = result;
    entry.detailMessage = detailMessage;
    m_storageService->appendHistory(entry);
    m_historyEntries.prepend(entry);
    while (m_historyEntries.size() > 100) {
        m_historyEntries.removeLast();
    }
    log(QStringLiteral("HISTORY"), QStringLiteral("event saved type=%1").arg(eventType));
}

QString MissionController::nextMissionId() const
{
    return QStringLiteral("MR-%1").arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMddhhmmsszzz")));
}

QString MissionController::nextQueueId() const
{
    return QStringLiteral("Q-%1").arg(QUuid::createUuid().toString(QUuid::WithoutBraces).left(8));
}

QString MissionController::displayNameForWaypoint(const QString &waypointId) const
{
    WaypointDef waypoint;
    if (m_waypointRepository->waypoint(waypointId, waypoint)) {
        return waypoint.label.isEmpty() ? waypoint.name : waypoint.label;
    }
    return waypointId;
}

QString MissionController::nearestWaypointName(double x, double y) const
{
    const WaypointMap waypoints = m_waypointRepository->waypoints();
    double nearestDistance = std::numeric_limits<double>::max();
    QString nearestName = QStringLiteral("현재 위치");
    for (auto it = waypoints.cbegin(); it != waypoints.cend(); ++it) {
        const double distance = std::hypot(it.value().x - x, it.value().y - y);
        if (distance < nearestDistance) {
            nearestDistance = distance;
            nearestName = it.value().label.isEmpty() ? it.key() : it.value().label;
        }
    }
    return nearestName;
}

void MissionController::updateProgress()
{
    if (m_initialDistanceMeters <= 0.0 || m_snapshot.distanceRemainingMeters < 0.0) {
        m_snapshot.progressPercent = -1;
        return;
    }
    const double ratio = 1.0 - (m_snapshot.distanceRemainingMeters / m_initialDistanceMeters);
    m_snapshot.progressPercent = qBound(0, static_cast<int>(ratio * 100.0), 100);
}

void MissionController::updateConnectionState()
{
    m_rosConnected = m_lastRosHeartbeat.isValid()
        && m_lastRosHeartbeat.secsTo(QDateTime::currentDateTime()) <= 5;
}

void MissionController::log(const QString &category, const QString &message) const
{
    qInfo().noquote() << QStringLiteral("[%1] %2").arg(category, message);
}
