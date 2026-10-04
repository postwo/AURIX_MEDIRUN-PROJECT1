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

void MissionController::enqueueCall(const QString &buttonId,
                                    const QString &requestSource,
                                    const QString &missionId,
                                    const QDateTime &requestedAt)
{
    // 등록 성공/실패와 무관하게, 실제 하드웨어가 보낸 raw button_id를 전부
    // 기록해둔다 — 웨이포인트관리 페이지가 콜존 등록 시 "감지된 콜존 ID"
    // 드롭다운으로 보여준다. 최근 감지된 것이 앞에 오게 유지.
    m_observedButtonIds.removeAll(buttonId);
    m_observedButtonIds.prepend(buttonId);
    QSettings().setValue(QLatin1String(kObservedButtonIdsSettingsKey), m_observedButtonIds);
    emit observedButtonIdsChanged();

    for (const MissionQueueItem &existing : std::as_const(m_queueItems)) {
        if (existing.waypointId == buttonId
            && (existing.status == MissionQueueStatus::Waiting || existing.status == MissionQueueStatus::Active)) {
            addHistory(QStringLiteral("QUEUE_DUPLICATE_IGNORED"), buttonId, displayNameForWaypoint(buttonId), missionId, existing.queueId,
                       QStringLiteral("IGNORED"), QStringLiteral("동일 위치 호출이 이미 대기 또는 진행 중"));
            emit historyChanged();
            return;
        }
    }

    MissionQueueItem item;
    item.queueId = nextQueueId();
    item.waypointId = buttonId;
    item.waypointName = displayNameForWaypoint(buttonId);
    item.requestedAt = requestedAt.isValid() ? requestedAt : QDateTime::currentDateTime();
    item.enqueueOrder = m_queueItems.size() + 1;
    item.status = MissionQueueStatus::Waiting;
    item.requestSource = requestSource.isEmpty() ? QStringLiteral("ros2") : requestSource;
    item.missionId = missionId;

    QString errorMessage;
    if (!m_waypointRepository->waypointPose(buttonId, item.pose, errorMessage)) {
        addHistory(QStringLiteral("QUEUE_FAILED"), buttonId, item.waypointName, missionId, item.queueId,
                   QStringLiteral("FAILED"), errorMessage);
        emit historyChanged();
        return;
    }

    m_queueItems.append(item);
    saveQueue();
    const MissionQueueItem persistedItem = m_queueItems.last();
    addHistory(QStringLiteral("QUEUE_REGISTERED"), persistedItem.waypointId, persistedItem.waypointName,
               persistedItem.missionId, persistedItem.queueId,
               QStringLiteral("WAITING"), QStringLiteral("호출 큐 등록"));
    log(QStringLiteral("QUEUE"), QStringLiteral("registered queue_id=%1 waypoint_id=%2")
                                    .arg(persistedItem.queueId, persistedItem.waypointId));
    emit queueChanged();
    emit historyChanged();
    startNextQueueIfPossible();
}

void MissionController::startNextQueueIfPossible()
{
    if (m_snapshot.state != MissionState::Idle && m_snapshot.state != MissionState::Completed
        && m_snapshot.state != MissionState::Cancelled && m_snapshot.state != MissionState::Failed) {
        return;
    }
    if (m_snapshot.returningHome) {
        return;
    }

    for (const MissionQueueItem &item : std::as_const(m_queueItems)) {
        if (item.status == MissionQueueStatus::Waiting) {
            log(QStringLiteral("QUEUE"), QStringLiteral("dequeue next queue_id=%1").arg(item.queueId));
            startMissionForQueue(item.queueId, false);
            return;
        }
    }

    resetToIdleSnapshot();
}

void MissionController::reconcileOrphanedActiveMission()
{
    for (const MissionQueueItem &item : std::as_const(m_queueItems)) {
        if (item.status != MissionQueueStatus::Active) {
            continue;
        }
        m_snapshot.missionId = item.missionId;
        m_snapshot.queueId = item.queueId;
        m_snapshot.originName = QStringLiteral("이전 세션");
        m_snapshot.destinationName = item.waypointName;
        m_snapshot.currentWaypointName = item.waypointName;
        m_snapshot.nextWaypointName = item.waypointName;
        m_snapshot.startedAt = item.requestedAt.isValid() ? item.requestedAt : QDateTime::currentDateTime();
        m_snapshot.elapsedSeconds = m_snapshot.startedAt.secsTo(QDateTime::currentDateTime());
        m_snapshot.waitingCount = std::count_if(m_queueItems.cbegin(), m_queueItems.cend(), [](const MissionQueueItem &candidate) {
            return candidate.status == MissionQueueStatus::Waiting;
        });
        m_snapshot.returningHome = false;
        m_snapshot.arrivedBeforeEnd = false;
        m_snapshot.payloadRemoved = false;
        m_snapshot.failureReason.clear();
        m_snapshot.distanceRemainingMeters = -1.0;
        // setMissionState()는 Idle -> Navigating 전환을 감지해 missionChanged()를
        // 알아서 emit한다.
        setMissionState(MissionState::Navigating);
        addHistory(QStringLiteral("MISSION_RECOVERED"), item.waypointId, item.waypointName, item.missionId, item.queueId,
                   QStringLiteral("RECOVERED"),
                   QStringLiteral("대시보드가 미션 진행 중 종료됐다가 다시 시작됨 — 정확한 진행 단계를 몰라 "
                                  "\"출동 중\"으로 임시 표시합니다. 실제 로봇 상태를 확인 후 필요하면 미션을 취소해주세요."));
        emit historyChanged();
        return; // 정상 흐름에서 Active 항목은 한 번에 하나뿐이라 첫 건만 처리
    }
}

void MissionController::startMissionForQueue(const QString &queueId, bool divertedFromActiveMission)
{
    const int index = queueIndex(queueId);
    if (index < 0) {
        return;
    }
    // 직전 미션의 완료 체크마크 표시를 위한 30초 대기 중이었더라도, 새 미션이
    // 실제로 시작되면 그 대기는 더 이상 의미가 없다 — 정리해둔다.
    m_completionResetTimer->stop();

    MissionQueueItem &item = m_queueItems[index];
    item.status = MissionQueueStatus::Active;
    item.missionId = item.missionId.isEmpty() ? nextMissionId() : item.missionId;

    m_snapshot.missionId = item.missionId;
    m_snapshot.queueId = item.queueId;
    m_snapshot.originName = m_robotPoseValid ? nearestWaypointName(m_lastRobotX, m_lastRobotY) : QStringLiteral("현재 위치");
    m_snapshot.destinationName = item.waypointName;
    m_snapshot.currentWaypointName = m_snapshot.originName;
    m_snapshot.nextWaypointName = item.waypointName;
    m_snapshot.startedAt = QDateTime::currentDateTime();
    m_snapshot.elapsedSeconds = 0;
    m_snapshot.waitingCount = std::count_if(m_queueItems.cbegin(), m_queueItems.cend(), [](const MissionQueueItem &candidate) {
        return candidate.status == MissionQueueStatus::Waiting;
    });
    m_snapshot.returningHome = false;
    m_snapshot.arrivedBeforeEnd = false;
    m_snapshot.payloadRemoved = false;
    m_snapshot.failureReason.clear();
    m_snapshot.distanceRemainingMeters = -1.0;
    m_initialDistanceMeters = -1.0;
    if (m_robotPoseValid) {
        m_initialDistanceMeters = std::hypot(item.pose.positionX - m_lastRobotX, item.pose.positionY - m_lastRobotY);
    }

    setMissionState(MissionState::Starting);
    addHistory(divertedFromActiveMission ? QStringLiteral("MISSION_DIVERTED_STARTED") : QStringLiteral("MISSION_STARTED"),
               item.waypointId, item.waypointName, item.missionId, item.queueId,
               QStringLiteral("STARTED"), divertedFromActiveMission ? QStringLiteral("확인 전환 후 새 미션 시작")
                                                                     : QStringLiteral("호출 미션 시작"));
    saveQueue();
    item = m_queueItems[index];
    m_snapshot.missionId = item.missionId;
    m_snapshot.queueId = item.queueId;
    emit queueChanged();
    emit historyChanged();
    emit missionChanged();
    log(QStringLiteral("NAV2"),
        QStringLiteral("goal send mission_id=%1 queue_id=%2 waypoint_id=%3").arg(item.missionId, item.queueId, item.waypointId));
    QVector<NavPose> poses;
    // 경로(trajectory) 다지점 주행이 단독 웨이포인트 주행에도 영향을 줘서
    // (2026-08-24) 다시 비활성화 — 항상 단일 지점(navigateToPose)으로 보낸다.
    // if (!trajectoryPosesForButton(item.waypointId, poses)) {
    //     poses = {NavPose{item.pose.positionX, item.pose.positionY, item.pose.yawRadians}};
    // }
    poses = {NavPose{item.pose.positionX, item.pose.positionY, item.pose.yawRadians}};
    dispatchNavigation(poses, item.pose.frameId, item.waypointName);

    // 문서 02 §7.1 음성 안내 #3(SR-F-005) "임무 시작" — 호출을 받아 실제로
    // 주행을 시작하는 순간 1회 안내한다. beginArrivedWait()의 "AED가
    // 도착했습니다"와 동일한 방식(Pi의 robot_audio_output이 처리).
    m_rosInterface->publishAudioAnnouncement(QStringLiteral("응급 호출을 접수했습니다. 출동합니다"));
}
