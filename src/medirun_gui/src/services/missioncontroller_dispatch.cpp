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

bool MissionController::trajectoryPosesForButton(const QString &buttonCode, QVector<NavPose> &poses) const
{
    if (!m_robotService || !m_overviewService || buttonCode.isEmpty()) {
        return false;
    }
    // trajectory는 waypoint_set(robot_id, type='trajectory')에 저장돼 로봇마다
    // 갈리므로, 운영현황 자동기록과 동일하게 로봇이 정확히 1대 등록돼 있을 때만
    // 조회한다(waypointmanagementpage.cpp::handleSaveTrajectory와 동일 패턴).
    const QVector<RobotRecord> robots = m_robotService->listRobots(false);
    if (robots.size() != 1) {
        return false;
    }

    QString trajectoryName;
    QVector<NavGoalPoint> dbPoses;
    QString errorMessage;
    if (!m_overviewService->loadTrajectoryPosesForButton(robots.first().id, buttonCode, trajectoryName, dbPoses, errorMessage)) {
        log(QStringLiteral("MISSION"), QStringLiteral("경로 조회 실패(버튼=%1): %2").arg(buttonCode, errorMessage));
        return false;
    }
    if (dbPoses.isEmpty()) {
        return false;
    }

    poses.clear();
    poses.reserve(dbPoses.size());
    for (const NavGoalPoint &point : dbPoses) {
        poses.append(NavPose{point.x, point.y, point.yaw});
    }
    log(QStringLiteral("MISSION"),
        QStringLiteral("버튼 %1 → 경로 '%2'(%3점) 사용").arg(buttonCode, trajectoryName).arg(poses.size()));
    return true;
}

void MissionController::dispatchNavigation(const QVector<NavPose> &poses, const QString &frameId, const QString &label)
{
    if (poses.isEmpty()) {
        return;
    }
    if (poses.size() == 1) {
        m_navigationService->navigateToPose(poses.first().x, poses.first().y, poses.first().yaw, frameId, label);
    } else {
        // navigateThroughPoses는 approach pose를 끼워넣어도(2026-08-24 수정) 또
        // 다른 코너에서 제자리 회전이 재현되어 일단 비활성화한다. followWaypoints는
        // 각 지점마다 잠깐 정지하지만 approach pose 포함 검증된 경로라 이걸로 대체.
        // m_navigationService->navigateThroughPoses(poses, frameId, label);
        m_navigationService->followWaypoints(poses, frameId, label);
    }
}

void MissionController::dispatchReturnHomeNavigation(const WaypointPose &homePose, const QString &label)
{
    QVector<NavPose> poses;
    // 출동과 동일한 이유(2026-08-24)로 trajectory 조회를 다시 비활성화 —
    // 항상 HOME 단일 지점으로만 복귀한다.
    // if (const MissionQueueItem *item = activeQueueItem()) {
    //     trajectoryPosesForButton(item->waypointId, poses);
    // }
    poses.append(NavPose{homePose.positionX, homePose.positionY, homePose.yawRadians});
    dispatchNavigation(poses, homePose.frameId, label);
}

bool MissionController::resolveHomeWaypoint(WaypointDef &homeWaypoint, WaypointPose &homePose, QString &errorMessage) const
{
    // Home 도착 후 항상 ArUco 전면주차로 이어진다(handleNavigationResult
    // 참고). simulate_docking(기본 true)이면 카메라 없는 로봇도 안전하게
    // 그대로 완료되므로 Home 웨이포인트 하나만 있으면 된다.
    const QVector<WaypointDef> homes = m_waypointRepository->waypointsByKind(WaypointMarkerKind::Home);
    if (homes.isEmpty()) {
        errorMessage = QStringLiteral("홈 waypoint가 설정되어 있지 않습니다.");
        log(QStringLiteral("MISSION"), QStringLiteral("home waypoint lookup failed: missing"));
        return false;
    }
    if (homes.size() > 1) {
        errorMessage = QStringLiteral("홈 waypoint가 여러 개입니다. 대표 홈 1개만 남겨야 합니다.");
        log(QStringLiteral("MISSION"), QStringLiteral("home waypoint lookup failed: duplicated"));
        return false;
    }

    homeWaypoint = homes.first();
    const bool ok = m_waypointRepository->waypointPose(homeWaypoint.name, homePose, errorMessage);
    if (ok) {
        log(QStringLiteral("MISSION"), QStringLiteral("returning home waypoint_id=%1").arg(homeWaypoint.name));
    }
    return ok;
}

void MissionController::finishMission(MissionState finalState,
                                      const QString &reason,
                                      const QString &historyType,
                                      const QString &historyResult,
                                      bool requiresQueueConfirmation)
{
    if (MissionQueueItem *item = activeQueueItem()) {
        item->status = (finalState == MissionState::Completed)
                           ? MissionQueueStatus::Completed
                           : (finalState == MissionState::Cancelled ? MissionQueueStatus::Cancelled : MissionQueueStatus::Failed);
    }
    addHistory(historyType, QString(), m_snapshot.destinationName, m_snapshot.missionId, m_snapshot.queueId, historyResult,
               reason.isEmpty() ? resultText(historyResult) : reason);
    m_snapshot.failureReason = reason;
    // returningHome은 바로 아래서 false로 리셋해야 다음 대기 호출이 안 막히니,
    // 실시간 관제 스텝퍼가 "복귀 단계까지는 갔었는지" 나중에 구분할 수 있게
    // 그 순간의 값을 arrivedBeforeEnd에 남겨둔다. Arrived(도착 후 반출 대기
    // 중 resolveHomeWaypoint 실패 등으로 바로 실패 처리되는 경우)도 "도착은
    // 했었다"이므로 포함한다.
    m_snapshot.arrivedBeforeEnd = m_snapshot.returningHome || m_snapshot.state == MissionState::Arrived
        || finalState == MissionState::Completed;
    // 미션이 어떤 이유로든 끝나는 시점 — Pi(arrival_return_manager)가 이
    // 미션에 대해 여전히 armed 상태로 남아 나중에 또 복귀 goal을 쏘는 일이
    // 없도록 해제 신호를 같이 보낸다(이미 해제돼 있으면 Pi가 그냥 무시함).
    if (!m_snapshot.missionId.isEmpty()) {
        m_rosInterface->publishArrivalNotice(false, m_snapshot.missionId, m_snapshot.queueId, 0.0, 0.0, 0.0, QString());
    }
    // Pi 주도 복귀(handlePiReturnStatus)로 DB 세션을 열어뒀으면, 이 미션이
    // 어떤 이유로 끝나든 그 세션도 같이 마감한다 — NavGoalLogRecorder가 이
    // 세션의 존재를 모르므로 여기서 직접 닫아야 한다.
    if (m_overviewService && m_piReturnSessionId >= 0) {
        QString dbErrorMessage;
        m_overviewService->finishRobotSession(m_piReturnSessionId, finalState == MissionState::Completed,
                                              finalState == MissionState::Cancelled, dbErrorMessage);
        m_piReturnSessionId = -1;
        m_piReturnLastMarkedIndex = -1;
    }
    setMissionState(finalState);
    m_pendingAction = PendingAction::None;
    m_pendingDivertQueueId.clear();
    m_snapshot.returningHome = false;
    saveQueue();
    emit queueChanged();
    emit historyChanged();
    emit missionChanged();

    if (requiresQueueConfirmation) {
        for (const MissionQueueItem &item : std::as_const(m_queueItems)) {
            if (item.status == MissionQueueStatus::Waiting) {
                log(QStringLiteral("QUEUE"), QStringLiteral("awaiting operator confirmation queue_id=%1").arg(item.queueId));
                resetToIdleSnapshot();
                emit nextQueueConfirmationRequested(item.queueId, item.waypointName);
                return;
            }
        }
    }

    if (finalState == MissionState::Completed) {
        // 완료 체크마크를 30초간 화면에 보여준 뒤에야 다음 큐 처리/Idle
        // 초기화로 넘어간다. 그 사이 새 호출이 들어오면 enqueueCall()이
        // 직접 startNextQueueIfPossible()을 불러 즉시 처리하므로 대기가
        // 필요 없다 — 그때는 이 타이머가 나중에 만료돼도 이미 Idle/Completed가
        // 아니라 startNextQueueIfPossible()의 상태 가드에서 아무 일도 안 한다.
        m_completionResetTimer->start(30000);
    } else {
        startNextQueueIfPossible();
    }
}

void MissionController::resetToIdleSnapshot()
{
    m_snapshot = MissionSnapshot();
    m_snapshot.waitingCount = std::count_if(m_queueItems.cbegin(), m_queueItems.cend(), [](const MissionQueueItem &item) {
        return item.status == MissionQueueStatus::Waiting;
    });
    emit missionChanged();
}
