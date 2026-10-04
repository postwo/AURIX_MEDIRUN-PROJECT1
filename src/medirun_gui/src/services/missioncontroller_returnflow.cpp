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

bool MissionController::beginReturnHomeFlow(const QString &reason)
{
    if (m_snapshot.state == MissionState::ReturningHome || m_snapshot.state == MissionState::Parking) {
        return false;
    }

    if (m_snapshot.state == MissionState::Idle) {
        WaypointDef homeWaypoint;
        WaypointPose homePose;
        QString errorMessage;
        if (!resolveHomeWaypoint(homeWaypoint, homePose, errorMessage)) {
            addHistory(QStringLiteral("RETURN_HOME_FAILED"), QString(), QString(), QString(), QString(),
                       QStringLiteral("FAILED"), errorMessage);
            emit historyChanged();
            return false;
        }

        m_snapshot.missionId = nextMissionId();
        m_snapshot.queueId.clear();
        m_snapshot.originName = m_robotPoseValid ? nearestWaypointName(m_lastRobotX, m_lastRobotY) : QStringLiteral("현재 위치");
        m_snapshot.destinationName = homeWaypoint.label;
        m_snapshot.nextWaypointName = homeWaypoint.label;
        m_snapshot.startedAt = QDateTime::currentDateTime();
        m_snapshot.returningHome = true;
        m_snapshot.payloadRemoved = false;
        m_pendingAction = PendingAction::ReturnHome;
        m_pendingReason = reason;
        setMissionState(MissionState::ReturningHome);
        addHistory(QStringLiteral("RETURN_HOME_STARTED"), homeWaypoint.name, homeWaypoint.label, m_snapshot.missionId, QString(),
                   QStringLiteral("STARTED"), reason);
        emit historyChanged();
        emit missionChanged();
        m_rosInterface->publishAudioAnnouncement(QStringLiteral("홈으로 복귀합니다"));
        m_navigationService->navigateToPose(homePose.positionX, homePose.positionY, homePose.yawRadians, homePose.frameId);
        return true;
    }

    if (m_snapshot.state == MissionState::Arrived) {
        // Arrived는 Nav2 goal이 이미 SUCCEEDED로 끝난 뒤라 취소할 활성 goal이
        // 없다 — 아래 else 분기(cancelGoal 후 CANCELED 콜백을 기다림)를 타면
        // CANCELED가 영영 안 와서 멈춘다. Idle 분기처럼 바로 홈 goal을
        // 보내되, 이미 진행 중인 미션이라 missionId/queueId는 새로 만들지
        // 않고 그대로 이어간다. 운영자가 지금 직접 개입하는 거라 PC가 바로
        // 처리하지만, Pi의 arrival_return_manager가 나중에 또 자기 판단으로
        // 복귀 goal을 쏘면 안 되므로 먼저 해제 신호를 보낸다.
        m_rosInterface->publishArrivalNotice(false, m_snapshot.missionId, m_snapshot.queueId, 0.0, 0.0, 0.0, QString());
        WaypointDef homeWaypoint;
        WaypointPose homePose;
        QString errorMessage;
        if (!resolveHomeWaypoint(homeWaypoint, homePose, errorMessage)) {
            finishMission(MissionState::Failed, errorMessage, QStringLiteral("RETURN_HOME_FAILED"), QStringLiteral("FAILED"));
            return false;
        }
        addHistory(QStringLiteral("RETURN_HOME_STARTED"), homeWaypoint.name, homeWaypoint.label, m_snapshot.missionId, m_snapshot.queueId,
                   QStringLiteral("STARTED"), reason);
        m_snapshot.destinationName = homeWaypoint.label;
        m_snapshot.nextWaypointName = homeWaypoint.label;
        m_snapshot.returningHome = true;
        setMissionState(MissionState::ReturningHome);
        emit historyChanged();
        emit missionChanged();
        m_rosInterface->publishAudioAnnouncement(QStringLiteral("홈으로 복귀합니다"));
        dispatchReturnHomeNavigation(homePose, homeWaypoint.label);
        return true;
    }

    if (m_snapshot.state == MissionState::Cancelling || m_snapshot.state == MissionState::Diverting) {
        return false;
    }

    m_pendingAction = PendingAction::ReturnHome;
    m_pendingReason = reason;
    setMissionState(MissionState::Cancelling);
    addHistory(QStringLiteral("RETURN_HOME_REQUESTED"),
               QString(), m_snapshot.destinationName, m_snapshot.missionId, m_snapshot.queueId,
               QStringLiteral("REQUESTED"), reason);
    emit historyChanged();
    m_navigationService->cancelGoal();
    return true;
}

bool MissionController::dispatchPendingReturnHome()
{
    m_pendingAction = PendingAction::None;
    WaypointDef homeWaypoint;
    WaypointPose homePose;
    QString errorMessage;
    if (!resolveHomeWaypoint(homeWaypoint, homePose, errorMessage)) {
        finishMission(MissionState::Failed, errorMessage, QStringLiteral("RETURN_HOME_FAILED"), QStringLiteral("FAILED"));
        return false;
    }
    addHistory(QStringLiteral("RETURN_HOME_STARTED"), homeWaypoint.name, homeWaypoint.label, m_snapshot.missionId, m_snapshot.queueId,
               QStringLiteral("STARTED"), QStringLiteral("홈 복귀 시작"));
    m_snapshot.destinationName = homeWaypoint.label;
    m_snapshot.nextWaypointName = homeWaypoint.label;
    m_snapshot.returningHome = true;
    setMissionState(MissionState::ReturningHome);
    m_rosInterface->publishAudioAnnouncement(QStringLiteral("홈으로 복귀합니다"));
    dispatchReturnHomeNavigation(homePose, homeWaypoint.label);
    return true;
}

void MissionController::beginArrivedWait()
{
    m_snapshot.payloadRemoved = false;
    setMissionState(MissionState::Arrived);
    addHistory(QStringLiteral("MISSION_ARRIVED"), QString(), m_snapshot.destinationName, m_snapshot.missionId, m_snapshot.queueId,
               QStringLiteral("ARRIVED"), QStringLiteral("호출구역 도착 — AED 반출 대기"));
    emit historyChanged();
    emit missionChanged();

    // 문서 02 §7.1 음성 안내 #6. 도착 판단 자체는 Nav2(PC)에서 나오는 거라
    // 이 발행은 여전히 그 순간의 Wi-Fi에 걸려있다 — 다만 이후 반출 감지/
    // 타이머/복귀 goal 전송은 전부 Pi(arrival_return_manager)가 로컬로
    // 이어받는다(2026-08-19).
    m_rosInterface->publishAudioAnnouncement(QStringLiteral("AED가 도착했습니다"));

    WaypointDef homeWaypoint;
    WaypointPose homePose;
    QString errorMessage;
    if (!resolveHomeWaypoint(homeWaypoint, homePose, errorMessage)) {
        // 반출 감시 자체를 못 켠다 — 자동 복귀는 안 되지만 미션을 실패
        // 처리하진 않는다(운영자가 홈을 나중에 설정하고 수동 "복귀" 버튼으로
        // 여전히 끝낼 수 있음). 기록만 남긴다.
        addHistory(QStringLiteral("ARRIVAL_WATCH_NOT_ARMED"), QString(), m_snapshot.destinationName, m_snapshot.missionId,
                   m_snapshot.queueId, QStringLiteral("FAILED"),
                   QStringLiteral("자동 복귀 감시를 시작하지 못했습니다: %1").arg(errorMessage));
        emit historyChanged();
        return;
    }

    // 이 미션을 유발한 버튼의 trajectory도 이 시점(Wi-Fi가 아직 살아있을 때)에
    // 통째로 같이 실어 보내는 기능이 있었으나, Pi 쪽 navigate_through_poses
    // 구현(arrival_return_manager.py)은 PC의 followWaypoints와 달리 approach
    // pose가 없어 코너에서 제자리 회전을 일으킬 수 있다 — Pi 쪽에도 같은
    // 보완이 되기 전까지는 의도적으로 비활성 상태로 둔다(2026-08-24). 항상
    // 빈 목록을 보내 Pi는 HOME 단일 지점으로만 복귀한다.
    QVector<NavPose> trajectoryPoses;
    // if (const MissionQueueItem *item = activeQueueItem()) {
    //     trajectoryPosesForButton(item->waypointId, trajectoryPoses);
    // }
    m_rosInterface->publishArrivalNotice(true, m_snapshot.missionId, m_snapshot.queueId, homePose.positionX,
                                         homePose.positionY, homePose.yawRadians, homePose.frameId, trajectoryPoses);

    // Pi가 실제로 타게 될 전체 경로(trajectory+HOME)를 캐싱해둔다 — "returning"
    // 상태를 받았을 때 DB에 다시 조회하지 않고 그대로 robot_session/nav_goal_log
    // 기록에 쓴다(handlePiReturnStatus 참고).
    m_pendingPiReturnPoses = trajectoryPoses;
    m_pendingPiReturnPoses.append(NavPose{homePose.positionX, homePose.positionY, homePose.yawRadians});
}

void MissionController::handleDockResult(bool success, const QString &detail)
{
    // 정상적으로는 항상 Parking 상태에서 온다. 다만 도착~복귀 전 구간이 이제
    // Pi(arrival_return_manager)가 로컬로 처리하는 부분이라, Wi-Fi가 그동안
    // 끊겨있었으면 PC는 Arrived나 ReturningHome에 멈춰있는 채로 이 결과를
    // 처음 받을 수 있다 — 그런 경우도 놓치지 않고 미션을 마무리하도록
    // 범위를 넓힌다(2026-08-19).
    if (m_snapshot.state != MissionState::Parking && m_snapshot.state != MissionState::ReturningHome
        && m_snapshot.state != MissionState::Arrived) {
        return;
    }
    if (success) {
        m_rosInterface->publishAudioAnnouncement(QStringLiteral("도킹 완료했습니다"));
        finishMission(MissionState::Completed, m_pendingReason, QStringLiteral("PARKING_COMPLETED"), QStringLiteral("SUCCEEDED"));
        return;
    }
    if (m_pendingReason.contains(QStringLiteral("취소"))) {
        finishMission(MissionState::Cancelled, m_pendingReason, QStringLiteral("PARKING_CANCELLED"), QStringLiteral("CANCELLED"));
        return;
    }
    const QString reason = detail.isEmpty() ? QStringLiteral("ArUco 전면주차 실패") : QStringLiteral("ArUco 전면주차 실패: %1").arg(detail);
    finishMission(MissionState::Failed, reason, QStringLiteral("PARKING_FAILED"), QStringLiteral("FAILED"));
}

void MissionController::handlePiReturnStatus(const QString &phase, const QString &missionId, const QString &queueId,
                                             int waypointIndex, int posesTotal, bool replayed)
{
    Q_UNUSED(posesTotal)
    // Pi가 지금 진행 중인 우리 미션 얘기를 하는 게 맞는지 확인 — 다른(예:
    // 이미 끝난) 미션의 지연 도착 메시지에 반응하지 않게.
    if (missionId != m_snapshot.missionId || queueId != m_snapshot.queueId) {
        return;
    }
    if (replayed) {
        // Wi-Fi 단절 중 Pi 로컬에 버퍼링돼 있다가 재연결 후 뒤늦게 온 이벤트 —
        // 처리 로직은 아래와 동일(상태 가드로 재실행에 안전), 로그만 표시.
        log(QStringLiteral("MISSION"), QStringLiteral("Pi 재전송 이벤트 수신 phase=%1").arg(phase));
    }

    if (phase == QStringLiteral("payload_removed")) {
        if (!m_snapshot.payloadRemoved) {
            m_snapshot.payloadRemoved = true;
            addHistory(QStringLiteral("PAYLOAD_REMOVED_DETECTED"), QString(), m_snapshot.destinationName, missionId, queueId,
                       QStringLiteral("DETECTED"), QStringLiteral("AED 반출 감지(Pi) — 잠시 후 자동 복귀"));
            emit historyChanged();
            emit missionChanged();
        }
        return;
    }

    if (phase == QStringLiteral("returning")) {
        // Pi가 이미 Nav2에 직접 복귀 goal을 보낸 뒤다 — 여기서 또
        // navigateToPose를 부르면 안 된다(중복 goal). 화면 상태만 맞춘다.
        if (m_snapshot.state == MissionState::Arrived) {
            addHistory(QStringLiteral("RETURN_HOME_STARTED"), QString(), m_snapshot.destinationName, missionId, queueId,
                       QStringLiteral("STARTED"), QStringLiteral("Pi 자동 복귀 시작"));
            m_snapshot.returningHome = true;
            setMissionState(MissionState::ReturningHome);
            emit historyChanged();
            emit missionChanged();

            // Pi가 실제로 주행을 시작한 시점 — NavGoalLogRecorder는 로컬
            // NavigationService 시그널만 보므로 이 Pi 주도 복귀는 여기서 직접
            // robot_session/nav_goal_log를 연다. beginArrivedWait()가 arrival_notice와
            // 함께 캐싱해둔 경로(m_pendingPiReturnPoses)를 그대로 쓴다.
            if (m_robotService && m_overviewService && !m_pendingPiReturnPoses.isEmpty()) {
                const QVector<RobotRecord> robots = m_robotService->listRobots(false);
                if (robots.size() == 1) {
                    QVector<NavGoalPoint> goals;
                    goals.reserve(m_pendingPiReturnPoses.size());
                    for (const NavPose &pose : m_pendingPiReturnPoses) {
                        goals.append(NavGoalPoint{pose.x, pose.y, pose.yaw});
                    }
                    const NavGoalPoint startPose{m_lastRobotX, m_lastRobotY, 0.0};
                    QString dbErrorMessage;
                    if (!m_overviewService->startRobotSession(robots.first().id, QStringLiteral("auto"),
                                                              QStringLiteral("Pi 로컬 복귀"), goals, m_robotPoseValid,
                                                              startPose, m_piReturnSessionId, dbErrorMessage)) {
                        log(QStringLiteral("MISSION"), QStringLiteral("Pi 복귀 세션 기록 실패: %1").arg(dbErrorMessage));
                        m_piReturnSessionId = -1;
                    }
                    m_piReturnLastMarkedIndex = -1;
                }
            }
        }

        // 매번(첫 진입 포함) 넘어온 진행률로, 이번에 새로 지나친 지점까지
        // 'arrived'로 마감한다. 첫 "returning"은 아직 goal도 안 나간 시점이라
        // waypointIndex가 -1(진행률 없음)로 온다 — 그때는 아무것도 안 함.
        if (m_piReturnSessionId >= 0 && waypointIndex > m_piReturnLastMarkedIndex) {
            QString dbErrorMessage;
            for (int i = m_piReturnLastMarkedIndex + 1; i < waypointIndex; ++i) {
                m_overviewService->markGoalReached(m_piReturnSessionId, i, dbErrorMessage);
            }
            m_piReturnLastMarkedIndex = waypointIndex - 1;
        }
        return;
    }

    if (phase == QStringLiteral("return_failed")) {
        if (m_snapshot.state == MissionState::Arrived || m_snapshot.state == MissionState::ReturningHome) {
            finishMission(MissionState::Failed, QStringLiteral("Pi 자동 복귀 실패"), QStringLiteral("RETURN_HOME_FAILED"),
                          QStringLiteral("FAILED"));
        }
        return;
    }
    // "armed"는 화면에 딱히 반영할 게 없어 로그만 남고 지나간다.
}
