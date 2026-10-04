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

void MissionController::announceTestDrive(const QString &waypointLabel)
{
    m_rosInterface->publishAudioAnnouncement(waypointLabel.isEmpty()
        ? QStringLiteral("지정한 위치로 이동합니다")
        : QStringLiteral("%1(으)로 이동합니다").arg(waypointLabel));
}

void MissionController::requestRetry()
{
    if (m_retryInProgress) {
        return;
    }
    m_retryInProgress = true;
    emit retryStateChanged();
    addHistory(QStringLiteral("ROS_RETRY_STARTED"), QString(), QString(), m_snapshot.missionId, m_snapshot.queueId,
               QStringLiteral("STARTED"), QStringLiteral("ROS2 재연결 시도 시작"));
    log(QStringLiteral("ROS2"), QStringLiteral("retry requested"));
    m_navigationService->retryConnections();
    m_rosInterface->retryCommunication();
}

void MissionController::requestReturnHome()
{
    // 미션 취소/홈 복귀는 조종 권한(리더 선출)과 무관하게 어느 창에서든
    // 요청할 수 있다 — cancelGoal()이 로컬 goal 핸들이 없어도 Nav2 액션
    // 서버에 전체 취소를 요청하는 "orphan-safe" 경로를 타므로, 팀원이 다른
    // 창에서 눌러도 안전하게 동작한다(nav2navigationservice.cpp 참고).
    if (beginReturnHomeFlow(QStringLiteral("운영자 홈 복귀 요청"))) {
        m_rosInterface->publishMissionCommand(QStringLiteral("return_home"), m_snapshot.missionId, m_snapshot.queueId, QString());
    }
}

void MissionController::requestMissionCancel()
{
    if (m_snapshot.state == MissionState::Idle) {
        addHistory(QStringLiteral("MISSION_CANCEL_IGNORED"), QString(), QString(), QString(), QString(),
                   QStringLiteral("IGNORED"), QStringLiteral("취소할 진행 중 미션이 없습니다."));
        emit historyChanged();
        return;
    }
    if (m_snapshot.state == MissionState::Cancelling || m_snapshot.state == MissionState::Diverting) {
        return;
    }

    // 이미 호출구역에 도착해 AED 반출 대기 중이면 "그 자리에서 정지"는 의미가
    // 없다(로봇이 이미 멈춰 있음) — 운영자 확인(2026-08-19)에 따라 즉시 홈
    // 복귀로 처리한다. 취소할 Nav2 goal 자체가 없으므로 beginReturnHomeFlow의
    // Arrived 분기(cancelGoal 없이 바로 홈으로 출발)를 그대로 탄다.
    if (m_snapshot.state == MissionState::Arrived) {
        if (beginReturnHomeFlow(QStringLiteral("운영자 미션 취소 요청 — 호출구역 도착 후라 즉시 홈 복귀로 처리"))) {
            m_rosInterface->publishMissionCommand(QStringLiteral("return_home"), m_snapshot.missionId, m_snapshot.queueId, QString());
        }
        return;
    }

    if (m_snapshot.state == MissionState::Parking) {
        // 전면주차는 Nav2 goal이 아니라 docking_manager가 /cmd_vel을 직접
        // 제어하는 중이라 cancelGoal()로는 못 멈춘다 — dock_request(false)로
        // 정지시키고, 결과는 handleDockResult가 "취소" 사유를 보고 Cancelled로
        // 마무리한다.
        m_pendingReason = QStringLiteral("운영자 미션 취소 요청 — 전면주차 중단");
        addHistory(QStringLiteral("MISSION_CANCEL_REQUESTED"), QString(), m_snapshot.destinationName, m_snapshot.missionId,
                   m_snapshot.queueId, QStringLiteral("REQUESTED"), m_pendingReason);
        emit historyChanged();
        m_rosInterface->cancelDocking();
        return;
    }

    // 홈 복귀 없이 그 자리에서 정지 + 진행 중이던 호출(큐 항목)은 삭제한다.
    // 실제 종료 처리(Cancelled 상태 전환, 큐 삭제)는 Nav2가 취소를 확인해 준
    // 뒤 handleNavigationResult()의 CANCELED 처리에서 수행한다.
    m_pendingAction = PendingAction::None;
    m_pendingReason = QStringLiteral("운영자 미션 취소 요청 — 현재 위치에서 정지");
    setMissionState(MissionState::Cancelling);
    addHistory(QStringLiteral("MISSION_CANCEL_REQUESTED"), QString(), m_snapshot.destinationName, m_snapshot.missionId, m_snapshot.queueId,
               QStringLiteral("REQUESTED"), m_pendingReason);
    emit historyChanged();
    m_navigationService->cancelGoal();
    m_rosInterface->publishMissionCommand(QStringLiteral("cancel"), m_snapshot.missionId, m_snapshot.queueId, QString());
}

void MissionController::requestQueueCancel(const QString &queueId)
{
    const int index = queueIndex(queueId);
    if (index < 0) {
        return;
    }
    if (m_queueItems.at(index).status == MissionQueueStatus::Active) {
        requestMissionCancel();
        return;
    }

    m_queueItems[index].status = MissionQueueStatus::Cancelled;
    const MissionQueueItem item = m_queueItems.at(index);
    saveQueue();
    m_queueItems.removeAt(index);
    addHistory(QStringLiteral("QUEUE_CANCELLED"), item.waypointId, item.waypointName, item.missionId, item.queueId,
               QStringLiteral("CANCELLED"), QStringLiteral("대기 호출 큐 취소"));
    log(QStringLiteral("QUEUE"), QStringLiteral("cancelled queue_id=%1 waypoint_id=%2").arg(item.queueId, item.waypointId));
    emit queueChanged();
    emit historyChanged();
}

void MissionController::requestQueueDivert(const QString &queueId)
{
    const int index = queueIndex(queueId);
    if (index < 0) {
        return;
    }
    if (m_snapshot.state == MissionState::Idle) {
        startMissionForQueue(queueId, false);
        return;
    }
    // Arrived(호출구역 도착 후 대기)는 취소할 활성 Nav2 goal이 없어 여기 아래의
    // cancelGoal()+CANCELED 대기 흐름을 못 탄다 — 도착 후 전환은 막고, 운영자는
    // 먼저 "복귀"나 "취소"(둘 다 즉시 홈 복귀로 처리됨)로 이 미션부터 끝내야 한다.
    if (m_snapshot.returningHome || m_snapshot.state == MissionState::Arrived
        || m_snapshot.state == MissionState::Cancelling || m_snapshot.state == MissionState::Diverting) {
        return;
    }

    m_pendingAction = PendingAction::Divert;
    m_pendingDivertQueueId = queueId;
    setMissionState(MissionState::Diverting);
    m_pendingReason = QStringLiteral("확인 전환");
    log(QStringLiteral("MISSION"), QStringLiteral("divert start queue_id=%1 mission_id=%2").arg(queueId, m_snapshot.missionId));
    addHistory(QStringLiteral("MISSION_DIVERT_REQUESTED"), m_queueItems.at(index).waypointId, m_queueItems.at(index).waypointName,
               m_snapshot.missionId, queueId, QStringLiteral("REQUESTED"), QStringLiteral("확인 전환 시작"));
    m_navigationService->cancelGoal();
}

void MissionController::handleDispatchRequested(const QString &buttonId,
                                                const QString &requestSource,
                                                const QString &missionId,
                                                const QDateTime &requestedAt)
{
    enqueueCall(buttonId, requestSource, missionId, requestedAt);
}

void MissionController::handleWaypointRepositoryChanged()
{
    if (!m_snapshot.missionId.isEmpty()) {
        m_snapshot.currentWaypointName = m_robotPoseValid ? nearestWaypointName(m_lastRobotX, m_lastRobotY) : m_snapshot.currentWaypointName;
        emit missionChanged();
    }
}

void MissionController::handleNavigationAccepted()
{
    m_goalAccepted = true;
    if (m_snapshot.state == MissionState::Starting) {
        setMissionState(MissionState::Navigating);
    }
    log(QStringLiteral("NAV2"), QStringLiteral("goal accepted mission_id=%1 queue_id=%2").arg(m_snapshot.missionId, m_snapshot.queueId));
}

void MissionController::handleNavigationRejected(const QString &reason)
{
    m_goalAccepted = false;
    const QString detail = reason.isEmpty() ? QStringLiteral("goal rejected") : reason;
    log(QStringLiteral("NAV2"), QStringLiteral("goal rejected mission_id=%1 detail=%2").arg(m_snapshot.missionId, detail));
    finishMission(MissionState::Failed, detail, QStringLiteral("NAV_GOAL_REJECTED"), QStringLiteral("FAILED"));
}

void MissionController::handleNavigationResult(const QString &resultCode, const QString &detail)
{
    log(QStringLiteral("NAV2"),
        QStringLiteral("goal result mission_id=%1 queue_id=%2 result=%3 detail=%4")
            .arg(m_snapshot.missionId, m_snapshot.queueId, resultCode, detail));

    if (resultCode == QStringLiteral("CANCELED")) {
        if (m_pendingAction == PendingAction::Divert && !m_pendingDivertQueueId.isEmpty()) {
            if (MissionQueueItem *item = activeQueueItem()) {
                item->status = MissionQueueStatus::Diverted;
            }
            addHistory(QStringLiteral("MISSION_DIVERTED"), QString(), m_snapshot.destinationName, m_snapshot.missionId, m_snapshot.queueId,
                       QStringLiteral("DIVERTED"), QStringLiteral("기존 미션을 중단하고 새 목적지로 전환"));
            const QString divertedQueueId = m_pendingDivertQueueId;
            m_pendingAction = PendingAction::None;
            m_pendingDivertQueueId.clear();
            startMissionForQueue(divertedQueueId, true);
            return;
        }
        if (m_pendingAction == PendingAction::ReturnHome) {
            dispatchPendingReturnHome();
            return;
        }

        // 미션 취소(그 자리에서 정지) — Nav2가 취소를 확인해준 시점. finishMission이
        // activeQueueItem()으로 이 항목을 찾아 Cancelled로 표시하고 DB에도 반영한다
        // (Completed/Failed와 동일한 패턴). 예전에는 여기서 removeQueueItem()을 먼저
        // 불러 큐에서 지워버린 뒤 finishMission을 호출했는데, 그러면 activeQueueItem()이
        // 이미 사라진 항목을 못 찾아 상태를 못 바꾸고 DB에는 마지막으로 저장된
        // "navigating"이 그대로 남는 버그가 있었다 — 취소해도 DB만 계속 주행 중으로
        // 보였다. Waiting/Active만 화면에 보여주는 필터(queueSummaryText 등) 덕분에
        // 큐에서 안 지워도 목록에서는 정상적으로 사라진다.
        // 다음 대기 호출로 바로 주행하지 않고, 운영자에게 확인을 받는다.
        finishMission(MissionState::Cancelled, m_pendingReason.isEmpty() ? QStringLiteral("운영자 미션 취소 요청") : m_pendingReason,
                      QStringLiteral("MISSION_CANCELLED"), QStringLiteral("CANCELLED"), true);
        return;
    }

    if (resultCode == QStringLiteral("SUCCEEDED")) {
        // 홈 복귀 요청으로 cancelGoal()을 보내고 CANCELED를 기다리는 중(state
        // == Cancelling)이었는데, 취소 확인보다 기존 목표의 SUCCEEDED가 먼저
        // 도착하는 경쟁 상태가 있었다 — 이 경우 아래로 흘러가 beginArrivedWait()를
        // 타면서 홈 복귀 요청 자체가 조용히 무시됐다("웨이포인트 도착 후 홈
        // 복귀 버튼을 눌러도 반응 없음" 버그). CANCELED로 오든 SUCCEEDED로
        // 오든 대기 중이던 홈 복귀 요청이 항상 이기도록 여기서도 처리한다.
        if (m_snapshot.state != MissionState::ReturningHome && m_pendingAction == PendingAction::ReturnHome) {
            dispatchPendingReturnHome();
            return;
        }
        if (m_snapshot.state == MissionState::ReturningHome) {
            if (m_pendingReason.contains(QStringLiteral("취소"))) {
                finishMission(MissionState::Cancelled, m_pendingReason, QStringLiteral("RETURN_HOME_COMPLETED"), QStringLiteral("SUCCEEDED"));
                return;
            }
            // 홈(또는 HOME_PRE_DOCK) 도착 — 여기서 바로 끝내지 않고 항상 ArUco
            // 전면주차로 이어간다. docking_manager는 simulate_docking 파라미터가
            // true(기본값)면 카메라/마커 없이도 잠깐 대기 후 SUCCEEDED를 돌려주므로,
            // 카메라가 없는 로봇도 안전하게 그대로 완료된다 — false로 바꾼
            // 로봇만 실제 ArUco 인식을 수행한다. /medirun/vision/dock_result가
            // 오면 handleDockResult에서 최종 마무리한다.
            addHistory(QStringLiteral("PARKING_STARTED"), QString(), m_snapshot.destinationName, m_snapshot.missionId,
                       m_snapshot.queueId, QStringLiteral("STARTED"), QStringLiteral("홈 도착 — ArUco 전면주차 시작"));
            setMissionState(MissionState::Parking);
            emit historyChanged();
            m_rosInterface->requestDocking();
            return;
        }
        // 호출구역 도착 — 여기서 바로 미션을 끝내지 않고 AED 반출 대기로
        // 전환한다(SR-F-010/TC-12: 반출 감지 15초 후 자동 복귀, 미반출 시
        // 60초 안내 반복 후 180초 경과 시 강제 복귀).
        beginArrivedWait();
        return;
    }

    finishMission(MissionState::Failed, detail, QStringLiteral("MISSION_FAILED"), QStringLiteral("FAILED"));
}

void MissionController::handleNav2ConnectionChanged(bool connected)
{
    m_nav2Connected = connected;
    updateConnectionState();
    emit connectionStateChanged();
}

void MissionController::handleNavigationFeedback(double metersRemaining)
{
    m_snapshot.distanceRemainingMeters = metersRemaining;
    updateProgress();
    emit missionChanged();
}

void MissionController::handleAmclPoseUpdated(double x, double y, double)
{
    m_lastRobotX = x;
    m_lastRobotY = y;
    m_robotPoseValid = true;
    if (!m_snapshot.missionId.isEmpty()) {
        m_snapshot.currentWaypointName = nearestWaypointName(x, y);
        updateProgress();
        emit missionChanged();
    }
}

void MissionController::handleRosHeartbeat(const QString &source, const QDateTime &receivedAt)
{
    m_lastRosHeartbeat = receivedAt;
    m_lastRosHeartbeatSource = source;
    updateConnectionState();
    emit connectionStateChanged();
}

void MissionController::handleRosRetryFinished(bool success, const QString &detail)
{
    const bool finalSuccess = success && m_nav2Connected;
    m_retryInProgress = false;
    addHistory(finalSuccess ? QStringLiteral("ROS_RETRY_SUCCEEDED") : QStringLiteral("ROS_RETRY_FAILED"),
               QString(), QString(), m_snapshot.missionId, m_snapshot.queueId,
               finalSuccess ? QStringLiteral("SUCCEEDED") : QStringLiteral("FAILED"), detail);
    log(QStringLiteral("ROS2"), QStringLiteral("retry result success=%1 detail=%2").arg(finalSuccess).arg(detail));
    updateConnectionState();
    emit retryStateChanged();
    emit historyChanged();
}

void MissionController::updateElapsed()
{
    if (m_snapshot.startedAt.isValid() && m_snapshot.state != MissionState::Idle) {
        m_snapshot.elapsedSeconds = m_snapshot.startedAt.secsTo(QDateTime::currentDateTime());
        emit missionChanged();
    }
    updateConnectionState();
}
