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

MissionController::MissionController(const AppSession &session,
                                     NavigationService *navigationService,
                                     WaypointRepository *waypointRepository,
                                     MissionStorageService *storageService,
                                     MissionRosInterface *rosInterface,
                                     RobotService *robotService,
                                     OperationsOverviewService *overviewService,
                                     QObject *parent)
    : QObject(parent)
    , m_session(session)
    , m_navigationService(navigationService)
    , m_waypointRepository(waypointRepository)
    , m_storageService(storageService)
    , m_rosInterface(rosInterface)
    , m_robotService(robotService)
    , m_overviewService(overviewService)
{
    m_storageService->setSessionContext(m_session);
    m_queueItems = m_storageService->loadQueue();
    m_historyEntries = m_storageService->loadTodayHistory();
    m_observedButtonIds = QSettings().value(QLatin1String(kObservedButtonIdsSettingsKey)).toStringList();
    m_elapsedTimer = new QTimer(this);
    m_elapsedTimer->setInterval(1000);
    connect(m_elapsedTimer, &QTimer::timeout, this, &MissionController::updateElapsed);
    m_elapsedTimer->start();

    // 복귀·도킹 완료(Completed) 직후 startNextQueueIfPossible()이 바로
    // resetToIdleSnapshot()을 불러버리면 실시간 관제 스텝퍼의 "완료" 체크마크가
    // 화면에 그려질 새도 없이 지워진다(2026-08-24). 그래서 Completed일 때만
    // startNextQueueIfPossible() 호출을 이 타이머로 30초 미룬다 — 그동안
    // 완료 체크마크가 보이고, 새 호출이 들어오면 enqueueCall()이 어차피
    // 즉시 startNextQueueIfPossible()을 부르니 대기할 필요 없이 바로 처리된다.
    m_completionResetTimer = new QTimer(this);
    m_completionResetTimer->setSingleShot(true);
    connect(m_completionResetTimer, &QTimer::timeout, this, &MissionController::startNextQueueIfPossible);

    reconcileOrphanedActiveMission();

    connect(m_waypointRepository, &WaypointRepository::repositoryChanged, this, &MissionController::handleWaypointRepositoryChanged);

    connect(m_navigationService, &NavigationService::navigationGoalAccepted, this, &MissionController::handleNavigationAccepted);
    connect(m_navigationService, &NavigationService::navigationGoalRejected, this, &MissionController::handleNavigationRejected);
    connect(m_navigationService, &NavigationService::navigationResult, this, &MissionController::handleNavigationResult);
    connect(m_navigationService, &NavigationService::nav2ConnectionChanged, this, &MissionController::handleNav2ConnectionChanged);
    connect(m_navigationService, &NavigationService::feedbackDistanceRemaining, this, &MissionController::handleNavigationFeedback);
    connect(m_navigationService, &NavigationService::amclPoseUpdated, this, &MissionController::handleAmclPoseUpdated);
    // Nav2가 이 컨트롤러보다 먼저 연결된 상태였다면 nav2ConnectionChanged를
    // 놓쳤을 수 있어 현재 상태로 한 번 동기화한다. amcl_pose는
    // TRANSIENT_LOCAL이라 amclPoseUpdated도 같은 문제가 있다.
    handleNav2ConnectionChanged(m_navigationService->isNav2Connected());
    if (double x, y, yaw; m_navigationService->lastKnownAmclPose(x, y, yaw)) {
        handleAmclPoseUpdated(x, y, yaw);
    }

    connect(m_rosInterface, &MissionRosInterface::dispatchRequested, this, &MissionController::handleDispatchRequested);
    connect(m_rosInterface, &MissionRosInterface::missionHeartbeat, this, &MissionController::handleRosHeartbeat);
    connect(m_rosInterface, &MissionRosInterface::communicationRetryFinished, this, &MissionController::handleRosRetryFinished);
    connect(m_rosInterface, &MissionRosInterface::dockResultReceived, this, &MissionController::handleDockResult);
    connect(m_rosInterface, &MissionRosInterface::piReturnStatusReceived, this, &MissionController::handlePiReturnStatus);

    updateConnectionState();
    startNextQueueIfPossible();
}

MissionSnapshot MissionController::missionSnapshot() const
{
    return m_snapshot;
}

QVector<MissionQueueItem> MissionController::waitingQueue() const
{
    return m_queueItems;
}

QVector<MissionHistoryEntry> MissionController::todayHistory() const
{
    // DB(system_event 등)에는 전부 남지만, GUI에는 최근 24시간 안의 항목만
    // 보여준다 — 자정 기준 "오늘"이 아니라 지금 시각 기준 rolling 24시간이라
    // GUI를 며칠씩 켜놔도 표시 목록이 무한정 쌓이지 않는다.
    const QDateTime cutoff = QDateTime::currentDateTime().addSecs(-24 * 60 * 60);
    QVector<MissionHistoryEntry> recent;
    recent.reserve(m_historyEntries.size());
    for (const MissionHistoryEntry &entry : m_historyEntries) {
        if (entry.occurredAt >= cutoff) {
            recent.append(entry);
        }
    }
    return recent;
}

QString MissionController::queueSummaryText() const
{
    const int count = std::count_if(m_queueItems.cbegin(), m_queueItems.cend(), [](const MissionQueueItem &item) {
        return item.status == MissionQueueStatus::Waiting || item.status == MissionQueueStatus::Active;
    });
    return QStringLiteral("대기 호출 큐 · %1건").arg(count);
}

QString MissionController::anomalyTitle() const
{
    if (!m_nav2Connected || !m_rosConnected) {
        return QStringLiteral("⚠ 이상 상황 · 통신 확인 필요");
    }
    if (m_snapshot.state == MissionState::Failed) {
        return QStringLiteral("⚠ 이상 상황 · 미션 실패");
    }
    return QStringLiteral("⚠ 이상 상황 · 모니터링 중");
}

QString MissionController::anomalyDetail() const
{
    if (!m_nav2Connected || !m_rosConnected) {
        return QStringLiteral("ROS2 graph 또는 Nav2 연결이 불안정합니다. 재시도로 graph와 action 연결을 다시 확인하세요.");
    }
    if (!m_snapshot.failureReason.isEmpty()) {
        return m_snapshot.failureReason;
    }
    return QStringLiteral("현재 연결은 정상입니다. 미션/큐 상태를 계속 감시합니다.");
}

QString MissionController::anomalyMeta() const
{
    const QString missionId = m_snapshot.missionId.isEmpty() ? QStringLiteral("-") : m_snapshot.missionId;
    const QString location = m_snapshot.destinationName.isEmpty() ? QStringLiteral("-") : m_snapshot.destinationName;
    return QStringLiteral("%1 · 미션 %2 · %3").arg(QTime::currentTime().toString(QStringLiteral("HH:mm:ss")), missionId, location);
}

bool MissionController::rosConnected() const
{
    return m_rosConnected;
}

bool MissionController::nav2Connected() const
{
    return m_nav2Connected;
}

bool MissionController::retryInProgress() const
{
    return m_retryInProgress;
}

QStringList MissionController::observedButtonIds() const
{
    return m_observedButtonIds;
}
