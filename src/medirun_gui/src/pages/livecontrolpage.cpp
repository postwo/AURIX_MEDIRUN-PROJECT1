#include "livecontrolpage.h"

#include "../ros/navigationservice.h"
#include "../services/camerafeedprovider.h"
#include "../services/drivingrecordservice.h"
#include "../services/missioncontroller.h"
#include "../services/operationsoverviewservice.h"
#include "../services/robotservice.h"
#include "../services/sensortelemetryprovider.h"
#include "../services/telemetryprovider.h"
#include "../services/waypointrepository.h"
#include "../theme/dashboardpalette.h"
#include "../widgets/badgelabel.h"
#include "../widgets/mapviewwidget.h"
#include "../widgets/proportionalbar.h"

#include <QDate>
#include <QDir>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayoutItem>
#include <QMessageBox>
#include <QPixmap>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QStandardPaths>
#include <QStyle>
#include <QTimer>
#include <QUuid>
#include <QVBoxLayout>
#include <QtMath>

#include <algorithm>

#include "livecontrolpage_helpers.h"

LiveControlPage::LiveControlPage(QWidget *parent)
    : QWidget(parent)
    , m_mapView(nullptr)
{
    auto *outerLayout = new QVBoxLayout(this);
    outerLayout->setContentsMargins(0, 0, 0, 0);
    auto *canvas = panelFrame(QStringLiteral("pageCanvas"));
    outerLayout->addWidget(canvas);

    auto *layout = new QVBoxLayout(canvas);
    layout->setContentsMargins(12, 12, 12, 12);
    layout->setSpacing(10);

    layout->addWidget(buildMissionStepper());

    auto *bodyRow = new QHBoxLayout;
    bodyRow->setSpacing(10);

    auto *leftCol = new QVBoxLayout;
    leftCol->setSpacing(8);

    m_mapView = new MapViewWidget(canvas);
    m_mapView->setMinimumHeight(250);
    m_mapView->setMode(MapViewMode::Live);
    m_mapView->setCaption(QStringLiteral("실시간 위치"));
    m_mapView->setRobotVisible(false);
    m_mapView->setOverlayChipsVisible(false);

    auto *mapCameraRow = new QHBoxLayout;
    mapCameraRow->setSpacing(8);
    mapCameraRow->addWidget(m_mapView, 1);
    mapCameraRow->addWidget(buildCameraPanel(), 1);

    auto *actionRow = buildActionRow();
    leftCol->addLayout(mapCameraRow, 1);
    leftCol->addWidget(actionRow);
    bodyRow->addLayout(leftCol, 7);

    auto *rightCol = new QVBoxLayout;
    rightCol->setContentsMargins(0, 0, 0, 0);
    rightCol->setSpacing(8);
    rightCol->addWidget(buildRobotStatusPanel());
    rightCol->addWidget(buildEventStreamPanel(), 1);
    auto *callBanner = buildWaitingCallBanner();
    rightCol->addWidget(callBanner);
    auto *rightColWidget = new QWidget(canvas);
    rightColWidget->setFixedWidth(200);
    rightColWidget->setLayout(rightCol);
    bodyRow->addWidget(rightColWidget);

    layout->addLayout(bodyRow, 1);

    callBanner->setFixedHeight(actionRow->sizeHint().height());

    loadDefaultMap();
    refreshMissionStepper();
    refreshWaitingCallBanner();

    m_eventPollTimer = new QTimer(this);
    connect(m_eventPollTimer, &QTimer::timeout, this, &LiveControlPage::refreshEventStream);

    // 속도 라벨은 /odom 콜백 주기(수십 Hz)가 아니라 이 타이머 주기로만 갱신한다
    // (handleOdomVelocityUpdated는 값만 다듬어 저장, 화면 반영은 여기서).
    m_speedUpdateTimer = new QTimer(this);
    connect(m_speedUpdateTimer, &QTimer::timeout, this, &LiveControlPage::updateSpeedLabel);
    m_speedUpdateTimer->start(300);

    m_recordingElapsedTimer = new QTimer(this);
    connect(m_recordingElapsedTimer, &QTimer::timeout, this, &LiveControlPage::tickRecordingElapsed);
}

void LiveControlPage::setSession(const AppSession &session)
{
    m_session = session;
}

void LiveControlPage::setMissionController(MissionController *controller)
{
    if (m_missionController == controller) {
        return;
    }
    m_missionController = controller;
    if (m_missionController != nullptr) {
        connect(m_missionController, &MissionController::missionChanged, this, &LiveControlPage::refreshMissionStepper);
        connect(m_missionController, &MissionController::queueChanged, this, &LiveControlPage::refreshWaitingCallBanner);
        connect(m_missionController, &MissionController::missionChanged, this, &LiveControlPage::refreshMap);
    }
    refreshMissionStepper();
    refreshWaitingCallBanner();
}

void LiveControlPage::setNavigationService(NavigationService *service)
{
    if (m_navigationService == service) {
        return;
    }
    m_navigationService = service;
    if (m_navigationService != nullptr) {
        connect(m_navigationService, &NavigationService::amclPoseUpdated, this, &LiveControlPage::handleAmclPoseUpdated);
        connect(m_navigationService, &NavigationService::odomVelocityUpdated, this, &LiveControlPage::handleOdomVelocityUpdated);
        connect(m_navigationService, &NavigationService::nav2ConnectionChanged, this, &LiveControlPage::handleNav2ConnectionChanged);
        // Nav2가 이 페이지보다 먼저 연결된 상태였다면 nav2ConnectionChanged를
        // 놓쳤을 수 있어 현재 상태로 한 번 동기화한다. amcl_pose는
        // TRANSIENT_LOCAL이라 amclPoseUpdated도 같은 문제가 있다.
        handleNav2ConnectionChanged(m_navigationService->isNav2Connected());
        if (double x, y, yaw; m_navigationService->lastKnownAmclPose(x, y, yaw)) {
            handleAmclPoseUpdated(x, y, yaw);
        }
    }
}

void LiveControlPage::setTelemetryProvider(TelemetryProvider *provider)
{
    if (m_telemetryProvider == provider) {
        return;
    }
    m_telemetryProvider = provider;
    if (m_telemetryProvider != nullptr) {
        connect(m_telemetryProvider, &TelemetryProvider::snapshotUpdated, this, &LiveControlPage::handleTelemetrySnapshot);
    }
}

void LiveControlPage::setSensorTelemetryProvider(SensorTelemetryProvider *provider)
{
    if (m_sensorTelemetryProvider == provider) {
        return;
    }
    m_sensorTelemetryProvider = provider;
    if (m_sensorTelemetryProvider != nullptr) {
        connect(m_sensorTelemetryProvider, &SensorTelemetryProvider::frontDistanceChanged, this, &LiveControlPage::handleFrontDistanceChanged);
        connect(m_sensorTelemetryProvider, &SensorTelemetryProvider::rearDistanceChanged, this, &LiveControlPage::handleRearDistanceChanged);
        connect(m_sensorTelemetryProvider, &SensorTelemetryProvider::payloadLoadedChanged, this, &LiveControlPage::handlePayloadLoadedChanged);
    }
}

void LiveControlPage::setCameraFeedProvider(CameraFeedProvider *provider)
{
    if (m_cameraFeedProvider == provider) {
        return;
    }
    m_cameraFeedProvider = provider;
    if (m_cameraFeedProvider != nullptr) {
        connect(m_cameraFeedProvider, &CameraFeedProvider::frameReady, this, &LiveControlPage::handleCameraFrameReady);
    }
}

void LiveControlPage::setWaypointRepository(WaypointRepository *repository)
{
    if (m_waypointRepository == repository) {
        return;
    }
    m_waypointRepository = repository;
    if (m_waypointRepository != nullptr) {
        connect(m_waypointRepository, &WaypointRepository::repositoryChanged, this, &LiveControlPage::refreshMap);
    }
    // 생성자 시점엔 아직 리포지토리가 주입되기 전이라 loadDefaultMap()이 항상
    // no-op이었다 — 실제로 리포지토리가 들어오는 지금 다시 시도해야 배경
    // 지도가 뜬다. 이미 로드돼 있으면(m_mapMeta.valid) 다시 읽지 않는다.
    if (!m_mapMeta.valid) {
        loadDefaultMap();
        // setNavigationService()가 setWaypointRepository()보다 먼저 불려서
        // (GuiApplicationController 순서) 그때는 m_mapMeta가 아직 무효라
        // handleAmclPoseUpdated()가 조용히 무시했다 — 지도가 이제 막 유효해졌으니
        // 마지막 위치를 다시 한번 반영한다.
        if (m_mapMeta.valid && m_navigationService != nullptr) {
            if (double x, y, yaw; m_navigationService->lastKnownAmclPose(x, y, yaw)) {
                handleAmclPoseUpdated(x, y, yaw);
            }
        }
    }
    refreshMap();
}

void LiveControlPage::setOperationsOverviewService(OperationsOverviewService *service)
{
    m_operationsOverviewService = service;
    if (m_operationsOverviewService != nullptr) {
        refreshEventStream();
        if (m_eventPollTimer != nullptr && !m_eventPollTimer->isActive()) {
            m_eventPollTimer->start(kEventPollIntervalMs);
        }
    }
}

void LiveControlPage::setRobotService(RobotService *service)
{
    m_robotService = service;
}

void LiveControlPage::setDrivingRecordService(DrivingRecordService *service)
{
    if (m_drivingRecordService == service) {
        return;
    }
    m_drivingRecordService = service;
    if (m_drivingRecordService != nullptr) {
        connect(m_drivingRecordService, &DrivingRecordService::recordingStateChanged, this, &LiveControlPage::handleRecordingStateChanged);
        connect(m_drivingRecordService, &DrivingRecordService::recordingPausedChanged, this, &LiveControlPage::handleRecordingPausedChanged);
        connect(m_drivingRecordService, &DrivingRecordService::recordingSaveFinished, this, &LiveControlPage::handleRecordingSaveFinished);
        handleRecordingStateChanged(m_drivingRecordService->isRecording());
        handleRecordingPausedChanged(m_drivingRecordService->isPaused());
    }
}
