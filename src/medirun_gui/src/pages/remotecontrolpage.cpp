#include "remotecontrolpage.h"

#include "../models/waypointtypes.h"
#include "../ros/navigationservice.h"
#include "../services/camerafeedprovider.h"
#include "../services/mapfile.h"
#include "../services/operationsoverviewservice.h"
#include "../services/robotservice.h"
#include "../services/sensortelemetryprovider.h"
#include "../theme/dashboardpalette.h"
#include "../widgets/badgelabel.h"
#include "../widgets/mapviewwidget.h"
#include "../widgets/proportionalbar.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHideEvent>
#include <QLabel>
#include <QMessageBox>
#include <QPixmap>
#include <QPushButton>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QStyle>
#include <QVBoxLayout>
#include <QtMath>

#include "remotecontrolpage_helpers.h"

RemoteControlPage::RemoteControlPage(RobotService &robotService, OperationsOverviewService &overviewService, QWidget *parent)
    : QWidget(parent)
    , m_robotService(robotService)
    , m_overviewService(overviewService)
{
    m_teleop = new TeleopController(this);
    connect(m_teleop, &TeleopController::velocityCommandChanged, this, &RemoteControlPage::handleTeleopVelocityChanged);
    connect(m_teleop, &TeleopController::activeChanged, this, &RemoteControlPage::handleTeleopActiveChanged);
    connect(m_teleop, &TeleopController::speedPresetChanged, this, &RemoteControlPage::handleTeleopSpeedPresetChanged);

    auto *outerLayout = new QVBoxLayout(this);
    outerLayout->setContentsMargins(0, 0, 0, 0);
    auto *canvas = panelFrame(QStringLiteral("pageCanvas"));
    outerLayout->addWidget(canvas);

    auto *layout = new QVBoxLayout(canvas);
    layout->setContentsMargins(12, 12, 12, 12);
    layout->setSpacing(10);

    layout->addWidget(buildManualControlBanner());

    auto *bodyRow = new QHBoxLayout;
    bodyRow->setSpacing(10);

    auto *bigSlotWidget = new QWidget(canvas);
    bigSlotWidget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    m_bigSlot = new QVBoxLayout(bigSlotWidget);
    m_bigSlot->setContentsMargins(0, 0, 0, 0);
    bodyRow->addWidget(bigSlotWidget, 15);

    auto *rightCol = new QVBoxLayout;
    rightCol->setContentsMargins(0, 0, 0, 0);
    rightCol->setSpacing(8);

    auto *smallSlotWidget = new QWidget;
    smallSlotWidget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    m_smallSlot = new QVBoxLayout(smallSlotWidget);
    m_smallSlot->setContentsMargins(0, 0, 0, 0);
    rightCol->addWidget(smallSlotWidget);

    auto *swapButton = buildViewSwapButton();
    swapButton->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    rightCol->addWidget(swapButton);

    auto *loadMapButton = buildLoadMapButton();
    loadMapButton->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    rightCol->addWidget(loadMapButton);

    auto *refreshMarkersButton = buildRefreshMarkersButton();
    refreshMarkersButton->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    rightCol->addWidget(refreshMarkersButton);

    m_cameraPanel = buildCameraPanel();
    m_minimapPanel = buildMinimapPanel();
    m_bigSlot->addWidget(m_cameraPanel);
    m_smallSlot->addWidget(m_minimapPanel);

    auto *directionPad = buildDirectionPad();
    directionPad->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    rightCol->addWidget(directionPad);

    auto *speedPresetPanel = buildSpeedPresetPanel();
    speedPresetPanel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    rightCol->addWidget(speedPresetPanel);

    auto *cmdVelPanel = buildCmdVelPanel();
    cmdVelPanel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    rightCol->addWidget(cmdVelPanel);

    auto *sensorStatusPanel = buildSensorStatusPanel();
    sensorStatusPanel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    rightCol->addWidget(sensorStatusPanel);

    rightCol->addStretch(1);

    auto *bottomButtonRow = new QHBoxLayout;
    bottomButtonRow->setSpacing(6);
    auto *dockButton = new QPushButton(QStringLiteral("도킹 재시도"));
    dockButton->setObjectName(QStringLiteral("cyanOutlineButton"));
    dockButton->setMinimumHeight(46);
    auto *autonomyButton = new QPushButton(QStringLiteral("자율 복귀"));
    autonomyButton->setMinimumHeight(46);
    bottomButtonRow->addWidget(dockButton, 1);
    bottomButtonRow->addWidget(autonomyButton, 1);
    rightCol->addLayout(bottomButtonRow);

    auto *rightColWidget = new QWidget(canvas);
    rightColWidget->setFixedWidth(224);
    rightColWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
    rightColWidget->setLayout(rightCol);
    bodyRow->addWidget(rightColWidget);

    layout->addLayout(bodyRow, 1);

    // 기본 큰 화면은 카메라가 아니라 지도 — 진입하자마자 로봇 위치/지도 정렬부터
    // 확인할 수 있도록 한다. swapCameraAndMap()이 이미 큰/작은 슬롯 크기·오버레이
    // 가시성을 전부 처리하므로 그대로 재사용.
    swapCameraAndMap();

    // 페이지 진입 시 지도가 비어 있지 않도록 기본 지도를 자동으로 불러온다
    // (WaypointManagementPage::autoLoadDefaultMap()과 동일한 방식).
    autoLoadDefaultMap();
}

void RemoteControlPage::setCameraFeedProvider(CameraFeedProvider *provider)
{
    if (m_cameraFeedProvider != nullptr) {
        disconnect(m_cameraFeedProvider, nullptr, this, nullptr);
    }

    m_cameraFeedProvider = provider;
    m_lastCameraFrame = QImage();
    applyCameraConnectionState(false);

    if (m_cameraFeedProvider == nullptr) {
        return;
    }

    connect(m_cameraFeedProvider, &CameraFeedProvider::frameReady, this, &RemoteControlPage::applyCameraFrame);
    connect(m_cameraFeedProvider, &CameraFeedProvider::connectionStateChanged, this, &RemoteControlPage::applyCameraConnectionState);
}

void RemoteControlPage::setNavigationService(NavigationService *service)
{
    if (m_navigationService != nullptr) {
        disconnect(m_navigationService, nullptr, this, nullptr);
    }

    m_navigationService = service;
    updateInitialPoseButtonEnabled();

    if (m_navigationService == nullptr) {
        return;
    }

    connect(m_navigationService, &NavigationService::amclPoseUpdated, this, &RemoteControlPage::handleAmclPoseUpdated);
    connect(m_navigationService, &NavigationService::odomVelocityUpdated, this, &RemoteControlPage::handleOdomVelocityUpdated);
    // 웨이포인트 관리 페이지 등 다른 페이지에서 시작한 주행이라도 같은
    // NavigationService를 공유하므로 여기서도 감지된다.
    connect(m_navigationService, &NavigationService::navSessionStarted, this, &RemoteControlPage::handleNavSessionStarted);
    connect(m_navigationService, &NavigationService::navSessionFinished, this, &RemoteControlPage::handleNavSessionFinished);
    // navSessionStarted를 놓친 뒤늦은 연결(이 페이지가 열리기 전에 이미
    // 주행이 시작된 경우)이라도, 즉시 현재 상태로 동기화해서 배너/방향
    // 패드를 맞춰준다.
    if (m_navigationService->isNavigating()) {
        handleNavSessionStarted({}, QString());
    }
    // amcl_pose는 TRANSIENT_LOCAL이라 앱 시작 직후(로그인 전) 이미 한 번
    // 와버렸을 수 있어, 이 페이지가 연결하기 전 시그널을 놓쳤을 수 있다.
    if (double x, y, yaw; m_navigationService->lastKnownAmclPose(x, y, yaw)) {
        handleAmclPoseUpdated(x, y, yaw);
    }
}

void RemoteControlPage::setSensorTelemetryProvider(SensorTelemetryProvider *provider)
{
    if (m_sensorTelemetryProvider == provider) {
        return;
    }
    m_sensorTelemetryProvider = provider;
    if (m_sensorTelemetryProvider != nullptr) {
        connect(m_sensorTelemetryProvider, &SensorTelemetryProvider::frontDistanceChanged, this, &RemoteControlPage::handleFrontDistanceChanged);
        connect(m_sensorTelemetryProvider, &SensorTelemetryProvider::rearDistanceChanged, this, &RemoteControlPage::handleRearDistanceChanged);
        connect(m_sensorTelemetryProvider, &SensorTelemetryProvider::payloadLoadedChanged, this, &RemoteControlPage::handlePayloadLoadedChanged);
    }
}

void RemoteControlPage::setSession(const AppSession &session)
{
    m_session = session;
}

bool RemoteControlPage::handleManualKeyPress(int key)
{
    if (m_navigationInProgress) {
        return false;
    }
    return m_teleop->consumeKey(key);
}

bool RemoteControlPage::handleManualKeyRelease(int key)
{
    if (m_navigationInProgress) {
        return false;
    }
    return m_teleop->releaseKey(key);
}

void RemoteControlPage::hideEvent(QHideEvent *event)
{
    // 다른 페이지로 전환될 때 눌려있던 키/버튼이 있어도 즉시 정지 —
    // 화면을 보고 있지 않은 상태에서 로봇이 계속 움직이면 안 된다.
    m_teleop->stop();
    QWidget::hideEvent(event);
}

void RemoteControlPage::showEvent(QShowEvent *event)
{
    // 웨이포인트 관리 페이지에서 콜존/홈을 추가·수정하고 돌아왔을 때 바로
    // 반영되도록, 페이지가 다시 보일 때마다 DB 마커를 새로 읽어온다.
    // 읽기 전용 조회라 teleop처럼 안전 문제(hideEvent와 비대칭)는 없다.
    refreshDbMarkers();
    QWidget::showEvent(event);
}

void RemoteControlPage::handleTeleopVelocityChanged(double linearX, double angularZ)
{
    if (m_navigationService != nullptr) {
        m_navigationService->driveManual(linearX, angularZ);
    }

    if (m_cmdVelLinearValueLabel != nullptr) {
        m_cmdVelLinearValueLabel->setText(QStringLiteral("%1 m/s").arg(linearX, 0, 'f', 2));
    }
    if (m_cmdVelAngularValueLabel != nullptr) {
        m_cmdVelAngularValueLabel->setText(QStringLiteral("%1 rad/s").arg(angularZ, 0, 'f', 2));
    }
}

void RemoteControlPage::handleOdomVelocityUpdated(double linearMps, double angularRadps)
{
    // teleop이 조작 중이면 그쪽이 명령한 값을 즉시 그대로 보여주는 기존
    // 경로가 더 정확하고 실기 검증도 끝나 있으므로, 실측 /odom 값으로
    // 덮어쓰지 않는다 — 여기서는 웨이포인트 자동주행 중이거나 완전히
    // 대기 중일 때만 실제 속도를 반영한다.
    if (m_teleopActive) {
        return;
    }
    // LiveControlPage와 동일한 alpha=0.25 지수이동평균 — 특히 angular.z가
    // raw로 보면 부호가 계속 뒤집힐 정도로 흔들려서 그대로 표시하면 안 된다.
    m_lastOdomLinearVel = m_lastOdomLinearVel * 0.75 + linearMps * 0.25;
    m_lastOdomAngularVel = m_lastOdomAngularVel * 0.75 + angularRadps * 0.25;
    if (m_cmdVelLinearValueLabel != nullptr) {
        m_cmdVelLinearValueLabel->setText(
            QStringLiteral("%1 m/s").arg(snapNearZero(m_lastOdomLinearVel, 0.01), 0, 'f', 2));
    }
    if (m_cmdVelAngularValueLabel != nullptr) {
        m_cmdVelAngularValueLabel->setText(
            QStringLiteral("%1 rad/s").arg(snapNearZero(m_lastOdomAngularVel, 0.01), 0, 'f', 2));
    }
    if (m_cmdVelTitleLabel != nullptr) {
        m_cmdVelTitleLabel->setText(m_navigationInProgress ? QStringLiteral("자동주행 중 · /odom")
                                                             : QStringLiteral("대기 · /cmd_vel"));
    }
}

void RemoteControlPage::handleTeleopActiveChanged(bool active)
{
    m_teleopActive = active;
    refreshRobotMarkerState();
    if (m_manualBannerTitleLabel != nullptr) {
        m_manualBannerTitleLabel->setText(active ? QStringLiteral("🎮  수동 제어 활성") : QStringLiteral("수동 제어 대기"));
        m_manualBannerTitleLabel->setStyleSheet(
            QStringLiteral("color: %1; font-size: 12px; font-weight: 600;")
                .arg(active ? DashboardPalette::Red.name() : DashboardPalette::TextTertiary.name()));
    }
    if (m_cmdVelTitleLabel != nullptr) {
        m_cmdVelTitleLabel->setText(active ? QStringLiteral("발행 중 · /cmd_vel") : QStringLiteral("대기 · /cmd_vel"));
    }
    if (m_manualBannerCaptionLabel != nullptr) {
        // 여러 PC 동시 조작을 막는 제어권 락은 아직 없다(추후 별도 기능) —
        // 지금은 실제로 이 PC에서 조작 중일 때만, 하드코딩이 아니라 로그인한
        // 계정을 그대로 보여준다.
        m_manualBannerCaptionLabel->setText(
            active ? QStringLiteral("방향 버튼/WASD로 조작 · 제어권 qt / %1").arg(m_session.username)
                   : QStringLiteral("방향 버튼/WASD로 조작"));
    }
}

void RemoteControlPage::handleTeleopSpeedPresetChanged(TeleopController::SpeedPreset preset)
{
    Q_UNUSED(preset);
    updateSpeedPresetButtons();
}

void RemoteControlPage::handleNavSessionStarted(const QVector<NavPose> &goals, const QString &label)
{
    Q_UNUSED(goals);
    Q_UNUSED(label);
    m_navigationInProgress = true;
    refreshRobotMarkerState();
    // 자동주행이 시작됐는데 방향 버튼/키가 눌려있던 상태로 남아있으면
    // teleop과 Nav2가 동시에 /cmd_vel을 발행해 충돌한다 — 즉시 정지.
    m_teleop->stop();
    if (m_directionPad != nullptr) {
        m_directionPad->setEnabled(false);
    }
    if (m_manualBannerTitleLabel != nullptr) {
        m_manualBannerTitleLabel->setText(QStringLiteral("🧭  웨이포인트 이동 중"));
        m_manualBannerTitleLabel->setStyleSheet(
            QStringLiteral("color: %1; font-size: 12px; font-weight: 600;").arg(DashboardPalette::Amber.name()));
    }
    if (m_manualBannerCaptionLabel != nullptr) {
        m_manualBannerCaptionLabel->setText(QStringLiteral("이동이 끝날 때까지 수동 제어를 할 수 없습니다"));
    }
}

void RemoteControlPage::handleNavSessionFinished(bool success, bool cancelled)
{
    Q_UNUSED(success);
    Q_UNUSED(cancelled);
    m_navigationInProgress = false;
    if (m_directionPad != nullptr) {
        m_directionPad->setEnabled(true);
    }
    // 배너 문구/색을 "수동 제어 대기" 상태로 되돌린다 — teleop은 이동 중
    // stop()된 채로 남아있으므로 active=false가 맞다.
    handleTeleopActiveChanged(false);
}

void RemoteControlPage::handleFrontDistanceChanged(int centimeters)
{
    if (m_frontObstacleValueLabel == nullptr) {
        return;
    }
    m_frontObstacleValueLabel->setText(QStringLiteral("%1cm").arg(centimeters));
    setLabelColor(m_frontObstacleValueLabel, centimeters < 20 ? DashboardPalette::Red : DashboardPalette::TextPrimary);
}

void RemoteControlPage::handleRearDistanceChanged(int centimeters)
{
    if (m_rearObstacleValueLabel == nullptr) {
        return;
    }
    m_rearObstacleValueLabel->setText(QStringLiteral("%1cm").arg(centimeters));
    setLabelColor(m_rearObstacleValueLabel, centimeters < 20 ? DashboardPalette::Red : DashboardPalette::TextPrimary);
}

void RemoteControlPage::handlePayloadLoadedChanged(bool loaded)
{
    if (m_payloadValueLabel == nullptr) {
        return;
    }
    m_payloadValueLabel->setText(loaded ? QStringLiteral("적재됨") : QStringLiteral("미적재"));
    setLabelColor(m_payloadValueLabel, loaded ? DashboardPalette::Green : DashboardPalette::TextMuted);
}

void RemoteControlPage::updateSpeedPresetButtons()
{
    if (m_speedFineButton == nullptr || m_speedNormalButton == nullptr || m_speedFastButton == nullptr) {
        return;
    }

    const TeleopController::SpeedPreset preset = m_teleop->speedPreset();
    auto applyStyle = [](QPushButton *button, bool selected) {
        button->setObjectName(selected ? QStringLiteral("cyanChipButton") : QStringLiteral("ghostChipButton"));
        button->style()->unpolish(button);
        button->style()->polish(button);
    };
    applyStyle(m_speedFineButton, preset == TeleopController::SpeedPreset::Fine);
    applyStyle(m_speedNormalButton, preset == TeleopController::SpeedPreset::Normal);
    applyStyle(m_speedFastButton, preset == TeleopController::SpeedPreset::Fast);

    if (m_linearSpeedValueLabel != nullptr) {
        m_linearSpeedValueLabel->setText(QStringLiteral("%1").arg(TeleopController::linearSpeedForPreset(preset), 0, 'f', 2));
    }
    if (m_angularSpeedValueLabel != nullptr) {
        m_angularSpeedValueLabel->setText(QStringLiteral("%1").arg(TeleopController::angularSpeedForPreset(preset), 0, 'f', 2));
    }
}

void RemoteControlPage::handleAmclPoseUpdated(double x, double y, double yawRad)
{
    if (!m_hasAmclPose) {
        // 초기 위치를 처음 받은 순간 — 안내 배너를 "일치 확인"으로 바꾸고,
        // 이제부터 로봇 마커를 그릴 수 있는 상태가 된다.
        m_hasAmclPose = true;
        updateInitialPoseBanner();
    }

    if (!m_mapIsMain || !m_currentMapMeta.valid) {
        return;
    }

    m_mapView->setRobotVisible(true);
    m_mapView->setRobotPose(worldToNormalized(x, y), qRadiansToDegrees(yawRad));
    refreshRobotMarkerState();
}

void RemoteControlPage::refreshRobotMarkerState()
{
    if (m_mapView == nullptr) {
        return;
    }
    const bool driving = m_teleopActive || m_navigationInProgress;
    m_mapView->setRobotState(driving ? RobotMarkerState::Moving : RobotMarkerState::Ready);
}

void RemoteControlPage::applyCameraFrame(const QImage &frame)
{
    if (m_cameraFrameLabel == nullptr) {
        return;
    }

    m_lastCameraFrame = frame;
    m_cameraFrameLabel->setText(QString());
    m_cameraFrameLabel->setPixmap(
        QPixmap::fromImage(frame).scaled(m_cameraFrameLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void RemoteControlPage::applyCameraConnectionState(bool connected)
{
    if (m_cameraFrameLabel == nullptr || connected) {
        return;
    }

    m_lastCameraFrame = QImage();
    m_cameraFrameLabel->setPixmap(QPixmap());
    m_cameraFrameLabel->setText(QStringLiteral("🎥  카메라 연결 대기"));
}

void RemoteControlPage::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);

    if (!m_lastCameraFrame.isNull()) {
        applyCameraFrame(m_lastCameraFrame);
    }
}
