#include "waypointmanagementpage.h"

#include "../services/missioncontroller.h"
#include "../services/waypointrepository.h"
#include "../services/waypointfile.h"
#include "../theme/dashboardpalette.h"
#include "../widgets/badgelabel.h"

#include <QAbstractItemModel>
#include <QAbstractItemView>
#include <QComboBox>
#include <QCoreApplication>
#include <QDate>
#include <QDebug>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPoint>
#include <QPushButton>
#include <QSet>
#include <QSlider>
#include <QStackedWidget>
#include <QStyle>
#include <QThread>
#include <QVBoxLayout>
#include <QtMath>

#include <algorithm>

#include "waypointmanagementpage_helpers.h"

WaypointManagementPage::WaypointManagementPage(RobotService &robotService, OperationsOverviewService &overviewService, QWidget *parent)
    : QWidget(parent)
    , m_robotService(robotService)
    , m_overviewService(overviewService)
    , m_creatingNew(false)
    , m_settingInitialPose(false)
    , m_waypointTabButton(nullptr)
    , m_trajectoryTabButton(nullptr)
    , m_tabStack(nullptr)
    , m_mapStatusLabel(nullptr)
    , m_navStatusLabel(nullptr)
    , m_setInitialPoseButton(nullptr)
    , m_speedButton(nullptr)
    , m_waypointMap(nullptr)
    , m_waypointList(nullptr)
    , m_waypointListCountLabel(nullptr)
    , m_editPanelTitle(nullptr)
    , m_nameEdit(nullptr)
    , m_xEdit(nullptr)
    , m_yEdit(nullptr)
    , m_thetaEdit(nullptr)
    , m_kindCombo(nullptr)
    , m_saveWaypointButton(nullptr)
    , m_deleteWaypointButton(nullptr)
    , m_navigateButton(nullptr)
    , m_cancelNavButton(nullptr)
    , m_waypointStatusLabel(nullptr)
    , m_routeMap(nullptr)
    , m_routeAddCombo(nullptr)
    , m_routeOrderList(nullptr)
    , m_routeOrderCountLabel(nullptr)
    , m_routeNameEdit(nullptr)
    , m_savedRoutesBar(nullptr)
    , m_navigateRouteButton(nullptr)
    , m_trajectoryStatusLabel(nullptr)
{
    auto *outerLayout = new QVBoxLayout(this);
    outerLayout->setContentsMargins(0, 0, 0, 0);
    auto *canvas = panelFrame(QStringLiteral("pageCanvas"));
    outerLayout->addWidget(canvas);

    auto *layout = new QVBoxLayout(canvas);
    layout->setContentsMargins(12, 12, 12, 12);
    layout->setSpacing(10);

    layout->addWidget(buildHeader());
    layout->addWidget(buildWarningBanner());

    m_tabStack = new QStackedWidget(canvas);
    m_tabStack->addWidget(buildWaypointTab());
    m_tabStack->addWidget(buildTrajectoryTab());
    layout->addWidget(m_tabStack, 1);

    autoLoadDefaultMap();
    refreshWaypointList();
    refreshMapMarkers();
    refreshRouteCombo();
    renumberRouteOrderItems();
    refreshSavedRoutesBar();
    selectTab(0);
    updateNavButtonsEnabled();
    handleNewWaypointRequested();
}

void WaypointManagementPage::setNavigationService(NavigationService *service)
{
    m_navigationService = service;
    if (m_navigationService == nullptr) {
        handleNav2ConnectionChanged(false);
        updateNavButtonsEnabled();
        return;
    }

    connect(m_navigationService, &NavigationService::nav2ConnectionChanged, this, &WaypointManagementPage::handleNav2ConnectionChanged);
    connect(m_navigationService, &NavigationService::statusChanged, this, &WaypointManagementPage::handleNavStatusChanged);
    connect(m_navigationService, &NavigationService::feedbackDistanceRemaining, this, &WaypointManagementPage::handleNavFeedback);
    connect(m_navigationService, &NavigationService::waypointProgress, this, &WaypointManagementPage::handleNavWaypointProgress);
    connect(m_navigationService, &NavigationService::amclPoseUpdated, this, &WaypointManagementPage::handleAmclPoseUpdated);
    connect(m_navigationService, &NavigationService::scanPointsUpdated, this, &WaypointManagementPage::handleScanPointsUpdated);
    connect(m_navigationService, &NavigationService::navSessionStarted, this, &WaypointManagementPage::handleNavSessionStarted);
    connect(m_navigationService, &NavigationService::navSessionFinished, this, &WaypointManagementPage::handleNavSessionFinished);
    connect(m_navigationService, &NavigationService::logEvent, this, [this](const QString &level, const QString &message) {
        if (level == QStringLiteral("ERROR") || level == QStringLiteral("WARN")) {
            m_navStatusLabel->setText(QStringLiteral("[%1] %2").arg(level, message));
        }
    });

    // Nav2가 이 페이지보다 먼저 연결된 상태였다면(예: 다른 사람이 이미 Nav2를
    // 띄워둔 뒤 이 GUI를 실행) nav2ConnectionChanged가 우리가 connect하기 전에
    // 이미 지나갔을 수 있다 — 현재 상태를 한 번 직접 조회해 동기화한다.
    handleNav2ConnectionChanged(m_navigationService->isNav2Connected());
    // amcl_pose는 TRANSIENT_LOCAL이라 앱 시작 직후(로그인 전) 이미 한 번
    // 와버렸을 수 있어 amclPoseUpdated도 같은 문제가 있다 — 마지막 값을 조회해 동기화.
    if (double x, y, yaw; m_navigationService->lastKnownAmclPose(x, y, yaw)) {
        handleAmclPoseUpdated(x, y, yaw);
    }
    updateNavButtonsEnabled();
}

void WaypointManagementPage::setWaypointRepository(WaypointRepository *repository)
{
    if (m_waypointRepository == repository) {
        return;
    }
    m_waypointRepository = repository;
    if (m_waypointRepository == nullptr) {
        return;
    }

    connect(m_waypointRepository, &WaypointRepository::repositoryChanged, this, &WaypointManagementPage::syncFromRepository);
    syncFromRepository();
}

void WaypointManagementPage::setMissionController(MissionController *controller)
{
    if (m_missionController == controller) {
        return;
    }
    m_missionController = controller;
    if (m_missionController != nullptr) {
        connect(m_missionController, &MissionController::observedButtonIdsChanged, this,
                &WaypointManagementPage::refreshCallZoneIdCombo);
    }
    refreshCallZoneIdCombo();
}

void WaypointManagementPage::updateCallZoneIdComboVisibility()
{
    if (m_callZoneIdLabel == nullptr || m_callZoneIdCombo == nullptr || m_kindCombo == nullptr) {
        return;
    }
    const auto kind = static_cast<WaypointMarkerKind>(m_kindCombo->currentData().toInt());
    const bool showCombo = (kind == WaypointMarkerKind::CallZone);
    m_callZoneIdLabel->setVisible(showCombo);
    m_callZoneIdCombo->setVisible(showCombo);
    if (showCombo) {
        refreshCallZoneIdCombo();
    }
}

void WaypointManagementPage::refreshCallZoneIdCombo()
{
    if (m_callZoneIdCombo == nullptr) {
        return;
    }
    // 지금 편집 중인 콜존의 code는 목록 재구성 후에도 계속 보여야 한다 —
    // 새로 감지된 버튼이 그 사이에 하나 더 들어와도 편집 화면이 흔들리면
    // 안 되므로, 재구성 기준값을 매번 다시 계산한다.
    QString mustKeep;
    if (!m_creatingNew && m_waypoints.contains(m_selectedWaypointId)) {
        const WaypointDef &record = m_waypoints.value(m_selectedWaypointId);
        if (record.kind == WaypointMarkerKind::CallZone) {
            mustKeep = record.name;
        }
    }
    const QString previousSelection = m_callZoneIdCombo->currentIndex() > 0 ? m_callZoneIdCombo->currentText() : QString();

    m_callZoneIdCombo->clear();
    m_callZoneIdCombo->addItem(QStringLiteral("선택 안 함 (자동 채번)"));
    if (m_missionController != nullptr) {
        m_callZoneIdCombo->addItems(m_missionController->observedButtonIds());
    }
    if (!mustKeep.isEmpty() && m_callZoneIdCombo->findText(mustKeep) < 0) {
        m_callZoneIdCombo->addItem(mustKeep);
    }

    const QString toSelect = !mustKeep.isEmpty() ? mustKeep : previousSelection;
    if (!toSelect.isEmpty()) {
        const int idx = m_callZoneIdCombo->findText(toSelect);
        if (idx >= 0) {
            m_callZoneIdCombo->setCurrentIndex(idx);
        }
    }
}

void WaypointManagementPage::syncFromRepository()
{
    if (m_waypointRepository == nullptr) {
        return;
    }

    m_frameId = m_waypointRepository->frameId();
    m_waypoints = m_waypointRepository->waypoints();
    m_trajectories = m_waypointRepository->trajectories();
    m_waypointFilePath = m_waypointRepository->filePath();

    refreshWaypointList();
    refreshMapMarkers();
    refreshRouteCombo();
    renumberRouteOrderItems();
    refreshSavedRoutesBar();

    if (!m_selectedWaypointId.isEmpty() && m_waypoints.contains(m_selectedWaypointId)) {
        selectWaypoint(m_selectedWaypointId);
    } else if (!m_waypoints.isEmpty()) {
        selectWaypoint(m_waypoints.firstKey());
    } else {
        handleNewWaypointRequested();
    }
}

void WaypointManagementPage::pushToRepository()
{
    if (m_waypointRepository == nullptr) {
        return;
    }
    m_waypointRepository->replaceAll(m_frameId, m_waypoints, m_trajectories, m_waypointFilePath);
}

void WaypointManagementPage::showEvent(QShowEvent *event)
{
    // 콜존/홈/경유점/경로 전부 DB가 원본이다 — 페이지가 다시 보일 때마다
    // 최신 값으로 다시 채운다(진행 중인 편집 폼 입력은 건드리지 않음).
    mergeCallButtonsInto(m_waypoints);
    mergeHomeWaypointInto(m_waypoints);
    mergeWaypointEntriesInto(m_waypoints);
    reloadTrajectoriesFromDb();
    refreshWaypointList();
    refreshMapMarkers();
    refreshRouteCombo();
    renumberRouteOrderItems();
    refreshSavedRoutesBar();
    QWidget::showEvent(event);
}
