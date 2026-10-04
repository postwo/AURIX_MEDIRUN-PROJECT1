#include "slammappingpage_internal.h"

SlamMappingPage::SlamMappingPage(QWidget *parent)
    : QWidget(parent)
    , m_mapView(nullptr)
    , m_statusBannerTitleLabel(nullptr)
    , m_statusBannerDetailLabel(nullptr)
    , m_mappingStatusValueLabel(nullptr)
    , m_saveFormatValueLabel(nullptr)
    , m_statusMessageLabel(nullptr)
    , m_liveStatusMessageLabel(nullptr)
    , m_logView(nullptr)
    , m_startButton(nullptr)
    , m_saveButton(nullptr)
    , m_resetButton(nullptr)
    , m_slamProcess(new QProcess(this))
    , m_saveProcess(new QProcess(this))
    , m_manualDriveTimer(new QTimer(this))
    , m_mapRefreshTimer(new QTimer(this))
    , m_mapTimeoutTimer(new QTimer(this))
    , m_mapBridge(std::make_unique<SlamMapBridge>())
    , m_mappingActive(false)
    , m_manualLinearX(0.0)
    , m_manualAngularZ(0.0)
{
    setFocusPolicy(Qt::StrongFocus);
    auto *outerLayout = new QVBoxLayout(this);
    outerLayout->setContentsMargins(0, 0, 0, 0);
    auto *canvas = panelFrame(QStringLiteral("pageCanvas"));
    outerLayout->addWidget(canvas);

    auto *layout = new QVBoxLayout(canvas);
    layout->setContentsMargins(12, 12, 12, 12);
    layout->setSpacing(10);

    layout->addWidget(buildStatusBanner());

    auto *bodyRow = new QHBoxLayout;
    bodyRow->setSpacing(10);

    auto *leftCol = new QVBoxLayout;
    leftCol->setSpacing(8);

    m_mapView = new MapViewWidget(canvas);
    m_mapView->setMinimumHeight(250);
    m_mapView->setMode(MapViewMode::Live);
    m_mapView->setOverlayChipsVisible(false);
    m_mapView->setCaption(QStringLiteral("매핑 시작 전 · Cartographer 대기 화면"));
    m_mapView->setRobotPose(QPointF(0.5, 0.5), 0.0);
    m_mapView->setRobotState(RobotMarkerState::Ready);
    m_mapView->setPlannedPath({});
    leftCol->addWidget(m_mapView, 1);
    leftCol->addWidget(buildActionRow());

    // 카메라 화면을 지도 캔버스 위에 우상단 모서리로 띄운다(원격제어/실시간
    // 관제 페이지처럼 별도 패널을 두면 지도가 좁아지는데, 매핑 중에는 지도
    // 캔버스가 화면의 대부분을 차지해야 해서 오버레이 방식을 택했다).
    // MapViewWidget 자신도 캡션/칩 라벨을 이 방식(자식 위젯 + move())으로
    // 띄우고 있어 같은 패턴을 그대로 따른다(mapviewwidget.cpp 참고).
    buildCameraOverlay();
    bodyRow->addLayout(leftCol, 7);

    auto *rightCol = new QVBoxLayout;
    rightCol->setContentsMargins(0, 0, 0, 0);
    rightCol->setSpacing(8);
    rightCol->addWidget(buildMappingStatusPanel());
    rightCol->addWidget(buildDirectionPad());
    rightCol->addWidget(buildSpeedLimitPanel());
    rightCol->addWidget(buildLogPanel(), 1);

    auto *rightColWidget = new QWidget(canvas);
    rightColWidget->setFixedWidth(200);
    rightColWidget->setLayout(rightCol);
    bodyRow->addWidget(rightColWidget);

    layout->addLayout(bodyRow, 1);

    connect(m_slamProcess, &QProcess::stateChanged, this, &SlamMappingPage::handleSlamProcessStateChanged);
    connect(m_slamProcess, &QProcess::errorOccurred, this, &SlamMappingPage::handleSlamProcessErrorOccurred);
    connect(m_slamProcess, &QProcess::readyReadStandardOutput, this, &SlamMappingPage::handleSlamProcessStdout);
    connect(m_slamProcess, &QProcess::readyReadStandardError, this, &SlamMappingPage::handleSlamProcessStderr);
    connect(m_slamProcess,
            qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
            this,
            &SlamMappingPage::handleSlamProcessFinished);
    connect(m_slamProcess, &QProcess::started, this, [this]() {
        qInfo() << "[SLAM] Cartographer process started";
        appendUiLog(QStringLiteral("[SLAM] Cartographer 프로세스 시작"));
        setMappingUiState(MappingUiState::WaitingForMap, QStringLiteral("/map 토픽 대기 중"));
        setLiveStatusMessage(QStringLiteral("Cartographer 프로세스 시작됨 · /map을 기다리는 중"));
        m_mapTimeoutTimer->start(kMapTimeoutMs);
        updateSlamUiState();
    });
    connect(m_saveProcess,
            qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
            this,
            &SlamMappingPage::handleSaveProcessFinished);
    connect(m_saveProcess, &QProcess::errorOccurred, this, &SlamMappingPage::handleSaveProcessErrorOccurred);
    connect(m_saveProcess, &QProcess::readyReadStandardOutput, this, &SlamMappingPage::handleSaveProcessStdout);
    connect(m_saveProcess, &QProcess::readyReadStandardError, this, &SlamMappingPage::handleSaveProcessStderr);
    // "대시보드 종료" 버튼은 MainWindow::close()만 부르고 WA_DeleteOnClose가
    // 없어서 위젯이 삭제되지 않는다 — 즉 이 페이지의 소멸자가 아예 안 불릴 수
    // 있다(cartographer가 계속 살아남던 버그의 원인). 앱 종료 시점엔 항상
    // aboutToQuit이 뜨므로 여기서 직접 정리한다. stopMappingProcess()가 아니라
    // 전용 핸들러를 쓰는 이유: nav2 RESUME까지 반드시 끝내야 하는데(소멸자가 안
    // 불릴 수 있으니 이번이 마지막 기회), 이 시점 이후 spin 스레드가 곧 죽으므로
    // 비동기로 던지면 응답이 오기 전에 유실될 수 있어 blocking으로 기다린다.
    connect(qApp, &QCoreApplication::aboutToQuit, this, &SlamMappingPage::handleAboutToQuit);
    m_manualDriveTimer->setInterval(100);
    connect(m_manualDriveTimer, &QTimer::timeout, this, &SlamMappingPage::handleManualDriveTick);
    m_mapRefreshTimer->setInterval(kMapRefreshIntervalMs);
    connect(m_mapRefreshTimer, &QTimer::timeout, this, &SlamMappingPage::handleMapRefreshTick);
    m_mapTimeoutTimer->setSingleShot(true);
    connect(m_mapTimeoutTimer, &QTimer::timeout, this, &SlamMappingPage::handleMapTimeout);

    attachMapBridgeCallbacks();
    m_mapBridge->start();
    refreshSensorStatus();
    updateSlamUiState();
    appendUiLog(QStringLiteral("[SLAM] SLAM 페이지 초기화 완료"));
}

SlamMappingPage::~SlamMappingPage()
{
    if (m_saveProcess->state() != QProcess::NotRunning) {
        m_saveProcess->kill();
        m_saveProcess->waitForFinished(1000);
    }
    m_mapTimeoutTimer->stop();
    m_mapRefreshTimer->stop();
    terminateProcessTree(m_slamProcess, 2000, 1000);
    // blocking으로 기다린다: 이 다음 줄에서 m_mapBridge->stop()이 spin 스레드를
    // 종료시키므로, 비동기로 던지기만 하면 응답이 오기 전에 요청이 유실된다.
    const auto failedManagers = m_mapBridge->resumeNav2IfPausedBlocking();
    if (!failedManagers.isEmpty()) {
        qWarning() << "[SLAM] Nav2 재개 실패, PAUSE 상태로 남아있을 수 있음:" << failedManagers;
    }
    m_mapBridge->stop();
}

// aboutToQuit 전용 정리 경로. 이 페이지의 소멸자는 "대시보드 종료" 버튼으로는
// 안 불릴 수 있어(위 커넥트 지점 주석 참고) 앱 종료 시 nav2를 재개할 마지막
// 기회일 수 있다 — blocking으로 응답을 기다려 확실히 반영한다.
void SlamMappingPage::handleAboutToQuit()
{
    stopMappingProcess();
    if (m_mapBridge != nullptr) {
        const auto failedManagers = m_mapBridge->resumeNav2IfPausedBlocking();
        if (!failedManagers.isEmpty()) {
            qWarning() << "[SLAM] Nav2 재개 실패, PAUSE 상태로 남아있을 수 있음:" << failedManagers;
        }
    }
}

void SlamMappingPage::showEvent(QShowEvent *event)
{
    QWidget::showEvent(event);
    setFocus(Qt::OtherFocusReason);
    qInfo() << "[TELEOP] keyboard focus acquired";
    appendUiLog(QStringLiteral("[TELEOP] keyboard focus acquired"));
    refreshSensorStatus();
}

void SlamMappingPage::hideEvent(QHideEvent *event)
{
    QWidget::hideEvent(event);
    stopManualDrive();
}

void SlamMappingPage::focusInEvent(QFocusEvent *event)
{
    QWidget::focusInEvent(event);
    qInfo() << "[TELEOP] keyboard focus acquired";
    appendUiLog(QStringLiteral("[TELEOP] keyboard focus acquired"));
}

void SlamMappingPage::focusOutEvent(QFocusEvent *event)
{
    QWidget::focusOutEvent(event);
    stopManualDrive();
    qWarning() << "[TELEOP] keyboard focus lost";
    appendUiLog(QStringLiteral("[TELEOP] keyboard focus lost"));
}

bool SlamMappingPage::handleManualKeyPress(int key)
{
    return consumeManualKey(key);
}

bool SlamMappingPage::handleManualKeyRelease(int key)
{
    return releaseManualKey(key);
}

bool SlamMappingPage::eventFilter(QObject *watched, QEvent *event)
{
    // m_mapView가 리사이즈될 때마다(창 크기 변경, 레이아웃 재계산 등) 카메라
    // 오버레이를 우상단 모서리에 다시 고정한다 — MapViewWidget 자신도 캡션/칩
    // 라벨을 같은 방식으로 재배치한다(mapviewwidget.cpp의 resizeEvent 참고).
    if (watched == m_mapView && event->type() == QEvent::Resize) {
        positionCameraOverlay();
    }
    return QWidget::eventFilter(watched, event);
}

void SlamMappingPage::setCameraFeedProvider(CameraFeedProvider *provider)
{
    if (m_cameraFeedProvider == provider) {
        return;
    }
    m_cameraFeedProvider = provider;
    if (m_cameraFeedProvider != nullptr) {
        connect(m_cameraFeedProvider, &CameraFeedProvider::frameReady, this, &SlamMappingPage::handleCameraFrameReady);
    }
}

void SlamMappingPage::handleCameraFrameReady(const QImage &frame)
{
    if (m_cameraViewLabel == nullptr || frame.isNull()) {
        return;
    }

    if (frame.size() != m_lastCameraFrameSize) {
        m_lastCameraFrameSize = frame.size();
        if (QWidget *overlay = m_cameraViewLabel->parentWidget()) {
            // 오버레이 폭(220)은 고정해두고, 실제 수신한 프레임의 가로세로
            // 비율에 맞춰 높이만 다시 계산한다 — camera.launch.py에 적어둔
            // image_size(1280x720)를 v4l2_camera가 그대로 지원 안 하면 다른
            // 비율로 뜰 수 있어서, KeepAspectRatio가 남기는 여백(좌우 또는
            // 상하 테두리)이 실제 수신 프레임 기준으로는 생기지 않게 한다.
            const int width = overlay->width();
            const int height = std::max(1, static_cast<int>(std::lround(width * static_cast<double>(frame.height()) / frame.width())));
            overlay->setFixedSize(width, height);
            positionCameraOverlay();
        }
    }

    const QPixmap pixmap = QPixmap::fromImage(frame).scaled(
        m_cameraViewLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    m_cameraViewLabel->setPixmap(pixmap);
}
