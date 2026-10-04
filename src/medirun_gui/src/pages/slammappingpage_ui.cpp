#include "slammappingpage_internal.h"

QWidget *SlamMappingPage::buildStatusBanner()
{
    auto *panel = panelFrame(QStringLiteral("anomalyCard"));
    auto *layout = new QHBoxLayout(panel);
    layout->setContentsMargins(12, 9, 12, 9);
    layout->setSpacing(8);

    m_statusBannerTitleLabel = makeLabel(QStringLiteral("🗺  Cartographer 대기 중"));
    m_statusBannerTitleLabel->setWordWrap(false);
    m_statusBannerTitleLabel->setStyleSheet(
        QStringLiteral("color: %1; font-size: 12px; font-weight: 600;").arg(DashboardPalette::Cyan.name()));
    layout->addWidget(m_statusBannerTitleLabel);
    m_statusBannerDetailLabel = makeLabel(QStringLiteral("TurtleBot3 bringup 후 매핑 시작 버튼으로 Cartographer를 실행합니다."),
                                          QStringLiteral("metricCaption"));
    m_statusBannerDetailLabel->setWordWrap(false);
    layout->addWidget(m_statusBannerDetailLabel);
    layout->addStretch();
    layout->addWidget(makeBadgeLabel(QStringLiteral("Cartographer 활성"), QStringLiteral("cyan"), panel));
    return panel;
}

QWidget *SlamMappingPage::buildActionRow()
{
    auto *row = new QWidget;
    auto *layout = new QHBoxLayout(row);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(7);

    m_startButton = new QPushButton(QStringLiteral("매핑 재생"));
    connect(m_startButton, &QPushButton::clicked, this, [this]() {
        if (m_slamProcess->state() == QProcess::NotRunning) {
            handleStartMapping();
            return;
        }
        handleResetMapping();
    });

    m_saveButton = new QPushButton(QStringLiteral("지도 저장"));
    m_saveButton->setObjectName(QStringLiteral("primaryCtaButton"));
    connect(m_saveButton, &QPushButton::clicked, this, &SlamMappingPage::handleSaveMap);

    for (QPushButton *button : {m_startButton, m_saveButton}) {
        layout->addWidget(button);
    }
    return row;
}

QWidget *SlamMappingPage::buildCameraOverlay()
{
    // LiveControlPage::buildCameraPanel()과 동일한 패널/라벨 조합(카메라 없을 때
    // 안내 문구, 프레임 도착 시 KeepAspectRatio로 스케일)을 재사용한다 — 녹화
    // 타임라인 등 이 페이지에 필요 없는 부분만 뺐다.
    auto *panel = panelFrame(QStringLiteral("contentPanel"));
    panel->setParent(m_mapView);
    // 폭은 운영자 요청으로 300으로 키웠다(처음엔 220이었음). 높이는 임시로
    // 카메라.launch.py의 image_size(1280x720, 16:9) 기준으로 잡아두지만,
    // v4l2_camera가 실제로 그 해상도를 지원 안 해 다른 비율로 뜰 수 있어
    // (실측: 좌우로 여백이 남았었다) handleCameraFrameReady()에서 첫 프레임이
    // 도착하면 실제 프레임 비율에 맞게 다시 계산해 덮어쓴다.
    panel->setFixedSize(300, 169);
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(0, 0, 0, 0);

    m_cameraViewLabel = new QLabel(QStringLiteral("🎥\n카메라 연결 대기 중"), panel);
    m_cameraViewLabel->setAlignment(Qt::AlignCenter);
    m_cameraViewLabel->setStyleSheet(QStringLiteral("color: %1;").arg(DashboardPalette::TextMuted.name()));
    m_cameraViewLabel->setMinimumSize(1, 1);
    layout->addWidget(m_cameraViewLabel, 1);

    m_mapView->installEventFilter(this);
    positionCameraOverlay();
    panel->show();
    return panel;
}

void SlamMappingPage::positionCameraOverlay()
{
    if (m_cameraViewLabel == nullptr || m_mapView == nullptr) {
        return;
    }
    auto *overlay = m_cameraViewLabel->parentWidget();
    if (overlay == nullptr) {
        return;
    }
    constexpr int kMargin = 10;
    overlay->move(std::max(0, m_mapView->width() - overlay->width() - kMargin), kMargin);
    overlay->raise();
}

QWidget *SlamMappingPage::buildMappingStatusPanel()
{
    auto *panel = panelFrame(QStringLiteral("contentPanel"));
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(12, 11, 12, 11);
    layout->setSpacing(6);
    layout->addWidget(makeLabel(QStringLiteral("매핑 상태"), QStringLiteral("metricCaption")));

    statRow(layout, QStringLiteral("지도 커버리지"), QStringLiteral("실행 중 확인"), DashboardPalette::Cyan);
    layout->addWidget(makeProportionalBar(62, DashboardPalette::Cyan, panel));
    layout->addSpacing(4);
    m_mappingStatusValueLabel = makeLabel(QStringLiteral("비활성"));
    m_mappingStatusValueLabel->setStyleSheet(QStringLiteral("color: %1; font-size: 11px;").arg(DashboardPalette::Amber.name()));
    auto *mappingStatusRow = new QHBoxLayout;
    mappingStatusRow->addWidget(makeLabel(QStringLiteral("Cartographer 상태"), QStringLiteral("metricCaption")));
    mappingStatusRow->addStretch();
    mappingStatusRow->addWidget(m_mappingStatusValueLabel);
    layout->addLayout(mappingStatusRow);

    statRow(layout, QStringLiteral("맵 저장 위치"), QStringLiteral("maps/"), DashboardPalette::Green);
    m_saveFormatValueLabel = makeLabel(QStringLiteral("pgm/yaml"));
    m_saveFormatValueLabel->setStyleSheet(QStringLiteral("color: %1; font-size: 11px;").arg(DashboardPalette::Green.name()));
    auto *saveFormatRow = new QHBoxLayout;
    saveFormatRow->addWidget(makeLabel(QStringLiteral("저장 형식"), QStringLiteral("metricCaption")));
    saveFormatRow->addStretch();
    saveFormatRow->addWidget(m_saveFormatValueLabel);
    layout->addLayout(saveFormatRow);

    m_statusMessageLabel = makeLabel(QStringLiteral("저장된 지도 없음"), QStringLiteral("pageSubtitle"));
    layout->addWidget(m_statusMessageLabel);
    m_liveStatusMessageLabel = makeLabel(QStringLiteral("실시간 상태 대기 중"), QStringLiteral("metricCaption"));
    layout->addWidget(m_liveStatusMessageLabel);
    return panel;
}

QWidget *SlamMappingPage::buildDirectionPad()
{
    auto *panel = panelFrame(QStringLiteral("contentPanel"));
    auto *layout = new QGridLayout(panel);
    layout->setContentsMargins(9, 10, 9, 10);
    layout->setSpacing(5);

    auto *forwardButton = directionButton(QStringLiteral("▲"));
    auto *leftButton = directionButton(QStringLiteral("↺"));
    auto *stopButton = directionButton(QStringLiteral("정지"), true);
    // dangerFilledButton은 전역 QPushButton의 padding: 0 12px를 물려받는데,
    // 38x38 고정 크기 방향패드 버튼 안에 "정지" 두 글자가 들어가면 잘린다.
    // 원격제어 페이지와 동일한 방향패드 전용 스타일로 여백/폰트를 다시 잡는다.
    stopButton->setObjectName(QStringLiteral("directionPadStopButton"));
    auto *rightButton = directionButton(QStringLiteral("↻"));
    auto *backwardButton = directionButton(QStringLiteral("▼"));

    for (QPushButton *button : {forwardButton, backwardButton, leftButton, rightButton, stopButton}) {
        button->setFocusPolicy(Qt::NoFocus);
    }

    connect(forwardButton, &QPushButton::pressed, this, [this]() {
        m_upPressed = true;
        m_downPressed = false;
        applyManualKeyState();
    });
    connect(forwardButton, &QPushButton::released, this, [this]() {
        m_upPressed = false;
        applyManualKeyState();
    });
    connect(backwardButton, &QPushButton::pressed, this, [this]() {
        m_downPressed = true;
        m_upPressed = false;
        applyManualKeyState();
    });
    connect(backwardButton, &QPushButton::released, this, [this]() {
        m_downPressed = false;
        applyManualKeyState();
    });
    connect(leftButton, &QPushButton::pressed, this, [this]() {
        m_leftPressed = true;
        m_rightPressed = false;
        applyManualKeyState();
    });
    connect(leftButton, &QPushButton::released, this, [this]() {
        m_leftPressed = false;
        applyManualKeyState();
    });
    connect(rightButton, &QPushButton::pressed, this, [this]() {
        m_rightPressed = true;
        m_leftPressed = false;
        applyManualKeyState();
    });
    connect(rightButton, &QPushButton::released, this, [this]() {
        m_rightPressed = false;
        applyManualKeyState();
    });
    connect(stopButton, &QPushButton::clicked, this, &SlamMappingPage::stopManualDrive);

    layout->addWidget(forwardButton, 0, 1);
    layout->addWidget(leftButton, 1, 0);
    layout->addWidget(stopButton, 1, 1);
    layout->addWidget(rightButton, 1, 2);
    layout->addWidget(backwardButton, 2, 1);
    return panel;
}

QWidget *SlamMappingPage::buildLogPanel()
{
    auto *panel = panelFrame(QStringLiteral("contentPanel"));
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(10, 8, 10, 8);
    layout->setSpacing(6);
    layout->addWidget(makeLabel(QStringLiteral("실행 로그"), QStringLiteral("metricCaption")));

    m_logView = new QPlainTextEdit(panel);
    m_logView->setReadOnly(true);
    m_logView->setObjectName(QStringLiteral("slamLogView"));
    m_logView->setMinimumHeight(140);
    m_logView->setMaximumBlockCount(200);
    layout->addWidget(m_logView);
    return panel;
}

QWidget *SlamMappingPage::buildSpeedLimitPanel()
{
    auto *panel = panelFrame(QStringLiteral("contentPanel"));
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(10, 8, 10, 8);
    layout->setSpacing(6);

    auto *topRow = new QHBoxLayout;
    topRow->addWidget(makeLabel(QStringLiteral("속도 제한"), QStringLiteral("metricCaption")));
    topRow->addStretch();
    auto *valueLabel = makeLabel(QStringLiteral("%1 m/s · %2 rad/s")
                                     .arg(kManualLinearSpeedMps, 0, 'f', 2)
                                     .arg(kManualAngularSpeedRadPerSec, 0, 'f', 2));
    valueLabel->setStyleSheet(QStringLiteral("color: %1; font-size: 10px;").arg(DashboardPalette::Cyan.name()));
    topRow->addWidget(valueLabel);
    layout->addLayout(topRow);
    layout->addWidget(makeProportionalBar(50, DashboardPalette::Cyan, panel));
    layout->addStretch();
    return panel;
}
