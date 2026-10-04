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

QWidget *LiveControlPage::buildMissionStepper()
{
    auto *panel = panelFrame(QStringLiteral("contentPanel"));
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(14, 12, 14, 12);
    layout->setSpacing(9);

    auto *infoRow = new QHBoxLayout;
    m_missionInfoLabel = makeLabel(QStringLiteral("대기 중"));
    m_missionInfoLabel->setStyleSheet(QStringLiteral("color: %1; font-size: 11px;").arg(DashboardPalette::Cyan.name()));
    infoRow->addWidget(m_missionInfoLabel);
    infoRow->addStretch();
    m_missionMetaLabel = makeLabel(QString(), QStringLiteral("metricCaption"));
    infoRow->addWidget(m_missionMetaLabel);
    layout->addLayout(infoRow);

    auto *stepRow = new QHBoxLayout;
    stepRow->setSpacing(0);
    for (int i = 0; i < kMissionSteps.size(); ++i) {
        auto *col = new QVBoxLayout;
        col->setSpacing(4);
        auto *circle = new QLabel;
        circle->setProperty("stepState", QStringLiteral("pending"));
        circle->setFixedSize(18, 18);
        circle->setAlignment(Qt::AlignCenter);
        m_stepCircles.push_back(circle);
        col->addWidget(circle, 0, Qt::AlignHCenter);
        auto *stepLabel = makeLabel(kMissionSteps.at(i).label, QStringLiteral("metricCaption"));
        stepLabel->setAlignment(Qt::AlignHCenter);
        m_stepLabels.push_back(stepLabel);
        col->addWidget(stepLabel);
        stepRow->addLayout(col, 10);

        if (i < kMissionSteps.size() - 1) {
            auto *connector = new QFrame(panel);
            connector->setFixedHeight(2);
            connector->setObjectName(QStringLiteral("stepConnectorPending"));
            m_stepConnectors.push_back(connector);
            stepRow->addWidget(connector, 6, Qt::AlignVCenter);
        }
    }
    layout->addLayout(stepRow);
    return panel;
}

QWidget *LiveControlPage::buildActionRow()
{
    auto *row = new QWidget;
    auto *layout = new QHBoxLayout(row);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(7);

    m_recordButton = new QPushButton(QStringLiteral("녹화 시작"));
    m_saveRecordingButton = new QPushButton(QStringLiteral("영상 저장"));
    m_saveRecordingButton->setEnabled(false);
    m_snapshotButton = new QPushButton(QStringLiteral("스냅샷"));
    m_cancelButton = new QPushButton(QStringLiteral("미션 취소"));
    m_homeButton = new QPushButton(QStringLiteral("홈 복귀"));

    for (QPushButton *button : {m_recordButton, m_saveRecordingButton, m_snapshotButton, m_cancelButton, m_homeButton}) {
        layout->addWidget(button);
    }

    connect(m_recordButton, &QPushButton::clicked, this, &LiveControlPage::handleRecordButtonClicked);
    connect(m_saveRecordingButton, &QPushButton::clicked, this, &LiveControlPage::handleSaveRecordingButtonClicked);
    connect(m_snapshotButton, &QPushButton::clicked, this, &LiveControlPage::handleSnapshotButtonClicked);
    connect(m_cancelButton, &QPushButton::clicked, this, &LiveControlPage::missionCancelRequested);
    connect(m_homeButton, &QPushButton::clicked, this, &LiveControlPage::returnHomeRequested);
    return row;
}

QWidget *LiveControlPage::buildRobotStatusPanel()
{
    auto *panel = panelFrame(QStringLiteral("contentPanel"));
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(12, 11, 12, 11);
    layout->setSpacing(6);
    layout->addWidget(makeLabel(QStringLiteral("로봇 상태"), QStringLiteral("metricCaption")));

    m_batteryValueLabel = statValueLabel(layout, QStringLiteral("배터리"));
    m_batteryBarHost = panel;
    m_batteryBar = makeProportionalBar(0, DashboardPalette::TextMuted, panel);
    layout->addWidget(m_batteryBar);
    layout->addSpacing(4);
    m_payloadValueLabel = statValueLabel(layout, QStringLiteral("AED 적재"));
    m_obstacleValueLabel = statValueLabel(layout, QStringLiteral("전방 장애물"));
    m_rearObstacleValueLabel = statValueLabel(layout, QStringLiteral("후방 장애물"));
    m_nav2ValueLabel = statValueLabel(layout, QStringLiteral("Nav2"));
    m_speedValueLabel = statValueLabel(layout, QStringLiteral("속도"));
    return panel;
}

QWidget *LiveControlPage::buildEventStreamPanel()
{
    auto *panel = panelFrame(QStringLiteral("contentPanel"));
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(12, 11, 12, 11);
    layout->setSpacing(6);
    layout->addWidget(makeLabel(QStringLiteral("이벤트 스트림"), QStringLiteral("metricCaption")));

    auto *streamContainer = new QWidget;
    m_eventStreamLayout = new QVBoxLayout(streamContainer);
    m_eventStreamLayout->setContentsMargins(0, 0, 0, 4);
    m_eventStreamLayout->setSpacing(6);

    // 미션 관제의 "오늘 처리 이력"과 동일한 패턴: 패널이 무한정 늘어나지
    // 않게 스크롤 영역에 담는다.
    auto *scrollArea = new QScrollArea(panel);
    scrollArea->setWidget(streamContainer);
    scrollArea->setWidgetResizable(true);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    layout->addWidget(scrollArea, 1);
    m_eventStreamLayout->addWidget(makeLabel(QStringLiteral("이벤트가 없습니다."), QStringLiteral("metricCaption")));

    return panel;
}

QWidget *LiveControlPage::buildWaitingCallBanner()
{
    auto *panel = panelFrame(QStringLiteral("callQueueCard"));
    auto *layout = new QHBoxLayout(panel);
    layout->setContentsMargins(11, 0, 11, 0);
    layout->setSpacing(7);
    m_waitingCallLabel = makeLabel(QStringLiteral("대기 호출 0건"));
    m_waitingCallLabel->setWordWrap(false);
    m_waitingCallLabel->setStyleSheet(QStringLiteral("color: %1;").arg(DashboardPalette::Cyan.name()));
    layout->addWidget(m_waitingCallLabel);
    layout->addStretch();
    auto *linkButton = new QPushButton(QStringLiteral("미션 관제 →"), panel);
    linkButton->setObjectName(QStringLiteral("linkChipButton"));
    linkButton->setCursor(Qt::PointingHandCursor);
    connect(linkButton, &QPushButton::clicked, this, &LiveControlPage::missionOpsRequested);
    layout->addWidget(linkButton);
    return panel;
}

QWidget *LiveControlPage::buildCameraPanel()
{
    auto *panel = panelFrame(QStringLiteral("contentPanel"));
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(0, 0, 0, 0);
    m_cameraViewLabel = new QLabel(QStringLiteral("🎥\n카메라 연결 대기 중"));
    m_cameraViewLabel->setAlignment(Qt::AlignCenter);
    m_cameraViewLabel->setStyleSheet(QStringLiteral("color: %1;").arg(DashboardPalette::TextMuted.name()));
    m_cameraViewLabel->setMinimumSize(1, 1);
    layout->addWidget(m_cameraViewLabel, 1);

    m_cameraTimelinePanel = new QWidget;
    auto *timelineLayout = new QVBoxLayout(m_cameraTimelinePanel);
    timelineLayout->setContentsMargins(9, 6, 9, 8);
    timelineLayout->setSpacing(6);
    timelineLayout->addWidget(makeProportionalBar(100, DashboardPalette::Red, m_cameraTimelinePanel));
    auto *controlsRow = new QHBoxLayout;
    controlsRow->setSpacing(9);
    controlsRow->addWidget(makeBadgeLabel(QStringLiteral("● REC"), QStringLiteral("red"), m_cameraTimelinePanel));
    m_recordingElapsedLabel = makeLabel(QStringLiteral("00:00"), QStringLiteral("metricCaption"));
    controlsRow->addWidget(m_recordingElapsedLabel);
    controlsRow->addStretch();
    m_pauseRecordingButton = new QPushButton(QStringLiteral("⏸"), m_cameraTimelinePanel);
    m_pauseRecordingButton->setObjectName(QStringLiteral("pauseChipButton"));
    m_pauseRecordingButton->setToolTip(QStringLiteral("일시정지"));
    connect(m_pauseRecordingButton, &QPushButton::clicked, this, &LiveControlPage::handlePauseRecordingButtonClicked);
    controlsRow->addWidget(m_pauseRecordingButton);
    timelineLayout->addLayout(controlsRow);
    m_cameraTimelinePanel->setVisible(false);
    layout->addWidget(m_cameraTimelinePanel);

    return panel;
}
