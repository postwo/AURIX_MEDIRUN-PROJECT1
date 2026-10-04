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

QWidget *RemoteControlPage::buildManualControlBanner()
{
    auto *panel = panelFrame(QStringLiteral("anomalyCard"));
    auto *layout = new QHBoxLayout(panel);
    layout->setContentsMargins(12, 9, 12, 9);
    layout->setSpacing(8);

    m_manualBannerTitleLabel = makeLabel(QStringLiteral("수동 제어 대기"));
    m_manualBannerTitleLabel->setWordWrap(false);
    m_manualBannerTitleLabel->setStyleSheet(
        QStringLiteral("color: %1; font-size: 12px; font-weight: 600;").arg(DashboardPalette::TextTertiary.name()));
    layout->addWidget(m_manualBannerTitleLabel);
    m_manualBannerCaptionLabel = makeLabel(QStringLiteral("방향 버튼/WASD로 조작"), QStringLiteral("metricCaption"));
    m_manualBannerCaptionLabel->setWordWrap(false);
    layout->addWidget(m_manualBannerCaptionLabel);
    layout->addStretch();
    return panel;
}

QWidget *RemoteControlPage::buildCameraPanel()
{
    auto *panel = panelFrame(QStringLiteral("contentPanel"));
    panel->setMinimumHeight(230);
    panel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(11, 9, 11, 9);

    m_cameraTopRow = new QWidget(panel);
    auto *topRow = new QHBoxLayout(m_cameraTopRow);
    topRow->setContentsMargins(0, 0, 0, 0);
    topRow->addWidget(makeBadgeLabel(QStringLiteral("ArUco 미검출"), QStringLiteral("amber"), panel));
    topRow->addStretch();
    topRow->addWidget(makeLabel(QStringLiteral("지연 120ms"), QStringLiteral("metricCaption")));
    layout->addWidget(m_cameraTopRow);

    m_cameraFrameLabel = makeLabel(QStringLiteral("🎥  카메라 연결 대기"));
    m_cameraFrameLabel->setAlignment(Qt::AlignCenter);
    m_cameraFrameLabel->setStyleSheet(QStringLiteral("color: %1;").arg(DashboardPalette::TextTertiary.name()));
    m_cameraFrameLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    m_cameraFrameLabel->setMinimumSize(1, 1);
    layout->addWidget(m_cameraFrameLabel, 1);

    return panel;
}

QWidget *RemoteControlPage::buildSensorStatusPanel()
{
    // LiveControlPage::buildRobotStatusPanel()의 전방/후방 장애물·AED 적재
    // 항목을 그대로 재사용한다 — 좌/우 초음파 센서는 실제 하드웨어가 없어
    // 표시하지 않는다.
    auto *panel = panelFrame(QStringLiteral("contentPanel"));
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(11, 9, 11, 9);
    layout->setSpacing(6);
    layout->addWidget(makeLabel(QStringLiteral("센서 상태"), QStringLiteral("metricCaption")));

    m_frontObstacleValueLabel = statValueLabel(layout, QStringLiteral("전방 장애물"));
    m_rearObstacleValueLabel = statValueLabel(layout, QStringLiteral("후방 장애물"));
    m_payloadValueLabel = statValueLabel(layout, QStringLiteral("AED 적재"));
    return panel;
}

QWidget *RemoteControlPage::buildMinimapPanel()
{
    auto *panel = panelFrame(QStringLiteral("contentPanel"));
    // MapViewWidget already paints its own dashed extent border, so the
    // panel's own QSS border is dropped here to avoid a doubled box outline.
    panel->setStyleSheet(QStringLiteral("background-color: %1; border: none; border-radius: 8px;").arg(DashboardPalette::PanelBg.name()));
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(6, 6, 6, 6);
    layout->setSpacing(6);

    // 미니맵(작은) 상태에서는 실시간 위치/초기 위치 설정이 의미 없고 클릭
    // 정밀도도 떨어지므로, 큰 화면(m_mapIsMain)일 때만 보이게 한다 —
    // swapCameraAndMap()에서 visibility를 토글.
    m_mapPoseHintBanner = buildPoseHintBanner();
    layout->addWidget(m_mapPoseHintBanner);

    m_mapView = new MapViewWidget(panel);
    // Fixed height only; width follows the panel so the dashed extent rect
    // (drawn relative to the widget's own size) hugs both left and right
    // edges symmetrically instead of leaving dead space on the right.
    m_mapView->setFixedHeight(100);
    m_mapView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    m_mapView->setMode(MapViewMode::Reference);
    m_mapView->setCaption(QStringLiteral("미니맵 · 방향 참조용"));
    // 초기 위치를 아직 못 받았으면(m_hasAmclPose==false) 로봇 마커를 그리지
    // 않는다 — 위치를 모르는 채로 마커부터 보이면 오해를 준다. 대신
    // buildPoseHintBanner()의 안내 배너로 "초기 위치를 설정하세요"를 보여준다.
    m_mapView->setRobotVisible(false);
    connect(m_mapView, &MapViewWidget::posePickRequested, this, &RemoteControlPage::handlePosePickRequested);
    layout->addWidget(m_mapView, 1);

    m_setInitialPoseButton = new QPushButton(QStringLiteral("📍  초기 위치 설정"));
    m_setInitialPoseButton->setObjectName(QStringLiteral("ghostChipButton"));
    m_setInitialPoseButton->setCheckable(true);
    m_setInitialPoseButton->setEnabled(false);
    m_setInitialPoseButton->setToolTip(
        QStringLiteral("RViz의 2D Pose Estimate와 동일 · 켠 뒤 지도를 클릭+드래그하면 그 위치·방향으로 AMCL 초기 위치를 설정합니다"));
    connect(m_setInitialPoseButton, &QPushButton::toggled, this, &RemoteControlPage::handleSetInitialPoseToggled);
    layout->addWidget(m_setInitialPoseButton);

    return panel;
}

QWidget *RemoteControlPage::buildPoseHintBanner()
{
    auto *panel = panelFrame(QStringLiteral("warningBanner"));
    auto *layout = new QHBoxLayout(panel);
    layout->setContentsMargins(9, 6, 9, 6);
    layout->setSpacing(6);

    m_poseHintIconLabel = makeLabel(QStringLiteral("⚠"));
    layout->addWidget(m_poseHintIconLabel);

    m_poseHintTextLabel = makeLabel(QString());
    layout->addWidget(m_poseHintTextLabel, 1);

    m_mapPoseHintBanner = panel;
    updateInitialPoseBanner();

    return panel;
}

void RemoteControlPage::updateInitialPoseBanner()
{
    if (m_mapPoseHintBanner == nullptr || m_poseHintIconLabel == nullptr || m_poseHintTextLabel == nullptr) {
        return;
    }

    if (!m_hasAmclPose) {
        // 초기 위치를 아예 모르는 상태 — QFrame#warningBanner(주황)보다 급한
        // 빨강으로 눈에 띄게 만든다(buildManualControlBanner의 anomalyCard와
        // 동일 톤).
        m_mapPoseHintBanner->setStyleSheet(
            QStringLiteral("background-color: #1a0f18; border: 1px solid #7c2438; border-radius: 8px;"));
        m_poseHintIconLabel->setStyleSheet(QStringLiteral("color: %1; font-size: 11px;").arg(DashboardPalette::Red.name()));
        m_poseHintTextLabel->setStyleSheet(QStringLiteral("color: %1; font-size: 10px;").arg(DashboardPalette::Red.name()));
        m_poseHintTextLabel->setText(
            QStringLiteral("초기 위치가 설정되지 않았습니다 · 아래 '📍 초기 위치 설정' 버튼을 누른 뒤 지도를 클릭+드래그하세요"));
        return;
    }

    // 초기 위치를 받은 뒤에는 원래의 온화한 확인 안내로 되돌아간다 —
    // 인라인 스타일을 비워서 QFrame#warningBanner(주황) qss가 다시 적용되게 한다.
    m_mapPoseHintBanner->setStyleSheet(QString());
    m_poseHintIconLabel->setStyleSheet(QStringLiteral("color: %1; font-size: 11px;").arg(DashboardPalette::Amber.name()));
    m_poseHintTextLabel->setStyleSheet(QStringLiteral("color: %1; font-size: 10px;").arg(DashboardPalette::Amber.name()));
    m_poseHintTextLabel->setText(QStringLiteral("로봇의 실제 위치와 지도 위 마커가 일치하는지 확인하세요 · 다르면 초기 위치 설정"));
}

QWidget *RemoteControlPage::buildViewSwapButton()
{
    auto *button = new QPushButton(QStringLiteral("⇄  지도/카메라 전환"));
    button->setObjectName(QStringLiteral("ghostChipButton"));
    connect(button, &QPushButton::clicked, this, &RemoteControlPage::swapCameraAndMap);
    return button;
}

QWidget *RemoteControlPage::buildLoadMapButton()
{
    auto *button = new QPushButton(QStringLiteral("🗂  지도 불러오기"));
    button->setObjectName(QStringLiteral("ghostChipButton"));
    connect(button, &QPushButton::clicked, this, &RemoteControlPage::handleLoadMap);
    return button;
}

QWidget *RemoteControlPage::buildRefreshMarkersButton()
{
    // 웨이포인트 관리 페이지에서 콜존/홈을 바꾼 뒤 페이지 재진입 없이도
    // 바로 반영하고 싶을 때 수동으로 다시 불러오는 용도(showEvent 자동
    // 갱신과 별개).
    auto *button = new QPushButton(QStringLiteral("🔄  마커 새로고침"));
    button->setObjectName(QStringLiteral("ghostChipButton"));
    connect(button, &QPushButton::clicked, this, &RemoteControlPage::refreshDbMarkers);
    return button;
}
