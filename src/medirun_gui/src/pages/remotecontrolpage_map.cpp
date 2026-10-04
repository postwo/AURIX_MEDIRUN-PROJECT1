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

void RemoteControlPage::handleLoadMap()
{
    const QString path = QFileDialog::getOpenFileName(
        this, QStringLiteral("지도 불러오기 (nav2 map_server YAML)"), QString(),
        QStringLiteral("맵 YAML (*.yaml *.yml)"));
    if (path.isEmpty()) {
        return;
    }

    QString errorMessage;
    if (!loadMapFromFile(path, errorMessage)) {
        QMessageBox::warning(this, QStringLiteral("지도 불러오기 실패"), errorMessage);
        return;
    }

    if (!errorMessage.isEmpty()) {
        // 회전(origin theta) 경고 등 — 실패는 아니므로 로드는 이미 적용된 상태.
        QMessageBox::information(this, QStringLiteral("지도 불러오기"), errorMessage);
    }
}

bool RemoteControlPage::loadMapFromFile(const QString &path, QString &errorMessage)
{
    MapMeta meta;
    if (!loadMapYaml(path, meta, errorMessage)) {
        return false;
    }

    m_currentMapMeta = meta;
    m_mapView->setBackgroundImage(meta.image);
    updateInitialPoseButtonEnabled();
    refreshDbMarkers();
    return true;
}

void RemoteControlPage::refreshDbMarkers()
{
    if (!m_currentMapMeta.valid) {
        return;
    }

    // 홈 — waypoint_set.name="home"(kind=Home). 로봇이 정확히 1대일 때만
    // 조회한다(WaypointManagementPage::mergeHomeWaypointInto와 동일 규칙).
    bool hasHome = false;
    QPointF homeNormalized;
    const QVector<RobotRecord> robots = m_robotService.listRobots(false);
    if (robots.size() == 1) {
        QString frameId;
        WaypointMap homeWaypoints;
        TrajectoryMap trajectories;
        QString homeErrorMessage;
        if (m_overviewService.loadWaypointLibraryByName(
                robots.first().id, QStringLiteral("home"), frameId, homeWaypoints, trajectories, homeErrorMessage)) {
            for (auto it = homeWaypoints.cbegin(); it != homeWaypoints.cend(); ++it) {
                if (it.value().kind == WaypointMarkerKind::Home) {
                    homeNormalized = worldToNormalized(it.value().x, it.value().y);
                    hasHome = true;
                    break;
                }
            }
        }
    }
    m_mapView->setHome(homeNormalized, hasHome);

    // 콜존 — call_button 테이블(현재 활성 항목). WaypointManagementPage의
    // 콜존 글리프를 재사용하는 MapViewWidget::setCallZones()로 홈과 구분되게 표시.
    QVector<CallButtonRow> callButtons;
    QString callButtonErrorMessage;
    QVector<QPointF> callZonePoints;
    QStringList callZoneLabels;
    if (m_overviewService.loadActiveCallButtons(callButtons, callButtonErrorMessage)) {
        callZonePoints.reserve(callButtons.size());
        callZoneLabels.reserve(callButtons.size());
        for (const CallButtonRow &row : std::as_const(callButtons)) {
            callZonePoints.append(worldToNormalized(row.x, row.y));
            callZoneLabels.append(row.zoneName.isEmpty() ? row.code : row.zoneName);
        }
    }
    m_mapView->setCallZones(callZonePoints, callZoneLabels);
}

QString RemoteControlPage::workspaceRootPath() const
{
    // WaypointManagementPage::workspaceRootPath()와 동일한 방식 — 실행 경로
    // 깊이는 --symlink-install 여부에 따라 달라지므로, maps/ 디렉터리가
    // 나올 때까지 상위로 탐색한다.
    QDir dir(QCoreApplication::applicationDirPath());
    for (int i = 0; i < 8; ++i) {
        if (QFileInfo::exists(dir.filePath(QStringLiteral("maps")))) {
            return dir.absolutePath();
        }
        if (!dir.cdUp()) {
            break;
        }
    }
    return QString();
}

void RemoteControlPage::autoLoadDefaultMap()
{
    const QString workspaceRoot = workspaceRootPath();
    if (workspaceRoot.isEmpty()) {
        return;
    }

    const QString defaultMapPath = QDir(workspaceRoot).filePath(QStringLiteral("maps/last_class_map_modi.yaml"));
    if (!QFileInfo::exists(defaultMapPath)) {
        return;
    }

    QString errorMessage;
    loadMapFromFile(defaultMapPath, errorMessage);
}

void RemoteControlPage::handleSetInitialPoseToggled(bool checked)
{
    m_settingInitialPose = checked;
    m_mapView->setPoseSettingEnabled(checked);
}

void RemoteControlPage::handlePosePickRequested(const QPointF &normalizedPos, double yawRadians)
{
    if (!m_settingInitialPose || m_navigationService == nullptr || !m_currentMapMeta.valid) {
        return;
    }

    // 한 번 찍으면 바로 꺼짐 — 웨이포인트 페이지의 "초기 위치 설정"과 동일한 UX.
    m_setInitialPoseButton->setChecked(false);

    const QPointF world = normalizedToWorld(normalizedPos);
    m_navigationService->publishInitialPose(world.x(), world.y(), yawRadians, QStringLiteral("map"));
}

QPointF RemoteControlPage::worldToNormalized(double x, double y) const
{
    const QPointF pixel = worldToPixel(QPointF(x, y), m_currentMapMeta);
    return QPointF(pixel.x() / m_currentMapMeta.width, pixel.y() / m_currentMapMeta.height);
}

QPointF RemoteControlPage::normalizedToWorld(const QPointF &normalized) const
{
    const QPointF pixel(normalized.x() * m_currentMapMeta.width, normalized.y() * m_currentMapMeta.height);
    return pixelToWorld(pixel, m_currentMapMeta);
}

void RemoteControlPage::updateInitialPoseButtonEnabled()
{
    if (m_setInitialPoseButton == nullptr) {
        return;
    }
    m_setInitialPoseButton->setEnabled(m_navigationService != nullptr && m_currentMapMeta.valid);
}

void RemoteControlPage::swapCameraAndMap()
{
    m_mapIsMain = !m_mapIsMain;

    m_bigSlot->removeWidget(m_cameraPanel);
    m_bigSlot->removeWidget(m_minimapPanel);
    m_smallSlot->removeWidget(m_cameraPanel);
    m_smallSlot->removeWidget(m_minimapPanel);

    QWidget *bigWidget = m_mapIsMain ? m_minimapPanel : m_cameraPanel;
    QWidget *smallWidget = m_mapIsMain ? m_cameraPanel : m_minimapPanel;
    m_bigSlot->addWidget(bigWidget);
    m_smallSlot->addWidget(smallWidget);

    if (m_mapIsMain) {
        m_mapView->setMinimumSize(0, 0);
        m_mapView->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
        m_mapView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        m_minimapPanel->setMinimumHeight(230);
        m_minimapPanel->setMaximumHeight(QWIDGETSIZE_MAX);
        m_minimapPanel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        m_cameraPanel->setMinimumHeight(0);
        // 배지 오버레이를 감춰서 축소된 높이 대부분을 실제 영상에 내준다
        // (100px로는 오버레이만 겨우 보이고 영상은 안 보이던 문제 수정).
        m_cameraTopRow->setVisible(false);
        m_cameraPanel->setFixedHeight(150);
        m_cameraPanel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);

        // 지도가 큰 화면이 됐을 때만 실시간 위치/초기 위치 설정 UI를 노출한다.
        m_mapPoseHintBanner->setVisible(true);
        m_setInitialPoseButton->setVisible(true);
    } else {
        m_mapView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        m_mapView->setMinimumWidth(0);
        m_mapView->setMaximumWidth(QWIDGETSIZE_MAX);
        m_mapView->setFixedHeight(100);
        m_minimapPanel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
        m_minimapPanel->setMinimumHeight(0);
        m_minimapPanel->setMaximumHeight(QWIDGETSIZE_MAX);
        m_cameraTopRow->setVisible(true);
        m_cameraPanel->setMaximumHeight(QWIDGETSIZE_MAX);
        m_cameraPanel->setMinimumHeight(230);
        m_cameraPanel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        // 미니맵으로 축소되면 초기 위치 설정 제스처도 같이 꺼서, 숨겨진 상태에서
        // 지도 클릭 좌표가 엉뚱하게 해석되는 일이 없도록 한다.
        m_mapPoseHintBanner->setVisible(false);
        m_setInitialPoseButton->setChecked(false);
        m_setInitialPoseButton->setVisible(false);
    }
}
