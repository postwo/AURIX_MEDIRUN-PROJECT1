#include "slammappingpage_internal.h"

void SlamMappingPage::handleManualDriveTick()
{
    if (m_mapBridge == nullptr) {
        qWarning() << "[TELEOP] cmd_vel publisher 없음";
        return;
    }
    m_mapBridge->publishVelocity(m_manualLinearX, m_manualAngularZ);
}

void SlamMappingPage::handleMapRefreshTick()
{
    if (!m_mappingActive || m_mapBridge == nullptr) {
        return;
    }

    if (!m_mapBridge->hasReceivedRealMap()) {
        refreshSensorStatus();
        setLiveStatusMessage(QStringLiteral("/map 토픽을 기다리는 중 · %1").arg(m_mapBridge->realtimeReadinessText()));
        return;
    }

    const MapRenderSnapshot mapSnapshot = m_mapBridge->takeLatestMapRenderSnapshot();
    if (mapSnapshot.map) {
        QImage image;
        if (!createOccupancyGridImage(*mapSnapshot.map, mapSnapshot.bounds, &image)) {
            appendUiLog(QStringLiteral("[SLAM][ERROR] OccupancyGrid 이미지를 생성하지 못했습니다."));
            setStatusMessage(QStringLiteral("OccupancyGrid 이미지를 생성하지 못했습니다."));
        } else {
            if (!m_hasReceivedRealMap) {
                qInfo() << "[SLAM] first /map received";
                qInfo() << "[SLAM] map width" << image.width() << "height" << image.height();
                appendUiLog(QStringLiteral("[SLAM] 첫 번째 지도 수신"));
                appendUiLog(QStringLiteral("[SLAM] 지도 크기: %1 x %2").arg(image.width()).arg(image.height()));
                m_mapTimeoutTimer->stop();
            }
            m_hasReceivedRealMap = true;
            setMappingUiState(MappingUiState::Running, QStringLiteral("실시간 지도를 생성하고 있습니다."));
            handleMapImageUpdated(image);
            m_mapBridge->markLatestMapDisplayed();
        }
    }

    const RobotPoseSnapshot pose = m_mapBridge->takeLatestRobotPose();
    if (m_hasReceivedRealMap && pose.valid) {
        handleRobotPoseUpdated(pose.normalizedPos, pose.headingDegrees);
    }
}

void SlamMappingPage::handleMapImageUpdated(const QImage &image)
{
    if (!m_mappingActive) {
        return;
    }
    m_mapView->setBackgroundImage(image);
    m_mapView->setRobotVisible(true);
    m_mapView->setCaption(QStringLiteral("실시간 Cartographer 매핑 화면"));
    setLiveStatusMessage(QStringLiteral("실시간 지도를 생성하고 있습니다."));
    appendUiLog(QStringLiteral("[SLAM] 지도 화면 갱신"));
}

void SlamMappingPage::handleRobotPoseUpdated(const QPointF &normalizedPos, qreal headingDegrees)
{
    if (!m_mappingActive) {
        return;
    }
    if (!m_hasReceivedRealMap) {
        return;
    }
    m_mapView->setRobotPose(normalizedPos, headingDegrees);
    m_mapView->setRobotVisible(true);
    m_mapView->setRobotState(RobotMarkerState::Moving);
    appendUiLog(QStringLiteral("[SLAM] map -> base 프레임 조회 성공"));
}

void SlamMappingPage::handleBridgeStatusUpdated(const QString &message)
{
    if (!m_mappingActive) {
        return;
    }
    if (m_hasReceivedRealMap) {
        return;
    }
    if (m_slamProcess->state() == QProcess::Running || m_slamProcess->state() == QProcess::Starting) {
        setLiveStatusMessage(message);
    }
}

void SlamMappingPage::handleMapTimeout()
{
    if (m_hasReceivedRealMap || m_slamProcess->state() != QProcess::Running) {
        return;
    }
    qWarning() << "[SLAM] /map timeout";
    appendUiLog(QStringLiteral("[SLAM][ERROR] /map timeout"));
    setMappingUiState(
        MappingUiState::Error,
        QStringLiteral("Cartographer는 실행되었지만 /map을 받지 못했습니다. /scan, /odom, TF 및 occupancy_grid_node를 확인하세요."));
    setLiveStatusMessage(QStringLiteral("/map timeout · /scan, /odom, TF 및 occupancy_grid_node 확인 필요"));
}
