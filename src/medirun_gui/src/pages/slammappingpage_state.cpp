#include "slammappingpage_internal.h"

void SlamMappingPage::attachMapBridgeCallbacks()
{
    m_mapBridge->onStatusUpdated = [this](const QString &message) {
        QMetaObject::invokeMethod(this, [this, message]() { handleBridgeStatusUpdated(message); }, Qt::QueuedConnection);
    };
}

void SlamMappingPage::resetMappingView(const QString &caption, bool mappingRunning)
{
    m_hasReceivedRealMap = false;
    m_mapView->clearBackgroundImage();
    m_mapView->setPlannedPath({});
    m_mapView->setCaption(caption);
    m_mapView->setRobotVisible(false);
    m_mapView->setRobotPose(QPointF(0.5, 0.5), 0.0);
    m_mapView->setRobotState(mappingRunning ? RobotMarkerState::Moving : RobotMarkerState::Ready);
    setLiveStatusMessage(mappingRunning ? QStringLiteral("새 Cartographer 맵 수신 대기 중")
                                        : QStringLiteral("실시간 상태 대기 중"));
}

void SlamMappingPage::setMappingUiState(MappingUiState state, const QString &detail)
{
    if (m_mappingUiState == state && m_mappingUiDetail == detail) {
        return;
    }
    m_mappingUiState = state;
    m_mappingUiDetail = detail;
    if (!detail.isEmpty()) {
        appendUiLog(QStringLiteral("[SLAM] %1").arg(detail));
    }
    updateSlamUiState();
}

void SlamMappingPage::refreshSensorStatus()
{
    if (m_mapBridge == nullptr) {
        return;
    }

    const TopicStateSnapshot snapshot = m_mapBridge->topicStateSnapshot();
    qInfo() << "[SLAM] scan ready:" << snapshot.scanReceived << snapshot.scanRecent;
    qInfo() << "[SLAM] odom ready:" << snapshot.odomReceived << snapshot.odomRecent;
    qInfo() << "[SLAM] tf ready:" << snapshot.tfReady
            << "tf=" << snapshot.tfReceived << snapshot.tfRecent
            << "tf_static=" << snapshot.tfStaticReceived;
    appendUiLog(QStringLiteral("[SLAM] Bringup 상태 확인"));
    appendUiLog(QStringLiteral("[SLAM] /scan publisher 수: %1").arg(snapshot.scanPublisherCount));
    appendUiLog(QStringLiteral("[SLAM] /scan 수신 확인: %1").arg(topicHealthText(snapshot.scanHealth)));
    appendUiLog(QStringLiteral("[SLAM] /odom publisher 수: %1").arg(snapshot.odomPublisherCount));
    appendUiLog(QStringLiteral("[SLAM] /odom 최초 수신 여부: %1").arg(snapshot.odomReceived ? QStringLiteral("true") : QStringLiteral("false")));
    appendUiLog(QStringLiteral("[SLAM] /odom 메시지 수신 횟수: %1").arg(snapshot.odomMessageCount));
    if (snapshot.odomReceived && snapshot.odomLastAgeSec >= 0.0) {
        appendUiLog(QStringLiteral("[SLAM] /odom 마지막 수신 경과 시간: %1초").arg(snapshot.odomLastAgeSec, 0, 'f', 1));
    }
    appendUiLog(QStringLiteral("[SLAM] /odom 수신 확인: %1").arg(topicHealthText(snapshot.odomHealth)));
    appendUiLog(QStringLiteral("[SLAM] /tf publisher 수: %1").arg(snapshot.tfPublisherCount));
    appendUiLog(QStringLiteral("[SLAM] /tf 수신 확인: %1").arg(topicHealthText(snapshot.tfHealth)));
    appendUiLog(QStringLiteral("[SLAM] /tf_static publisher 수: %1").arg(snapshot.tfStaticPublisherCount));
    appendUiLog(QStringLiteral("[SLAM] /tf_static 수신 확인: %1").arg(topicHealthText(snapshot.tfStaticHealth)));

    if (snapshot.odomHealth == TopicHealth::Stale && snapshot.odomUsable) {
        appendUiLog(QStringLiteral("[SLAM][WARN] /odom의 최근 갱신이 없습니다."));
        appendUiLog(QStringLiteral("[SLAM][WARN] publisher와 이전 수신 데이터가 확인되어 매핑을 계속 시작합니다."));
    }

    if (snapshot.readyForMapping) {
        if (m_mappingUiState == MappingUiState::Idle || m_mappingUiState == MappingUiState::CheckingBringup) {
            setMappingUiState(MappingUiState::SensorsReady, QStringLiteral("센서 준비 완료"));
        }
        setLiveStatusMessage(QStringLiteral("/scan, /odom, TF 준비 완료"));
        return;
    }

    if (m_slamProcess->state() == QProcess::NotRunning && !m_mappingActive) {
        setMappingUiState(MappingUiState::CheckingBringup, QStringLiteral("Bringup 연결 확인 중"));
    }
    setLiveStatusMessage(readinessFailureMessage(snapshot));
}

void SlamMappingPage::appendUiLog(const QString &message)
{
    if (m_logView == nullptr || message.isEmpty()) {
        return;
    }

    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss"));
    const QString line = QStringLiteral("[%1] %2").arg(timestamp, message);
    const QString current = m_logView->toPlainText();
    const QString lastLine = current.section(QLatin1Char('\n'), -1);
    if (lastLine == line) {
        return;
    }
    m_logView->appendPlainText(line);
}

void SlamMappingPage::updateSlamUiState()
{
    const bool slamRunning = (m_slamProcess->state() == QProcess::Running);
    const bool slamStarting = (m_slamProcess->state() == QProcess::Starting);
    const bool saveRunning = (m_saveProcess->state() != QProcess::NotRunning);

    QString bannerTitle = QStringLiteral("🗺  Cartographer 대기 중");
    QString bannerDetail = QStringLiteral("TurtleBot3 bringup 후 매핑 시작 버튼으로 Cartographer를 실행합니다.");
    QString mappingValue = QStringLiteral("비활성");
    QColor mappingColor = DashboardPalette::Amber;

    switch (m_mappingUiState) {
    case MappingUiState::Idle:
        break;
    case MappingUiState::CheckingBringup:
        bannerTitle = QStringLiteral("🗺  Bringup 연결 확인 중");
        bannerDetail = QStringLiteral("매핑 시작 전 /scan, /odom, /tf, /tf_static 수신 상태를 확인합니다.");
        mappingValue = QStringLiteral("준비 확인 중");
        mappingColor = DashboardPalette::Amber;
        break;
    case MappingUiState::SensorsReady:
        bannerTitle = QStringLiteral("🗺  센서 준비 완료");
        bannerDetail = QStringLiteral("Bringup 입력이 준비되었습니다. 매핑 시작을 누르면 Cartographer를 실행합니다.");
        mappingValue = QStringLiteral("센서 준비 완료");
        mappingColor = DashboardPalette::Green;
        break;
    case MappingUiState::Starting:
        bannerTitle = QStringLiteral("🗺  Cartographer 시작 중");
        bannerDetail = QStringLiteral("ROS 환경을 로드하고 Cartographer launch를 실행하는 중입니다.");
        mappingValue = QStringLiteral("시작 중");
        mappingColor = DashboardPalette::Cyan;
        break;
    case MappingUiState::WaitingForMap:
        bannerTitle = QStringLiteral("🗺  /map 토픽 대기 중");
        bannerDetail = QStringLiteral("Cartographer 프로세스는 시작되었지만 아직 실제 OccupancyGrid를 받지 못했습니다.");
        mappingValue = QStringLiteral("/map 대기");
        mappingColor = DashboardPalette::Cyan;
        break;
    case MappingUiState::Running:
        bannerTitle = QStringLiteral("🗺  Cartographer 매핑 실행 중");
        bannerDetail = QStringLiteral("실제 /map을 수신 중이며 로봇 이동에 따라 지도가 확장됩니다.");
        mappingValue = QStringLiteral("실행 중");
        mappingColor = DashboardPalette::Green;
        break;
    case MappingUiState::Error:
        bannerTitle = QStringLiteral("🗺  Cartographer 오류");
        bannerDetail = QStringLiteral("프로세스 로그 또는 센서 연결 상태를 확인하세요.");
        mappingValue = QStringLiteral("오류");
        mappingColor = DashboardPalette::Red;
        break;
    }

    if (m_statusBannerTitleLabel != nullptr) {
        m_statusBannerTitleLabel->setText(bannerTitle);
    }
    if (m_statusBannerDetailLabel != nullptr) {
        if (saveRunning) {
            m_statusBannerDetailLabel->setText(QStringLiteral("현재 점유격자를 maps 폴더에 저장하는 중입니다."));
        } else {
            m_statusBannerDetailLabel->setText(bannerDetail);
        }
    }
    if (m_mappingStatusValueLabel != nullptr) {
        m_mappingStatusValueLabel->setText(mappingValue);
        m_mappingStatusValueLabel->setStyleSheet(QStringLiteral("color: %1; font-size: 11px;").arg(mappingColor.name()));
    }
    if (m_startButton != nullptr) {
        m_startButton->setText((slamRunning || slamStarting) ? QStringLiteral("초기화")
                                                             : QStringLiteral("매핑 재생"));
        m_startButton->setEnabled(!saveRunning);
    }
    if (m_saveButton != nullptr) {
        m_saveButton->setEnabled((slamRunning || slamStarting) && !saveRunning);
    }
    if (m_resetButton != nullptr) {
        m_resetButton->setEnabled(!saveRunning);
    }
    if (m_saveFormatValueLabel != nullptr && !m_lastSavedBasePath.isEmpty()) {
        m_saveFormatValueLabel->setText(QStringLiteral("pgm/yaml · %1").arg(QFileInfo(m_lastSavedBasePath).baseName()));
    }
    if (!m_mappingUiDetail.isEmpty() && !saveRunning) {
        setStatusMessage(m_mappingUiDetail);
    }
}

void SlamMappingPage::setStatusMessage(const QString &message, const QString &kind)
{
    if (m_statusMessageLabel == nullptr) {
        return;
    }
    if (m_statusMessageLabel->text() == message && m_statusMessageLabel->objectName() == kind) {
        return;
    }
    m_statusMessageLabel->setObjectName(kind);
    m_statusMessageLabel->setText(message);
    m_statusMessageLabel->style()->unpolish(m_statusMessageLabel);
    m_statusMessageLabel->style()->polish(m_statusMessageLabel);
}

void SlamMappingPage::setLiveStatusMessage(const QString &message, const QString &kind)
{
    if (m_liveStatusMessageLabel == nullptr) {
        return;
    }
    if (m_liveStatusMessageLabel->text() == message && m_liveStatusMessageLabel->objectName() == kind) {
        return;
    }
    m_liveStatusMessageLabel->setObjectName(kind);
    m_liveStatusMessageLabel->setText(message);
}
