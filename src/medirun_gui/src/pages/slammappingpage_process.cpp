#include "slammappingpage_internal.h"

QString SlamMappingPage::mapsDirectoryPath() const
{
    const QString workspaceRoot = findWorkspaceRoot();
    if (workspaceRoot.isEmpty()) {
        return QString();
    }

    QDir mapsDir(workspaceRoot);
    if (!mapsDir.exists(QStringLiteral("maps")) && !mapsDir.mkdir(QStringLiteral("maps"))) {
        return QString();
    }
    return mapsDir.filePath(QStringLiteral("maps"));
}

QString SlamMappingPage::nextMapBasePath() const
{
    const QString mapsDir = mapsDirectoryPath();
    if (mapsDir.isEmpty()) {
        return QString();
    }

    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
    return QDir(mapsDir).filePath(QStringLiteral("slam_map_%1").arg(timestamp));
}

bool SlamMappingPage::startMappingProcess()
{
    qInfo() << "[SLAM] mapping start requested";
    appendUiLog(QStringLiteral("[SLAM] 매핑 시작 요청"));
    stopMappingProcess();
    m_mapBridge->beginFreshSession();
    refreshSensorStatus();

    const TopicStateSnapshot snapshot = m_mapBridge->topicStateSnapshot();
    if (!snapshot.readyForMapping) {
        const QString message = readinessFailureMessage(snapshot);
        qWarning() << "[SLAM]" << message;
        appendUiLog(QStringLiteral("[SLAM][ERROR] %1").arg(message));
        setMappingUiState(MappingUiState::CheckingBringup, message);
        resetMappingView(QStringLiteral("Bringup 입력이 준비되지 않아 매핑을 시작하지 않았습니다."), false);
        setLiveStatusMessage(message);
        return false;
    }

    if (snapshot.odomHealth == TopicHealth::Stale && snapshot.odomUsable) {
        appendUiLog(QStringLiteral("[SLAM][WARN] /odom publisher는 존재하지만 최근 메시지가 없습니다."));
        appendUiLog(QStringLiteral("[SLAM][WARN] 마지막으로 수신한 odometry 데이터를 사용하여 매핑을 시작합니다."));
    }

    stopManualDrive();
    setMappingUiState(MappingUiState::Starting, QStringLiteral("Cartographer 시작 중"));
    resetMappingView(QStringLiteral("새 SLAM 세션 준비 중 · 이전 맵을 버리고 실시간 맵만 기다리는 중"), false);
    const QString turtlebotModel = QString::fromLocal8Bit(qgetenv("TURTLEBOT3_MODEL")).isEmpty()
        ? QStringLiteral("burger")
        : QString::fromLocal8Bit(qgetenv("TURTLEBOT3_MODEL"));
    const QString slamCommand = shellCommandPrefix()
        + QStringLiteral("export TURTLEBOT3_MODEL=\"%1\" && export RMW_IMPLEMENTATION=\"${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}\" && exec ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=false use_rviz:=false")
              .arg(turtlebotModel);
    qInfo() << "[SLAM] starting Cartographer process:" << slamCommand;
    appendUiLog(QStringLiteral("[SLAM] Cartographer 실행 시작"));
    appendUiLog(QStringLiteral("[SLAM] Cartographer 실행 명령 준비 완료"));
    m_slamProcess->setProgram(QStringLiteral("setsid"));
    m_slamProcess->setArguments({
        QStringLiteral("bash"),
        QStringLiteral("-lc"),
        slamCommand
    });
    m_slamProcess->start();
    if (!m_slamProcess->waitForStarted(5000)) {
        qWarning() << "[SLAM] Cartographer process failed:" << m_slamProcess->errorString();
        appendUiLog(QStringLiteral("[SLAM][ERROR] Cartographer 시작 실패: %1").arg(m_slamProcess->errorString()));
        setMappingUiState(MappingUiState::Error,
                          QStringLiteral("Cartographer 시작 실패: %1").arg(m_slamProcess->errorString()));
        return false;
    }
    m_mappingActive = true;
    m_mapRefreshTimer->start();
    resetMappingView(QStringLiteral("실시간 Cartographer 맵 수신 대기 중"), true);
    setMappingUiState(MappingUiState::WaitingForMap, QStringLiteral("Cartographer 프로세스 시작됨"));
    setLiveStatusMessage(QStringLiteral("Cartographer 실행 후 /map을 기다리는 중"));
    return true;
}

// Cartographer 프로세스만 정리한다 — nav2 PAUSE/RESUME은 호출자가 상황에 맞는
// 방식(인터랙티브면 비동기, 앱 종료 직전이면 blocking)으로 별도로 처리한다.
// aboutToQuit(handleAboutToQuit) / 초기화 버튼(handleResetMapping) 양쪽에서
// 공유하기 때문에 이 함수 자체에 특정 방식을 강제하지 않는다.
void SlamMappingPage::stopMappingProcess()
{
    stopManualDrive();
    m_mapTimeoutTimer->stop();
    m_mapRefreshTimer->stop();
    m_mappingActive = false;
    if (m_slamProcess->state() == QProcess::NotRunning) {
        return;
    }

    m_stoppingSlamProcess = true;
    terminateProcessTree(m_slamProcess, 4000, 2000);
}

// 성공/실패와 무관하게 UI 로그·상태 메시지로 남긴다. 초기화 버튼 경로와
// 매핑 시작 실패 시 롤백 경로 양쪽에서 공유한다.
void SlamMappingPage::logNav2ResumeResult(const QStringList &failedManagers)
{
    if (failedManagers.isEmpty()) {
        appendUiLog(QStringLiteral("[SLAM] Nav2 재개 완료"));
        return;
    }
    appendUiLog(QStringLiteral("[SLAM][ERROR] Nav2 재개 실패: %1 (여전히 PAUSE 상태일 수 있습니다)")
                    .arg(failedManagers.join(QStringLiteral(", "))));
    setStatusMessage(QStringLiteral(
        "Nav2 재개 실패 · 웨이포인트 주행이 동작하지 않을 수 있습니다. 다시 초기화를 눌러 재시도하거나 run.sh를 재시작하세요."));
}

void SlamMappingPage::handleStartMapping()
{
    if (m_slamProcess->state() != QProcess::NotRunning) {
        setStatusMessage(QStringLiteral("Cartographer가 이미 실행 중입니다."));
        return;
    }

    if (m_mapBridge == nullptr || !m_mapBridge->isNav2LocalizationActive()) {
        beginCartographerLaunch();
        return;
    }

    const auto answer = QMessageBox::question(
        this,
        QStringLiteral("Nav2/AMCL 감지됨"),
        QStringLiteral("이 로봇에서 Nav2(AMCL)가 실행 중입니다. 같이 켜져 있으면 map→odom 변환이 "
                        "충돌해 지도 생성이 정상 동작하지 않습니다.\n\n"
                        "지도 생성을 위해 Nav2를 일시정지(PAUSE)하고 진행할까요?\n"
                        "(매핑을 초기화하거나 종료하면 자동으로 Nav2를 재개합니다)"),
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::Yes);
    if (answer != QMessageBox::Yes) {
        appendUiLog(QStringLiteral("[SLAM] Nav2/AMCL이 실행 중이라 매핑을 시작하지 않았습니다."));
        setStatusMessage(QStringLiteral("Nav2가 실행 중이라 매핑을 시작하지 않았습니다."));
        return;
    }

    setMappingUiState(MappingUiState::CheckingBringup, QStringLiteral("Nav2 일시정지 중"));
    if (m_startButton != nullptr) {
        // 비동기 응답을 기다리는 동안 중복 클릭으로 PAUSE 요청이 겹치지 않게 막는다.
        m_startButton->setEnabled(false);
    }
    appendUiLog(QStringLiteral("[SLAM] Nav2/AMCL 감지됨 · lifecycle PAUSE 요청"));
    m_mapBridge->pauseNav2ForMappingAsync([this](bool localizationPaused) {
        QMetaObject::invokeMethod(this, [this, localizationPaused]() {
            handleNav2PauseCompleted(localizationPaused);
        }, Qt::QueuedConnection);
    });
}

void SlamMappingPage::handleNav2PauseCompleted(bool localizationPaused)
{
    if (m_startButton != nullptr) {
        m_startButton->setEnabled(true);
    }
    if (!localizationPaused) {
        appendUiLog(QStringLiteral("[SLAM][ERROR] Nav2 일시정지 실패"));
        setMappingUiState(MappingUiState::Error, QStringLiteral("Nav2 일시정지 실패"));
        QMessageBox::warning(
            this,
            QStringLiteral("Nav2 일시정지 실패"),
            QStringLiteral("Nav2(AMCL) lifecycle 서비스 호출에 실패했습니다.\n"
                            "다른 PC에서 run.sh로 띄운 대시보드를 직접 종료한 뒤 다시 시도하세요."));
        return;
    }
    appendUiLog(QStringLiteral("[SLAM] Nav2 일시정지 완료"));
    beginCartographerLaunch();
}

void SlamMappingPage::beginCartographerLaunch()
{
    setMappingUiState(MappingUiState::CheckingBringup, QStringLiteral("Bringup 연결 확인 중"));
    if (!startMappingProcess()) {
        if (m_mapBridge != nullptr) {
            m_mapBridge->resumeNav2IfPausedAsync([this](const QStringList &failedManagers) {
                QMetaObject::invokeMethod(this, [this, failedManagers]() {
                    logNav2ResumeResult(failedManagers);
                }, Qt::QueuedConnection);
            });
        }
        return;
    }
    setStatusMessage(QStringLiteral("새 Cartographer 세션을 시작했습니다. 기존 맵은 가져오지 않고 새 매핑만 표시합니다."));
}

void SlamMappingPage::handleResetMapping()
{
    stopMappingProcess();
    m_lastSavedBasePath.clear();
    if (m_mapBridge != nullptr) {
        m_mapBridge->beginFreshSession();
        // 비동기 요청 — 여기서 기다리지 않고 바로 UI를 "초기화 완료"로 되돌린다.
        // 응답은 나중에 콜백으로 도착해 로그/상태 메시지만 갱신한다. 그 사이
        // (아직 nav2가 재개되지 않은 동안) 웨이포인트 페이지의 "Nav2 미연결"
        // 배지·버튼 비활성화가 실제 재개 완료 시점까지 사용자를 보호해 준다.
        m_mapBridge->resumeNav2IfPausedAsync([this](const QStringList &failedManagers) {
            QMetaObject::invokeMethod(this, [this, failedManagers]() {
                logNav2ResumeResult(failedManagers);
            }, Qt::QueuedConnection);
        });
    }
    setMappingUiState(MappingUiState::Idle, QStringLiteral("매핑이 초기화되었습니다. 다시 시작하려면 매핑 재생을 누르세요."));
    resetMappingView(QStringLiteral("초기화 완료 · 새 매핑을 시작하려면 매핑 재생을 누르세요."), false);
    updateSlamUiState();
    setStatusMessage(QStringLiteral("매핑을 초기화했습니다. 다시 시작하려면 매핑 재생을 누르세요."));
}

void SlamMappingPage::handleSaveMap()
{
    if (m_saveProcess->state() != QProcess::NotRunning) {
        setStatusMessage(QStringLiteral("이미 지도 저장 작업이 실행 중입니다."));
        return;
    }

    appendUiLog(QStringLiteral("[MAP SAVE] 지도 저장 요청"));
    const MapTopicSnapshot mapSnapshot = m_mapBridge->mapTopicSnapshot();
    appendUiLog(QStringLiteral("[MAP SAVE] /map publisher 수: %1").arg(mapSnapshot.publisherCount));
    if (!mapSnapshot.publisherNames.isEmpty()) {
        appendUiLog(QStringLiteral("[MAP SAVE] publisher: %1").arg(mapSnapshot.publisherNames.join(QStringLiteral(", "))));
    }
    if (mapSnapshot.publisherCount == 0) {
        appendUiLog(QStringLiteral("[MAP SAVE][ERROR] /map publisher가 없습니다."));
        setStatusMessage(QStringLiteral("/map publisher가 없습니다."));
        return;
    }
    if (mapSnapshot.publisherCount > 1) {
        appendUiLog(QStringLiteral("[MAP SAVE][ERROR] /map publisher가 여러 개 존재합니다."));
        appendUiLog(QStringLiteral("[MAP SAVE][ERROR] publishers: %1").arg(mapSnapshot.publisherNames.join(QStringLiteral(", "))));
        setStatusMessage(QStringLiteral("/map publisher가 여러 개 존재합니다. 중복 SLAM 실행을 확인하세요."));
        return;
    }
    if (!mapSnapshot.hasValidMap) {
        appendUiLog(QStringLiteral("[MAP SAVE][ERROR] 유효한 OccupancyGrid를 아직 수신하지 못했습니다."));
        setStatusMessage(QStringLiteral("유효한 OccupancyGrid를 아직 수신하지 못했습니다."));
        return;
    }

    appendUiLog(QStringLiteral("[MAP SAVE] 마지막 지도 크기: %1 x %2").arg(mapSnapshot.width).arg(mapSnapshot.height));
    appendUiLog(QStringLiteral("[MAP SAVE] resolution: %1").arg(mapSnapshot.resolution, 0, 'f', 4));
    appendUiLog(QStringLiteral("[MAP SAVE] origin: [%1, %2]").arg(mapSnapshot.originX, 0, 'f', 3).arg(mapSnapshot.originY, 0, 'f', 3));

    const QString defaultMapsDir = mapsDirectoryPath();
    if (defaultMapsDir.isEmpty()) {
        QMessageBox::warning(this,
                             QStringLiteral("저장 경로 오류"),
                             QStringLiteral("workspace의 maps 폴더를 찾지 못했습니다."));
        return;
    }

    const QString selectedDirectory = QFileDialog::getExistingDirectory(
        this,
        QStringLiteral("지도 저장 폴더 선택"),
        defaultMapsDir,
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (selectedDirectory.isEmpty()) {
        return;
    }

    const QFileInfo directoryInfo(selectedDirectory);
    if (!directoryInfo.exists() || !directoryInfo.isDir() || !directoryInfo.isWritable()) {
        appendUiLog(QStringLiteral("[MAP SAVE][ERROR] 저장 폴더에 쓰기 권한이 없습니다."));
        setStatusMessage(QStringLiteral("저장 폴더에 쓰기 권한이 없습니다."));
        return;
    }

    const QString defaultBaseName = QFileInfo(nextMapBasePath()).fileName();
    bool accepted = false;
    const QString rawBaseName = QInputDialog::getText(
        this,
        QStringLiteral("지도 이름 입력"),
        QStringLiteral("저장할 지도 기본 이름을 입력하세요"),
        QLineEdit::Normal,
        defaultBaseName,
        &accepted);
    if (!accepted) {
        return;
    }

    const QString normalizedBaseName = normalizeMapBaseName(rawBaseName);
    if (normalizedBaseName.isEmpty()) {
        appendUiLog(QStringLiteral("[MAP SAVE][ERROR] 지도 기본 이름이 비어 있습니다."));
        setStatusMessage(QStringLiteral("지도 기본 이름이 비어 있습니다."));
        return;
    }

    const QString basePath = QDir(selectedDirectory).filePath(normalizedBaseName);
    const QString pgmPath = basePath + QStringLiteral(".pgm");
    const QString yamlPath = basePath + QStringLiteral(".yaml");
    if (QFile::exists(pgmPath) || QFile::exists(yamlPath)) {
        const auto answer = QMessageBox::question(
            this,
            QStringLiteral("기존 지도 덮어쓰기"),
            QStringLiteral("같은 이름의 지도 파일이 이미 있습니다. 덮어쓰시겠습니까?\n%1\n%2")
                .arg(pgmPath, yamlPath));
        if (answer != QMessageBox::Yes) {
            return;
        }
    }

    m_pendingSaveBasePath = basePath;
    m_lastSaveStdout.clear();
    m_lastSaveStderr.clear();
    appendUiLog(QStringLiteral("[MAP SAVE] 저장 기본 경로: %1").arg(basePath));
    const QString saveCommand = shellCommandPrefix()
        + QStringLiteral("ros2 run nav2_map_server map_saver_cli -f %1 --ros-args -p map_subscribe_transient_local:=true")
              .arg(shellSingleQuoted(basePath));
    appendUiLog(QStringLiteral("[MAP SAVE] map_saver_cli 실행"));
    m_saveProcess->setProgram(QStringLiteral("bash"));
    m_saveProcess->setArguments({
        QStringLiteral("-lc"),
        saveCommand
    });
    m_saveProcess->start();
    updateSlamUiState();
    setStatusMessage(QStringLiteral("지도 저장을 시작했습니다: %1").arg(QFileInfo(basePath).fileName()));
}

void SlamMappingPage::handleSlamProcessStateChanged()
{
    if (m_slamProcess->state() == QProcess::Starting) {
        appendUiLog(QStringLiteral("[SLAM] Cartographer 프로세스 시작 중"));
        setMappingUiState(MappingUiState::Starting, QStringLiteral("Cartographer 시작 중"));
    }
    updateSlamUiState();
}

void SlamMappingPage::handleSlamProcessErrorOccurred()
{
    if (m_stoppingSlamProcess) {
        appendUiLog(QStringLiteral("[SLAM] Cartographer 종료 요청 처리 중"));
        return;
    }
    qWarning() << "[SLAM] process failed:" << m_slamProcess->errorString();
    appendUiLog(QStringLiteral("[SLAM][ERROR] Cartographer 프로세스 오류: %1").arg(m_slamProcess->errorString()));
    setMappingUiState(MappingUiState::Error,
                      QStringLiteral("Cartographer 실행 오류: %1").arg(m_slamProcess->errorString()));
    if (!m_hasReceivedRealMap) {
        resetMappingView(QStringLiteral("Cartographer 실행 실패 · 실시간 지도를 받지 못했습니다."), false);
    }
}

void SlamMappingPage::handleSlamProcessFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    qInfo() << "[SLAM] Cartographer process finished:" << exitCode << "status=" << static_cast<int>(exitStatus);
    appendUiLog(QStringLiteral("[SLAM] Cartographer 프로세스 종료: exit=%1 status=%2")
                    .arg(exitCode)
                    .arg(static_cast<int>(exitStatus)));
    m_mapTimeoutTimer->stop();
    const bool stoppingRequested = m_stoppingSlamProcess;
    m_stoppingSlamProcess = false;
    if (stoppingRequested) {
        appendUiLog(QStringLiteral("[SLAM] Cartographer 프로세스를 요청에 따라 종료했습니다."));
        return;
    }
    if (!m_hasReceivedRealMap) {
        setMappingUiState(
            MappingUiState::Error,
            QStringLiteral("Cartographer는 실행되었지만 /map을 받지 못했습니다. /scan, /odom, TF 및 occupancy_grid_node를 확인하세요."));
    } else {
        setMappingUiState(MappingUiState::Idle, QStringLiteral("Cartographer 세션이 종료되었습니다."));
    }
    if (!m_hasReceivedRealMap) {
        resetMappingView(QStringLiteral("Cartographer 종료 · 실시간 지도를 받지 못했습니다."), false);
    }
}

void SlamMappingPage::handleSlamProcessStdout()
{
    const QString output = QString::fromLocal8Bit(m_slamProcess->readAllStandardOutput());
    if (output.trimmed().isEmpty()) {
        return;
    }
    for (const QString &line : output.split(QLatin1Char('\n'), Qt::SkipEmptyParts)) {
        const QString trimmed = line.trimmed();
        qInfo().noquote() << "[SLAM][stdout]" << trimmed;
        appendUiLog(QStringLiteral("[SLAM][stdout] %1").arg(trimmed));
    }
}

void SlamMappingPage::handleSlamProcessStderr()
{
    const QString output = QString::fromLocal8Bit(m_slamProcess->readAllStandardError());
    if (output.trimmed().isEmpty()) {
        return;
    }
    for (const QString &line : output.split(QLatin1Char('\n'), Qt::SkipEmptyParts)) {
        const QString trimmed = line.trimmed();
        qWarning().noquote() << "[SLAM][stderr]" << trimmed;
        appendUiLog(QStringLiteral("[SLAM][stderr] %1").arg(trimmed));
        if (!m_stoppingSlamProcess && isSlamFatalLogLine(trimmed)) {
            setMappingUiState(MappingUiState::Error, trimmed);
        }
    }
}

void SlamMappingPage::handleSaveProcessFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    updateSlamUiState();
    const QString basePath = m_pendingSaveBasePath;
    m_pendingSaveBasePath.clear();
    if (exitStatus != QProcess::NormalExit || exitCode != 0) {
        appendUiLog(QStringLiteral("[MAP SAVE][ERROR] map_saver_cli 실행에 실패했습니다."));
        setStatusMessage(QStringLiteral("지도 저장 실패: %1").arg(m_lastSaveStderr.trimmed().isEmpty() ? m_lastSaveStdout.trimmed()
                                                                                                       : m_lastSaveStderr.trimmed()));
        return;
    }

    const QString pgmPath = basePath + QStringLiteral(".pgm");
    const QString yamlPath = basePath + QStringLiteral(".yaml");
    if (!QFile::exists(pgmPath) || !QFile::exists(yamlPath)) {
        appendUiLog(QStringLiteral("[MAP SAVE][ERROR] PGM 또는 YAML 파일이 생성되지 않았습니다."));
        setStatusMessage(QStringLiteral("PGM 또는 YAML 파일이 생성되지 않았습니다."));
        return;
    }

    MapMeta meta;
    QString validationMessage;
    if (!loadMapYaml(yamlPath, meta, validationMessage) || !meta.valid) {
        appendUiLog(QStringLiteral("[MAP SAVE][ERROR] 저장된 YAML 검증에 실패했습니다."));
        setStatusMessage(QStringLiteral("저장된 YAML 검증에 실패했습니다: %1").arg(validationMessage));
        return;
    }
    try {
        const YAML::Node root = YAML::LoadFile(yamlPath.toStdString());
        if (!root["image"] || !root["mode"] || !root["resolution"] || !root["origin"]
            || !root["negate"] || !root["occupied_thresh"] || !root["free_thresh"]) {
            appendUiLog(QStringLiteral("[MAP SAVE][ERROR] YAML 필수 항목이 누락되었습니다."));
            setStatusMessage(QStringLiteral("저장된 YAML 필수 항목이 누락되었습니다."));
            return;
        }
    } catch (const std::exception &exc) {
        appendUiLog(QStringLiteral("[MAP SAVE][ERROR] YAML 파싱 실패: %1").arg(QString::fromStdString(exc.what())));
        setStatusMessage(QStringLiteral("저장된 YAML 파싱 실패: %1").arg(QString::fromStdString(exc.what())));
        return;
    }
    if (!validationMessage.isEmpty()) {
        appendUiLog(QStringLiteral("[MAP SAVE][WARN] %1").arg(validationMessage));
    }

    appendUiLog(QStringLiteral("[MAP SAVE] PGM 생성 확인"));
    appendUiLog(QStringLiteral("[MAP SAVE] YAML 생성 확인"));
    appendUiLog(QStringLiteral("[MAP SAVE] 지도 저장 완료"));

    m_lastSavedBasePath = basePath;
    const QString baseName = QFileInfo(m_lastSavedBasePath).baseName();
    setStatusMessage(QStringLiteral("지도 저장 완료: %1, %2")
                         .arg(QFileInfo(pgmPath).fileName(), QFileInfo(yamlPath).fileName()));
    if (m_saveFormatValueLabel != nullptr) {
        m_saveFormatValueLabel->setText(QStringLiteral("pgm/yaml · %1").arg(baseName));
    }
}

void SlamMappingPage::handleSaveProcessErrorOccurred()
{
    updateSlamUiState();
    m_pendingSaveBasePath.clear();
    setStatusMessage(QStringLiteral("지도 저장 실행 오류: %1").arg(m_saveProcess->errorString()));
}

void SlamMappingPage::handleSaveProcessStdout()
{
    const QString output = QString::fromLocal8Bit(m_saveProcess->readAllStandardOutput());
    if (output.trimmed().isEmpty()) {
        return;
    }
    m_lastSaveStdout.append(output);
    for (const QString &line : output.split(QLatin1Char('\n'), Qt::SkipEmptyParts)) {
        const QString trimmed = line.trimmed();
        qInfo().noquote() << "[MAP SAVE][stdout]" << trimmed;
        appendUiLog(QStringLiteral("[MAP SAVE][stdout] %1").arg(trimmed));
    }
}

void SlamMappingPage::handleSaveProcessStderr()
{
    const QString output = QString::fromLocal8Bit(m_saveProcess->readAllStandardError());
    if (output.trimmed().isEmpty()) {
        return;
    }
    m_lastSaveStderr.append(output);
    for (const QString &line : output.split(QLatin1Char('\n'), Qt::SkipEmptyParts)) {
        const QString trimmed = line.trimmed();
        qWarning().noquote() << "[MAP SAVE][stderr]" << trimmed;
        appendUiLog(QStringLiteral("[MAP SAVE][stderr] %1").arg(trimmed));
    }
}
