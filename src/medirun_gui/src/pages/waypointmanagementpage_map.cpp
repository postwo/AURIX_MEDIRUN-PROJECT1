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

void WaypointManagementPage::handleLoadMap()
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
        // 회전 warning 등 — 실패는 아니었으므로 로드는 계속 진행한다.
        m_waypointStatusLabel->setText(errorMessage);
    }
}

bool WaypointManagementPage::loadMapFromFile(const QString &path, QString &errorMessage)
{
    MapMeta meta;
    if (!loadMapYaml(path, meta, errorMessage)) {
        return false;
    }

    m_currentMapMeta = meta;
    m_waypointMap->setMapMeta(meta);
    m_waypointMap->setBackgroundImage(meta.image);
    m_routeMap->setMapMeta(meta);
    m_routeMap->setBackgroundImage(meta.image);

    m_mapStatusLabel->setText(QStringLiteral("불러온 지도: %1 (%2×%3, %4 m/px)")
                                   .arg(QFileInfo(path).fileName())
                                   .arg(meta.width)
                                   .arg(meta.height)
                                   .arg(meta.resolution));
    refreshMapMarkers();
    return true;
}

QString WaypointManagementPage::workspaceRootPath() const
{
    // 실행 파일 경로에서 고정 단계 수만큼 올라가는 방식은 --symlink-install
    // 여부에 따라 실제 깊이가 달라져 신뢰할 수 없다: OS가 install/의
    // 심볼릭 링크를 실행 전에 미리 풀어버리면 QCoreApplication::
    // applicationDirPath()가 install/medirun_gui/lib/medirun_gui가 아니라
    // 그 링크가 가리키는 build/medirun_gui를 반환해 깊이가 2단계 차이 남.
    // 대신 maps/ 디렉터리가 나올 때까지 상위로 탐색한다(계정별 절대 경로
    // 하드코딩 금지, CONVENTIONS.md 3.2절 관례는 유지).
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

void WaypointManagementPage::autoLoadDefaultMap()
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
    if (loadMapFromFile(defaultMapPath, errorMessage) && !errorMessage.isEmpty()) {
        m_waypointStatusLabel->setText(errorMessage);
    }
}

void WaypointManagementPage::mergeCallButtonsInto(WaypointMap &waypoints) const
{
    QVector<CallButtonRow> rows;
    QString errorMessage;
    if (!m_overviewService.loadActiveCallButtons(rows, errorMessage)) {
        return;
    }
    for (const CallButtonRow &row : std::as_const(rows)) {
        WaypointDef def;
        def.name = row.code;
        def.label = row.zoneName.isEmpty() ? row.code : row.zoneName;
        def.x = row.x;
        def.y = row.y;
        def.yaw = row.yaw;
        def.kind = WaypointMarkerKind::CallZone;
        def.dbId = row.id;
        waypoints.insert(def.name, def);
    }
}

void WaypointManagementPage::mergeHomeWaypointInto(WaypointMap &waypoints) const
{
    const QVector<RobotRecord> robots = m_robotService.listRobots(false);
    if (robots.size() != 1) {
        return;
    }

    QString frameId;
    WaypointMap homeWaypoints;
    TrajectoryMap trajectories;
    QString errorMessage;
    if (!m_overviewService.loadWaypointLibraryByName(
            robots.first().id, QStringLiteral("home"), frameId, homeWaypoints, trajectories, errorMessage)) {
        return;
    }

    for (auto it = homeWaypoints.cbegin(); it != homeWaypoints.cend(); ++it) {
        if (it.value().kind == WaypointMarkerKind::Home) {
            waypoints.insert(it.key(), it.value());
        }
    }
}

void WaypointManagementPage::mergeWaypointEntriesInto(WaypointMap &waypoints) const
{
    const QVector<RobotRecord> robots = m_robotService.listRobots(false);
    if (robots.size() != 1) {
        return;
    }

    QVector<WaypointDef> rows;
    QString errorMessage;
    if (!m_overviewService.loadActiveWaypointEntries(robots.first().id, rows, errorMessage)) {
        return;
    }
    for (const WaypointDef &def : std::as_const(rows)) {
        waypoints.insert(def.name, def);
    }
}

void WaypointManagementPage::reloadTrajectoriesFromDb()
{
    const QVector<RobotRecord> robots = m_robotService.listRobots(false);
    if (robots.size() != 1) {
        return;
    }

    QVector<TrajectoryRow> rows;
    QString errorMessage;
    if (!m_overviewService.loadActiveTrajectoryEntries(robots.first().id, rows, errorMessage)) {
        return;
    }
    m_trajectories.clear();
    for (const TrajectoryRow &row : std::as_const(rows)) {
        m_trajectories.insert(row.name, row.waypointCodes);
    }
}

void WaypointManagementPage::backupWaypointEntryToFile(const WaypointDef &def) const
{
    const QString workspaceRoot = workspaceRootPath();
    if (workspaceRoot.isEmpty()) {
        return;
    }

    const QString fileName =
        QStringLiteral("waypoint_%1.yaml").arg(sanitizeForFileName(def.label.isEmpty() ? def.name : def.label));
    const QString path = QDir(workspaceRoot).filePath(QStringLiteral("maps/waypoints/%1").arg(fileName));

    WaypointMap single;
    single.insert(def.name, def);
    QString errorMessage;
    if (!saveWaypointYaml(path, m_frameId, single, TrajectoryMap(), errorMessage)) {
        qWarning() << "경유점 백업 저장 실패:" << errorMessage;
    }
}

void WaypointManagementPage::backupTrajectoryToFile(const QString &name, const QStringList &waypointCodes) const
{
    const QString workspaceRoot = workspaceRootPath();
    if (workspaceRoot.isEmpty()) {
        return;
    }

    const QString fileName = QStringLiteral("%1_%2.yaml")
                                  .arg(QDate::currentDate().toString(QStringLiteral("yyMMdd")))
                                  .arg(sanitizeForFileName(name));
    const QString path = QDir(workspaceRoot).filePath(QStringLiteral("maps/waypoints/%1").arg(fileName));

    TrajectoryMap single;
    single.insert(name, waypointCodes);
    QString errorMessage;
    if (!saveWaypointYaml(path, m_frameId, WaypointMap(), single, errorMessage)) {
        qWarning() << "경로 백업 저장 실패:" << errorMessage;
    }
}

void WaypointManagementPage::handleClearMap()
{
    m_waypointMap->clearBackgroundImage();
    m_routeMap->clearBackgroundImage();
    m_currentMapMeta = MapMeta();
    m_mapStatusLabel->setText(QStringLiteral("불러온 지도 없음"));
    refreshMapMarkers();
}
