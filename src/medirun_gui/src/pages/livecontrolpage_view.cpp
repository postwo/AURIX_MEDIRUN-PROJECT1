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

void LiveControlPage::refreshMissionStepper()
{
    if (m_missionInfoLabel == nullptr) {
        return;
    }

    if (m_missionController == nullptr) {
        m_missionInfoLabel->setText(QStringLiteral("대기 중"));
        m_missionMetaLabel->clear();
    } else {
        const MissionSnapshot snapshot = m_missionController->missionSnapshot();
        if (snapshot.missionId.isEmpty()) {
            m_missionInfoLabel->setText(QStringLiteral("대기 중"));
            m_missionMetaLabel->clear();
        } else {
            m_missionInfoLabel->setText(QStringLiteral("%1 · %2").arg(snapshot.missionId, snapshot.destinationName));
            const QString elapsed = QStringLiteral("%1:%2")
                                         .arg(snapshot.elapsedSeconds / 60, 2, 10, QChar('0'))
                                         .arg(snapshot.elapsedSeconds % 60, 2, 10, QChar('0'));
            const QString remaining = snapshot.distanceRemainingMeters < 0.0
                ? QStringLiteral("--")
                : QString::number(snapshot.distanceRemainingMeters, 'f', 1);
            m_missionMetaLabel->setText(QStringLiteral("경과 %1 · 남은 %2m").arg(elapsed, remaining));
        }

        const QVector<QString> states = computeStepStates(snapshot);
        for (int i = 0; i < m_stepCircles.size() && i < states.size(); ++i) {
            QLabel *circle = m_stepCircles.at(i);
            if (states.at(i) == QStringLiteral("done")) {
                circle->setText(QStringLiteral("✓"));
            } else if (states.at(i) == QStringLiteral("failed")) {
                circle->setText(QStringLiteral("✕"));
            } else if (states.at(i) == QStringLiteral("cancelled")) {
                circle->setText(QStringLiteral("■"));
            } else {
                circle->setText(QString());
            }
            circle->setProperty("stepState", states.at(i));
            circle->style()->unpolish(circle);
            circle->style()->polish(circle);
        }
        for (int i = 0; i < m_stepConnectors.size(); ++i) {
            const bool done = i < states.size() - 1 && states.at(i) == QStringLiteral("done") && states.at(i + 1) != QStringLiteral("pending");
            QFrame *connector = m_stepConnectors.at(i);
            connector->setObjectName(done ? QStringLiteral("stepConnectorDone") : QStringLiteral("stepConnectorPending"));
            connector->style()->unpolish(connector);
            connector->style()->polish(connector);
        }
        return;
    }

    for (QLabel *circle : m_stepCircles) {
        circle->setText(QString());
        circle->setProperty("stepState", QStringLiteral("pending"));
        circle->style()->unpolish(circle);
        circle->style()->polish(circle);
    }
    for (QFrame *connector : m_stepConnectors) {
        connector->setObjectName(QStringLiteral("stepConnectorPending"));
        connector->style()->unpolish(connector);
        connector->style()->polish(connector);
    }
}

void LiveControlPage::refreshWaitingCallBanner()
{
    if (m_waitingCallLabel == nullptr) {
        return;
    }
    // waitingQueue()는 Completed/Cancelled 등 종료된 항목도 그대로 담고 있어서
    // (미션 관제 페이지의 큐 요약과 동일하게) Waiting/Active 상태만 센다.
    int count = 0;
    if (m_missionController != nullptr) {
        for (const MissionQueueItem &item : m_missionController->waitingQueue()) {
            if (item.status == MissionQueueStatus::Waiting || item.status == MissionQueueStatus::Active) {
                ++count;
            }
        }
    }
    m_waitingCallLabel->setText(QStringLiteral("대기 호출 %1건").arg(count));
}

void LiveControlPage::refreshEventStream()
{
    if (m_eventStreamLayout == nullptr || m_operationsOverviewService == nullptr) {
        return;
    }

    QVector<OperationsEventRow> rows;
    QString errorMessage;
    // 6건만 가져오면 항목이 패널 높이 안에 다 들어가버려서 스크롤이 생길
    // 일이 거의 없다 — "오늘 처리 이력"처럼 실제로 스크롤이 필요할 만큼
    // 더 넉넉히 가져온다.
    if (!m_operationsOverviewService->fetchRecentEvents(30, rows, errorMessage)) {
        return;
    }

    clearVBoxLayout(m_eventStreamLayout);
    if (rows.isEmpty()) {
        m_eventStreamLayout->addWidget(makeLabel(QStringLiteral("이벤트가 없습니다."), QStringLiteral("metricCaption")));
        return;
    }
    // 최신 이벤트가 맨 위에 오도록 표시 직전에 명시적으로 정렬한다.
    std::sort(rows.begin(), rows.end(), [](const OperationsEventRow &a, const OperationsEventRow &b) {
        return a.occurredAt > b.occurredAt;
    });
    for (const OperationsEventRow &event : rows) {
        m_eventStreamLayout->addWidget(buildEventRow(event));
    }
    m_eventStreamLayout->addStretch();
}

void LiveControlPage::loadDefaultMap()
{
    if (m_waypointRepository == nullptr) {
        return;
    }
    const QString path = m_waypointRepository->workspaceRootPath() + QStringLiteral("/maps/last_class_map_modi.yaml");
    MapMeta meta;
    QString errorMessage;
    if (!loadMapYaml(path, meta, errorMessage) || !meta.valid) {
        return;
    }
    m_mapMeta = meta;
    m_mapView->setBackgroundImage(meta.image);
    refreshMapMarkers();
}

QPointF LiveControlPage::worldToNormalized(double x, double y) const
{
    const QPointF pixel = worldToPixel(QPointF(x, y), m_mapMeta);
    return QPointF(pixel.x() / m_mapMeta.width, pixel.y() / m_mapMeta.height);
}

void LiveControlPage::refreshMapMarkers()
{
    // 원격제어 페이지(RemoteControlPage::refreshDbMarkers)와 동일한 규칙 —
    // 콜존/홈 마커는 공유 WaypointRepository 스냅샷이 아니라 항상 DB
    // (call_button / waypoint_set name="home")를 직접 재조회해서 그린다.
    // WaypointRepository는 미션 목적지 좌표 조회(refreshMap) 용도로만 남긴다.
    if (!m_mapMeta.valid || m_operationsOverviewService == nullptr) {
        return;
    }

    bool hasHome = false;
    QPointF homeNormalized;
    if (m_robotService != nullptr) {
        const QVector<RobotRecord> robots = m_robotService->listRobots(false);
        if (robots.size() == 1) {
            QString frameId;
            WaypointMap homeWaypoints;
            TrajectoryMap trajectories;
            QString homeErrorMessage;
            if (m_operationsOverviewService->loadWaypointLibraryByName(
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
    }
    m_mapView->setHome(homeNormalized, hasHome);

    QVector<CallButtonRow> callButtons;
    QString callButtonErrorMessage;
    QVector<QPointF> callZonePoints;
    QStringList callZoneLabels;
    if (m_operationsOverviewService->loadActiveCallButtons(callButtons, callButtonErrorMessage)) {
        callZonePoints.reserve(callButtons.size());
        callZoneLabels.reserve(callButtons.size());
        for (const CallButtonRow &row : std::as_const(callButtons)) {
            callZonePoints.append(worldToNormalized(row.x, row.y));
            callZoneLabels.append(row.zoneName.isEmpty() ? row.code : row.zoneName);
        }
    }
    m_mapView->setCallZones(callZonePoints, callZoneLabels);
}

void LiveControlPage::showEvent(QShowEvent *event)
{
    // 웨이포인트 관리 페이지에서 콜존/홈을 추가·수정·삭제하고 이 페이지로
    // 돌아왔을 때 바로 반영되도록, 페이지가 다시 보일 때마다 DB 마커를 새로
    // 읽어온다(RemoteControlPage::showEvent와 동일한 이유).
    refreshMapMarkers();
    QWidget::showEvent(event);
}

void LiveControlPage::refreshMap()
{
    refreshMapMarkers();

    if (m_missionController == nullptr || !m_mapMeta.valid) {
        m_mapView->setDestination(QPointF(), false);
        return;
    }
    const MissionSnapshot snapshot = m_missionController->missionSnapshot();
    if (snapshot.missionId.isEmpty() || m_waypointRepository == nullptr) {
        m_mapView->setDestination(QPointF(), false);
        return;
    }

    // MissionSnapshot은 표시용 라벨(destinationName)만 갖고 있고 원본 waypoint id는
    // 없어서, 라벨(없으면 id) 일치로 역으로 찾는다. 라벨이 겹치면 먼저 찾은 것을 쓴다.
    const WaypointMap waypoints = m_waypointRepository->waypoints();
    for (auto it = waypoints.cbegin(); it != waypoints.cend(); ++it) {
        const WaypointDef &def = it.value();
        const QString displayName = def.label.isEmpty() ? def.name : def.label;
        if (displayName == snapshot.destinationName || def.name == snapshot.destinationName) {
            m_mapView->setDestination(worldToNormalized(def.x, def.y));
            return;
        }
    }
    m_mapView->setDestination(QPointF(), false);
}

void LiveControlPage::handleAmclPoseUpdated(double x, double y, double yawRad)
{
    if (!m_mapMeta.valid) {
        return;
    }
    m_mapView->setRobotVisible(true);
    m_mapView->setRobotPose(worldToNormalized(x, y), qRadiansToDegrees(yawRad));
    m_mapView->setRobotState(RobotMarkerState::Moving);
}

void LiveControlPage::handleOdomVelocityUpdated(double linearMps, double /*angularRadps*/)
{
    // 지수이동평균(alpha=0.25)으로 다듬는다 — 화면 갱신 자체는 m_speedUpdateTimer가 맡는다.
    m_lastLinearVel = m_lastLinearVel * 0.75 + linearMps * 0.25;
}

void LiveControlPage::updateSpeedLabel()
{
    if (m_speedValueLabel == nullptr) {
        return;
    }
    m_speedValueLabel->setText(QStringLiteral("%1 m/s").arg(snapNearZero(m_lastLinearVel, 0.01), 0, 'f', 2));
}
