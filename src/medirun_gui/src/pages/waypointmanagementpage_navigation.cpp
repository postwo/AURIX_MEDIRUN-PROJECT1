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

void WaypointManagementPage::updateNavButtonsEnabled()
{
    const bool hasService = (m_navigationService != nullptr);
    // 실제 주행 가능하려면 (1) navigate_to_pose/follow_waypoints 액션 서버가
    // ACTIVE여야(m_nav2Connected) 하고 (2) AMCL이 초기 위치를 받아 로컬라이즈
    // 돼 있어야(m_hasAmclPose) 한다 — nav2Connected만으로는 goal을 받아줄 수는
    // 있어도 위치를 모르는 채로 주행하면 실패하거나 엉뚱하게 움직인다.
    // 초기 위치 설정 자체는 AMCL(localization)만 있으면 되므로 이 조건과
    // 무관하게 hasService만으로 활성화한다.
    const bool nav2Ready = hasService && m_nav2Connected && m_hasAmclPose;
    const QString blockedText = !hasService || !m_nav2Connected
        ? QStringLiteral("Nav2 부팅 중")
        : QStringLiteral("위치 설정 필요");
    if (m_navigateButton != nullptr) {
        m_navigateButton->setEnabled(nav2Ready && !m_navigating && !m_creatingNew && !m_selectedWaypointId.isEmpty());
        m_navigateButton->setText(
            m_navigating ? QStringLiteral("주행 중...") : (nav2Ready ? QStringLiteral("주행 시작") : blockedText));
    }
    if (m_cancelNavButton != nullptr) {
        m_cancelNavButton->setEnabled(hasService && m_navigating);
    }
    if (m_navigateRouteButton != nullptr) {
        m_navigateRouteButton->setEnabled(nav2Ready && !m_navigating && !m_selectedSavedRoute.isEmpty());
        m_navigateRouteButton->setText(
            m_navigating ? QStringLiteral("주행 중...") : (nav2Ready ? QStringLiteral("선택 경로 주행") : blockedText));
    }
    if (m_setInitialPoseButton != nullptr) {
        m_setInitialPoseButton->setEnabled(hasService);
        if (!hasService) {
            m_setInitialPoseButton->setChecked(false);
        }
    }
}

void WaypointManagementPage::refreshNav2Badge()
{
    if (m_nav2ConnectionBadge != nullptr) {
        QString text;
        QString kind;
        if (!m_nav2Connected) {
            text = QStringLiteral("Nav2 부팅 중");
            kind = QStringLiteral("amber");
        } else if (!m_hasAmclPose) {
            text = QStringLiteral("위치 설정 필요");
            kind = QStringLiteral("amber");
        } else {
            text = QStringLiteral("주행 가능");
            kind = QStringLiteral("green");
        }
        m_nav2ConnectionBadge->setText(text);
        m_nav2ConnectionBadge->setProperty("badgeKind", kind);
        m_nav2ConnectionBadge->style()->unpolish(m_nav2ConnectionBadge);
        m_nav2ConnectionBadge->style()->polish(m_nav2ConnectionBadge);
    }
    updateNavButtonsEnabled();
}

void WaypointManagementPage::handleNavigateToSelectedWaypoint()
{
    if (m_navigationService == nullptr) {
        m_waypointStatusLabel->setText(QStringLiteral("Nav2 연동이 준비되지 않았습니다."));
        return;
    }
    // 버튼이 비활성화돼 있어야 정상이지만(updateNavButtonsEnabled), 클릭과
    // 연결 끊김이 겹치는 race를 대비해 여기서도 한 번 더 막는다.
    if (!m_nav2Connected) {
        m_waypointStatusLabel->setText(QStringLiteral("Nav2가 아직 부팅 중입니다. 잠시 후 다시 시도하세요."));
        return;
    }
    if (!m_hasAmclPose) {
        m_waypointStatusLabel->setText(QStringLiteral("먼저 초기 위치를 설정하세요."));
        return;
    }
    if (m_creatingNew || !m_waypoints.contains(m_selectedWaypointId)) {
        m_waypointStatusLabel->setText(QStringLiteral("먼저 저장된 웨이포인트를 선택하세요."));
        return;
    }

    const WaypointDef &wp = m_waypoints.value(m_selectedWaypointId);
    const QString label = wp.label.isEmpty() ? wp.name : wp.label;
    m_activeRouteWaypointIds.clear();
    m_waypointMap->setNavigationTargetId(m_selectedWaypointId);
    m_routeMap->setNavigationTargetId(m_selectedWaypointId);
    m_navigationService->navigateToPose(wp.x, wp.y, wp.yaw, m_frameId, label);
    if (m_missionController != nullptr) {
        m_missionController->announceTestDrive(label);
    }
}

void WaypointManagementPage::handleNavigateSelectedRoute()
{
    if (m_navigationService == nullptr) {
        m_trajectoryStatusLabel->setText(QStringLiteral("Nav2 연동이 준비되지 않았습니다."));
        return;
    }
    if (!m_nav2Connected) {
        m_trajectoryStatusLabel->setText(QStringLiteral("Nav2가 아직 부팅 중입니다. 잠시 후 다시 시도하세요."));
        return;
    }
    if (!m_hasAmclPose) {
        m_trajectoryStatusLabel->setText(QStringLiteral("먼저 초기 위치를 설정하세요."));
        return;
    }
    if (m_selectedSavedRoute.isEmpty() || !m_trajectories.contains(m_selectedSavedRoute)) {
        m_trajectoryStatusLabel->setText(QStringLiteral("주행할 경로를 먼저 선택하세요."));
        return;
    }

    QVector<NavPose> poses;
    QVector<QString> orderedIds;
    for (const QString &wpId : m_trajectories.value(m_selectedSavedRoute)) {
        if (!m_waypoints.contains(wpId)) {
            continue;
        }
        const WaypointDef &wp = m_waypoints.value(wpId);
        poses.append({wp.x, wp.y, wp.yaw});
        orderedIds.append(wpId);
    }
    if (poses.isEmpty()) {
        m_trajectoryStatusLabel->setText(QStringLiteral("경로에 유효한 웨이포인트가 없습니다."));
        return;
    }

    // FollowWaypoints 피드백의 current_waypoint(0-based)가 이 목록의 인덱스와
    // 그대로 대응하도록 poses와 같은 순서로 채운다 — handleNavWaypointProgress에서
    // 지금 어느 웨이포인트로 가고 있는지 지도에 표시하는 데 쓴다.
    m_activeRouteWaypointIds = orderedIds;
    m_waypointMap->setNavigationTargetId(orderedIds.first());
    m_routeMap->setNavigationTargetId(orderedIds.first());

    m_navigationService->followWaypoints(poses, m_frameId, m_selectedSavedRoute);
    if (m_missionController != nullptr) {
        m_missionController->announceTestDrive(m_selectedSavedRoute);
    }
}

void WaypointManagementPage::handleCancelNavigation()
{
    if (m_navigationService == nullptr) {
        return;
    }
    m_navigationService->cancelGoal();
}

void WaypointManagementPage::handleNavStatusChanged(const QString &text)
{
    m_navStatusLabel->setText(text);
}

void WaypointManagementPage::handleNavSessionStarted()
{
    m_navigating = true;
    updateNavButtonsEnabled();
}

void WaypointManagementPage::handleNavSessionFinished()
{
    m_navigating = false;
    m_activeRouteWaypointIds.clear();
    m_waypointMap->setNavigationTargetId(QString());
    m_routeMap->setNavigationTargetId(QString());
    updateNavButtonsEnabled();
}

void WaypointManagementPage::handleNav2ConnectionChanged(bool connected)
{
    m_nav2Connected = connected;
    if (!connected) {
        // nav2가 다시 내려가면(SLAM 매핑 PAUSE 등) 이전에 알던 위치는 더 이상
        // 유효하다고 볼 수 없으니 같이 초기화한다 — 재연결 후 새 amclPoseUpdated를
        // 받기 전까지는 "위치 설정 필요" 단계로 되돌아간다.
        m_hasAmclPose = false;
    }
    refreshNav2Badge();
}

void WaypointManagementPage::handleNavFeedback(double metersRemaining)
{
    m_navStatusLabel->setText(QStringLiteral("목적지까지 남은 거리: %1 m").arg(metersRemaining, 0, 'f', 2));
}

void WaypointManagementPage::handleNavWaypointProgress(int currentIndex, int total)
{
    m_navStatusLabel->setText(QStringLiteral("경로 주행 중 (%1/%2)").arg(currentIndex).arg(total));

    // current_waypoint(0-based)가 지금 로봇이 향하는 웨이포인트 인덱스이므로,
    // 경로 진행에 맞춰 지도의 목표 마커도 같이 옮긴다.
    if (currentIndex >= 0 && currentIndex < m_activeRouteWaypointIds.size()) {
        const QString &targetId = m_activeRouteWaypointIds.at(currentIndex);
        m_waypointMap->setNavigationTargetId(targetId);
        m_routeMap->setNavigationTargetId(targetId);
    }
}

void WaypointManagementPage::handleAmclPoseUpdated(double x, double y, double yawRad)
{
    if (!m_hasAmclPose) {
        m_hasAmclPose = true;
        refreshNav2Badge();
    }

    if (!m_currentMapMeta.valid) {
        return;
    }
    const QPointF normalized = worldToNormalized(x, y);
    m_waypointMap->setRobotPose(normalized, yawRad, true);
    m_routeMap->setRobotPose(normalized, yawRad, true);
}

void WaypointManagementPage::handleScanPointsUpdated(const QVector<QPointF> &mapPoints)
{
    if (!m_currentMapMeta.valid) {
        return;
    }

    QVector<QPointF> normalized;
    normalized.reserve(mapPoints.size());
    for (const QPointF &point : mapPoints) {
        normalized.append(worldToNormalized(point.x(), point.y()));
    }

    m_waypointMap->setScanPoints(normalized);
    m_routeMap->setScanPoints(normalized);
}
