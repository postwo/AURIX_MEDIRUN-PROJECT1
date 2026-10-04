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

void WaypointManagementPage::handleAddToRoute()
{
    const QString id = m_routeAddCombo->currentData().toString();
    if (id.isEmpty()) {
        return;
    }

    auto *item = new QListWidgetItem;
    item->setData(Qt::UserRole, id);
    m_routeOrderList->addItem(item);

    refreshRouteCombo();
    renumberRouteOrderItems();
    m_trajectoryStatusLabel->setText(QStringLiteral("%1 을(를) 경로에 추가했습니다.").arg(id));
}

void WaypointManagementPage::handleRemoveSelectedFromRoute()
{
    QListWidgetItem *item = m_routeOrderList->currentItem();
    if (!item) {
        m_trajectoryStatusLabel->setText(QStringLiteral("제거할 항목을 목록에서 선택하세요."));
        return;
    }
    delete m_routeOrderList->takeItem(m_routeOrderList->row(item));
    refreshRouteCombo();
    renumberRouteOrderItems();
}

void WaypointManagementPage::handleClearRoute()
{
    m_routeOrderList->clear();
    refreshRouteCombo();
    renumberRouteOrderItems();
    m_trajectoryStatusLabel->setText(QStringLiteral("경로를 초기화했습니다."));
}

void WaypointManagementPage::handleSaveTrajectory()
{
    const QString name = m_routeNameEdit->text().trimmed();
    if (name.isEmpty()) {
        m_trajectoryStatusLabel->setText(QStringLiteral("경로 이름을 입력하세요."));
        return;
    }
    if (m_routeOrderList->count() < 2) {
        m_trajectoryStatusLabel->setText(QStringLiteral("경로에는 웨이포인트가 2개 이상 필요합니다."));
        return;
    }
    if (m_trajectories.contains(name)) {
        m_trajectoryStatusLabel->setText(QStringLiteral("이미 존재하는 경로 이름입니다."));
        return;
    }

    // 각 스텝을 DB PK(테이블+id)로 바로 변환한다 — 이름을 나중에 다시
    // 찾는 대신, 그 웨이포인트를 저장했을 때 이미 받아둔 dbId를 그대로 쓴다
    // (call_button/waypoint_set은 서로 독립된 PK 시퀀스라 이름 문자열로
    // 되찾으면 대소문자·중복 등으로 어긋나기 쉽다). 아직 저장 안 된
    // 웨이포인트(dbId 없음)가 섞여 있으면 저장 자체를 막는다.
    QStringList sequence;
    QVector<TrajectoryStep> steps;
    for (int i = 0; i < m_routeOrderList->count(); ++i) {
        const QString code = m_routeOrderList->item(i)->data(Qt::UserRole).toString();
        sequence.append(code);

        if (!m_waypoints.contains(code) || m_waypoints.value(code).dbId <= 0) {
            m_trajectoryStatusLabel->setText(
                QStringLiteral("'%1' 웨이포인트가 아직 DB에 저장되지 않아 경로에 추가할 수 없습니다.").arg(code));
            return;
        }
        const WaypointDef &stepDef = m_waypoints.value(code);
        TrajectoryStep step;
        step.table = (stepDef.kind == WaypointMarkerKind::CallZone) ? QStringLiteral("call_button") : QStringLiteral("waypoint");
        step.id = stepDef.dbId;
        steps.append(step);
    }
    m_trajectories.insert(name, sequence);

    // 경로도 경유점/콜존과 동일하게 저장 즉시 개별 DB row(waypoint_set
    // type='trajectory')로 반영하고, 로컬 백업 yaml도 자동 이름으로 남긴다.
    const QVector<RobotRecord> robots = m_robotService.listRobots(false);
    if (robots.size() == 1) {
        QString dbErrorMessage;
        if (!m_overviewService.upsertTrajectoryEntry(robots.first().id, name, steps, dbErrorMessage)) {
            // DB 반영 실패해도 화면(m_trajectories)엔 이미 들어간 상태라, 실패를
            // 콘솔 로그로만 남기면 사용자는 저장된 줄 알고 지나친다 — 상태
            // 라벨에도 그대로 보여준다.
            qWarning() << "경로 등록 실패:" << dbErrorMessage;
            m_trajectoryStatusLabel->setText(
                QStringLiteral("%1 경로를 화면에는 추가했지만 DB 저장 실패: %2").arg(name, dbErrorMessage));
        } else {
            backupTrajectoryToFile(name, sequence);
            m_trajectoryStatusLabel->setText(QStringLiteral("%1 경로를 Trajectory로 저장했습니다.").arg(name));
        }
    } else {
        m_trajectoryStatusLabel->setText(
            QStringLiteral("%1 경로를 화면에는 추가했지만 등록된 로봇이 1대가 아니어서 DB에는 저장하지 않음").arg(name));
    }

    m_routeNameEdit->clear();
    m_routeOrderList->clear();
    refreshRouteCombo();
    renumberRouteOrderItems();
    refreshSavedRoutesBar();
    pushToRepository();
}

void WaypointManagementPage::handleSavedRouteChipClicked(const QString &name)
{
    m_selectedSavedRoute = (m_selectedSavedRoute == name) ? QString() : name;
    refreshSavedRoutesBar();

    // 저장된 경로 칩을 고르면 지도(m_routeMap)에 순서대로 그려지도록, 그
    // 시퀀스를 경로 순서 목록에 채운다 — renumberRouteOrderItems가 번호/
    // 연결선을 실제로 그린다(그동안 이 목록은 "새 경로 만들기" 전용이라
    // 저장된 경로를 선택해도 지도에 반영되지 않고 있었다).
    m_routeOrderList->clear();
    if (!m_selectedSavedRoute.isEmpty() && m_trajectories.contains(m_selectedSavedRoute)) {
        for (const QString &id : m_trajectories.value(m_selectedSavedRoute)) {
            if (!m_waypoints.contains(id)) {
                continue;
            }
            auto *item = new QListWidgetItem;
            item->setData(Qt::UserRole, id);
            m_routeOrderList->addItem(item);
        }
    }
    refreshRouteCombo();
    renumberRouteOrderItems();
}

void WaypointManagementPage::handleDeleteSelectedSavedRoute()
{
    if (m_selectedSavedRoute.isEmpty()) {
        m_trajectoryStatusLabel->setText(QStringLiteral("삭제할 저장된 경로를 선택하세요."));
        return;
    }

    const QString deletedName = m_selectedSavedRoute;
    m_trajectories.remove(deletedName);
    m_selectedSavedRoute.clear();

    const QVector<RobotRecord> robots = m_robotService.listRobots(false);
    if (robots.size() == 1) {
        QString dbErrorMessage;
        if (!m_overviewService.deregisterTrajectoryEntry(robots.first().id, deletedName, dbErrorMessage)) {
            qWarning() << "경로 등록 해제 실패:" << dbErrorMessage;
        }
    }
    m_trajectoryStatusLabel->setText(QStringLiteral("%1 경로를 삭제했습니다.").arg(deletedName));
    refreshSavedRoutesBar();

    // 삭제한 경로가 handleSavedRouteChipClicked로 경로 순서 목록에 채워져
    // 지도에 그려져 있었을 수 있으므로 같이 지운다.
    m_routeOrderList->clear();
    refreshRouteCombo();
    renumberRouteOrderItems();
    pushToRepository();
}
