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

void WaypointManagementPage::selectTab(int index)
{
    m_tabStack->setCurrentIndex(index);
    m_waypointTabButton->setObjectName(index == 0 ? QStringLiteral("tabButtonActive") : QStringLiteral("tabButton"));
    m_trajectoryTabButton->setObjectName(index == 1 ? QStringLiteral("tabButtonActive") : QStringLiteral("tabButton"));
    for (QPushButton *button : {m_waypointTabButton, m_trajectoryTabButton}) {
        button->style()->unpolish(button);
        button->style()->polish(button);
        button->update();
    }
}

void WaypointManagementPage::refreshWaypointList()
{
    m_waypointList->clear();
    const QStringList ids = m_waypoints.keys();
    for (const QString &id : ids) {
        const WaypointDef &wp = m_waypoints.value(id);
        auto *item = new QListWidgetItem(m_waypointList);
        item->setData(Qt::UserRole, id);
        item->setSizeHint(QSize(0, 30));

        auto *row = new QWidget;
        auto *rowLayout = new QHBoxLayout(row);
        rowLayout->setContentsMargins(4, 2, 4, 2);
        rowLayout->setSpacing(6);

        auto *nameLabel = makeLabel(QStringLiteral("%1  %2").arg(id, wp.label));
        nameLabel->setStyleSheet(QStringLiteral("font-size: 11px;"));
        rowLayout->addWidget(nameLabel, 1);
        rowLayout->addWidget(makeBadgeLabel(kindLabel(wp.kind), QStringLiteral("muted"), row));

        m_waypointList->setItemWidget(item, row);
    }
    m_waypointListCountLabel->setText(QStringLiteral("웨이포인트 · %1개").arg(m_waypoints.size()));
}

void WaypointManagementPage::refreshMapMarkers()
{
    QVector<WaypointMapMarker> markers;
    const QStringList ids = m_waypoints.keys();
    for (const QString &id : ids) {
        const WaypointDef &wp = m_waypoints.value(id);
        markers.append({id, id, worldToNormalized(wp.x, wp.y), wp.kind, wp.yaw});
    }
    m_waypointMap->setMarkers(markers);
    m_routeMap->setMarkers(markers);
}

void WaypointManagementPage::refreshRouteCombo()
{
    QSet<QString> inRoute;
    for (int i = 0; i < m_routeOrderList->count(); ++i) {
        inRoute.insert(m_routeOrderList->item(i)->data(Qt::UserRole).toString());
    }

    const QString previousSelection = m_routeAddCombo->currentData().toString();
    m_routeAddCombo->clear();
    const QStringList ids = m_waypoints.keys();
    for (const QString &id : ids) {
        if (!inRoute.contains(id)) {
            m_routeAddCombo->addItem(QStringLiteral("%1  %2").arg(id, m_waypoints.value(id).label), id);
        }
    }
    const int idx = m_routeAddCombo->findData(previousSelection);
    if (idx >= 0) {
        m_routeAddCombo->setCurrentIndex(idx);
    }
}

void WaypointManagementPage::renumberRouteOrderItems()
{
    static const QStringList circled = {
        QStringLiteral("①"), QStringLiteral("②"), QStringLiteral("③"), QStringLiteral("④"), QStringLiteral("⑤"),
        QStringLiteral("⑥"), QStringLiteral("⑦"), QStringLiteral("⑧"), QStringLiteral("⑨"), QStringLiteral("⑩"),
    };

    QVector<QString> order;
    for (int i = 0; i < m_routeOrderList->count(); ++i) {
        QListWidgetItem *item = m_routeOrderList->item(i);
        const QString id = item->data(Qt::UserRole).toString();
        order.append(id);

        const QString name = m_waypoints.contains(id) ? m_waypoints.value(id).label : id;
        const QString number = i < circled.size() ? circled.at(i) : QString::number(i + 1);
        item->setText(QStringLiteral("%1  %2  %3").arg(number, id, name));
    }

    m_routeOrderCountLabel->setText(QStringLiteral("경로 순서 · %1개").arg(order.size()));
    m_routeMap->setRouteOrder(order);
}

void WaypointManagementPage::refreshSavedRoutesBar()
{
    auto *layout = qobject_cast<QHBoxLayout *>(m_savedRoutesBar->layout());
    QLayoutItem *child = nullptr;
    QVector<QLayoutItem *> keep;
    // 처음 두 아이템(주행/정지 버튼)은 그대로 두고 나머지(칩 목록)만 다시 그린다.
    QWidget *navButton = layout->takeAt(0)->widget();
    QWidget *cancelButton = layout->takeAt(0)->widget();
    while ((child = layout->takeAt(0)) != nullptr) {
        delete child->widget();
        delete child;
    }

    layout->addWidget(navButton);
    layout->addWidget(cancelButton);

    auto *divider = new QFrame(m_savedRoutesBar);
    divider->setObjectName(QStringLiteral("sectionDivider"));
    divider->setFixedWidth(1);
    divider->setFixedHeight(22);
    layout->addWidget(divider);

    layout->addWidget(makeLabel(QStringLiteral("저장된 경로"), QStringLiteral("metricCaption")));

    const QStringList names = m_trajectories.keys();
    for (const QString &name : names) {
        auto *chip = new QPushButton(QStringLiteral("%1 · %2점").arg(name).arg(m_trajectories.value(name).size()), m_savedRoutesBar);
        chip->setObjectName(name == m_selectedSavedRoute ? QStringLiteral("routeChipActive") : QStringLiteral("routeChip"));
        connect(chip, &QPushButton::clicked, this, [this, name]() { handleSavedRouteChipClicked(name); });
        layout->addWidget(chip);
    }

    layout->addStretch();

    auto *deleteButton = new QPushButton(QStringLiteral("선택 삭제"), m_savedRoutesBar);
    deleteButton->setObjectName(QStringLiteral("inlineGhostButton"));
    connect(deleteButton, &QPushButton::clicked, this, &WaypointManagementPage::handleDeleteSelectedSavedRoute);
    layout->addWidget(deleteButton);

    updateNavButtonsEnabled();
}

void WaypointManagementPage::selectWaypoint(const QString &id)
{
    if (!m_waypoints.contains(id)) {
        return;
    }
    m_waypointStatusLabel->setStyleSheet(QString());

    m_creatingNew = false;
    m_selectedWaypointId = id;
    loadWaypointIntoForm(m_waypoints.value(id));
    m_waypointMap->setSelectedId(id);

    for (int i = 0; i < m_waypointList->count(); ++i) {
        QListWidgetItem *item = m_waypointList->item(i);
        if (item->data(Qt::UserRole).toString() == id) {
            m_waypointList->setCurrentItem(item);
            break;
        }
    }
    updateNavButtonsEnabled();
}

void WaypointManagementPage::loadWaypointIntoForm(const WaypointDef &record)
{
    m_editPanelTitle->setText(QStringLiteral("%1 편집").arg(record.name));
    m_nameEdit->setText(record.label);
    m_xEdit->setText(QString::number(record.x, 'f', 2));
    m_xEdit->setCursorPosition(0);
    m_yEdit->setText(QString::number(record.y, 'f', 2));
    m_yEdit->setCursorPosition(0);
    m_thetaEdit->setText(QString::number(qRadiansToDegrees(record.yaw), 'f', 0));
    m_thetaEdit->setCursorPosition(0);
    m_kindCombo->setCurrentIndex(m_kindCombo->findData(static_cast<int>(record.kind)));
    m_saveWaypointButton->setText(QStringLiteral("저장"));
    m_deleteWaypointButton->setEnabled(true);
    if (m_callZoneIdCombo != nullptr) {
        // 이미 등록된 콜존의 code는 저장 시 바꾸지 않는다(핸들러가 m_creatingNew일
        // 때만 드롭다운 값을 읽음) — 골라도 반영 안 되는 걸 막기 위해 편집 중엔
        // 아예 비활성화하되, refreshCallZoneIdCombo가 m_selectedWaypointId
        // 기준으로 지금 이 웨이포인트의 실제 code를 그대로 보여준다.
        refreshCallZoneIdCombo();
        m_callZoneIdCombo->setEnabled(false);
    }
}
