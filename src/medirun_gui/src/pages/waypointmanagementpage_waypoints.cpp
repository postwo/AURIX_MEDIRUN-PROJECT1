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

void WaypointManagementPage::handleNewWaypointRequested()
{
    prefillNewWaypointForm(0.0, 0.0, 0.0);
}

void WaypointManagementPage::prefillNewWaypointForm(double x, double y, double yawDeg)
{
    m_creatingNew = true;
    m_selectedWaypointId.clear();
    m_waypointList->clearSelection();
    m_waypointMap->setSelectedId(QString());

    m_editPanelTitle->setText(QStringLiteral("새 웨이포인트 추가"));
    m_nameEdit->clear();
    if (m_callZoneIdCombo != nullptr) {
        m_callZoneIdCombo->setCurrentIndex(0);
        m_callZoneIdCombo->setEnabled(true);
    }
    m_xEdit->setText(QString::number(x, 'f', 2));
    m_yEdit->setText(QString::number(y, 'f', 2));
    m_thetaEdit->setText(QString::number(yawDeg, 'f', 0));
    m_kindCombo->setCurrentIndex(m_kindCombo->findData(static_cast<int>(WaypointMarkerKind::Waypoint)));
    m_saveWaypointButton->setText(QStringLiteral("웨이포인트 추가"));
    m_deleteWaypointButton->setEnabled(false);
    m_nameEdit->setFocus();
    updateNavButtonsEnabled();
}

void WaypointManagementPage::handleSaveWaypoint()
{
    // 이전 시도에서 남은 에러 강조(빨간 글씨/테두리)가 있으면 새 시도 시작 시
    // 먼저 지운다 — 이번에 검증을 통과하면 그대로 기본 스타일로 유지된다.
    m_waypointStatusLabel->setStyleSheet(QString());
    if (m_callZoneIdCombo != nullptr) {
        m_callZoneIdCombo->setStyleSheet(QString());
    }

    const QString name = m_nameEdit->text().trimmed();
    if (name.isEmpty()) {
        m_waypointStatusLabel->setText(QStringLiteral("이름을 입력하세요."));
        return;
    }

    bool xOk = false;
    bool yOk = false;
    const double x = m_xEdit->text().trimmed().toDouble(&xOk);
    const double y = m_yEdit->text().trimmed().toDouble(&yOk);
    const double thetaDeg = m_thetaEdit->text().trimmed().toDouble();
    if (!xOk || !yOk) {
        m_waypointStatusLabel->setText(QStringLiteral("X, Y 좌표는 숫자로 입력해야 합니다."));
        return;
    }

    const auto kind = static_cast<WaypointMarkerKind>(m_kindCombo->currentData().toInt());

    if (m_creatingNew) {
        if (kind == WaypointMarkerKind::Home) {
            const bool homeExists = std::any_of(m_waypoints.cbegin(), m_waypoints.cend(),
                                                 [](const WaypointDef &wp) { return wp.kind == WaypointMarkerKind::Home; });
            if (homeExists) {
                m_waypointStatusLabel->setText(QStringLiteral("HOME은 하나만 존재할 수 있습니다."));
                return;
            }
        }

        // 콜존은 감지된 콜존 ID 드롭다운에서 고른 값이 있으면 그걸 그대로
        // code로 쓴다(ESP32가 실제로 보내는 값과 정확히 일치시키기 위함) —
        // 자동 채번(BTN_01, BTN_02...)에 의존하면 삭제 후 재생성 시 하드웨어와
        // 어긋날 수 있다. "선택 안 함"이면 기존처럼 자동 채번.
        QString callZoneCode;
        if (kind == WaypointMarkerKind::CallZone && m_callZoneIdCombo != nullptr && m_callZoneIdCombo->currentIndex() > 0) {
            callZoneCode = m_callZoneIdCombo->currentText().trimmed();
            if (!callZoneCode.isEmpty() && m_waypoints.contains(callZoneCode)) {
                m_waypointStatusLabel->setStyleSheet(
                    QStringLiteral("color: %1; font-weight: 600;").arg(DashboardPalette::Red.name()));
                m_waypointStatusLabel->setText(QStringLiteral("이미 등록된 콜존 ID입니다: %1").arg(callZoneCode));
                m_callZoneIdCombo->setStyleSheet(
                    QStringLiteral("QComboBox { border: 1px solid %1; }").arg(DashboardPalette::Red.name()));
                return;
            }
        }

        WaypointDef record;
        record.name = !callZoneCode.isEmpty() ? callZoneCode : nextIdForKind(kind);
        record.label = name;
        record.x = x;
        record.y = y;
        record.yaw = qDegreesToRadians(thetaDeg);
        record.kind = kind;
        m_waypoints.insert(record.name, record);
        m_selectedWaypointId = record.name;
        m_creatingNew = false;
        m_waypointStatusLabel->setText(QStringLiteral("%1 웨이포인트를 추가했습니다.").arg(record.name));
    } else {
        WaypointDef &wp = m_waypoints[m_selectedWaypointId];
        wp.label = name;
        wp.x = x;
        wp.y = y;
        wp.yaw = qDegreesToRadians(thetaDeg);
        wp.kind = kind;
        m_waypointStatusLabel->setText(QStringLiteral("%1 웨이포인트를 수정했습니다.").arg(m_selectedWaypointId));
    }

    // 콜존/경유점/홈 모두 저장 즉시 각자의 DB 소스에 개별 반영한다 — call_button
    // 은 이전부터 이랬고, 이번에 경유점(waypoint_set type='waypoint')과
    // 홈(waypoint_set type='home')도 같은 방식으로 맞췄다.
    const WaypointDef saved = m_waypoints.value(m_selectedWaypointId);
    if (kind == WaypointMarkerKind::CallZone) {
        int dbId = 0;
        QString dbErrorMessage;
        if (!m_overviewService.upsertCallButton(saved.name, saved.label, saved.x, saved.y, saved.yaw, dbId, dbErrorMessage)) {
            qWarning() << "call_button upsert 실패:" << dbErrorMessage;
        } else {
            // 저장된 행의 실제 PK를 바로 붙여둔다 — 페이지를 새로고침하지 않고
            // 바로 경로에 추가해도 trajectory 저장이 이 id를 쓸 수 있게.
            m_waypoints[m_selectedWaypointId].dbId = dbId;
        }
    } else if (kind == WaypointMarkerKind::Waypoint) {
        const QVector<RobotRecord> robots = m_robotService.listRobots(false);
        if (robots.size() == 1) {
            int dbId = 0;
            QString dbErrorMessage;
            if (!m_overviewService.upsertWaypointEntry(
                    robots.first().id, saved.name, saved.label, saved.x, saved.y, saved.yaw, dbId, dbErrorMessage)) {
                qWarning() << "경유점 등록 실패:" << dbErrorMessage;
            } else {
                m_waypoints[m_selectedWaypointId].dbId = dbId;
                backupWaypointEntryToFile(saved);
            }
        }
    } else if (kind == WaypointMarkerKind::Home) {
        const QVector<RobotRecord> robots = m_robotService.listRobots(false);
        if (robots.size() == 1) {
            WaypointMap homeMap;
            homeMap.insert(saved.name, saved);
            QString dbErrorMessage;
            if (!m_overviewService.saveWaypointLibrary(
                    robots.first().id, QStringLiteral("home"), m_frameId, homeMap, TrajectoryMap(), dbErrorMessage)) {
                qWarning() << "홈 저장 실패:" << dbErrorMessage;
            } else {
                // saveWaypointLibrary는 PK를 안 돌려주므로, 방금 저장된 행을
                // 다시 읽어와서 dbId를 채운다(경로에 홈을 바로 추가할 수 있게).
                mergeHomeWaypointInto(m_waypoints);
            }
        }
    }

    refreshWaypointList();
    refreshMapMarkers();
    refreshRouteCombo();
    renumberRouteOrderItems();
    pushToRepository();
    selectWaypoint(m_selectedWaypointId);
}

void WaypointManagementPage::handleDeleteWaypoint()
{
    if (m_selectedWaypointId.isEmpty()) {
        return;
    }

    // 이전 시도에서 남은 에러 강조(빨간 글씨)가 있으면 새 시도 시작 시 지운다.
    m_waypointStatusLabel->setStyleSheet(QString());

    const QString deletedId = m_selectedWaypointId;
    const WaypointDef deletedRecord = m_waypoints.value(deletedId);

    // 저장된 경로가 이 웨이포인트/콜존을 스텝으로 쓰고 있으면 삭제를 막는다
    // — 삭제해도 trajectory 행 자체는 안 건드리므로, 막지 않으면 저장된
    // 경로가 나중에 조용히 그 스텝을 잃는다(화면엔 안 보이는데 DB엔 죽은
    // 참조가 남음).
    if (deletedRecord.dbId > 0
        && (deletedRecord.kind == WaypointMarkerKind::CallZone || deletedRecord.kind == WaypointMarkerKind::Waypoint)) {
        const QVector<RobotRecord> robots = m_robotService.listRobots(false);
        if (robots.size() == 1) {
            const QString table = (deletedRecord.kind == WaypointMarkerKind::CallZone)
                ? QStringLiteral("call_button")
                : QStringLiteral("waypoint");
            QStringList referencingTrajectories;
            QString checkErrorMessage;
            if (m_overviewService.findTrajectoriesReferencing(
                    robots.first().id, table, deletedRecord.dbId, referencingTrajectories, checkErrorMessage)) {
                if (!referencingTrajectories.isEmpty()) {
                    m_waypointStatusLabel->setStyleSheet(
                        QStringLiteral("color: %1; font-weight: 600;").arg(DashboardPalette::Red.name()));
                    m_waypointStatusLabel->setText(QStringLiteral("삭제할 수 없습니다 — 저장된 경로에서 사용 중: %1")
                                                        .arg(referencingTrajectories.join(QStringLiteral(", "))));
                    return;
                }
            } else {
                qWarning() << "경로 참조 확인 실패:" << checkErrorMessage;
            }
        }
    }

    const auto confirm = QMessageBox::question(
        this, QStringLiteral("웨이포인트 삭제"), QStringLiteral("%1 웨이포인트를 삭제하시겠습니까?").arg(deletedId));
    if (confirm != QMessageBox::Yes) {
        return;
    }

    const WaypointMarkerKind deletedKind = deletedRecord.kind;
    m_waypoints.remove(deletedId);

    for (int i = m_routeOrderList->count() - 1; i >= 0; --i) {
        if (m_routeOrderList->item(i)->data(Qt::UserRole).toString() == deletedId) {
            delete m_routeOrderList->takeItem(i);
        }
    }

    if (deletedKind == WaypointMarkerKind::CallZone) {
        // 콜존 삭제는 로컬뿐 아니라 call_button도 같이 등록 해제한다 — 안 그러면
        // 원격제어/실시간관제가 여전히 이 버튼을 보여준다.
        QString dbErrorMessage;
        if (!m_overviewService.deregisterCallButton(deletedId, dbErrorMessage)) {
            qWarning() << "call_button 등록 해제 실패:" << dbErrorMessage;
        }
    } else if (deletedKind == WaypointMarkerKind::Waypoint) {
        const QVector<RobotRecord> robots = m_robotService.listRobots(false);
        if (robots.size() == 1) {
            QString dbErrorMessage;
            if (!m_overviewService.deregisterWaypointEntry(robots.first().id, deletedId, dbErrorMessage)) {
                qWarning() << "경유점 등록 해제 실패:" << dbErrorMessage;
            }
        }
    } else if (deletedKind == WaypointMarkerKind::Home) {
        const QVector<RobotRecord> robots = m_robotService.listRobots(false);
        if (robots.size() == 1) {
            QString dbErrorMessage;
            if (!m_overviewService.deregisterHomeEntry(robots.first().id, dbErrorMessage)) {
                qWarning() << "홈 등록 해제 실패:" << dbErrorMessage;
            }
        }
    }

    m_waypointStatusLabel->setText(QStringLiteral("%1 웨이포인트를 삭제했습니다.").arg(deletedId));
    refreshWaypointList();
    refreshMapMarkers();
    refreshRouteCombo();
    renumberRouteOrderItems();
    pushToRepository();

    if (!m_waypoints.isEmpty()) {
        selectWaypoint(m_waypoints.firstKey());
    } else {
        handleNewWaypointRequested();
    }
}

void WaypointManagementPage::handleMapPickRequested(const QPointF &normalizedPos, double yawRadians)
{
    if (!m_currentMapMeta.valid) {
        m_waypointStatusLabel->setText(QStringLiteral("먼저 지도를 불러오세요."));
        return;
    }
    const QPointF world = normalizedToWorld(normalizedPos);

    if (m_settingInitialPose) {
        // Unchecking triggers handleSetInitialPoseToggled(false), which
        // resets the flag and restores the waypoint-picking caption.
        m_setInitialPoseButton->setChecked(false);
        if (m_navigationService == nullptr) {
            m_waypointStatusLabel->setText(QStringLiteral("Nav2 연동이 준비되지 않았습니다."));
            return;
        }
        m_navigationService->publishInitialPose(world.x(), world.y(), yawRadians, m_frameId);
        m_waypointStatusLabel->setText(QStringLiteral("초기 위치를 설정했습니다: x=%1, y=%2")
                                            .arg(world.x(), 0, 'f', 2)
                                            .arg(world.y(), 0, 'f', 2));
        return;
    }

    prefillNewWaypointForm(world.x(), world.y(), qRadiansToDegrees(yawRadians));
}

void WaypointManagementPage::handleSetInitialPoseToggled(bool checked)
{
    m_settingInitialPose = checked;
    m_waypointMap->setPoseSettingEnabled(checked);
    if (checked) {
        selectTab(0);
        m_waypointMap->setCaption(
            QStringLiteral("지도 클릭 + 드래그 → 로봇의 실제 위치·방향으로 초기 위치 설정 (RViz 2D Pose Estimate와 동일)"));
    } else {
        m_waypointMap->setCaption(QStringLiteral("지도 클릭 → 웨이포인트 생성 · 드래그 → 방향 지정 · 마커 클릭 → 편집"));
    }
}

void WaypointManagementPage::showSpeedPopup()
{
    // Qt::Popup auto-closes on outside click/focus loss — matches the
    // "click the chip → adjust in a small tooltip-like panel" behavior
    // requested, without needing a modal dialog.
    auto *popup = new QWidget(this, Qt::Popup);
    popup->setAttribute(Qt::WA_DeleteOnClose);
    popup->setObjectName(QStringLiteral("speedPopup"));
    popup->setFixedWidth(190);

    auto *layout = new QVBoxLayout(popup);
    layout->setContentsMargins(14, 12, 14, 12);
    layout->setSpacing(6);

    layout->addWidget(makeLabel(QStringLiteral("웨이포인트 주행 속도"), QStringLiteral("metricCaption")));

    auto *valueLabel = makeLabel(QStringLiteral("%1%").arg(m_speedPercent), QStringLiteral("panelTitle"));
    valueLabel->setAlignment(Qt::AlignCenter);
    layout->addWidget(valueLabel);

    auto *slider = new QSlider(Qt::Horizontal, popup);
    slider->setRange(10, 100);
    slider->setSingleStep(5);
    slider->setPageStep(10);
    slider->setValue(m_speedPercent);
    layout->addWidget(slider);

    connect(slider, &QSlider::valueChanged, this, [this, valueLabel](int value) {
        valueLabel->setText(QStringLiteral("%1%").arg(value));
        applySpeedLimit(value);
    });

    popup->move(m_speedButton->mapToGlobal(QPoint(0, m_speedButton->height() + 4)));
    popup->show();
}

void WaypointManagementPage::applySpeedLimit(int percent)
{
    m_speedPercent = percent;
    m_speedButton->setText(QStringLiteral("주행 속도 %1%").arg(percent));
    if (m_navigationService != nullptr) {
        m_navigationService->setSpeedLimit(percent);
    }
}

QString WaypointManagementPage::nextIdForKind(WaypointMarkerKind kind) const
{
    if (kind == WaypointMarkerKind::Home) {
        return QStringLiteral("HOME");
    }

    int maxNum = 0;
    const QStringList ids = m_waypoints.keys();
    for (const QString &id : ids) {
        const WaypointMarkerKind existingKind = m_waypoints.value(id).kind;
        if (existingKind != kind) {
            continue;
        }
        if (kind == WaypointMarkerKind::CallZone && id.startsWith(QStringLiteral("BTN_"))) {
            maxNum = std::max(maxNum, id.mid(4).toInt());
        } else if (kind == WaypointMarkerKind::Waypoint && id.startsWith(QStringLiteral("W-"))) {
            maxNum = std::max(maxNum, id.mid(2).toInt());
        }
    }

    if (kind == WaypointMarkerKind::CallZone) {
        return QStringLiteral("BTN_%1").arg(maxNum + 1, 2, 10, QChar('0'));
    }
    return QStringLiteral("W-%1").arg(maxNum + 1, 2, 10, QChar('0'));
}

QPointF WaypointManagementPage::normalizedFromMeters(double x, double y) const
{
    const double nx = std::clamp(0.5 + x / 20.0, 0.06, 0.94);
    const double ny = std::clamp(0.5 - y / 12.0, 0.06, 0.94);
    return QPointF(nx, ny);
}

QPointF WaypointManagementPage::worldToNormalized(double x, double y) const
{
    if (!m_currentMapMeta.valid) {
        return normalizedFromMeters(x, y);
    }
    const QPointF pixel = worldToPixel(QPointF(x, y), m_currentMapMeta);
    return QPointF(pixel.x() / m_currentMapMeta.width, pixel.y() / m_currentMapMeta.height);
}

QPointF WaypointManagementPage::normalizedToWorld(const QPointF &normalized) const
{
    if (!m_currentMapMeta.valid) {
        // 지도 없이 호출되지 않도록 handleMapPickRequested에서 가드하지만,
        // 방어적으로 역변환 근사치를 반환한다.
        const double x = (normalized.x() - 0.5) * 20.0;
        const double y = (0.5 - normalized.y()) * 12.0;
        return QPointF(x, y);
    }
    const QPointF pixel(normalized.x() * m_currentMapMeta.width, normalized.y() * m_currentMapMeta.height);
    return pixelToWorld(pixel, m_currentMapMeta);
}
