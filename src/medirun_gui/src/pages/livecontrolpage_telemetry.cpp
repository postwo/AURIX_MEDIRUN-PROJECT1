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

void LiveControlPage::handleTelemetrySnapshot(const TelemetrySnapshot &snapshot)
{
    m_lastBatteryPercent = snapshot.batteryPercent;
    if (m_batteryValueLabel == nullptr) {
        return;
    }

    const int displayPercent = snapshot.batteryPercent < 0 ? 0 : snapshot.batteryPercent;
    m_batteryValueLabel->setText(snapshot.batteryPercent < 0 ? QStringLiteral("--") : QStringLiteral("%1%").arg(snapshot.batteryPercent));
    const QColor color = displayPercent < 20 ? DashboardPalette::Red
        : displayPercent < 40 ? DashboardPalette::Amber
        : DashboardPalette::Green;
    setLabelColor(m_batteryValueLabel, color);

    // 값이 실제로 바뀔 때만 막대 위젯을 새로 만든다 — 매 스냅샷마다 다시 만들면
    // (배터리 %가 그대로여도) 위젯이 계속 교체되면서 깜빡여 보인다.
    if (displayPercent != m_lastDisplayedBatteryPercent) {
        m_lastDisplayedBatteryPercent = displayPercent;
        replaceBatteryBar(displayPercent);
    }
}

void LiveControlPage::replaceBatteryBar(int percent)
{
    if (m_batteryBarHost == nullptr || m_batteryBar == nullptr) {
        return;
    }
    const QColor color = percent < 20 ? DashboardPalette::Red : percent < 40 ? DashboardPalette::Amber : DashboardPalette::Green;
    auto *newBar = makeProportionalBar(percent, color, m_batteryBarHost);
    auto *layout = qobject_cast<QVBoxLayout *>(m_batteryBarHost->layout());
    if (layout != nullptr) {
        layout->replaceWidget(m_batteryBar, newBar);
    }
    delete m_batteryBar;
    m_batteryBar = newBar;
}

void LiveControlPage::handleFrontDistanceChanged(int centimeters)
{
    if (m_obstacleValueLabel == nullptr) {
        return;
    }
    m_obstacleValueLabel->setText(QStringLiteral("%1cm").arg(centimeters));
    setLabelColor(m_obstacleValueLabel, centimeters < 20 ? DashboardPalette::Red : DashboardPalette::TextPrimary);
}

void LiveControlPage::handleRearDistanceChanged(int centimeters)
{
    if (m_rearObstacleValueLabel == nullptr) {
        return;
    }
    m_rearObstacleValueLabel->setText(QStringLiteral("%1cm").arg(centimeters));
    setLabelColor(m_rearObstacleValueLabel, centimeters < 20 ? DashboardPalette::Red : DashboardPalette::TextPrimary);
}

void LiveControlPage::handlePayloadLoadedChanged(bool loaded)
{
    if (m_payloadValueLabel == nullptr) {
        return;
    }
    m_payloadValueLabel->setText(loaded ? QStringLiteral("적재됨") : QStringLiteral("미적재"));
    setLabelColor(m_payloadValueLabel, loaded ? DashboardPalette::Green : DashboardPalette::TextMuted);
}

void LiveControlPage::handleNav2ConnectionChanged(bool connected)
{
    m_lastNav2Connected = connected;
    updateNav2StatusLabel();
}

void LiveControlPage::updateNav2StatusLabel()
{
    if (m_nav2ValueLabel == nullptr) {
        return;
    }
    m_nav2ValueLabel->setText(m_lastNav2Connected ? QStringLiteral("연결됨") : QStringLiteral("연결 안 됨"));
    setLabelColor(m_nav2ValueLabel, m_lastNav2Connected ? DashboardPalette::Green : DashboardPalette::Red);
}
