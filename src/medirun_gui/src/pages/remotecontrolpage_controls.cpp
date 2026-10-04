#include "remotecontrolpage.h"

#include "../models/waypointtypes.h"
#include "../ros/navigationservice.h"
#include "../services/camerafeedprovider.h"
#include "../services/mapfile.h"
#include "../services/operationsoverviewservice.h"
#include "../services/robotservice.h"
#include "../services/sensortelemetryprovider.h"
#include "../theme/dashboardpalette.h"
#include "../widgets/badgelabel.h"
#include "../widgets/mapviewwidget.h"
#include "../widgets/proportionalbar.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHideEvent>
#include <QLabel>
#include <QMessageBox>
#include <QPixmap>
#include <QPushButton>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QStyle>
#include <QVBoxLayout>
#include <QtMath>

#include "remotecontrolpage_helpers.h"

QWidget *RemoteControlPage::buildDirectionPad()
{
    auto *panel = panelFrame(QStringLiteral("contentPanel"));
    auto *layout = new QGridLayout(panel);
    layout->setContentsMargins(9, 10, 9, 10);
    layout->setSpacing(5);

    auto *forwardButton = directionButton(QStringLiteral("▲"));
    auto *leftButton = directionButton(QStringLiteral("↺"));
    auto *stopButton = directionButton(QStringLiteral("정지"), true);
    // dangerFilledButton은 전역 QPushButton의 padding: 0 12px를 물려받는데,
    // 38x38 고정 크기 방향패드 버튼 안에 "정지" 두 글자가 들어가면 잘린다.
    // 방향패드 전용 스타일(directionPadStopButton)로 여백/폰트를 다시 잡는다.
    stopButton->setObjectName(QStringLiteral("directionPadStopButton"));
    auto *rightButton = directionButton(QStringLiteral("↻"));
    auto *backwardButton = directionButton(QStringLiteral("▼"));

    for (QPushButton *button : {forwardButton, backwardButton, leftButton, rightButton, stopButton}) {
        button->setFocusPolicy(Qt::NoFocus);
    }

    connect(forwardButton, &QPushButton::pressed, this, [this]() { m_teleop->pressDirection(TeleopController::Direction::Forward); });
    connect(forwardButton, &QPushButton::released, this, [this]() { m_teleop->releaseDirection(TeleopController::Direction::Forward); });
    connect(backwardButton, &QPushButton::pressed, this, [this]() { m_teleop->pressDirection(TeleopController::Direction::Backward); });
    connect(backwardButton, &QPushButton::released, this, [this]() { m_teleop->releaseDirection(TeleopController::Direction::Backward); });
    connect(leftButton, &QPushButton::pressed, this, [this]() { m_teleop->pressDirection(TeleopController::Direction::Left); });
    connect(leftButton, &QPushButton::released, this, [this]() { m_teleop->releaseDirection(TeleopController::Direction::Left); });
    connect(rightButton, &QPushButton::pressed, this, [this]() { m_teleop->pressDirection(TeleopController::Direction::Right); });
    connect(rightButton, &QPushButton::released, this, [this]() { m_teleop->releaseDirection(TeleopController::Direction::Right); });
    connect(stopButton, &QPushButton::clicked, this, [this]() { m_teleop->stop(); });

    layout->addWidget(forwardButton, 0, 1);
    layout->addWidget(leftButton, 1, 0);
    layout->addWidget(stopButton, 1, 1);
    layout->addWidget(rightButton, 1, 2);
    layout->addWidget(backwardButton, 2, 1);
    m_directionPad = panel;
    return panel;
}

QWidget *RemoteControlPage::buildSpeedPresetPanel()
{
    auto *panel = panelFrame(QStringLiteral("contentPanel"));
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(11, 9, 11, 9);
    layout->setSpacing(7);

    auto *headerRow = new QHBoxLayout;
    headerRow->addWidget(makeLabel(QStringLiteral("속도 프리셋"), QStringLiteral("metricCaption")));
    headerRow->addStretch();
    auto *sharedCaption = makeLabel(QStringLiteral("세 방식 값 공유"));
    sharedCaption->setStyleSheet(QStringLiteral("color: %1; font-size: 9px;").arg(DashboardPalette::TextMuted.name()));
    headerRow->addWidget(sharedCaption);
    layout->addLayout(headerRow);

    auto *presetRow = new QHBoxLayout;
    presetRow->setSpacing(5);
    m_speedFineButton = new QPushButton(QStringLiteral("미세"));
    m_speedNormalButton = new QPushButton(QStringLiteral("보통"));
    m_speedFastButton = new QPushButton(QStringLiteral("빠름"));
    presetRow->addWidget(m_speedFineButton, 1);
    presetRow->addWidget(m_speedNormalButton, 1);
    presetRow->addWidget(m_speedFastButton, 1);
    layout->addLayout(presetRow);

    connect(m_speedFineButton, &QPushButton::clicked, this, [this]() { m_teleop->setSpeedPreset(TeleopController::SpeedPreset::Fine); });
    connect(m_speedNormalButton, &QPushButton::clicked, this, [this]() { m_teleop->setSpeedPreset(TeleopController::SpeedPreset::Normal); });
    connect(m_speedFastButton, &QPushButton::clicked, this, [this]() { m_teleop->setSpeedPreset(TeleopController::SpeedPreset::Fast); });

    layout->addWidget(buildSpeedAxisRow(QStringLiteral("▲ 직진"), QStringLiteral("m/s"), 55, &m_linearSpeedValueLabel));
    layout->addWidget(buildSpeedAxisRow(QStringLiteral("↻ 회전"), QStringLiteral("rad/s"), 28, &m_angularSpeedValueLabel));

    updateSpeedPresetButtons();

    return panel;
}

QWidget *RemoteControlPage::buildSpeedAxisRow(const QString &label, const QString &unit, int percent, QLabel **valueLabelOut)
{
    auto *container = new QWidget;
    auto *layout = new QVBoxLayout(container);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(6);

    auto *row = new QHBoxLayout;
    auto *labelWidget = makeLabel(label);
    labelWidget->setStyleSheet(QStringLiteral("color: %1; font-size: 10px;").arg(DashboardPalette::TextSecondary.name()));
    row->addWidget(labelWidget);
    row->addStretch();

    auto *valueBox = makeLabel(QString());
    valueBox->setAlignment(Qt::AlignCenter);
    valueBox->setFixedSize(50, 22);
    valueBox->setStyleSheet(QStringLiteral("color: %1; font-size: 10px; background-color: %2; border: 1px solid %3; border-radius: 5px;")
                                 .arg(DashboardPalette::TextPrimary.name(), DashboardPalette::CardBg.name(), DashboardPalette::BorderGuide.name()));
    row->addWidget(valueBox);
    if (valueLabelOut != nullptr) {
        *valueLabelOut = valueBox;
    }

    auto *unitLabel = makeLabel(unit);
    unitLabel->setStyleSheet(QStringLiteral("color: %1; font-size: 9px;").arg(DashboardPalette::TextTertiary.name()));
    row->addWidget(unitLabel);
    layout->addLayout(row);

    layout->addWidget(makeProportionalBar(percent, DashboardPalette::Cyan, container));
    return container;
}

QWidget *RemoteControlPage::buildCmdVelPanel()
{
    auto *panel = panelFrame(QStringLiteral("callQueueCard"));
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(11, 8, 11, 8);
    layout->setSpacing(4);

    m_cmdVelTitleLabel = makeLabel(QStringLiteral("대기 · /cmd_vel"));
    m_cmdVelTitleLabel->setStyleSheet(QStringLiteral("color: %1; font-size: 9px; letter-spacing: 0.5px;").arg(DashboardPalette::Cyan.name()));
    layout->addWidget(m_cmdVelTitleLabel);

    auto *linearRow = new QHBoxLayout;
    linearRow->addWidget(makeLabel(QStringLiteral("linear.x"), QStringLiteral("metricCaption")));
    linearRow->addStretch();
    m_cmdVelLinearValueLabel = makeLabel(QStringLiteral("0.00 m/s"));
    m_cmdVelLinearValueLabel->setStyleSheet(QStringLiteral("color: %1; font-size: 11px;").arg(DashboardPalette::TextPrimary.name()));
    linearRow->addWidget(m_cmdVelLinearValueLabel);
    layout->addLayout(linearRow);

    auto *angularRow = new QHBoxLayout;
    angularRow->addWidget(makeLabel(QStringLiteral("angular.z"), QStringLiteral("metricCaption")));
    angularRow->addStretch();
    m_cmdVelAngularValueLabel = makeLabel(QStringLiteral("0.00 rad/s"));
    m_cmdVelAngularValueLabel->setStyleSheet(QStringLiteral("color: %1; font-size: 11px;").arg(DashboardPalette::TextPrimary.name()));
    angularRow->addWidget(m_cmdVelAngularValueLabel);
    layout->addLayout(angularRow);

    return panel;
}
