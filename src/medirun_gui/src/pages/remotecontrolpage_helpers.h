#pragma once

// remotecontrolpage.cpp 분할 시 공유하는 파일 내부용 헬퍼(각 번역 단위가 자기 사본을 가진다).
#include <QtGlobal>
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#endif

namespace {
QLabel *makeLabel(const QString &text, const QString &objectName = {})
{
    auto *label = new QLabel(text);
    if (!objectName.isEmpty()) {
        label->setObjectName(objectName);
    }
    label->setWordWrap(true);
    return label;
}

QFrame *panelFrame(const QString &objectName)
{
    auto *frame = new QFrame;
    frame->setObjectName(objectName);
    frame->setFrameShape(QFrame::NoFrame);
    return frame;
}

QPushButton *directionButton(const QString &text, bool danger = false)
{
    auto *button = new QPushButton(text);
    button->setObjectName(danger ? QStringLiteral("dangerFilledButton") : QStringLiteral("directionPadButton"));
    button->setFixedSize(38, 38);
    return button;
}

// LiveControlPage::statValueLabel/setLabelColor와 동일한 모양의 "로봇 상태" 행 —
// 센서 상태 패널에서 같은 스타일을 재사용하기 위해 그대로 옮겨왔다.
QLabel *statValueLabel(QBoxLayout *parentLayout, const QString &label)
{
    auto *row = new QHBoxLayout;
    row->addWidget(makeLabel(label, QStringLiteral("metricCaption")));
    row->addStretch();
    auto *valueLabel = makeLabel(QStringLiteral("--"));
    valueLabel->setStyleSheet(QStringLiteral("color: %1; font-size: 11px;").arg(DashboardPalette::TextPrimary.name()));
    row->addWidget(valueLabel);
    parentLayout->addLayout(row);
    return valueLabel;
}

void setLabelColor(QLabel *label, const QColor &color)
{
    label->setStyleSheet(QStringLiteral("color: %1; font-size: 11px;").arg(color.name()));
}

// 로봇이 정지 상태여도 /odom 노이즈가 0 근처에서 부호만 미세하게 흔들려
// "-0.00"/"0.00"이 번갈아 표시되는 문제를 막는다 — EMA로 다듬은 값이라도
// 이 정도 잔류 노이즈는 남는다.
double snapNearZero(double value, double threshold)
{
    return qAbs(value) < threshold ? 0.0 : value;
}
} // namespace

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
