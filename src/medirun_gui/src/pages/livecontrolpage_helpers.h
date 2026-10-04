#pragma once

// livecontrolpage.cpp 분할 시 공유하는 파일 내부용 헬퍼(각 번역 단위가 자기 사본을 가진다).
#include <QtGlobal>
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#endif

namespace {

constexpr int kEventPollIntervalMs = 5000;

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

struct StepInfo
{
    QString label;
};

const QVector<StepInfo> kMissionSteps = {
    {QStringLiteral("호출 접수")},
    {QStringLiteral("출동 중")},
    {QStringLiteral("현장 도착")},
    {QStringLiteral("AED 반출")},
    {QStringLiteral("복귀·도킹")},
};

// MissionState::Arrived + MissionSnapshot::payloadRemoved로 "현장 도착"과
// "AED 반출"을 구분해서 보여준다(missiontypes.h 참고).
QVector<QString> computeStepStates(const MissionSnapshot &snapshot)
{
    QVector<QString> states(kMissionSteps.size(), QStringLiteral("pending"));
    if (snapshot.missionId.isEmpty()) {
        return states;
    }
    states[0] = QStringLiteral("done");

    const bool arrivedOrLater = snapshot.state == MissionState::Arrived || snapshot.returningHome
        || snapshot.state == MissionState::Completed;
    const bool dispatching = snapshot.state == MissionState::Starting || snapshot.state == MissionState::Navigating
        || snapshot.state == MissionState::Diverting;
    const bool payloadDoneOrLater = snapshot.payloadRemoved || snapshot.returningHome || snapshot.state == MissionState::Completed;

    states[1] = arrivedOrLater ? QStringLiteral("done") : (dispatching ? QStringLiteral("active") : QStringLiteral("pending"));
    states[2] = arrivedOrLater ? QStringLiteral("done") : QStringLiteral("pending");
    states[3] = payloadDoneOrLater
        ? QStringLiteral("done")
        : (snapshot.state == MissionState::Arrived ? QStringLiteral("active") : QStringLiteral("pending"));

    if (snapshot.state == MissionState::Completed) {
        states[4] = QStringLiteral("done");
    } else if (snapshot.returningHome) {
        states[4] = QStringLiteral("active");
    } else if (snapshot.state == MissionState::Failed || snapshot.state == MissionState::Cancelled) {
        // finishMission()이 returningHome을 리셋하기 직전 값을 저장해둔
        // arrivedBeforeEnd로, 실패/취소가 복귀 단계까지 가서 난 건지 그 전인
        // 출동/주행 중에 난 건지를 구분해 그 단계에 표시를 준다. 시스템이
        // 감지한 실패는 빨강("failed"), 운영자가 의도적으로 취소한 건 주황
        // ("cancelled")으로 구분한다.
        const QString marker = snapshot.state == MissionState::Failed ? QStringLiteral("failed") : QStringLiteral("cancelled");
        if (snapshot.arrivedBeforeEnd) {
            states[1] = states[2] = states[3] = QStringLiteral("done");
            states[4] = marker;
        } else {
            states[1] = marker;
        }
    }
    return states;
}

// Tailwind의 break-words(overflow-wrap: break-word)와 같은 효과를 낸다.
// QLabel은 단어 경계(공백)에서만 줄바꿈해서, JSON 원문이나 UUID처럼 공백 없이
// 아주 긴 토큰은 끊을 자리를 못 찾아 패널 폭을 넘어가 버린다 — 그런 토큰
// 안에만 제로폭 공백(U+200B, 유니코드 줄바꿈 규칙상 항상 끊어도 되는 지점)을
// 끼워 넣어 강제로 끊기게 하고, 일반 단어는 건드리지 않아 공백에서 자연스럽게
// 줄바꿈되던 동작 그대로 둔다.
QString breakLongTokens(const QString &text)
{
    constexpr int kSoftBreakEvery = 10;
    QString result;
    result.reserve(text.size() + text.size() / kSoftBreakEvery);
    int runLength = 0;
    for (const QChar &ch : text) {
        if (ch.isSpace()) {
            runLength = 0;
        } else {
            if (++runLength > kSoftBreakEvery) {
                result.append(QChar(0x200B));
                runLength = 1;
            }
        }
        result.append(ch);
    }
    return result;
}

QColor colorForEventKind(const QString &kind)
{
    if (kind == QStringLiteral("amber")) {
        return DashboardPalette::Amber;
    }
    if (kind == QStringLiteral("red")) {
        return DashboardPalette::Red;
    }
    if (kind == QStringLiteral("cyan")) {
        return DashboardPalette::Cyan;
    }
    return DashboardPalette::TextMuted;
}

QWidget *buildEventRow(const OperationsEventRow &event)
{
    // 예전엔 왼쪽에 심각도 색상 막대(고정 26px 높이)를 세로로 세워뒀는데,
    // 메시지가 길어서 여러 줄로 접히면 막대만 짧게 남아 가운데 붕 떠 보였다.
    // 막대를 없애고 그 색 정보는 시간/심각도 라벨 자체를 더 크고 굵게 키워
    // 대신하게 한다 — 줄 수와 무관하게 항상 자연스럽다.
    auto *rowWidget = new QWidget;
    // textCol을 addLayout()으로 바로 붙이면(예전 미션 이력 목록의 크래시
    // 원인과 같은 패턴) 스크롤 영역 안에서 폭 협상이 제대로 안 돼 메시지가
    // 줄바꿈되지 않고 잘려 보인다 — rowWidget 자체를 QVBoxLayout으로 만들어
    // addWidget()으로 붙인다.
    auto *textCol = new QVBoxLayout(rowWidget);
    textCol->setContentsMargins(0, 0, 0, 0);
    textCol->setSpacing(2);
    auto *typeLabel = makeLabel(QStringLiteral("%1 · %2").arg(event.occurredAt.toString(QStringLiteral("HH:mm:ss")), event.severityLabel));
    typeLabel->setStyleSheet(QStringLiteral("color: %1; font-size: 12px; font-weight: 700;").arg(colorForEventKind(event.kind).name()));
    textCol->addWidget(typeLabel);
    auto *messageLabel = makeLabel(breakLongTokens(event.message), QStringLiteral("metricCaption"));
    // Preferred(기본값)면 줄바꿈을 켜놔도 "한 줄로 쓴 원래 텍스트 너비"를 계속
    // 선호 크기로 요청해서, 가로 스크롤바가 꺼진 QScrollArea 안에서 그 너비만큼
    // 패널이 넓어지길 시도하다가 넘치는 부분이 그냥 잘려 보인다. Ignored로
    // 바꿔서 라벨이 부모가 주는 너비를 그대로 받아들이고 그 안에서 줄바꿈하게 한다.
    messageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    textCol->addWidget(messageLabel);
    return rowWidget;
}

void clearVBoxLayout(QVBoxLayout *layout)
{
    while (QLayoutItem *item = layout->takeAt(0)) {
        delete item->widget();
        delete item;
    }
}

// 로봇이 정지 상태여도 /odom 노이즈가 0 근처에서 부호만 미세하게 흔들려
// "-0.00"/"0.00"이 번갈아 표시되는 문제를 막는다 — EMA로 다듬은 값이라도
// 이 정도 잔류 노이즈는 남는다(remotecontrolpage.cpp의 동일 문제와 같은 수정).
double snapNearZero(double value, double threshold)
{
    return qAbs(value) < threshold ? 0.0 : value;
}

} // namespace

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
