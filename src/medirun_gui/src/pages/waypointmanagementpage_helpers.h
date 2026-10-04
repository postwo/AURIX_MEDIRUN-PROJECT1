#pragma once

// waypointmanagementpage.cpp 분할 시 공유하는 파일 내부용 헬퍼(각 번역 단위가 자기 사본을 가진다).
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

QString kindLabel(WaypointMarkerKind kind)
{
    switch (kind) {
    case WaypointMarkerKind::CallZone:
        return QStringLiteral("호출");
    case WaypointMarkerKind::Waypoint:
        return QStringLiteral("경유");
    case WaypointMarkerKind::Home:
        return QStringLiteral("홈");
    }
    return QString();
}

// 자동 생성 백업 파일명에 쓸 수 있게, 파일명에 못 쓰는 문자를 제거하고
// 공백은 밑줄로 바꾼다.
QString sanitizeForFileName(const QString &text)
{
    QString result;
    result.reserve(text.size());
    for (const QChar &ch : text) {
        if (ch.isLetterOrNumber() || ch == QLatin1Char('_') || ch == QLatin1Char('-')) {
            result.append(ch);
        } else if (ch.isSpace()) {
            result.append(QLatin1Char('_'));
        }
    }
    return result.isEmpty() ? QStringLiteral("untitled") : result;
}
} // namespace

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
