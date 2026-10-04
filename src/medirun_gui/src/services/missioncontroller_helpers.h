#pragma once

// missioncontroller.cpp 분할 시 공유하는 파일 내부용 헬퍼(각 번역 단위가 자기 사본을 가진다).
#include <QtGlobal>
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#endif

namespace {

QString historyId()
{
    return QUuid::createUuid().toString(QUuid::WithoutBraces);
}

// observedButtonIds()를 앱 재시작 후에도 남기기 위한 로컬 설정 키 — 매번
// 실제 버튼을 다시 눌러야만 웨이포인트관리 페이지 드롭다운에 뜨는 걸 막는다
// (main.cpp에서 QApplication에 조직/앱 이름이 이미 설정돼 있어 QSettings()를
// 인자 없이 그대로 쓸 수 있다).
constexpr auto kObservedButtonIdsSettingsKey = "mission/observedCallButtonIds";

QString resultText(const QString &resultCode)
{
    if (resultCode == QStringLiteral("SUCCEEDED")) {
        return QStringLiteral("성공");
    }
    if (resultCode == QStringLiteral("CANCELED")) {
        return QStringLiteral("취소");
    }
    return QStringLiteral("실패");
}

} // namespace

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
