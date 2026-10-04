#pragma once

#include <QString>

// 최초 실행 시 만들 기본 계정의 비밀번호를 정한다.
// 환경변수(예: AURIX_ADMIN_PASSWORD)가 있으면 그 값을 쓰고, 없으면 무작위 16자를
// 만들어 로그에 한 번만 출력한다. 소스에 고정된 비밀번호를 두지 않기 위함이다.
QString initialPasswordFromEnv(const char *envName, const QString &loginId);
