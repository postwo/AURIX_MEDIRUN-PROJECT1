// DB 없이 MissionStorageService 의 JSON 파일 경로를 시험하기 위한 dbconnection 대역.
#include "stub_dbconnection.h"

#include "../src/services/dbconnection.h"

namespace StubDb {
int openCalls = 0;
}

bool openDbConnection(QString &errorMessage)
{
    ++StubDb::openCalls;
    errorMessage = QStringLiteral("시험 환경: DB 없음");
    return false;
}

QDateTime dbTimestampToLocal(const QVariant &value)
{
    return value.toDateTime().toLocalTime();
}
