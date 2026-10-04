// yaml-cpp 없이 WaypointRepository 를 링크하기 위한 waypointfile 대역.
// 파일 입출력은 항상 실패로 응답한다(저장소 로직만 시험하려는 목적).
#include "../src/services/waypointfile.h"

bool loadWaypointYaml(const QString &, QString &, WaypointMap &, TrajectoryMap &, QString &errorMessage)
{
    errorMessage = QStringLiteral("시험 환경: YAML 로드 미지원");
    return false;
}

bool saveWaypointYaml(const QString &, const QString &, const WaypointMap &, const TrajectoryMap &, QString &errorMessage)
{
    errorMessage = QStringLiteral("시험 환경: YAML 저장 미지원");
    return false;
}
