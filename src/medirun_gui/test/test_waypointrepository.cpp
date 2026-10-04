// WaypointRepository 시험: 웨이포인트 저장·조회, 자세 계산, 변경 알림.
// YAML 입출력은 stub_waypointfile.cpp 가 항상 실패로 응답한다.

#include "../src/services/waypointrepository.h"

#include <QSignalSpy>
#include <QtMath>
#include <QtTest>

namespace {

WaypointDef def(const QString &name, double x, double y, double yaw, WaypointMarkerKind kind, const QString &label = QString())
{
    WaypointDef d;
    d.name = name;
    d.label = label;
    d.x = x;
    d.y = y;
    d.yaw = yaw;
    d.kind = kind;
    return d;
}

} // namespace

class WaypointRepositoryTest : public QObject
{
    Q_OBJECT

private slots:
    void replaceAllStoresDataAndNotifiesOnce();
    void emptyFrameIdFallsBackToMap();
    void lookupFindsOnlyExistingWaypoints();
    void poseUsesFrameAndComputesQuaternionFromYaw();
    void poseOfUnknownWaypointFailsWithId();
    void waypointsByKindFiltersCorrectly();
    void upsertAddsOrOverwritesByName();
    void upsertWithEmptyListDoesNotNotify();
    void batchUpsertNotifiesOnlyOnce();
    void failedLoadKeepsExistingData();
    void failedSaveKeepsPathAndDoesNotNotify();
};

void WaypointRepositoryTest::replaceAllStoresDataAndNotifiesOnce()
{
    WaypointRepository repo;
    QSignalSpy spy(&repo, &WaypointRepository::repositoryChanged);
    WaypointMap waypoints;
    waypoints.insert(QStringLiteral("A"), def(QStringLiteral("A"), 1, 2, 0, WaypointMarkerKind::Waypoint));
    TrajectoryMap trajectories;
    trajectories.insert(QStringLiteral("t1"), {QStringLiteral("A"), QStringLiteral("B")});

    repo.replaceAll(QStringLiteral("odom"), waypoints, trajectories, QStringLiteral("/tmp/x.yaml"));

    QCOMPARE(spy.count(), 1);
    QCOMPARE(repo.frameId(), QStringLiteral("odom"));
    QCOMPARE(repo.filePath(), QStringLiteral("/tmp/x.yaml"));
    QCOMPARE(repo.waypoints().size(), 1);
    QCOMPARE(repo.trajectories().value(QStringLiteral("t1")), (QStringList{QStringLiteral("A"), QStringLiteral("B")}));
}

void WaypointRepositoryTest::emptyFrameIdFallsBackToMap()
{
    WaypointRepository repo;
    repo.replaceAll(QString(), {}, {}, QString());
    QCOMPARE(repo.frameId(), QStringLiteral("map"));
}

void WaypointRepositoryTest::lookupFindsOnlyExistingWaypoints()
{
    WaypointRepository repo;
    WaypointMap waypoints;
    waypoints.insert(QStringLiteral("A"), def(QStringLiteral("A"), 1, 2, 0.5, WaypointMarkerKind::CallZone, QStringLiteral("병실A")));
    repo.replaceAll(QStringLiteral("map"), waypoints, {}, QString());

    WaypointDef found;
    QVERIFY(repo.waypoint(QStringLiteral("A"), found));
    QCOMPARE(found.label, QStringLiteral("병실A"));
    QCOMPARE(found.x, 1.0);

    WaypointDef untouched;
    untouched.label = QStringLiteral("그대로");
    QVERIFY(!repo.waypoint(QStringLiteral("NOPE"), untouched));
    QCOMPARE(untouched.label, QStringLiteral("그대로")); // 실패 시 출력 인자를 건드리지 않는다
}

void WaypointRepositoryTest::poseUsesFrameAndComputesQuaternionFromYaw()
{
    WaypointRepository repo;
    WaypointMap waypoints;
    waypoints.insert(QStringLiteral("A"), def(QStringLiteral("A"), 1.5, -2.5, M_PI / 2.0, WaypointMarkerKind::Waypoint));
    repo.replaceAll(QStringLiteral("map"), waypoints, {}, QString());

    WaypointPose pose;
    QString error;
    QVERIFY(repo.waypointPose(QStringLiteral("A"), pose, error));

    QVERIFY(error.isEmpty());
    QVERIFY(pose.valid);
    QCOMPARE(pose.frameId, QStringLiteral("map"));
    QCOMPARE(pose.positionX, 1.5);
    QCOMPARE(pose.positionY, -2.5);
    QCOMPARE(pose.positionZ, 0.0);
    QCOMPARE(pose.yawRadians, M_PI / 2.0);
    // yaw 90° → z = sin(45°), w = cos(45°)
    QVERIFY(qAbs(pose.orientationZ - qSin(M_PI / 4.0)) < 1e-9);
    QVERIFY(qAbs(pose.orientationW - qCos(M_PI / 4.0)) < 1e-9);
    QCOMPARE(pose.orientationX, 0.0);
    QCOMPARE(pose.orientationY, 0.0);
}

void WaypointRepositoryTest::poseOfUnknownWaypointFailsWithId()
{
    WaypointRepository repo;
    WaypointPose pose;
    QString error;

    QVERIFY(!repo.waypointPose(QStringLiteral("NOPE"), pose, error));

    QVERIFY(error.contains(QStringLiteral("NOPE")));
}

void WaypointRepositoryTest::waypointsByKindFiltersCorrectly()
{
    WaypointRepository repo;
    WaypointMap waypoints;
    waypoints.insert(QStringLiteral("H"), def(QStringLiteral("H"), 0, 0, 0, WaypointMarkerKind::Home));
    waypoints.insert(QStringLiteral("C1"), def(QStringLiteral("C1"), 1, 0, 0, WaypointMarkerKind::CallZone));
    waypoints.insert(QStringLiteral("C2"), def(QStringLiteral("C2"), 2, 0, 0, WaypointMarkerKind::CallZone));
    repo.replaceAll(QStringLiteral("map"), waypoints, {}, QString());

    QCOMPARE(repo.waypointsByKind(WaypointMarkerKind::Home).size(), 1);
    QCOMPARE(repo.waypointsByKind(WaypointMarkerKind::CallZone).size(), 2);
    QVERIFY(repo.waypointsByKind(WaypointMarkerKind::Waypoint).isEmpty());
}

void WaypointRepositoryTest::upsertAddsOrOverwritesByName()
{
    WaypointRepository repo;
    QSignalSpy spy(&repo, &WaypointRepository::repositoryChanged);

    repo.upsertWaypoint(def(QStringLiteral("A"), 1, 1, 0, WaypointMarkerKind::Waypoint, QStringLiteral("처음")));
    repo.upsertWaypoint(def(QStringLiteral("A"), 9, 9, 0, WaypointMarkerKind::Waypoint, QStringLiteral("바뀜")));

    QCOMPARE(repo.waypoints().size(), 1);
    QCOMPARE(repo.waypoints().value(QStringLiteral("A")).label, QStringLiteral("바뀜"));
    QCOMPARE(repo.waypoints().value(QStringLiteral("A")).x, 9.0);
    QCOMPARE(spy.count(), 2);
}

void WaypointRepositoryTest::upsertWithEmptyListDoesNotNotify()
{
    WaypointRepository repo;
    QSignalSpy spy(&repo, &WaypointRepository::repositoryChanged);

    repo.upsertWaypoints({});

    QCOMPARE(spy.count(), 0);
}

void WaypointRepositoryTest::batchUpsertNotifiesOnlyOnce()
{
    WaypointRepository repo;
    QSignalSpy spy(&repo, &WaypointRepository::repositoryChanged);

    repo.upsertWaypoints({def(QStringLiteral("A"), 1, 1, 0, WaypointMarkerKind::Waypoint),
                          def(QStringLiteral("B"), 2, 2, 0, WaypointMarkerKind::Waypoint),
                          def(QStringLiteral("C"), 3, 3, 0, WaypointMarkerKind::Waypoint)});

    QCOMPARE(repo.waypoints().size(), 3);
    QCOMPARE(spy.count(), 1); // 화면이 항목 수만큼 다시 그려지지 않는다
}

void WaypointRepositoryTest::failedLoadKeepsExistingData()
{
    WaypointRepository repo;
    WaypointMap waypoints;
    waypoints.insert(QStringLiteral("A"), def(QStringLiteral("A"), 1, 2, 0, WaypointMarkerKind::Waypoint));
    repo.replaceAll(QStringLiteral("map"), waypoints, {}, QStringLiteral("/keep.yaml"));
    QSignalSpy spy(&repo, &WaypointRepository::repositoryChanged);
    QString error;

    QVERIFY(!repo.loadFromFile(QStringLiteral("/no/such/file.yaml"), error));

    QVERIFY(!error.isEmpty());
    QCOMPARE(repo.waypoints().size(), 1);
    QCOMPARE(repo.filePath(), QStringLiteral("/keep.yaml"));
    QCOMPARE(spy.count(), 0);
}

void WaypointRepositoryTest::failedSaveKeepsPathAndDoesNotNotify()
{
    WaypointRepository repo;
    repo.replaceAll(QStringLiteral("map"), {}, {}, QStringLiteral("/keep.yaml"));
    QSignalSpy spy(&repo, &WaypointRepository::repositoryChanged);
    QString error;

    QVERIFY(!repo.saveToFile(QStringLiteral("/other.yaml"), error));

    QCOMPARE(repo.filePath(), QStringLiteral("/keep.yaml"));
    QCOMPARE(spy.count(), 0);
}

QTEST_GUILESS_MAIN(WaypointRepositoryTest)
#include "test_waypointrepository.moc"
