// MissionController 상태 전이 시험.
// 협력 객체(Nav2, ROS 인터페이스, 저장소)는 fakes.h/fakes.cpp 의 대역을 쓰므로
// ROS·DB·YAML 없이 실행된다.

#include "fakes.h"

#include "../src/services/missioncontroller.h"
#include "../src/services/missionrosinterface.h"
#include "../src/services/missionstorageservice.h"
#include "../src/services/waypointrepository.h"

#include <QCoreApplication>
#include <QSettings>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QtTest>

#include <memory>

namespace {

FakeWorld &world()
{
    return FakeWorld::instance();
}

WaypointDef makeWaypoint(const QString &name, const QString &label, double x, double y, double yaw, WaypointMarkerKind kind)
{
    WaypointDef def;
    def.name = name;
    def.label = label;
    def.x = x;
    def.y = y;
    def.yaw = yaw;
    def.kind = kind;
    return def;
}

MissionQueueItem makeQueueItem(const QString &queueId, const QString &waypointId, MissionQueueStatus status)
{
    MissionQueueItem item;
    item.queueId = queueId;
    item.waypointId = waypointId;
    item.waypointName = waypointId;
    item.missionId = QStringLiteral("MR-OLD-") + queueId;
    item.status = status;
    item.requestedAt = QDateTime::currentDateTime();
    item.pose.valid = true;
    item.pose.positionX = 1.0;
    item.pose.positionY = 2.0;
    return item;
}

// 컨트롤러와 협력 객체 묶음. 웨이포인트: CZ1(1,2), CZ2(3,4), HOME(0,0).
struct Fixture
{
    FakeNavigationService nav;
    WaypointRepository repo;
    MissionStorageService storage;
    MissionRosInterface ros;
    FakeRobotService robots;
    FakeOverviewService overview;
    bool useDb {false};
    std::unique_ptr<MissionController> controller;

    explicit Fixture(bool withHome = true)
    {
        WaypointMap waypoints;
        waypoints.insert(QStringLiteral("CZ1"), makeWaypoint(QStringLiteral("CZ1"), QStringLiteral("병실1"), 1.0, 2.0, 0.5, WaypointMarkerKind::CallZone));
        waypoints.insert(QStringLiteral("CZ2"), makeWaypoint(QStringLiteral("CZ2"), QStringLiteral("병실2"), 3.0, 4.0, 1.0, WaypointMarkerKind::CallZone));
        if (withHome) {
            waypoints.insert(QStringLiteral("HOME"), makeWaypoint(QStringLiteral("HOME"), QStringLiteral("홈"), 0.0, 0.0, 0.0, WaypointMarkerKind::Home));
        }
        repo.replaceAll(QStringLiteral("map"), waypoints, {}, QString());
    }

    void build()
    {
        controller = std::make_unique<MissionController>(AppSession{}, &nav, &repo, &storage, &ros,
                                                         useDb ? static_cast<RobotService *>(&robots) : nullptr,
                                                         useDb ? static_cast<OperationsOverviewService *>(&overview) : nullptr);
    }

    // 로봇 1대가 등록된 DB 협력 객체를 켠다(Pi 복귀 세션 기록 경로용).
    void enableDb(int robotCount = 1)
    {
        useDb = true;
        for (int i = 0; i < robotCount; ++i) {
            RobotRecord record;
            record.id = 7 + i;
            record.name = QStringLiteral("robot%1").arg(i);
            robots.robots.append(record);
        }
    }

    // 호출 → 도착(Arrived)까지 진행한다. 미션/큐 ID를 돌려준다.
    QPair<QString, QString> arriveAtCallZone(const QString &buttonId = QStringLiteral("CZ1"))
    {
        startNavigating(buttonId);
        nav.navigationResult(QStringLiteral("SUCCEEDED"), QString());
        const MissionSnapshot snapshot = controller->missionSnapshot();
        return {snapshot.missionId, snapshot.queueId};
    }

    void call(const QString &buttonId)
    {
        ros.dispatchRequested(buttonId, QStringLiteral("test"), QString(), QDateTime());
    }

    // 호출 → Nav2 goal 접수까지 진행해 Navigating 상태로 만든다.
    void startNavigating(const QString &buttonId = QStringLiteral("CZ1"))
    {
        call(buttonId);
        nav.navigationGoalAccepted();
    }

    MissionState state() const { return controller->missionSnapshot().state; }
    QString queueIdAt(int index) const { return controller->waitingQueue().at(index).queueId; }
    MissionQueueStatus statusAt(int index) const { return controller->waitingQueue().at(index).status; }
};

} // namespace

class MissionControllerTest : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void init();

    // 시작·큐
    void startsIdleWithEmptyQueue();
    void unknownWaypointIsRejectedAndRecorded();
    void callStartsMissionAndSendsGoal();
    void goalAcceptedMovesToNavigating();
    void duplicateCallForSameWaypointIsIgnored();
    void secondCallWaitsInQueue();
    void waitingItemAtStartupStartsImmediately();
    void orphanedActiveMissionIsRecoveredAsNavigating();
    void observedButtonIdsKeepNewestFirstWithoutDuplicates();
    void todayHistoryDropsEntriesOlderThan24Hours();

    // 결과 처리
    void arrivalMovesToArrivedAndArmsReturnWatch();
    void arrivalWithoutHomeStillArrivesButRecordsUnarmedWatch();
    void navFailureMarksItemFailedAndStartsNextCall();
    void goalRejectionFailsMission();
    void nonSuccessResultCodeFailsMission();

    // 취소·전환
    void cancelWithoutMissionIsIgnored();
    void cancelWhileNavigatingWaitsForNav2Confirmation();
    void cancelConfirmedAsksBeforeStartingNextCall();
    void cancelAfterArrivalReturnsHomeImmediately();
    void cancelDuringParkingStopsDocking();
    void cancelingWaitingQueueItemRemovesItWithoutTouchingNav2();
    void cancelingActiveQueueItemCancelsMission();
    void divertCancelsCurrentGoalThenStartsSelectedCall();
    void divertIsRefusedAfterArrival();

    // 홈 복귀·주차
    void returnHomeFromIdleDrivesToHome();
    void returnHomeWithoutHomeWaypointFailsCleanly();
    void returnHomeWhileNavigatingCancelsThenDrivesHome();
    void returnHomeSucceedsEvenIfSuccessArrivesBeforeCancel();
    void homeArrivalStartsParkingAndCompletes();
    void parkingFailureFailsMission();
    void newCallStartsImmediatelyAfterCompletion();

    // Pi 연동·연결 상태·진행률
    void piPayloadRemovedIsRecordedOnce();
    void piStatusForOtherMissionIsIgnored();
    void nav2ConnectionIsSyncedAtStartupAndOnChange();
    void anomalyTitleFollowsConnectionState();
    void retryRunsOnceAndRecordsOutcome();
    void progressFollowsRemainingDistance();

    // Pi 자동 복귀와 DB 세션 기록
    void piReturningOnlyUpdatesScreenAndDoesNotResendGoal();
    void piReturningOpensDbSessionWithCachedRoute();
    void piReturningMarksEachPassedGoalOnce();
    void piReturningSessionNeedsExactlyOneRobot();
    void piReturningContinuesWhenDbSessionFails();
    void piReturningIsIgnoredUnlessArrived();
    void piReplayedReturningDoesNotOpenSecondSession();
    void piReturnFailedFailsMissionAndClosesSession();
    void piReturnFailedIsIgnoredWhileNavigating();
    void dockSuccessClosesPiSessionAsSucceeded();
    void cancelWhileReturningClosesPiSessionAsCancelled();
    void dockResultWhileArrivedStillFinishesMission();
    void dockResultIsIgnoredWhileNavigating();
    void piArmedPhaseHasNoEffect();

    // 경계 조건
    void announceTestDriveSpeaksLabelOrDefault();
    void confirmStartNextQueueRequiresIdleAndWaitingItem();
    void declineNextQueueKeepsWaitingItemAndRecords();
    void divertFromIdleStartsSelectedCallDirectly();
    void divertIsIgnoredWhileCancelling();
    void divertWithUnknownQueueIdIsIgnored();
    void cancelBeforeGoalAcceptedIsAllowed();
    void returnHomeFromArrivalWithoutHomeFailsMission();
    void returnHomeIsNotRequestedTwice();
    void returnHomeIsRefusedWhileParking();
    void repositoryChangeNotifiesOnlyWhileMissionActive();
    void amclPoseUpdatesCurrentWaypointNameDuringMission();

private:
    QTemporaryDir m_settingsDir;
};

void MissionControllerTest::initTestCase()
{
    QCoreApplication::setOrganizationName(QStringLiteral("AURIX-test"));
    QCoreApplication::setApplicationName(QStringLiteral("MissionControllerTest"));
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, m_settingsDir.path());
}

void MissionControllerTest::init()
{
    world().reset();
    QSettings().clear();
}

// ---- 시작·큐 ------------------------------------------------------------------

void MissionControllerTest::startsIdleWithEmptyQueue()
{
    Fixture f;
    f.build();

    QCOMPARE(f.state(), MissionState::Idle);
    QVERIFY(f.controller->waitingQueue().isEmpty());
    QCOMPARE(f.controller->queueSummaryText(), QStringLiteral("대기 호출 큐 · 0건"));
    QVERIFY(!f.controller->rosConnected());
    QVERIFY(!f.controller->retryInProgress());
}

void MissionControllerTest::unknownWaypointIsRejectedAndRecorded()
{
    Fixture f;
    f.build();

    f.call(QStringLiteral("NOPE"));

    QVERIFY(f.controller->waitingQueue().isEmpty());
    QVERIFY(f.nav.navigateCalls.isEmpty());
    QVERIFY(world().historyTypes().contains(QStringLiteral("QUEUE_FAILED")));
    // 등록 실패여도 실제로 받은 버튼 ID는 기억해 둔다(웨이포인트 관리 화면용).
    QCOMPARE(f.controller->observedButtonIds(), QStringList{QStringLiteral("NOPE")});
}

void MissionControllerTest::callStartsMissionAndSendsGoal()
{
    Fixture f;
    f.build();
    QSignalSpy queueSpy(f.controller.get(), &MissionController::queueChanged);

    f.call(QStringLiteral("CZ1"));

    QCOMPARE(f.state(), MissionState::Starting);
    QCOMPARE(f.nav.navigateCalls.size(), 1);
    QCOMPARE(f.nav.navigateCalls.first().x, 1.0);
    QCOMPARE(f.nav.navigateCalls.first().y, 2.0);
    QCOMPARE(f.nav.navigateCalls.first().yaw, 0.5);
    QCOMPARE(f.nav.navigateCalls.first().frameId, QStringLiteral("map"));
    QCOMPARE(f.nav.navigateCalls.first().label, QStringLiteral("병실1"));

    QCOMPARE(f.controller->waitingQueue().size(), 1);
    QCOMPARE(f.statusAt(0), MissionQueueStatus::Active);
    QVERIFY(f.controller->missionSnapshot().missionId.startsWith(QStringLiteral("MR-")));
    QCOMPARE(f.controller->missionSnapshot().destinationName, QStringLiteral("병실1"));

    QCOMPARE(world().savedQueue.size(), 1);
    QCOMPARE(world().savedQueue.first().status, MissionQueueStatus::Active);
    QVERIFY(world().historyTypes().contains(QStringLiteral("QUEUE_REGISTERED")));
    QVERIFY(world().historyTypes().contains(QStringLiteral("MISSION_STARTED")));
    QVERIFY(world().audio.contains(QStringLiteral("응급 호출을 접수했습니다. 출동합니다")));
    QVERIFY(!queueSpy.isEmpty());
}

void MissionControllerTest::goalAcceptedMovesToNavigating()
{
    Fixture f;
    f.build();
    f.call(QStringLiteral("CZ1"));
    QCOMPARE(f.state(), MissionState::Starting);

    f.nav.navigationGoalAccepted();

    QCOMPARE(f.state(), MissionState::Navigating);
}

void MissionControllerTest::duplicateCallForSameWaypointIsIgnored()
{
    Fixture f;
    f.build();

    f.call(QStringLiteral("CZ1"));
    f.call(QStringLiteral("CZ1"));

    QCOMPARE(f.controller->waitingQueue().size(), 1);
    QCOMPARE(f.nav.navigateCalls.size(), 1);
    QVERIFY(world().historyTypes().contains(QStringLiteral("QUEUE_DUPLICATE_IGNORED")));
}

void MissionControllerTest::secondCallWaitsInQueue()
{
    Fixture f;
    f.build();

    f.call(QStringLiteral("CZ1"));
    f.call(QStringLiteral("CZ2"));

    QCOMPARE(f.controller->waitingQueue().size(), 2);
    QCOMPARE(f.statusAt(0), MissionQueueStatus::Active);
    QCOMPARE(f.statusAt(1), MissionQueueStatus::Waiting);
    QCOMPARE(f.controller->queueSummaryText(), QStringLiteral("대기 호출 큐 · 2건"));
    QCOMPARE(f.nav.navigateCalls.size(), 1);
}

void MissionControllerTest::waitingItemAtStartupStartsImmediately()
{
    Fixture f;
    world().initialQueue = {makeQueueItem(QStringLiteral("Q-1"), QStringLiteral("CZ2"), MissionQueueStatus::Waiting)};

    f.build();

    QCOMPARE(f.state(), MissionState::Starting);
    QCOMPARE(f.nav.navigateCalls.size(), 1);
    // 저장돼 있던 항목의 pose(1,2)로 출발한다.
    QCOMPARE(f.nav.navigateCalls.first().x, 1.0);
    QCOMPARE(f.nav.navigateCalls.first().y, 2.0);
}

void MissionControllerTest::orphanedActiveMissionIsRecoveredAsNavigating()
{
    Fixture f;
    world().initialQueue = {makeQueueItem(QStringLiteral("Q-OLD"), QStringLiteral("CZ1"), MissionQueueStatus::Active)};

    f.build();

    QCOMPARE(f.state(), MissionState::Navigating);
    QCOMPARE(f.controller->missionSnapshot().originName, QStringLiteral("이전 세션"));
    QVERIFY(f.nav.navigateCalls.isEmpty()); // 이미 나간 goal을 다시 보내지 않는다
    QVERIFY(world().historyTypes().contains(QStringLiteral("MISSION_RECOVERED")));

    // 복구된 미션도 취소할 수 있어야 한다.
    f.controller->requestMissionCancel();
    QCOMPARE(f.state(), MissionState::Cancelling);
    QCOMPARE(f.nav.cancelCalls, 1);
}

void MissionControllerTest::observedButtonIdsKeepNewestFirstWithoutDuplicates()
{
    Fixture f;
    f.build();
    QSignalSpy spy(f.controller.get(), &MissionController::observedButtonIdsChanged);

    f.call(QStringLiteral("CZ1"));
    f.call(QStringLiteral("CZ2"));
    f.call(QStringLiteral("CZ1"));

    QCOMPARE(f.controller->observedButtonIds(), (QStringList{QStringLiteral("CZ1"), QStringLiteral("CZ2")}));
    QCOMPARE(spy.count(), 3);
}

void MissionControllerTest::todayHistoryDropsEntriesOlderThan24Hours()
{
    Fixture f;
    MissionHistoryEntry old;
    old.eventType = QStringLiteral("OLD");
    old.occurredAt = QDateTime::currentDateTime().addDays(-2);
    MissionHistoryEntry recent;
    recent.eventType = QStringLiteral("RECENT");
    recent.occurredAt = QDateTime::currentDateTime().addSecs(-60);
    world().initialHistory = {old, recent};

    f.build();

    const QVector<MissionHistoryEntry> history = f.controller->todayHistory();
    QCOMPARE(history.size(), 1);
    QCOMPARE(history.first().eventType, QStringLiteral("RECENT"));
}

// ---- 결과 처리 ----------------------------------------------------------------

void MissionControllerTest::arrivalMovesToArrivedAndArmsReturnWatch()
{
    Fixture f;
    f.build();
    f.startNavigating();

    f.nav.navigationResult(QStringLiteral("SUCCEEDED"), QString());

    QCOMPARE(f.state(), MissionState::Arrived);
    QVERIFY(world().audio.contains(QStringLiteral("AED가 도착했습니다")));
    QVERIFY(world().historyTypes().contains(QStringLiteral("MISSION_ARRIVED")));
    QVERIFY(!world().arrivalNotices.isEmpty());
    QVERIFY(world().arrivalNotices.last().armed); // Pi 자동 복귀 감시 시작
    QCOMPARE(world().arrivalNotices.last().missionId, f.controller->missionSnapshot().missionId);
}

void MissionControllerTest::arrivalWithoutHomeStillArrivesButRecordsUnarmedWatch()
{
    Fixture f(false);
    f.build();
    f.startNavigating();

    f.nav.navigationResult(QStringLiteral("SUCCEEDED"), QString());

    QCOMPARE(f.state(), MissionState::Arrived);
    QVERIFY(world().historyTypes().contains(QStringLiteral("ARRIVAL_WATCH_NOT_ARMED")));
    for (const FakeWorld::ArrivalNotice &notice : world().arrivalNotices) {
        QVERIFY(!notice.armed);
    }
}

void MissionControllerTest::navFailureMarksItemFailedAndStartsNextCall()
{
    Fixture f;
    f.build();
    f.call(QStringLiteral("CZ1"));
    f.call(QStringLiteral("CZ2"));
    f.nav.navigationGoalAccepted();

    f.nav.navigationResult(QStringLiteral("ABORTED"), QStringLiteral("planner failed"));

    QCOMPARE(f.statusAt(0), MissionQueueStatus::Failed);
    QVERIFY(world().historyTypes().contains(QStringLiteral("MISSION_FAILED")));
    // 실패한 미션의 Pi 감시는 반드시 해제한다.
    QVERIFY(!world().arrivalNotices.isEmpty());
    QVERIFY(!world().arrivalNotices.last().armed);
    // 확인 절차 없이 다음 대기 호출로 넘어간다(취소와 다름).
    QCOMPARE(f.nav.navigateCalls.size(), 2);
    QCOMPARE(f.nav.navigateCalls.last().x, 3.0);
    QCOMPARE(f.statusAt(1), MissionQueueStatus::Active);
}

void MissionControllerTest::goalRejectionFailsMission()
{
    Fixture f;
    f.build();
    f.call(QStringLiteral("CZ1"));

    f.nav.navigationGoalRejected(QStringLiteral("server busy"));

    QCOMPARE(f.statusAt(0), MissionQueueStatus::Failed);
    QVERIFY(world().historyTypes().contains(QStringLiteral("NAV_GOAL_REJECTED")));
    QCOMPARE(f.state(), MissionState::Idle); // 대기 호출이 없으면 Idle로 정리
}

void MissionControllerTest::nonSuccessResultCodeFailsMission()
{
    Fixture f;
    f.build();
    f.startNavigating();

    f.nav.navigationResult(QStringLiteral("UNKNOWN_CODE"), QStringLiteral("detail"));

    QCOMPARE(f.statusAt(0), MissionQueueStatus::Failed);
    QVERIFY(!world().historyTypes().contains(QStringLiteral("MISSION_ARRIVED")));
}

// ---- 취소·전환 ----------------------------------------------------------------

void MissionControllerTest::cancelWithoutMissionIsIgnored()
{
    Fixture f;
    f.build();

    f.controller->requestMissionCancel();

    QCOMPARE(f.nav.cancelCalls, 0);
    QCOMPARE(f.state(), MissionState::Idle);
    QVERIFY(world().historyTypes().contains(QStringLiteral("MISSION_CANCEL_IGNORED")));
}

void MissionControllerTest::cancelWhileNavigatingWaitsForNav2Confirmation()
{
    Fixture f;
    f.build();
    f.startNavigating();

    f.controller->requestMissionCancel();

    QCOMPARE(f.state(), MissionState::Cancelling);
    QCOMPARE(f.nav.cancelCalls, 1);
    QCOMPARE(world().commands.size(), 1);
    QCOMPARE(world().commands.first().command, QStringLiteral("cancel"));
    QCOMPARE(f.statusAt(0), MissionQueueStatus::Active); // 아직 확정 전

    // 취소 중에 또 눌러도 중복 요청하지 않는다.
    f.controller->requestMissionCancel();
    QCOMPARE(f.nav.cancelCalls, 1);

    f.nav.navigationResult(QStringLiteral("CANCELED"), QString());

    QCOMPARE(f.statusAt(0), MissionQueueStatus::Cancelled);
    QCOMPARE(world().savedQueue.first().status, MissionQueueStatus::Cancelled);
    QVERIFY(world().historyTypes().contains(QStringLiteral("MISSION_CANCELLED")));
    QCOMPARE(f.state(), MissionState::Idle);
}

void MissionControllerTest::cancelConfirmedAsksBeforeStartingNextCall()
{
    Fixture f;
    f.build();
    f.call(QStringLiteral("CZ1"));
    f.call(QStringLiteral("CZ2"));
    f.nav.navigationGoalAccepted();
    const QString nextQueueId = f.queueIdAt(1);
    QSignalSpy confirmSpy(f.controller.get(), &MissionController::nextQueueConfirmationRequested);

    f.controller->requestMissionCancel();
    f.nav.navigationResult(QStringLiteral("CANCELED"), QString());

    // 자동으로 다음 호출로 출발하지 않고 운영자 확인을 요청한다.
    QCOMPARE(confirmSpy.count(), 1);
    QCOMPARE(confirmSpy.first().at(0).toString(), nextQueueId);
    QCOMPARE(f.nav.navigateCalls.size(), 1);
    QCOMPARE(f.state(), MissionState::Idle);

    // 운영자가 확인하면 그때 출발한다.
    f.controller->confirmStartNextQueue(nextQueueId);
    QCOMPARE(f.nav.navigateCalls.size(), 2);
    QCOMPARE(f.nav.navigateCalls.last().x, 3.0);
}

void MissionControllerTest::cancelAfterArrivalReturnsHomeImmediately()
{
    Fixture f;
    f.build();
    f.startNavigating();
    f.nav.navigationResult(QStringLiteral("SUCCEEDED"), QString());
    QCOMPARE(f.state(), MissionState::Arrived);
    const int cancelCallsBefore = f.nav.cancelCalls;

    f.controller->requestMissionCancel();

    // 도착 후엔 취소할 Nav2 goal이 없으므로 바로 홈 goal을 보낸다.
    QCOMPARE(f.state(), MissionState::ReturningHome);
    QCOMPARE(f.nav.cancelCalls, cancelCallsBefore);
    QCOMPARE(f.nav.navigateCalls.last().x, 0.0);
    QCOMPARE(f.nav.navigateCalls.last().label, QStringLiteral("홈"));
    QVERIFY(!world().arrivalNotices.last().armed); // Pi가 따로 복귀하지 않도록 해제
}

void MissionControllerTest::cancelDuringParkingStopsDocking()
{
    Fixture f;
    f.build();
    f.startNavigating();
    f.nav.navigationResult(QStringLiteral("SUCCEEDED"), QString());
    f.controller->requestReturnHome();
    f.nav.navigationResult(QStringLiteral("SUCCEEDED"), QString());
    QCOMPARE(f.state(), MissionState::Parking);

    f.controller->requestMissionCancel();

    QCOMPARE(world().dockCancels, 1);
    QVERIFY(world().historyTypes().contains(QStringLiteral("MISSION_CANCEL_REQUESTED")));

    f.ros.dockResultReceived(false, QString());

    QVERIFY(world().historyTypes().contains(QStringLiteral("PARKING_CANCELLED")));
    QCOMPARE(f.statusAt(0), MissionQueueStatus::Cancelled);
}

void MissionControllerTest::cancelingWaitingQueueItemRemovesItWithoutTouchingNav2()
{
    Fixture f;
    f.build();
    f.call(QStringLiteral("CZ1"));
    f.call(QStringLiteral("CZ2"));
    const QString waitingId = f.queueIdAt(1);

    f.controller->requestQueueCancel(waitingId);

    QCOMPARE(f.controller->waitingQueue().size(), 1);
    QCOMPARE(f.controller->waitingQueue().first().status, MissionQueueStatus::Active);
    QCOMPARE(f.nav.cancelCalls, 0);
    QVERIFY(world().historyTypes().contains(QStringLiteral("QUEUE_CANCELLED")));
    // 삭제 전에 Cancelled 상태가 저장소에 반영돼야 DB에 남는다.
    bool persistedAsCancelled = false;
    for (const MissionQueueItem &item : world().savedQueue) {
        persistedAsCancelled = persistedAsCancelled
            || (item.queueId == waitingId && item.status == MissionQueueStatus::Cancelled);
    }
    QVERIFY(persistedAsCancelled);

    // 모르는 ID는 아무 일도 하지 않는다.
    f.controller->requestQueueCancel(QStringLiteral("Q-UNKNOWN"));
    QCOMPARE(f.controller->waitingQueue().size(), 1);
}

void MissionControllerTest::cancelingActiveQueueItemCancelsMission()
{
    Fixture f;
    f.build();
    f.startNavigating();

    f.controller->requestQueueCancel(f.queueIdAt(0));

    QCOMPARE(f.state(), MissionState::Cancelling);
    QCOMPARE(f.nav.cancelCalls, 1);
}

void MissionControllerTest::divertCancelsCurrentGoalThenStartsSelectedCall()
{
    Fixture f;
    f.build();
    f.call(QStringLiteral("CZ1"));
    f.call(QStringLiteral("CZ2"));
    f.nav.navigationGoalAccepted();
    const QString divertTarget = f.queueIdAt(1);

    f.controller->requestQueueDivert(divertTarget);

    QCOMPARE(f.state(), MissionState::Diverting);
    QCOMPARE(f.nav.cancelCalls, 1);
    QCOMPARE(f.nav.navigateCalls.size(), 1); // 취소 확인 전에는 새 goal을 보내지 않는다

    f.nav.navigationResult(QStringLiteral("CANCELED"), QString());

    QCOMPARE(f.statusAt(0), MissionQueueStatus::Diverted);
    QCOMPARE(f.statusAt(1), MissionQueueStatus::Active);
    QCOMPARE(f.nav.navigateCalls.size(), 2);
    QCOMPARE(f.nav.navigateCalls.last().x, 3.0);
    QVERIFY(world().historyTypes().contains(QStringLiteral("MISSION_DIVERTED")));
    QVERIFY(world().historyTypes().contains(QStringLiteral("MISSION_DIVERTED_STARTED")));
}

void MissionControllerTest::divertIsRefusedAfterArrival()
{
    Fixture f;
    f.build();
    f.call(QStringLiteral("CZ1"));
    f.call(QStringLiteral("CZ2"));
    f.nav.navigationGoalAccepted();
    f.nav.navigationResult(QStringLiteral("SUCCEEDED"), QString());
    QCOMPARE(f.state(), MissionState::Arrived);

    f.controller->requestQueueDivert(f.queueIdAt(1));

    QCOMPARE(f.state(), MissionState::Arrived);
    QCOMPARE(f.nav.cancelCalls, 0);
    QCOMPARE(f.statusAt(1), MissionQueueStatus::Waiting);
}

// ---- 홈 복귀·주차 -------------------------------------------------------------

void MissionControllerTest::returnHomeFromIdleDrivesToHome()
{
    Fixture f;
    f.build();

    f.controller->requestReturnHome();

    QCOMPARE(f.state(), MissionState::ReturningHome);
    QCOMPARE(f.nav.navigateCalls.size(), 1);
    QCOMPARE(f.nav.navigateCalls.first().x, 0.0);
    QCOMPARE(f.nav.navigateCalls.first().y, 0.0);
    QVERIFY(world().audio.contains(QStringLiteral("홈으로 복귀합니다")));
    QCOMPARE(world().commands.size(), 1);
    QCOMPARE(world().commands.first().command, QStringLiteral("return_home"));
    QVERIFY(f.controller->missionSnapshot().returningHome);
}

void MissionControllerTest::returnHomeWithoutHomeWaypointFailsCleanly()
{
    Fixture f(false);
    f.build();

    f.controller->requestReturnHome();

    QCOMPARE(f.state(), MissionState::Idle);
    QVERIFY(f.nav.navigateCalls.isEmpty());
    QVERIFY(world().commands.isEmpty());
    QVERIFY(world().historyTypes().contains(QStringLiteral("RETURN_HOME_FAILED")));
}

void MissionControllerTest::returnHomeWhileNavigatingCancelsThenDrivesHome()
{
    Fixture f;
    f.build();
    f.startNavigating();

    f.controller->requestReturnHome();

    QCOMPARE(f.state(), MissionState::Cancelling);
    QCOMPARE(f.nav.cancelCalls, 1);
    QCOMPARE(f.nav.navigateCalls.size(), 1); // 취소 확인 전

    f.nav.navigationResult(QStringLiteral("CANCELED"), QString());

    QCOMPARE(f.state(), MissionState::ReturningHome);
    QCOMPARE(f.nav.navigateCalls.size(), 2);
    QCOMPARE(f.nav.navigateCalls.last().x, 0.0);
}

void MissionControllerTest::returnHomeSucceedsEvenIfSuccessArrivesBeforeCancel()
{
    // 취소 확인(CANCELED)보다 기존 goal의 SUCCEEDED가 먼저 도착하는 경쟁 상태에서도
    // 홈 복귀 요청이 무시되면 안 된다.
    Fixture f;
    f.build();
    f.startNavigating();
    f.controller->requestReturnHome();
    QCOMPARE(f.state(), MissionState::Cancelling);

    f.nav.navigationResult(QStringLiteral("SUCCEEDED"), QString());

    QCOMPARE(f.state(), MissionState::ReturningHome);
    QVERIFY(!world().historyTypes().contains(QStringLiteral("MISSION_ARRIVED")));
    QCOMPARE(f.nav.navigateCalls.last().x, 0.0);
}

void MissionControllerTest::homeArrivalStartsParkingAndCompletes()
{
    Fixture f;
    f.build();
    f.controller->requestReturnHome();

    f.nav.navigationResult(QStringLiteral("SUCCEEDED"), QString());

    QCOMPARE(f.state(), MissionState::Parking);
    QCOMPARE(world().dockRequests, 1);
    QVERIFY(world().historyTypes().contains(QStringLiteral("PARKING_STARTED")));

    f.ros.dockResultReceived(true, QString());

    // 완료 체크마크를 보여주려고 바로 Idle로 지우지 않는다.
    QCOMPARE(f.state(), MissionState::Completed);
    QVERIFY(world().audio.contains(QStringLiteral("도킹 완료했습니다")));
    QVERIFY(world().historyTypes().contains(QStringLiteral("PARKING_COMPLETED")));
}

void MissionControllerTest::parkingFailureFailsMission()
{
    Fixture f;
    f.build();
    f.controller->requestReturnHome();
    f.nav.navigationResult(QStringLiteral("SUCCEEDED"), QString());
    QCOMPARE(f.state(), MissionState::Parking);

    f.ros.dockResultReceived(false, QStringLiteral("marker not found"));

    QVERIFY(world().historyTypes().contains(QStringLiteral("PARKING_FAILED")));
    QVERIFY(!world().historyTypes().contains(QStringLiteral("PARKING_COMPLETED")));
}

void MissionControllerTest::newCallStartsImmediatelyAfterCompletion()
{
    Fixture f;
    f.build();
    f.controller->requestReturnHome();
    f.nav.navigationResult(QStringLiteral("SUCCEEDED"), QString());
    f.ros.dockResultReceived(true, QString());
    QCOMPARE(f.state(), MissionState::Completed);
    const int goalsBefore = f.nav.navigateCalls.size();

    f.call(QStringLiteral("CZ2"));

    QCOMPARE(f.state(), MissionState::Starting);
    QCOMPARE(f.nav.navigateCalls.size(), goalsBefore + 1);
    QCOMPARE(f.nav.navigateCalls.last().x, 3.0);
}

// ---- Pi 연동·연결 상태·진행률 ---------------------------------------------------

void MissionControllerTest::piPayloadRemovedIsRecordedOnce()
{
    Fixture f;
    f.build();
    f.startNavigating();
    f.nav.navigationResult(QStringLiteral("SUCCEEDED"), QString());
    const MissionSnapshot arrived = f.controller->missionSnapshot();
    QVERIFY(!arrived.payloadRemoved);

    f.ros.piReturnStatusReceived(QStringLiteral("payload_removed"), arrived.missionId, arrived.queueId, 0, 1, false);
    f.ros.piReturnStatusReceived(QStringLiteral("payload_removed"), arrived.missionId, arrived.queueId, 0, 1, true); // 재전송

    QVERIFY(f.controller->missionSnapshot().payloadRemoved);
    QCOMPARE(world().historyTypes().count(QStringLiteral("PAYLOAD_REMOVED_DETECTED")), 1);
}

void MissionControllerTest::piStatusForOtherMissionIsIgnored()
{
    Fixture f;
    f.build();
    f.startNavigating();
    f.nav.navigationResult(QStringLiteral("SUCCEEDED"), QString());

    f.ros.piReturnStatusReceived(QStringLiteral("payload_removed"), QStringLiteral("MR-OTHER"), QStringLiteral("Q-OTHER"), 0, 1, false);

    QVERIFY(!f.controller->missionSnapshot().payloadRemoved);
    QVERIFY(!world().historyTypes().contains(QStringLiteral("PAYLOAD_REMOVED_DETECTED")));
}

void MissionControllerTest::nav2ConnectionIsSyncedAtStartupAndOnChange()
{
    Fixture f;
    f.nav.nav2Connected = true; // 컨트롤러보다 먼저 연결된 경우
    f.build();
    QVERIFY(f.controller->nav2Connected());

    QSignalSpy spy(f.controller.get(), &MissionController::connectionStateChanged);
    f.nav.nav2ConnectionChanged(false);

    QVERIFY(!f.controller->nav2Connected());
    QCOMPARE(spy.count(), 1);
}

void MissionControllerTest::anomalyTitleFollowsConnectionState()
{
    Fixture f;
    f.nav.nav2Connected = true;
    f.build();
    // 아직 ROS 하트비트가 없으므로 통신 이상으로 보인다.
    QVERIFY(f.controller->anomalyTitle().contains(QStringLiteral("통신 확인 필요")));

    f.ros.missionHeartbeat(QStringLiteral("pi"), QDateTime::currentDateTime());

    QVERIFY(f.controller->rosConnected());
    QVERIFY(f.controller->anomalyTitle().contains(QStringLiteral("모니터링 중")));

    // 오래된 하트비트는 연결 끊김으로 본다.
    f.ros.missionHeartbeat(QStringLiteral("pi"), QDateTime::currentDateTime().addSecs(-30));
    QVERIFY(!f.controller->rosConnected());
}

void MissionControllerTest::retryRunsOnceAndRecordsOutcome()
{
    Fixture f;
    f.nav.nav2Connected = true;
    f.build();

    f.controller->requestRetry();
    f.controller->requestRetry(); // 진행 중에는 무시

    QVERIFY(f.controller->retryInProgress());
    QCOMPARE(f.nav.retryCalls, 1);
    QCOMPARE(world().rosRetries, 1);

    f.ros.communicationRetryFinished(true, QStringLiteral("ok"));

    QVERIFY(!f.controller->retryInProgress());
    QVERIFY(world().historyTypes().contains(QStringLiteral("ROS_RETRY_SUCCEEDED")));

    // Nav2가 안 붙어 있으면 ROS가 성공해도 실패로 기록한다.
    f.nav.nav2ConnectionChanged(false);
    f.controller->requestRetry();
    f.ros.communicationRetryFinished(true, QStringLiteral("ok"));
    QVERIFY(world().historyTypes().contains(QStringLiteral("ROS_RETRY_FAILED")));
}

void MissionControllerTest::progressFollowsRemainingDistance()
{
    Fixture f;
    f.nav.hasAmclPose = true; // 로봇이 (0,0)에 있다 → CZ1(1,2)까지 √5 m
    f.build();
    f.startNavigating();
    QCOMPARE(f.controller->missionSnapshot().originName, QStringLiteral("홈"));
    const double total = std::hypot(1.0, 2.0);

    f.nav.feedbackDistanceRemaining(total);
    QCOMPARE(f.controller->missionSnapshot().progressPercent, 0);

    f.nav.feedbackDistanceRemaining(total / 2.0);
    QCOMPARE(f.controller->missionSnapshot().progressPercent, 50);

    f.nav.feedbackDistanceRemaining(0.0);
    QCOMPARE(f.controller->missionSnapshot().progressPercent, 100);

    // 처음 거리보다 멀어져도 0~100 범위를 벗어나지 않는다.
    f.nav.feedbackDistanceRemaining(total * 3.0);
    QCOMPARE(f.controller->missionSnapshot().progressPercent, 0);
}

// ---- Pi 자동 복귀와 DB 세션 기록 -------------------------------------------------

void MissionControllerTest::piReturningOnlyUpdatesScreenAndDoesNotResendGoal()
{
    Fixture f;
    f.build();
    const auto ids = f.arriveAtCallZone();
    const int goalsBefore = f.nav.navigateCalls.size();

    f.ros.piReturnStatusReceived(QStringLiteral("returning"), ids.first, ids.second, -1, 1, false);

    QCOMPARE(f.state(), MissionState::ReturningHome);
    QVERIFY(f.controller->missionSnapshot().returningHome);
    QCOMPARE(f.nav.navigateCalls.size(), goalsBefore); // 복귀 goal은 Pi가 이미 보냈다
    QCOMPARE(world().historyTypes().count(QStringLiteral("RETURN_HOME_STARTED")), 1);
}

void MissionControllerTest::piReturningOpensDbSessionWithCachedRoute()
{
    Fixture f;
    f.enableDb();
    f.nav.hasAmclPose = true;
    f.nav.amclX = 1.5;
    f.nav.amclY = 2.5;
    f.build();
    const auto ids = f.arriveAtCallZone();

    f.ros.piReturnStatusReceived(QStringLiteral("returning"), ids.first, ids.second, -1, 1, false);

    QCOMPARE(f.overview.started.size(), 1);
    const FakeOverviewService::StartedSession session = f.overview.started.first();
    QCOMPARE(session.robotId, 7);
    QCOMPARE(session.mode, QStringLiteral("auto"));
    QCOMPARE(session.name, QStringLiteral("Pi 로컬 복귀"));
    // 도착 시 Pi에 알려준 경로(HOME 한 점)를 그대로 기록한다.
    QCOMPARE(session.goals.size(), 1);
    QCOMPARE(session.goals.first().x, 0.0);
    QCOMPARE(session.goals.first().y, 0.0);
    QVERIFY(session.hasStartPose);
    QCOMPARE(session.startPose.x, 1.5);
    QCOMPARE(session.startPose.y, 2.5);
}

void MissionControllerTest::piReturningMarksEachPassedGoalOnce()
{
    Fixture f;
    f.enableDb();
    f.build();
    const auto ids = f.arriveAtCallZone();
    f.ros.piReturnStatusReceived(QStringLiteral("returning"), ids.first, ids.second, -1, 3, false);
    QVERIFY(f.overview.marked.isEmpty()); // 아직 goal이 나가기 전(-1)

    f.ros.piReturnStatusReceived(QStringLiteral("returning"), ids.first, ids.second, 2, 3, false);
    QCOMPARE(f.overview.marked.size(), 2); // 0, 1번 지점이 지나갔다
    QCOMPARE(f.overview.marked.at(0).second, 0);
    QCOMPARE(f.overview.marked.at(1).second, 1);

    // 같은 진행률을 다시 받아도 중복 기록하지 않는다.
    f.ros.piReturnStatusReceived(QStringLiteral("returning"), ids.first, ids.second, 2, 3, false);
    QCOMPARE(f.overview.marked.size(), 2);

    f.ros.piReturnStatusReceived(QStringLiteral("returning"), ids.first, ids.second, 3, 3, false);
    QCOMPARE(f.overview.marked.size(), 3);
    QCOMPARE(f.overview.marked.last().second, 2);
}

void MissionControllerTest::piReturningSessionNeedsExactlyOneRobot()
{
    for (const int robotCount : {0, 2}) {
        world().reset();
        Fixture f;
        f.enableDb(robotCount);
        f.build();
        const auto ids = f.arriveAtCallZone();

        f.ros.piReturnStatusReceived(QStringLiteral("returning"), ids.first, ids.second, -1, 1, false);

        // 로봇이 몇 대인지 모르면 엉뚱한 로봇의 세션으로 기록하지 않는다.
        QVERIFY2(f.overview.started.isEmpty(), qPrintable(QStringLiteral("robots=%1").arg(robotCount)));
        QCOMPARE(f.state(), MissionState::ReturningHome); // 화면 상태는 그대로 따라간다
    }
}

void MissionControllerTest::piReturningContinuesWhenDbSessionFails()
{
    Fixture f;
    f.enableDb();
    f.overview.startShouldFail = true;
    f.build();
    const auto ids = f.arriveAtCallZone();

    f.ros.piReturnStatusReceived(QStringLiteral("returning"), ids.first, ids.second, -1, 1, false);
    f.ros.piReturnStatusReceived(QStringLiteral("returning"), ids.first, ids.second, 1, 1, false);

    QCOMPARE(f.state(), MissionState::ReturningHome);
    QCOMPARE(f.overview.started.size(), 1);
    QVERIFY(f.overview.marked.isEmpty()); // 세션이 없으니 진행률도 기록하지 않는다

    f.ros.dockResultReceived(true, QString());
    QVERIFY(f.overview.finished.isEmpty()); // 닫을 세션도 없다
    QCOMPARE(f.state(), MissionState::Completed);
}

void MissionControllerTest::piReturningIsIgnoredUnlessArrived()
{
    Fixture f;
    f.enableDb();
    f.build();
    f.startNavigating();
    const MissionSnapshot snapshot = f.controller->missionSnapshot();

    f.ros.piReturnStatusReceived(QStringLiteral("returning"), snapshot.missionId, snapshot.queueId, -1, 1, false);

    QCOMPARE(f.state(), MissionState::Navigating);
    QVERIFY(f.overview.started.isEmpty());
    QVERIFY(!world().historyTypes().contains(QStringLiteral("RETURN_HOME_STARTED")));
}

void MissionControllerTest::piReplayedReturningDoesNotOpenSecondSession()
{
    Fixture f;
    f.enableDb();
    f.build();
    const auto ids = f.arriveAtCallZone();

    f.ros.piReturnStatusReceived(QStringLiteral("returning"), ids.first, ids.second, -1, 1, false);
    f.ros.piReturnStatusReceived(QStringLiteral("returning"), ids.first, ids.second, -1, 1, true); // Wi-Fi 복구 후 재전송

    QCOMPARE(f.overview.started.size(), 1);
    QCOMPARE(world().historyTypes().count(QStringLiteral("RETURN_HOME_STARTED")), 1);
}

void MissionControllerTest::piReturnFailedFailsMissionAndClosesSession()
{
    Fixture f;
    f.enableDb();
    f.build();
    const auto ids = f.arriveAtCallZone();
    f.ros.piReturnStatusReceived(QStringLiteral("returning"), ids.first, ids.second, -1, 1, false);

    f.ros.piReturnStatusReceived(QStringLiteral("return_failed"), ids.first, ids.second, 0, 1, false);

    QCOMPARE(f.statusAt(0), MissionQueueStatus::Failed);
    QVERIFY(world().historyTypes().contains(QStringLiteral("RETURN_HOME_FAILED")));
    QCOMPARE(f.overview.finished.size(), 1);
    QCOMPARE(f.overview.finished.first().sessionId, 100);
    QVERIFY(!f.overview.finished.first().success);
    QVERIFY(!f.overview.finished.first().cancelled);
}

void MissionControllerTest::piReturnFailedIsIgnoredWhileNavigating()
{
    Fixture f;
    f.build();
    f.startNavigating();
    const MissionSnapshot snapshot = f.controller->missionSnapshot();

    f.ros.piReturnStatusReceived(QStringLiteral("return_failed"), snapshot.missionId, snapshot.queueId, 0, 1, false);

    QCOMPARE(f.state(), MissionState::Navigating);
    QCOMPARE(f.statusAt(0), MissionQueueStatus::Active);
}

void MissionControllerTest::dockSuccessClosesPiSessionAsSucceeded()
{
    Fixture f;
    f.enableDb();
    f.build();
    const auto ids = f.arriveAtCallZone();
    f.ros.piReturnStatusReceived(QStringLiteral("returning"), ids.first, ids.second, -1, 1, false);

    f.ros.dockResultReceived(true, QString());

    QCOMPARE(f.state(), MissionState::Completed);
    QCOMPARE(f.overview.finished.size(), 1);
    QVERIFY(f.overview.finished.first().success);
    QVERIFY(!f.overview.finished.first().cancelled);
}

void MissionControllerTest::cancelWhileReturningClosesPiSessionAsCancelled()
{
    Fixture f;
    f.enableDb();
    f.build();
    const auto ids = f.arriveAtCallZone();
    f.ros.piReturnStatusReceived(QStringLiteral("returning"), ids.first, ids.second, -1, 1, false);
    QCOMPARE(f.state(), MissionState::ReturningHome);

    f.controller->requestMissionCancel();
    QCOMPARE(f.state(), MissionState::Cancelling);
    QCOMPARE(f.nav.cancelCalls, 1);
    f.nav.navigationResult(QStringLiteral("CANCELED"), QString());

    QCOMPARE(f.overview.finished.size(), 1);
    QVERIFY(!f.overview.finished.first().success);
    QVERIFY(f.overview.finished.first().cancelled);
}

void MissionControllerTest::dockResultWhileArrivedStillFinishesMission()
{
    // Wi-Fi가 끊긴 동안 Pi가 복귀·주차를 끝냈다면, PC는 Arrived 상태에서 결과를 처음 받는다.
    Fixture f;
    f.build();
    f.arriveAtCallZone();
    QCOMPARE(f.state(), MissionState::Arrived);

    f.ros.dockResultReceived(true, QString());

    QCOMPARE(f.state(), MissionState::Completed);
    QVERIFY(world().historyTypes().contains(QStringLiteral("PARKING_COMPLETED")));
}

void MissionControllerTest::dockResultIsIgnoredWhileNavigating()
{
    Fixture f;
    f.build();
    f.startNavigating();

    f.ros.dockResultReceived(true, QString());

    QCOMPARE(f.state(), MissionState::Navigating);
    QVERIFY(!world().historyTypes().contains(QStringLiteral("PARKING_COMPLETED")));
}

void MissionControllerTest::piArmedPhaseHasNoEffect()
{
    Fixture f;
    f.build();
    const auto ids = f.arriveAtCallZone();
    const QStringList before = world().historyTypes();

    f.ros.piReturnStatusReceived(QStringLiteral("armed"), ids.first, ids.second, -1, 1, false);

    QCOMPARE(f.state(), MissionState::Arrived);
    QCOMPARE(world().historyTypes(), before);
}

// ---- 경계 조건 ----------------------------------------------------------------

void MissionControllerTest::announceTestDriveSpeaksLabelOrDefault()
{
    Fixture f;
    f.build();

    f.controller->announceTestDrive(QStringLiteral("병실1"));
    f.controller->announceTestDrive(QString());

    QCOMPARE(world().audio.size(), 2);
    QCOMPARE(world().audio.at(0), QStringLiteral("병실1(으)로 이동합니다"));
    QCOMPARE(world().audio.at(1), QStringLiteral("지정한 위치로 이동합니다"));
}

void MissionControllerTest::confirmStartNextQueueRequiresIdleAndWaitingItem()
{
    Fixture f;
    f.build();
    f.call(QStringLiteral("CZ1"));
    f.call(QStringLiteral("CZ2"));
    f.nav.navigationGoalAccepted();
    const QString waitingId = f.queueIdAt(1);

    // 미션 진행 중에는 확인 요청이 와도 무시한다.
    f.controller->confirmStartNextQueue(waitingId);
    QCOMPARE(f.nav.navigateCalls.size(), 1);

    // 모르는 ID, 또는 이미 진행 중인 항목도 무시한다.
    f.controller->requestMissionCancel();
    f.nav.navigationResult(QStringLiteral("CANCELED"), QString());
    QCOMPARE(f.state(), MissionState::Idle);
    f.controller->confirmStartNextQueue(QStringLiteral("Q-UNKNOWN"));
    f.controller->confirmStartNextQueue(f.queueIdAt(0)); // 이미 Cancelled
    QCOMPARE(f.nav.navigateCalls.size(), 1);

    f.controller->confirmStartNextQueue(waitingId);
    QCOMPARE(f.nav.navigateCalls.size(), 2);
}

void MissionControllerTest::declineNextQueueKeepsWaitingItemAndRecords()
{
    Fixture f;
    f.build();
    f.call(QStringLiteral("CZ1"));
    f.call(QStringLiteral("CZ2"));
    f.nav.navigationGoalAccepted();
    f.controller->requestMissionCancel();
    f.nav.navigationResult(QStringLiteral("CANCELED"), QString());

    f.controller->declineNextQueue();

    QVERIFY(world().historyTypes().contains(QStringLiteral("QUEUE_START_DECLINED")));
    QCOMPARE(f.statusAt(1), MissionQueueStatus::Waiting); // 보류일 뿐 삭제되지 않는다
    QCOMPARE(f.nav.navigateCalls.size(), 1);
    QCOMPARE(f.state(), MissionState::Idle);
}

void MissionControllerTest::divertFromIdleStartsSelectedCallDirectly()
{
    Fixture f;
    f.build();
    f.call(QStringLiteral("CZ1"));
    f.call(QStringLiteral("CZ2"));
    f.nav.navigationGoalAccepted();
    f.controller->requestMissionCancel();
    f.nav.navigationResult(QStringLiteral("CANCELED"), QString());
    QCOMPARE(f.state(), MissionState::Idle);
    const int cancelCallsBefore = f.nav.cancelCalls;

    f.controller->requestQueueDivert(f.queueIdAt(1));

    QCOMPARE(f.nav.cancelCalls, cancelCallsBefore); // 취소할 goal이 없으니 바로 출발
    QCOMPARE(f.nav.navigateCalls.size(), 2);
    QCOMPARE(f.state(), MissionState::Starting);
}

void MissionControllerTest::divertIsIgnoredWhileCancelling()
{
    Fixture f;
    f.build();
    f.call(QStringLiteral("CZ1"));
    f.call(QStringLiteral("CZ2"));
    f.nav.navigationGoalAccepted();
    f.controller->requestMissionCancel();
    QCOMPARE(f.state(), MissionState::Cancelling);

    f.controller->requestQueueDivert(f.queueIdAt(1));

    QCOMPARE(f.state(), MissionState::Cancelling);
    QCOMPARE(f.nav.cancelCalls, 1);
}

void MissionControllerTest::divertWithUnknownQueueIdIsIgnored()
{
    Fixture f;
    f.build();
    f.startNavigating();

    f.controller->requestQueueDivert(QStringLiteral("Q-UNKNOWN"));

    QCOMPARE(f.state(), MissionState::Navigating);
    QCOMPARE(f.nav.cancelCalls, 0);
}

void MissionControllerTest::cancelBeforeGoalAcceptedIsAllowed()
{
    Fixture f;
    f.build();
    f.call(QStringLiteral("CZ1"));
    QCOMPARE(f.state(), MissionState::Starting);

    f.controller->requestMissionCancel();

    QCOMPARE(f.state(), MissionState::Cancelling);
    QCOMPARE(f.nav.cancelCalls, 1);
}

void MissionControllerTest::returnHomeFromArrivalWithoutHomeFailsMission()
{
    Fixture f(false);
    f.build();
    f.arriveAtCallZone();
    QCOMPARE(f.state(), MissionState::Arrived);

    f.controller->requestReturnHome();

    QVERIFY(world().historyTypes().contains(QStringLiteral("RETURN_HOME_FAILED")));
    QCOMPARE(f.statusAt(0), MissionQueueStatus::Failed);
    for (const FakeWorld::Command &command : world().commands) {
        QVERIFY(command.command != QStringLiteral("return_home")); // 실패한 요청을 Pi에 알리지 않는다
    }
}

void MissionControllerTest::returnHomeIsNotRequestedTwice()
{
    Fixture f;
    f.build();

    f.controller->requestReturnHome();
    f.controller->requestReturnHome();

    QCOMPARE(f.nav.navigateCalls.size(), 1);
    QCOMPARE(world().commands.size(), 1);
}

void MissionControllerTest::returnHomeIsRefusedWhileParking()
{
    Fixture f;
    f.build();
    f.controller->requestReturnHome();
    f.nav.navigationResult(QStringLiteral("SUCCEEDED"), QString());
    QCOMPARE(f.state(), MissionState::Parking);
    const int goalsBefore = f.nav.navigateCalls.size();
    const int commandsBefore = world().commands.size();

    f.controller->requestReturnHome();

    QCOMPARE(f.state(), MissionState::Parking);
    QCOMPARE(f.nav.navigateCalls.size(), goalsBefore);
    QCOMPARE(world().commands.size(), commandsBefore);
}

void MissionControllerTest::repositoryChangeNotifiesOnlyWhileMissionActive()
{
    Fixture f;
    f.build();
    QSignalSpy spy(f.controller.get(), &MissionController::missionChanged);

    f.repo.upsertWaypoint(makeWaypoint(QStringLiteral("CZ3"), QStringLiteral("병실3"), 5.0, 5.0, 0.0, WaypointMarkerKind::CallZone));
    QCOMPARE(spy.count(), 0); // 미션이 없으면 화면을 다시 그릴 필요가 없다

    f.call(QStringLiteral("CZ1"));
    spy.clear();
    f.repo.upsertWaypoint(makeWaypoint(QStringLiteral("CZ3"), QStringLiteral("병실3-변경"), 5.0, 5.0, 0.0, WaypointMarkerKind::CallZone));
    QVERIFY(spy.count() >= 1);
}

void MissionControllerTest::amclPoseUpdatesCurrentWaypointNameDuringMission()
{
    Fixture f;
    f.build();
    QSignalSpy spy(f.controller.get(), &MissionController::missionChanged);

    f.nav.amclPoseUpdated(0.1, 0.1, 0.0); // 미션 전: 화면 갱신 없음
    QCOMPARE(spy.count(), 0);

    f.startNavigating();
    f.nav.amclPoseUpdated(2.9, 3.9, 0.0); // CZ2(3,4)에 가장 가깝다
    QCOMPARE(f.controller->missionSnapshot().currentWaypointName, QStringLiteral("병실2"));

    f.nav.amclPoseUpdated(0.0, 0.1, 0.0); // HOME(0,0)
    QCOMPARE(f.controller->missionSnapshot().currentWaypointName, QStringLiteral("홈"));
}

QTEST_GUILESS_MAIN(MissionControllerTest)
#include "test_missioncontroller.moc"
