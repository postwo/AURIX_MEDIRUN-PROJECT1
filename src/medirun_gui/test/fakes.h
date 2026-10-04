#pragma once

// MissionController 시험용 대역(fake).
// MissionRosInterface / MissionStorageService / waypointfile 은 fakes.cpp 가 실제
// 구현 대신 같은 이름으로 정의한다. 시험 실행 파일에는 실제 *.cpp 를 링크하지
// 않으므로 ROS·DB·YAML 없이 컨트롤러 로직만 검증할 수 있다.

#include "../src/models/missiontypes.h"
#include "../src/ros/navigationservice.h"
#include "../src/services/operationsoverviewservice.h"
#include "../src/services/robotservice.h"

#include <QStringList>
#include <QVector>

// 컨트롤러가 협력 객체에 보낸 호출을 기록하고, 협력 객체가 돌려줄 초기값을 보관한다.
struct FakeWorld
{
    struct Command
    {
        QString command;
        QString missionId;
        QString queueId;
    };
    struct ArrivalNotice
    {
        bool armed {false};
        QString missionId;
    };

    // 저장소가 컨트롤러 생성 시 돌려줄 값
    QVector<MissionQueueItem> initialQueue;
    QVector<MissionHistoryEntry> initialHistory;

    // 컨트롤러가 저장소·ROS 인터페이스로 보낸 값
    QVector<MissionQueueItem> savedQueue;
    QVector<MissionHistoryEntry> appendedHistory;
    QStringList audio;
    QVector<Command> commands;
    QVector<ArrivalNotice> arrivalNotices;
    int dockRequests {0};
    int dockCancels {0};
    int rosRetries {0};

    static FakeWorld &instance();
    void reset() { *this = FakeWorld(); }

    QStringList historyTypes() const
    {
        QStringList types;
        for (const MissionHistoryEntry &entry : appendedHistory) {
            types << entry.eventType;
        }
        return types;
    }
};

class FakeNavigationService : public NavigationService
{
    Q_OBJECT

public:
    struct PoseCall
    {
        double x {0.0};
        double y {0.0};
        double yaw {0.0};
        QString frameId;
        QString label;
    };

    QVector<PoseCall> navigateCalls;
    int followCalls {0};
    int cancelCalls {0};
    int retryCalls {0};
    bool nav2Connected {false};
    bool hasAmclPose {false};
    double amclX {0.0};
    double amclY {0.0};

    void navigateToPose(double x, double y, double yawRad, const QString &frameId, const QString &label = QString()) override
    {
        navigateCalls.append({x, y, yawRad, frameId, label});
    }
    void followWaypoints(const QVector<NavPose> &, const QString &, const QString & = QString()) override { ++followCalls; }
    void navigateThroughPoses(const QVector<NavPose> &, const QString &, const QString & = QString()) override {}
    void cancelGoal() override { ++cancelCalls; }
    void retryConnections() override { ++retryCalls; }
    void publishInitialPose(double, double, double, const QString &) override {}
    void setSpeedLimit(int) override {}
    void driveManual(double, double) override {}
    bool isNav2Connected() const override { return nav2Connected; }
    bool lastKnownAmclPose(double &x, double &y, double &yaw) const override
    {
        if (!hasAmclPose) {
            return false;
        }
        x = amclX;
        y = amclY;
        yaw = 0.0;
        return true;
    }
    bool isNavigating() const override { return false; }
};

class FakeRobotService : public RobotService
{
public:
    QVector<RobotRecord> robots;

    QVector<RobotRecord> listRobots(bool = false) override { return robots; }
    bool registerRobot(const QString &, const QString &, QString &) override { return false; }
    bool deregisterRobot(int, QString &) override { return false; }
};

// DB 운영 기록 대역: MissionController 가 쓰는 세션 기록 호출만 저장하고, 나머지는 실패를 돌려준다.
class FakeOverviewService : public OperationsOverviewService
{
public:
    struct StartedSession
    {
        int robotId {0};
        QString mode;
        QString name;
        QVector<NavGoalPoint> goals;
        bool hasStartPose {false};
        NavGoalPoint startPose;
    };
    struct FinishedSession
    {
        int sessionId {0};
        bool success {false};
        bool cancelled {false};
    };

    bool startShouldFail {false};
    int nextSessionId {100};
    QVector<StartedSession> started;
    QVector<QPair<int, int>> marked; // (sessionId, index)
    QVector<FinishedSession> finished;

    bool startRobotSession(int robotId, const QString &mode, const QString &sessionName, const QVector<NavGoalPoint> &goals,
                           bool hasStartPose, const NavGoalPoint &startPose, int &sessionId, QString &errorMessage) override
    {
        started.append({robotId, mode, sessionName, goals, hasStartPose, startPose});
        if (startShouldFail) {
            errorMessage = QStringLiteral("db down");
            return false;
        }
        sessionId = nextSessionId++;
        return true;
    }
    bool markGoalReached(int sessionId, int reachedIndex, QString &) override
    {
        marked.append({sessionId, reachedIndex});
        return true;
    }
    bool finishRobotSession(int sessionId, bool success, bool cancelled, QString &) override
    {
        finished.append({sessionId, success, cancelled});
        return true;
    }

    // 컨트롤러가 쓰지 않는 나머지 인터페이스
    bool fetchSummary(OperationsSummaryMetrics &, QString &) override { return false; }
    bool fetchRobotStatus(QVector<RobotStatusRow> &, QString &) override { return false; }
    bool fetchSystemHealth(QVector<SystemHealthRow> &, QString &) override { return false; }
    bool fetchRecentEvents(int, QVector<OperationsEventRow> &, QString &) override { return false; }
    bool fetchSystemEvents(const QDate &, const QDate &, const QString &, QVector<OperationsEventRow> &, QString &) override { return false; }
    bool recordConnectionStatus(int, const QString &, const QString &, const QString &, QString &) override { return false; }
    bool recordSystemEvent(const QString &, const QString &, const QString &, QString &) override { return false; }
    bool recordControlCommand(int, const QString &, const QString &, bool, const QString &, QString &) override { return false; }
    bool recordUserAction(int, int, int, const QString &, const QString &, const QString &, QString &) override { return false; }
    bool loadWaypointLibraryByName(int, const QString &, QString &, WaypointMap &, TrajectoryMap &, QString &) override { return false; }
    bool saveWaypointLibrary(int, const QString &, const QString &, const WaypointMap &, const TrajectoryMap &, QString &) override { return false; }
    bool deregisterHomeEntry(int, QString &) override { return false; }
    bool loadActiveWaypointEntries(int, QVector<WaypointDef> &, QString &) override { return false; }
    bool upsertWaypointEntry(int, const QString &, const QString &, double, double, double, int &, QString &) override { return false; }
    bool deregisterWaypointEntry(int, const QString &, QString &) override { return false; }
    bool loadActiveTrajectoryEntries(int, QVector<TrajectoryRow> &, QString &) override { return false; }
    bool upsertTrajectoryEntry(int, const QString &, const QVector<TrajectoryStep> &, QString &) override { return false; }
    bool deregisterTrajectoryEntry(int, const QString &, QString &) override { return false; }
    bool findTrajectoriesReferencing(int, const QString &, int, QStringList &, QString &) override { return false; }
    bool loadTrajectoryPosesForButton(int, const QString &, QString &, QVector<NavGoalPoint> &, QString &) override { return false; }
    bool startDrivingSession(int, int &, int &, QString &) override { return false; }
    bool appendDrivingRecord(int, double, double, double, double, double, double, QString &) override { return false; }
    bool finishDrivingSession(int, QString &) override { return false; }
    bool startCameraSession(int, int, const QString &, const QString &, int &, QString &) override { return false; }
    bool recordCameraRecording(int, int, int, int, int, double, const QString &, int &, QString &) override { return false; }
    bool recordFile(const QString &, const QString &, const QString &, const QString &, const QString &, const QString &, const QString &,
                    QString &) override { return false; }
    bool loadActiveCallButtons(QVector<CallButtonRow> &, QString &) override { return false; }
    bool upsertCallButton(const QString &, const QString &, double, double, double, int &, QString &) override { return false; }
    bool deregisterCallButton(const QString &, QString &) override { return false; }
};
