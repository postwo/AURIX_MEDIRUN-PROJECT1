#pragma once

// MissionController 시험용 대역(fake).
// MissionRosInterface / MissionStorageService / waypointfile 은 fakes.cpp 가 실제
// 구현 대신 같은 이름으로 정의한다. 시험 실행 파일에는 실제 *.cpp 를 링크하지
// 않으므로 ROS·DB·YAML 없이 컨트롤러 로직만 검증할 수 있다.

#include "../src/models/missiontypes.h"
#include "../src/ros/navigationservice.h"

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
