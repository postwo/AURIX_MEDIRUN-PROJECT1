#include "fakes.h"

#include "../src/services/missionrosinterface.h"
#include "../src/services/missionstorageservice.h"
#include "../src/services/waypointfile.h"

FakeWorld &FakeWorld::instance()
{
    static FakeWorld world;
    return world;
}

// ---- MissionRosInterface 대역 -------------------------------------------------

struct MissionRosInterface::Impl
{
};

MissionRosInterface::MissionRosInterface(QObject *parent)
    : QObject(parent)
{
}

MissionRosInterface::~MissionRosInterface() = default;

void MissionRosInterface::publishMissionCommand(const QString &command, const QString &missionId, const QString &queueId, const QString &)
{
    FakeWorld::instance().commands.append({command, missionId, queueId});
}

void MissionRosInterface::publishAudioAnnouncement(const QString &text)
{
    FakeWorld::instance().audio << text;
}

void MissionRosInterface::retryCommunication()
{
    ++FakeWorld::instance().rosRetries;
}

void MissionRosInterface::requestDocking()
{
    ++FakeWorld::instance().dockRequests;
}

void MissionRosInterface::cancelDocking()
{
    ++FakeWorld::instance().dockCancels;
}

void MissionRosInterface::publishArrivalNotice(bool armed, const QString &missionId, const QString &, double, double, double,
                                               const QString &, const QVector<NavPose> &)
{
    FakeWorld::instance().arrivalNotices.append({armed, missionId});
}

// ---- MissionStorageService 대역 -----------------------------------------------

MissionStorageService::MissionStorageService(QObject *parent)
    : QObject(parent)
{
}

void MissionStorageService::setSessionContext(const AppSession &session)
{
    m_session = session;
}

QVector<MissionQueueItem> MissionStorageService::loadQueue()
{
    return FakeWorld::instance().initialQueue;
}

QVector<MissionHistoryEntry> MissionStorageService::loadTodayHistory()
{
    return FakeWorld::instance().initialHistory;
}

void MissionStorageService::saveQueue(QVector<MissionQueueItem> &queueItems)
{
    FakeWorld::instance().savedQueue = queueItems;
}

void MissionStorageService::appendHistory(const MissionHistoryEntry &entry)
{
    FakeWorld::instance().appendedHistory.append(entry);
}

// ---- waypointfile 대역 (yaml-cpp 불필요) ---------------------------------------

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
