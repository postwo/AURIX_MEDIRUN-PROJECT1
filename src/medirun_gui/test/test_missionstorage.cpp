// MissionStorageService 의 JSON 파일 저장(DB 미연결 시 대체 경로) 시험.
// openDbConnection() 은 stub_dbconnection.cpp 가 항상 실패로 응답한다.

#include "stub_dbconnection.h"

#include "../src/services/missionstorageservice.h"

#include <QDir>
#include <QFile>
#include <QStandardPaths>
#include <QtTest>

namespace {

QString storageDir()
{
    return QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
}

void writeFile(const QString &name, const QByteArray &content)
{
    QDir().mkpath(storageDir());
    QFile file(QDir(storageDir()).filePath(name));
    QVERIFY(file.open(QIODevice::WriteOnly | QIODevice::Truncate));
    file.write(content);
}

MissionQueueItem makeItem(const QString &queueId, MissionQueueStatus status)
{
    MissionQueueItem item;
    item.queueId = queueId;
    item.waypointId = QStringLiteral("CZ-") + queueId;
    item.waypointName = QStringLiteral("병실 ") + queueId;
    item.requestedAt = QDateTime(QDate(2026, 9, 1), QTime(12, 30, 15));
    item.enqueueOrder = 3;
    item.status = status;
    item.requestSource = QStringLiteral("mqtt");
    item.missionId = QStringLiteral("MR-") + queueId;
    item.pose.frameId = QStringLiteral("odom");
    item.pose.positionX = 1.25;
    item.pose.positionY = -3.5;
    item.pose.positionZ = 0.1;
    item.pose.orientationX = 0.01;
    item.pose.orientationY = 0.02;
    item.pose.orientationZ = 0.6;
    item.pose.orientationW = 0.8;
    item.pose.yawRadians = 1.2;
    item.pose.valid = true;
    return item;
}

MissionHistoryEntry makeHistory(const QString &type, const QDateTime &when)
{
    MissionHistoryEntry entry;
    entry.historyId = type + QLatin1Char('_') + when.toString(Qt::ISODate);
    entry.occurredAt = when;
    entry.eventType = type;
    entry.waypointId = QStringLiteral("CZ1");
    entry.waypointName = QStringLiteral("병실1");
    entry.missionId = QStringLiteral("MR-1");
    entry.queueId = QStringLiteral("Q-1");
    entry.result = QStringLiteral("STARTED");
    entry.detailMessage = QStringLiteral("상세 \"따옴표\"와 줄바꿈\n포함");
    return entry;
}

} // namespace

class MissionStorageTest : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void init();

    void startsEmptyWithoutFiles();
    void queueRoundTripKeepsEveryField();
    void everyQueueStatusSurvivesRoundTrip();
    void savingAgainReplacesPreviousQueue();
    void savingEmptyQueueClearsFile();
    void corruptQueueFileYieldsEmptyQueue();
    void nonArrayQueueFileYieldsEmptyQueue();
    void unknownStatusFallsBackToWaiting();
    void historyRoundTripKeepsFieldsAndSpecialCharacters();
    void historyKeepsNewestAppendedFirst();
    void todayHistoryExcludesOtherDaysAndSortsNewestFirst();
    void corruptHistoryFileDoesNotBreakAppend();
    void databaseConnectionIsTriedOnlyOncePerService();
};

void MissionStorageTest::initTestCase()
{
    QStandardPaths::setTestModeEnabled(true); // 실제 사용자 데이터 폴더를 건드리지 않는다
    QCoreApplication::setOrganizationName(QStringLiteral("AURIX-test"));
    QCoreApplication::setApplicationName(QStringLiteral("MissionStorageTest"));
}

void MissionStorageTest::init()
{
    QDir(storageDir()).removeRecursively();
    StubDb::openCalls = 0;
}

void MissionStorageTest::startsEmptyWithoutFiles()
{
    MissionStorageService storage;

    QVERIFY(storage.loadQueue().isEmpty());
    QVERIFY(storage.loadTodayHistory().isEmpty());
}

void MissionStorageTest::queueRoundTripKeepsEveryField()
{
    QVector<MissionQueueItem> items = {makeItem(QStringLiteral("1"), MissionQueueStatus::Active)};
    {
        MissionStorageService writer;
        writer.saveQueue(items);
    }

    MissionStorageService reader; // 새 인스턴스 = 앱 재시작
    const QVector<MissionQueueItem> loaded = reader.loadQueue();

    QCOMPARE(loaded.size(), 1);
    const MissionQueueItem &a = items.first();
    const MissionQueueItem &b = loaded.first();
    QCOMPARE(b.queueId, a.queueId);
    QCOMPARE(b.waypointId, a.waypointId);
    QCOMPARE(b.waypointName, a.waypointName);
    QCOMPARE(b.requestedAt, a.requestedAt);
    QCOMPARE(b.enqueueOrder, a.enqueueOrder);
    QCOMPARE(b.status, a.status);
    QCOMPARE(b.requestSource, a.requestSource);
    QCOMPARE(b.missionId, a.missionId);
    QCOMPARE(b.pose.frameId, a.pose.frameId);
    QCOMPARE(b.pose.positionX, a.pose.positionX);
    QCOMPARE(b.pose.positionY, a.pose.positionY);
    QCOMPARE(b.pose.positionZ, a.pose.positionZ);
    QCOMPARE(b.pose.orientationX, a.pose.orientationX);
    QCOMPARE(b.pose.orientationY, a.pose.orientationY);
    QCOMPARE(b.pose.orientationZ, a.pose.orientationZ);
    QCOMPARE(b.pose.orientationW, a.pose.orientationW);
    QCOMPARE(b.pose.yawRadians, a.pose.yawRadians);
    QCOMPARE(b.pose.valid, a.pose.valid);
}

void MissionStorageTest::everyQueueStatusSurvivesRoundTrip()
{
    const QVector<MissionQueueStatus> statuses = {
        MissionQueueStatus::Waiting, MissionQueueStatus::Active, MissionQueueStatus::Completed,
        MissionQueueStatus::Cancelled, MissionQueueStatus::Diverted, MissionQueueStatus::Failed};
    QVector<MissionQueueItem> items;
    for (int i = 0; i < statuses.size(); ++i) {
        items.append(makeItem(QString::number(i), statuses.at(i)));
    }
    MissionStorageService().saveQueue(items);

    const QVector<MissionQueueItem> loaded = MissionStorageService().loadQueue();

    QCOMPARE(loaded.size(), statuses.size());
    for (int i = 0; i < statuses.size(); ++i) {
        QCOMPARE(loaded.at(i).status, statuses.at(i));
    }
}

void MissionStorageTest::savingAgainReplacesPreviousQueue()
{
    QVector<MissionQueueItem> first = {makeItem(QStringLiteral("1"), MissionQueueStatus::Waiting),
                                       makeItem(QStringLiteral("2"), MissionQueueStatus::Waiting)};
    MissionStorageService storage;
    storage.saveQueue(first);
    QVector<MissionQueueItem> second = {makeItem(QStringLiteral("3"), MissionQueueStatus::Waiting)};

    storage.saveQueue(second);

    const QVector<MissionQueueItem> loaded = MissionStorageService().loadQueue();
    QCOMPARE(loaded.size(), 1);
    QCOMPARE(loaded.first().queueId, QStringLiteral("3"));
}

void MissionStorageTest::savingEmptyQueueClearsFile()
{
    QVector<MissionQueueItem> items = {makeItem(QStringLiteral("1"), MissionQueueStatus::Waiting)};
    MissionStorageService storage;
    storage.saveQueue(items);
    QVector<MissionQueueItem> none;

    storage.saveQueue(none);

    QVERIFY(MissionStorageService().loadQueue().isEmpty());
}

void MissionStorageTest::corruptQueueFileYieldsEmptyQueue()
{
    writeFile(QStringLiteral("mission_queue.json"), "{ not valid json [[[");

    QVERIFY(MissionStorageService().loadQueue().isEmpty());
}

void MissionStorageTest::nonArrayQueueFileYieldsEmptyQueue()
{
    writeFile(QStringLiteral("mission_queue.json"), R"({"queue_id": "Q-1"})");

    QVERIFY(MissionStorageService().loadQueue().isEmpty());
}

void MissionStorageTest::unknownStatusFallsBackToWaiting()
{
    writeFile(QStringLiteral("mission_queue.json"),
              R"([{"queue_id":"Q-1","waypoint_id":"CZ1","status":"SOMETHING_NEW"}])");

    const QVector<MissionQueueItem> loaded = MissionStorageService().loadQueue();

    QCOMPARE(loaded.size(), 1);
    QCOMPARE(loaded.first().status, MissionQueueStatus::Waiting);
    // 빠진 필드는 기본값으로 채운다.
    QCOMPARE(loaded.first().pose.frameId, QStringLiteral("map"));
    QCOMPARE(loaded.first().pose.orientationW, 1.0);
}

void MissionStorageTest::historyRoundTripKeepsFieldsAndSpecialCharacters()
{
    const MissionHistoryEntry entry = makeHistory(QStringLiteral("MISSION_STARTED"), QDateTime::currentDateTime());
    MissionStorageService().appendHistory(entry);

    const QVector<MissionHistoryEntry> loaded = MissionStorageService().loadTodayHistory();

    QCOMPARE(loaded.size(), 1);
    QCOMPARE(loaded.first().historyId, entry.historyId);
    QCOMPARE(loaded.first().eventType, entry.eventType);
    QCOMPARE(loaded.first().waypointId, entry.waypointId);
    QCOMPARE(loaded.first().waypointName, entry.waypointName);
    QCOMPARE(loaded.first().missionId, entry.missionId);
    QCOMPARE(loaded.first().queueId, entry.queueId);
    QCOMPARE(loaded.first().result, entry.result);
    QCOMPARE(loaded.first().detailMessage, entry.detailMessage);
    QCOMPARE(loaded.first().occurredAt.toSecsSinceEpoch(), entry.occurredAt.toSecsSinceEpoch());
}

void MissionStorageTest::historyKeepsNewestAppendedFirst()
{
    const QDateTime now = QDateTime::currentDateTime();
    MissionStorageService storage;
    storage.appendHistory(makeHistory(QStringLiteral("FIRST"), now.addSecs(-30)));
    storage.appendHistory(makeHistory(QStringLiteral("SECOND"), now.addSecs(-20)));
    storage.appendHistory(makeHistory(QStringLiteral("THIRD"), now.addSecs(-10)));

    const QVector<MissionHistoryEntry> loaded = MissionStorageService().loadTodayHistory();

    QCOMPARE(loaded.size(), 3);
    QCOMPARE(loaded.at(0).eventType, QStringLiteral("THIRD"));
    QCOMPARE(loaded.at(2).eventType, QStringLiteral("FIRST"));
}

void MissionStorageTest::todayHistoryExcludesOtherDaysAndSortsNewestFirst()
{
    const QDate today = QDate::currentDate();
    MissionStorageService storage;
    storage.appendHistory(makeHistory(QStringLiteral("YESTERDAY"), QDateTime(today.addDays(-1), QTime(23, 59, 0))));
    storage.appendHistory(makeHistory(QStringLiteral("MORNING"), QDateTime(today, QTime(8, 0, 0))));
    storage.appendHistory(makeHistory(QStringLiteral("NOON"), QDateTime(today, QTime(12, 0, 0))));

    const QVector<MissionHistoryEntry> loaded = MissionStorageService().loadTodayHistory();

    QCOMPARE(loaded.size(), 2);
    QCOMPARE(loaded.at(0).eventType, QStringLiteral("NOON"));
    QCOMPARE(loaded.at(1).eventType, QStringLiteral("MORNING"));
}

void MissionStorageTest::corruptHistoryFileDoesNotBreakAppend()
{
    writeFile(QStringLiteral("mission_history.json"), "garbage");
    MissionStorageService storage;

    storage.appendHistory(makeHistory(QStringLiteral("AFTER_CORRUPT"), QDateTime::currentDateTime()));

    const QVector<MissionHistoryEntry> loaded = MissionStorageService().loadTodayHistory();
    QCOMPARE(loaded.size(), 1);
    QCOMPARE(loaded.first().eventType, QStringLiteral("AFTER_CORRUPT"));
}

void MissionStorageTest::databaseConnectionIsTriedOnlyOncePerService()
{
    MissionStorageService storage;
    QVector<MissionQueueItem> items;

    storage.loadQueue();
    storage.loadTodayHistory();
    storage.saveQueue(items);
    storage.appendHistory(makeHistory(QStringLiteral("X"), QDateTime::currentDateTime()));

    QCOMPARE(StubDb::openCalls, 1);
}

QTEST_GUILESS_MAIN(MissionStorageTest)
#include "test_missionstorage.moc"
