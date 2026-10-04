#pragma once

// SlamMappingPage 내부 구현 전용 헤더: ROS 지도 브리지(SlamMapBridge)와 헬퍼.
// slammappingpage*.cpp 에서만 include 한다.

#include "slammappingpage.h"

#include "../services/camerafeedprovider.h"
#include "../services/mapfile.h"
#include "../theme/dashboardpalette.h"
#include "../widgets/badgelabel.h"
#include "../widgets/mapviewwidget.h"
#include "../widgets/proportionalbar.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QFrame>
#include <QFile>
#include <QFileInfo>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QImage>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QStringList>
#include <QStyle>
#include <QTimer>
#include <QVBoxLayout>
#include <QDebug>
#include <QEvent>
#include <QPixmap>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <functional>
#include <mutex>
#include <memory>
#include <string>
#include <thread>

#include <sys/types.h>

#include <geometry_msgs/msg/twist.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_options.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

namespace slam_mapping_detail {
inline QLabel *makeLabel(const QString &text, const QString &objectName = {})
{
    auto *label = new QLabel(text);
    if (!objectName.isEmpty()) {
        label->setObjectName(objectName);
    }
    label->setWordWrap(true);
    return label;
}

inline QFrame *panelFrame(const QString &objectName)
{
    auto *frame = new QFrame;
    frame->setObjectName(objectName);
    frame->setFrameShape(QFrame::NoFrame);
    return frame;
}

inline QHBoxLayout *statRow(QBoxLayout *parentLayout, const QString &label, const QString &value, const QColor &valueColor = DashboardPalette::TextPrimary)
{
    auto *row = new QHBoxLayout;
    row->addWidget(makeLabel(label, QStringLiteral("metricCaption")));
    row->addStretch();
    auto *valueLabel = makeLabel(value);
    valueLabel->setStyleSheet(QStringLiteral("color: %1; font-size: 11px;").arg(valueColor.name()));
    row->addWidget(valueLabel);
    parentLayout->addLayout(row);
    return row;
}

inline QPushButton *directionButton(const QString &text, bool danger = false)
{
    auto *button = new QPushButton(text);
    button->setObjectName(danger ? QStringLiteral("dangerFilledButton") : QStringLiteral("directionPadButton"));
    button->setFixedSize(38, 38);
    return button;
}

inline QString findWorkspaceRoot()
{
    const QStringList candidateRoots = {
        QDir::currentPath(),
        QCoreApplication::applicationDirPath()
    };

    for (const QString &candidate : candidateRoots) {
        QDir dir(candidate);
        for (int depth = 0; depth < 8; ++depth) {
            if (dir.exists(QStringLiteral("maps")) && dir.exists(QStringLiteral("src/medirun_gui"))) {
                return dir.absolutePath();
            }
            if (!dir.cdUp()) {
                break;
            }
        }
    }
    return QString();
}

inline QString shellCommandPrefix()
{
    QStringList commands;
    if (QFile::exists(QStringLiteral("/opt/ros/humble/setup.bash"))) {
        commands.append(QStringLiteral("source /opt/ros/humble/setup.bash"));
    }

    const QString turtlebotWorkspaceSetup = QDir::home().filePath(QStringLiteral("turtlebot3_ws/install/setup.bash"));
    if (QFile::exists(turtlebotWorkspaceSetup)) {
        commands.append(QStringLiteral("source \"%1\"").arg(turtlebotWorkspaceSetup));
    }

    const QString workspaceRoot = findWorkspaceRoot();
    if (!workspaceRoot.isEmpty()) {
        const QString installSetup = QDir(workspaceRoot).filePath(QStringLiteral("install/setup.bash"));
        if (QFile::exists(installSetup)) {
            commands.append(QStringLiteral("source \"%1\"").arg(installSetup));
        }
    }

    if (commands.isEmpty()) {
        return QString();
    }

    return commands.join(QStringLiteral(" && ")) + QStringLiteral(" && ");
}

inline QString shellSingleQuoted(const QString &value)
{
    QString escaped = value;
    escaped.replace(QStringLiteral("'"), QStringLiteral("'\"'\"'"));
    return QStringLiteral("'%1'").arg(escaped);
}

inline QString normalizeMapBaseName(QString fileName)
{
    fileName = fileName.trimmed();
    while (fileName.endsWith(QStringLiteral(".png"), Qt::CaseInsensitive)
           || fileName.endsWith(QStringLiteral(".pgm"), Qt::CaseInsensitive)
           || fileName.endsWith(QStringLiteral(".yaml"), Qt::CaseInsensitive)
           || fileName.endsWith(QStringLiteral(".yml"), Qt::CaseInsensitive)) {
        const int dotIndex = fileName.lastIndexOf(QLatin1Char('.'));
        if (dotIndex <= 0) {
            break;
        }
        fileName = fileName.left(dotIndex).trimmed();
    }
    return fileName;
}

// m_slamProcess는 setsid로 자기 자신을 새 프로세스 그룹의 리더로 띄운다
// (startMappingProcess 참고). ros2 launch는 cartographer_node /
// cartographer_occupancy_grid_node를 자식으로 fork하는데, QProcess::terminate()
// /kill()은 직접 자식(ros2 launch)에만 신호를 보내므로, SIGTERM 처리 시간 안에
// ros2 launch가 죽지 않고 SIGKILL로 넘어가면 그 자식들이 부모 잃은 채 살아남는다
// (실측: 대시보드를 꺼도 cartographer_node가 계속 떠 있던 버그). 음수 PID로
// kill()해서 프로세스 그룹 전체(ros2 launch + 그 자식들)에 신호를 보낸다.
inline void terminateProcessTree(QProcess *process, int termTimeoutMs, int killTimeoutMs)
{
    if (process->state() == QProcess::NotRunning) {
        return;
    }

    const qint64 pid = process->processId();
    if (pid > 0) {
        ::kill(-static_cast<pid_t>(pid), SIGTERM);
    } else {
        process->terminate();
    }

    if (!process->waitForFinished(termTimeoutMs)) {
        if (pid > 0) {
            ::kill(-static_cast<pid_t>(pid), SIGKILL);
        } else {
            process->kill();
        }
        process->waitForFinished(killTimeoutMs);
    }
}

enum class LifecycleCallResult {
    Success,
    ServiceUnavailable,
    CallFailed,
};

// PAUSE/RESUME 요청을 보낸다(blocking). m_node는 SlamMapBridge 소유 executor가
// 별도 스레드에서 계속 spin 중이므로, 이 함수를 호출한 스레드(Qt 메인 스레드)에서
// future를 blocking wait해도 응답 콜백은 그 spin 스레드에서 처리되어 데드락 없이
// 완료된다. GUI 종료(소멸자)처럼 "이 다음엔 재시도 기회가 없는" 경로에서만 쓴다 —
// 인터랙티브 경로(초기화 버튼 등)는 UI를 막지 않는 callNav2LifecycleManagerAsync를 쓴다.
inline LifecycleCallResult callNav2LifecycleManagerBlocking(
    const rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr &client,
    uint8_t command,
    int timeoutMs)
{
    if (!client || !client->wait_for_service(std::chrono::milliseconds(timeoutMs))) {
        return LifecycleCallResult::ServiceUnavailable;
    }

    auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    request->command = command;
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::milliseconds(timeoutMs)) != std::future_status::ready) {
        client->remove_pending_request(future);
        return LifecycleCallResult::CallFailed;
    }
    return future.get()->success ? LifecycleCallResult::Success : LifecycleCallResult::CallFailed;
}

// 비blocking 버전. 서비스가 지금 당장 그래프에 없으면(service_is_ready()==false)
// 바로 실패로 응답하고, 그렇지 않으면 요청을 보내고 즉시 반환한다 — 응답이 오면
// (SlamMapBridge의 executor spin 스레드에서) onDone이 호출된다. Nav2 lifecycle
// RESUME은 관리 노드 수에 따라 걸리는 시간이 크게 달라(costmap+controller+
// bt_navigator 등은 amcl+map_server보다 훨씬 오래 걸림) 고정 타임아웃으로 끊지
// 않고 실제로 끝날 때까지 기다린다.
inline void callNav2LifecycleManagerAsync(
    const rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr &client,
    uint8_t command,
    std::function<void(LifecycleCallResult)> onDone)
{
    if (!client || !client->service_is_ready()) {
        onDone(LifecycleCallResult::ServiceUnavailable);
        return;
    }

    auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    request->command = command;
    client->async_send_request(
        request,
        [onDone](rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedFuture future) {
            onDone(future.get()->success ? LifecycleCallResult::Success : LifecycleCallResult::CallFailed);
        });
}

inline bool isSlamFatalLogLine(const QString &line)
{
    const QString lower = line.toLower();

    static const QStringList errorMarkers = {
        QStringLiteral("[fatal]"),
        QStringLiteral(" fatal"),
        QStringLiteral("terminate called"),
        QStringLiteral("segmentation fault"),
        QStringLiteral("stack trace"),
        QStringLiteral("traceback"),
        QStringLiteral("std::runtime_error"),
        QStringLiteral("bad_alloc"),
        QStringLiteral("process has died"),
        QStringLiteral("failed to load"),
        QStringLiteral("no such file"),
        QStringLiteral("package '"),
        QStringLiteral("launch file"),
        QStringLiteral("could not open"),
        QStringLiteral("permission denied")
    };

    for (const QString &marker : errorMarkers) {
        if (lower.contains(marker)) {
            return true;
        }
    }

    return false;
}

constexpr double kManualLinearSpeedMps = 0.06;
constexpr double kManualAngularSpeedRadPerSec = 0.20;
constexpr int kMapRefreshIntervalMs = 300;
constexpr int kMapTimeoutMs = 10000;
constexpr auto kRecentSensorTimeout = std::chrono::seconds(3);
constexpr auto kRecentTfTimeout = std::chrono::seconds(5);

struct RobotPoseSnapshot
{
    bool valid {false};
    QPointF normalizedPos {0.5, 0.5};
    qreal headingDegrees {0.0};
};

struct StableMapBounds
{
    bool valid {false};
    double minX {0.0};
    double minY {0.0};
    double maxX {0.0};
    double maxY {0.0};
    double resolution {0.0};
    int widthCells {0};
    int heightCells {0};
};

struct MapRenderSnapshot
{
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
    StableMapBounds bounds;
};

struct MapTopicSnapshot
{
    size_t publisherCount {0};
    QStringList publisherNames;
    bool hasValidMap {false};
    int width {0};
    int height {0};
    double resolution {0.0};
    double originX {0.0};
    double originY {0.0};
    size_t dataSize {0};
};

constexpr int kStableMapPaddingCells = 128;
constexpr int kStableMapGrowthChunkCells = 64;

inline int roundUpCells(int cellCount, int chunkSize)
{
    if (chunkSize <= 0) {
        return std::max(1, cellCount);
    }
    const int safeCount = std::max(1, cellCount);
    return ((safeCount + chunkSize - 1) / chunkSize) * chunkSize;
}

enum class TopicHealth {
    Unavailable,
    Waiting,
    Ready,
    Stale,
};

struct TopicStateSnapshot
{
    TopicHealth scanHealth {TopicHealth::Unavailable};
    TopicHealth odomHealth {TopicHealth::Unavailable};
    TopicHealth tfHealth {TopicHealth::Unavailable};
    TopicHealth tfStaticHealth {TopicHealth::Unavailable};
    size_t scanPublisherCount {0};
    size_t odomPublisherCount {0};
    size_t tfPublisherCount {0};
    size_t tfStaticPublisherCount {0};
    bool scanReceived {false};
    bool odomReceived {false};
    bool tfReceived {false};
    bool tfStaticReceived {false};
    bool scanRecent {false};
    bool odomRecent {false};
    bool tfRecent {false};
    bool tfReady {false};
    bool odomUsable {false};
    bool readyForMapping {false};
    uint64_t odomMessageCount {0};
    double odomLastAgeSec {-1.0};
    QString odomFrameId;
    QString odomChildFrameId;
};

inline QString topicHealthText(TopicHealth health)
{
    switch (health) {
    case TopicHealth::Unavailable:
        return QStringLiteral("없음");
    case TopicHealth::Waiting:
        return QStringLiteral("대기");
    case TopicHealth::Ready:
        return QStringLiteral("OK");
    case TopicHealth::Stale:
        return QStringLiteral("경고");
    }
    return QStringLiteral("알 수 없음");
}

inline QString readinessFailureMessage(const TopicStateSnapshot &snapshot)
{
    if (snapshot.scanPublisherCount == 0) {
        return QStringLiteral("/scan publisher가 존재하지 않습니다.");
    }
    if (!snapshot.scanReceived || !snapshot.scanRecent) {
        return QStringLiteral("LiDAR 데이터가 없습니다: /scan 확인 필요");
    }
    if (snapshot.odomPublisherCount == 0) {
        return QStringLiteral("/odom publisher가 존재하지 않습니다.");
    }
    if (!snapshot.odomReceived) {
        return QStringLiteral("odom 메시지를 아직 수신하지 못했습니다: /odom 확인 필요");
    }
    if (!snapshot.tfReady) {
        return QStringLiteral("TF 데이터가 없습니다: /tf 또는 /tf_static 확인 필요");
    }
    return QStringLiteral("Bringup 준비 상태를 다시 확인하세요.");
}

inline bool createOccupancyGridImage(const nav_msgs::msg::OccupancyGrid &map,
                              const StableMapBounds &bounds,
                              QImage *imageOut)
{
    static bool hasLoggedImageCreated = false;

    if (imageOut == nullptr) {
        return false;
    }

    const int mapWidth = static_cast<int>(map.info.width);
    const int mapHeight = static_cast<int>(map.info.height);
    const size_t expectedSize = static_cast<size_t>(mapWidth) * static_cast<size_t>(mapHeight);
    if (mapWidth <= 0 || mapHeight <= 0) {
        qWarning() << "Invalid map dimensions:" << mapWidth << "x" << mapHeight;
        return false;
    }
    if (map.data.size() != expectedSize) {
        qWarning() << "Invalid map data size:" << map.data.size() << "expected:" << expectedSize;
        return false;
    }
    if (!bounds.valid || bounds.widthCells <= 0 || bounds.heightCells <= 0 || bounds.resolution <= 0.0) {
        qWarning() << "Invalid stable map bounds";
        return false;
    }

    QImage image(bounds.widthCells, bounds.heightCells, QImage::Format_RGB32);
    if (image.isNull()) {
        qWarning() << "Failed to allocate map image:" << bounds.widthCells << "x" << bounds.heightCells;
        return false;
    }
    image.fill(qRgb(236, 240, 244));

    const int offsetX = static_cast<int>(std::llround((map.info.origin.position.x - bounds.minX) / bounds.resolution));
    const int offsetY = static_cast<int>(std::llround((map.info.origin.position.y - bounds.minY) / bounds.resolution));

    for (int y = 0; y < mapHeight; ++y) {
        const int targetY = offsetY + y;
        if (targetY < 0 || targetY >= bounds.heightCells) {
            continue;
        }
        const int imageY = bounds.heightCells - 1 - targetY;
        QRgb *scanLine = reinterpret_cast<QRgb *>(image.scanLine(imageY));
        for (int x = 0; x < mapWidth; ++x) {
            const int targetX = offsetX + x;
            if (targetX < 0 || targetX >= bounds.widthCells) {
                continue;
            }
            const std::size_t index =
                static_cast<std::size_t>(y) * static_cast<std::size_t>(mapWidth)
                + static_cast<std::size_t>(x);
            const int value = static_cast<int>(map.data.at(index));

            QRgb color = qRgb(255, 255, 255);
            if (value < 0) {
                color = qRgb(236, 240, 244);
            } else if (value == 0) {
                color = qRgb(255, 255, 255);
            } else if (value >= 65) {
                color = qRgb(18, 22, 28);
            } else {
                const int gray = std::clamp(
                    255 - (value * 205 / 64),
                    48,
                    252);
                color = qRgb(gray, gray, gray);
            }

            scanLine[targetX] = color;
        }
    }

    *imageOut = image;
    if (!hasLoggedImageCreated) {
        qInfo() << "[MapView] map image created:" << bounds.widthCells << "x" << bounds.heightCells;
        hasLoggedImageCreated = true;
    }
    return true;
}

} // namespace slam_mapping_detail

using namespace slam_mapping_detail;

class SlamMapBridge
{
public:
    SlamMapBridge()
        : m_node(std::make_shared<rclcpp::Node>("medirun_gui_slam_map_bridge"))
        , m_executor(std::make_unique<rclcpp::executors::SingleThreadedExecutor>())
        , m_running(false)
        , m_mapReceived(false)
    {
        m_cmdVelPublisher = m_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10));
        // nav2_bringup의 localization_launch.py/navigation_launch.py가 기본으로 띄우는
        // lifecycle manager 이름(namespace 없음). ./scripts/run.sh -> bringup.launch.py가
        // 이 기본값 그대로 Nav2를 띄운다.
        m_localizationLifecycleClient = m_node->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
            "/lifecycle_manager_localization/manage_nodes");
        m_navigationLifecycleClient = m_node->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
            "/lifecycle_manager_navigation/manage_nodes");
        m_amclStateClient = m_node->create_client<lifecycle_msgs::srv::GetState>("/amcl/get_state");
        m_tfBuffer = std::make_unique<tf2_ros::Buffer>(m_node->get_clock());
        m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);

        rclcpp::SubscriptionOptions mapOptions;
        mapOptions.event_callbacks.incompatible_qos_callback =
            [this](rclcpp::QOSRequestedIncompatibleQoSInfo &info) {
                RCLCPP_ERROR(
                    m_node->get_logger(),
                    "[SLAM] /map QoS incompatible: policy=%d total=%d",
                    info.last_policy_kind,
                    info.total_count);
                if (onStatusUpdated) {
                    onStatusUpdated(QStringLiteral("/map QoS가 publisher와 맞지 않습니다."));
                }
            };

        const auto mapQos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        m_mapSubscription = m_node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            mapQos,
            [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { handleMap(msg); },
            mapOptions);
        m_odomSubscription = m_node->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            rclcpp::SystemDefaultsQoS(),
            [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { handleOdom(msg); });
        m_scanSubscription = m_node->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { handleScan(msg); });
        m_tfSubscription = m_node->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf",
            rclcpp::SystemDefaultsQoS(),
            [this](const tf2_msgs::msg::TFMessage::ConstSharedPtr msg) { handleTf(msg, false); });
        m_tfStaticSubscription = m_node->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf_static",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            [this](const tf2_msgs::msg::TFMessage::ConstSharedPtr msg) { handleTf(msg, true); });
        resetMapSessionTracking();
        resetTopicTracking();
    }

    ~SlamMapBridge()
    {
        stop();
    }

    void start()
    {
        if (m_running.exchange(true)) {
            return;
        }
        m_executor->add_node(m_node);
        m_spinThread = std::thread([this]() { m_executor->spin(); });
        if (onStatusUpdated) {
            onStatusUpdated(QStringLiteral("Bringup 토픽 구독 대기 중 (/scan, /odom, /tf, /tf_static, /map)"));
        }
    }

    void stop()
    {
        if (!m_running.exchange(false)) {
            return;
        }
        m_executor->cancel();
        if (m_spinThread.joinable()) {
            m_spinThread.join();
        }
        m_executor->remove_node(m_node);
    }

    void resetMapSessionTracking()
    {
        {
            std::lock_guard<std::mutex> lock(m_dataMutex);
            m_latestMap.reset();
            m_mapDirty = false;
            m_latestRobotPose = RobotPoseSnapshot();
            m_sessionBounds = StableMapBounds();
        }
        m_mapReceived = false;
        m_hasInitialOdom = false;
        m_hasMapAnchor = false;
        m_mapWidth = 0;
        m_mapHeight = 0;
        m_mapResolution = 0.0;
        m_mapOriginX = 0.0;
        m_mapOriginY = 0.0;
        m_hasPublisherInfoLogged = false;
        m_hasLoggedFirstAcceptedMap = false;
        m_hasLoggedMapInfo = false;
        m_lastLoggedWidth = 0;
        m_lastLoggedHeight = 0;
        m_lastLoggedResolution = 0.0;
        m_lastLoggedOriginX = 0.0;
        m_lastLoggedOriginY = 0.0;
        m_lastLoggedDataSize = 0;
        m_lastLoggedPublisherCount = 0;
        m_lastInvalidWidth = -1;
        m_lastInvalidHeight = -1;
        m_lastInvalidDataSize = 0;
    }

    void resetTopicTracking()
    {
        m_hasScan = false;
        m_hasTf = false;
        m_hasTfStatic = false;
        m_hasLatestOdom = false;
        m_odomMessageCount = 0;
        m_lastScanAt = {};
        m_lastOdomAt = {};
        m_lastTfAt = {};
        m_lastTfStaticAt = {};
        m_lastOdomFrameId.clear();
        m_lastOdomChildFrameId.clear();
    }

    void beginFreshSession()
    {
        resetMapSessionTracking();
    }

    std::function<void(const QString &)> onStatusUpdated;

    void publishVelocity(double linearX, double angularZ)
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = linearX;
        cmd.angular.z = angularZ;
        m_cmdVelPublisher->publish(cmd);
    }

    nav_msgs::msg::OccupancyGrid::ConstSharedPtr peekLatestMap()
    {
        std::lock_guard<std::mutex> lock(m_dataMutex);
        if (!m_mapDirty || !m_latestMap) {
            return nullptr;
        }
        return m_latestMap;
    }

    MapRenderSnapshot takeLatestMapRenderSnapshot()
    {
        std::lock_guard<std::mutex> lock(m_dataMutex);
        MapRenderSnapshot snapshot;
        if (!m_mapDirty || !m_latestMap) {
            return snapshot;
        }
        snapshot.map = m_latestMap;
        snapshot.bounds = m_sessionBounds;
        return snapshot;
    }

    void markLatestMapDisplayed()
    {
        std::lock_guard<std::mutex> lock(m_dataMutex);
        m_mapDirty = false;
    }

    RobotPoseSnapshot takeLatestRobotPose()
    {
        std::lock_guard<std::mutex> lock(m_dataMutex);
        return m_latestRobotPose;
    }

    TopicStateSnapshot topicStateSnapshot() const
    {
        const auto now = std::chrono::steady_clock::now();
        TopicStateSnapshot snapshot;
        snapshot.scanPublisherCount = m_node->count_publishers("/scan");
        snapshot.odomPublisherCount = m_node->count_publishers("/odom");
        snapshot.tfPublisherCount = m_node->count_publishers("/tf");
        snapshot.tfStaticPublisherCount = m_node->count_publishers("/tf_static");
        snapshot.scanReceived = m_hasScan;
        snapshot.odomReceived = m_hasLatestOdom;
        snapshot.tfReceived = m_hasTf;
        snapshot.tfStaticReceived = m_hasTfStatic;
        snapshot.scanRecent = m_hasScan && (now - m_lastScanAt <= kRecentSensorTimeout);
        snapshot.odomRecent = m_hasLatestOdom && (now - m_lastOdomAt <= kRecentSensorTimeout);
        snapshot.tfRecent = m_hasTf && (now - m_lastTfAt <= kRecentTfTimeout);
        snapshot.tfReady = snapshot.tfRecent || hasTransformChainReady();
        if (!snapshot.tfReady && snapshot.tfStaticReceived) {
            snapshot.tfReady = hasTransformChainReady();
        }
        snapshot.scanHealth = snapshot.scanPublisherCount == 0
            ? TopicHealth::Unavailable
            : (!snapshot.scanReceived ? TopicHealth::Waiting
                                      : (snapshot.scanRecent ? TopicHealth::Ready : TopicHealth::Stale));
        snapshot.odomHealth = snapshot.odomPublisherCount == 0
            ? TopicHealth::Unavailable
            : (!snapshot.odomReceived ? TopicHealth::Waiting
                                      : (snapshot.odomRecent ? TopicHealth::Ready : TopicHealth::Stale));
        snapshot.tfHealth = snapshot.tfPublisherCount == 0
            ? TopicHealth::Unavailable
            : (!snapshot.tfReady ? TopicHealth::Waiting : TopicHealth::Ready);
        snapshot.tfStaticHealth = snapshot.tfStaticPublisherCount == 0
            ? TopicHealth::Unavailable
            : (!snapshot.tfStaticReceived ? TopicHealth::Waiting : TopicHealth::Ready);
        snapshot.odomUsable = snapshot.odomPublisherCount > 0 && snapshot.odomReceived;
        snapshot.readyForMapping =
            snapshot.scanReceived && snapshot.scanRecent
            && snapshot.odomUsable
            && snapshot.tfReady
            && snapshot.tfStaticPublisherCount > 0;
        snapshot.odomMessageCount = m_odomMessageCount.load();
        if (snapshot.odomReceived) {
            snapshot.odomLastAgeSec = std::chrono::duration<double>(now - m_lastOdomAt).count();
        }
        snapshot.odomFrameId = m_lastOdomFrameId;
        snapshot.odomChildFrameId = m_lastOdomChildFrameId;
        return snapshot;
    }

    QString realtimeReadinessText() const
    {
        const TopicStateSnapshot snapshot = topicStateSnapshot();
        if (snapshot.readyForMapping) {
            return QStringLiteral("/scan, /odom, TF 준비 완료");
        }
        return readinessFailureMessage(snapshot);
    }

private:
    bool hasTransformChainReady() const
    {
        static constexpr const char *kBaseFrames[] = {"base_link", "base_footprint"};
        static constexpr const char *kLaserFrames[] = {"base_scan", "laser", "base_laser"};

        for (const char *baseFrame : kBaseFrames) {
            if (m_tfBuffer->canTransform("odom", baseFrame, tf2::TimePointZero)) {
                return true;
            }
            for (const char *laserFrame : kLaserFrames) {
                if (m_tfBuffer->canTransform(baseFrame, laserFrame, tf2::TimePointZero)) {
                    return true;
                }
            }
        }
        return false;
    }

    bool tryLookupRobotPose(double &worldX, double &worldY)
    {
        static constexpr const char *kBaseFrames[] = {"base_link", "base_footprint"};

        for (const char *baseFrame : kBaseFrames) {
            try {
                const auto transform = m_tfBuffer->lookupTransform("map", baseFrame, tf2::TimePointZero);
                worldX = transform.transform.translation.x;
                worldY = transform.transform.translation.y;
                return true;
            } catch (const tf2::TransformException &) {
            }
        }

        return false;
    }

    void handleMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &msg)
    {
        const int width = static_cast<int>(msg->info.width);
        const int height = static_cast<int>(msg->info.height);
        const size_t expectedSize = (width > 0 && height > 0)
            ? static_cast<size_t>(width) * static_cast<size_t>(height)
            : 0;
        if (width <= 0 || height <= 0 || msg->data.size() != expectedSize) {
            if (width != m_lastInvalidWidth || height != m_lastInvalidHeight || msg->data.size() != m_lastInvalidDataSize) {
                RCLCPP_WARN(m_node->get_logger(),
                            "Invalid /map received: width=%d height=%d data_size=%zu expected=%zu",
                            width,
                            height,
                            msg->data.size(),
                            expectedSize);
                m_lastInvalidWidth = width;
                m_lastInvalidHeight = height;
                m_lastInvalidDataSize = msg->data.size();
            }
            if (onStatusUpdated) {
                onStatusUpdated(QStringLiteral("/map 토픽 형식이 올바르지 않습니다."));
            }
            return;
        }

        logMapPublishersOnce();

        const QStringList publisherNames = effectiveMapPublisherNames();
        if (publisherNames.size() > 1) {
            RCLCPP_ERROR(
                m_node->get_logger(),
                "[MAP] multiple /map publishers detected: %s",
                publisherNames.join(QStringLiteral(", ")).toStdString().c_str());
            if (onStatusUpdated) {
                onStatusUpdated(QStringLiteral("/map publisher가 여러 개 존재합니다. 중복 SLAM 실행을 확인하세요."));
            }
            return;
        }
        const size_t publisherCount = static_cast<size_t>(publisherNames.size());

        const TopicStateSnapshot topicSnapshot = topicStateSnapshot();
        if (!m_mapReceived && !topicSnapshot.readyForMapping) {
            RCLCPP_WARN(
                m_node->get_logger(),
                "[MapSource] ignoring /map until bringup inputs are ready");
            if (onStatusUpdated) {
                onStatusUpdated(QStringLiteral("Bringup 입력이 준비되지 않아 /map 표시를 보류합니다."));
            }
            return;
        }

        if (!m_hasLoggedMapInfo) {
            RCLCPP_INFO(m_node->get_logger(), "[MAP] width: %d", width);
            RCLCPP_INFO(m_node->get_logger(), "[MAP] height: %d", height);
            RCLCPP_INFO(m_node->get_logger(), "[MAP] resolution: %.4f", msg->info.resolution);
            RCLCPP_INFO(m_node->get_logger(), "[MAP] origin.x: %.3f", msg->info.origin.position.x);
            RCLCPP_INFO(m_node->get_logger(), "[MAP] origin.y: %.3f", msg->info.origin.position.y);
            RCLCPP_INFO(m_node->get_logger(), "[MAP] data size: %zu", msg->data.size());
            RCLCPP_INFO(m_node->get_logger(), "[MAP] publisher count: %zu", publisherCount);
        } else {
            if (static_cast<uint32_t>(width) != m_lastLoggedWidth) {
                RCLCPP_INFO(m_node->get_logger(), "[MAP] width: %u -> %d", m_lastLoggedWidth, width);
            }
            if (static_cast<uint32_t>(height) != m_lastLoggedHeight) {
                RCLCPP_INFO(m_node->get_logger(), "[MAP] height: %u -> %d", m_lastLoggedHeight, height);
            }
            if (std::abs(msg->info.resolution - m_lastLoggedResolution) > 1e-9) {
                RCLCPP_INFO(m_node->get_logger(), "[MAP] resolution: %.4f -> %.4f", m_lastLoggedResolution, msg->info.resolution);
            }
            if (std::abs(msg->info.origin.position.x - m_lastLoggedOriginX) > 1e-9) {
                RCLCPP_INFO(m_node->get_logger(), "[MAP] origin.x: %.3f -> %.3f", m_lastLoggedOriginX, msg->info.origin.position.x);
            }
            if (std::abs(msg->info.origin.position.y - m_lastLoggedOriginY) > 1e-9) {
                RCLCPP_INFO(m_node->get_logger(), "[MAP] origin.y: %.3f -> %.3f", m_lastLoggedOriginY, msg->info.origin.position.y);
            }
            if (msg->data.size() != m_lastLoggedDataSize) {
                RCLCPP_INFO(m_node->get_logger(), "[MAP] data size: %zu -> %zu", m_lastLoggedDataSize, msg->data.size());
            }
            if (publisherCount != m_lastLoggedPublisherCount) {
                RCLCPP_INFO(m_node->get_logger(), "[MAP] publisher count: %zu -> %zu", m_lastLoggedPublisherCount, publisherCount);
            }
        }

        {
            const double mapMinX = msg->info.origin.position.x;
            const double mapMinY = msg->info.origin.position.y;
            const double mapMaxX = mapMinX + (msg->info.resolution * static_cast<double>(width));
            const double mapMaxY = mapMinY + (msg->info.resolution * static_cast<double>(height));
            const double resolution = msg->info.resolution;

            if (!m_sessionBounds.valid || std::abs(resolution - m_sessionBounds.resolution) > 1e-9) {
                m_sessionBounds.valid = true;
                m_sessionBounds.resolution = resolution;
                m_sessionBounds.minX = mapMinX - (static_cast<double>(kStableMapPaddingCells) * resolution);
                m_sessionBounds.minY = mapMinY - (static_cast<double>(kStableMapPaddingCells) * resolution);
                m_sessionBounds.widthCells = roundUpCells(width + (kStableMapPaddingCells * 2), kStableMapGrowthChunkCells);
                m_sessionBounds.heightCells = roundUpCells(height + (kStableMapPaddingCells * 2), kStableMapGrowthChunkCells);
                m_sessionBounds.maxX = m_sessionBounds.minX + (static_cast<double>(m_sessionBounds.widthCells) * resolution);
                m_sessionBounds.maxY = m_sessionBounds.minY + (static_cast<double>(m_sessionBounds.heightCells) * resolution);
            } else {
                bool boundsExpanded = false;

                while (mapMinX < m_sessionBounds.minX) {
                    m_sessionBounds.minX -= static_cast<double>(kStableMapGrowthChunkCells) * resolution;
                    m_sessionBounds.widthCells += kStableMapGrowthChunkCells;
                    boundsExpanded = true;
                }
                while (mapMinY < m_sessionBounds.minY) {
                    m_sessionBounds.minY -= static_cast<double>(kStableMapGrowthChunkCells) * resolution;
                    m_sessionBounds.heightCells += kStableMapGrowthChunkCells;
                    boundsExpanded = true;
                }
                while (mapMaxX > m_sessionBounds.maxX) {
                    m_sessionBounds.widthCells += kStableMapGrowthChunkCells;
                    boundsExpanded = true;
                    m_sessionBounds.maxX = m_sessionBounds.minX
                        + (static_cast<double>(m_sessionBounds.widthCells) * resolution);
                }
                while (mapMaxY > m_sessionBounds.maxY) {
                    m_sessionBounds.heightCells += kStableMapGrowthChunkCells;
                    boundsExpanded = true;
                    m_sessionBounds.maxY = m_sessionBounds.minY
                        + (static_cast<double>(m_sessionBounds.heightCells) * resolution);
                }

                if (boundsExpanded) {
                    m_sessionBounds.maxX = m_sessionBounds.minX
                        + (static_cast<double>(m_sessionBounds.widthCells) * resolution);
                    m_sessionBounds.maxY = m_sessionBounds.minY
                        + (static_cast<double>(m_sessionBounds.heightCells) * resolution);
                }
            }
        }

        if (!m_hasLoggedMapInfo
            || static_cast<uint32_t>(width) != m_lastLoggedWidth
            || static_cast<uint32_t>(height) != m_lastLoggedHeight
            || std::abs(msg->info.resolution - m_lastLoggedResolution) > 1e-9
            || std::abs(msg->info.origin.position.x - m_lastLoggedOriginX) > 1e-9
            || std::abs(msg->info.origin.position.y - m_lastLoggedOriginY) > 1e-9
            || msg->data.size() != m_lastLoggedDataSize
            || publisherCount != m_lastLoggedPublisherCount) {
            m_hasLoggedMapInfo = true;
            m_lastLoggedWidth = static_cast<uint32_t>(width);
            m_lastLoggedHeight = static_cast<uint32_t>(height);
            m_lastLoggedResolution = msg->info.resolution;
            m_lastLoggedOriginX = msg->info.origin.position.x;
            m_lastLoggedOriginY = msg->info.origin.position.y;
            m_lastLoggedDataSize = msg->data.size();
            m_lastLoggedPublisherCount = publisherCount;
        }

        m_mapWidth = width;
        m_mapHeight = height;
        m_mapResolution = msg->info.resolution;
        m_mapOriginX = msg->info.origin.position.x;
        m_mapOriginY = msg->info.origin.position.y;
        m_mapReceived = true;
        if (!m_hasLoggedFirstAcceptedMap) {
            RCLCPP_INFO(m_node->get_logger(), "[Cartographer] first real map received");
            m_hasLoggedFirstAcceptedMap = true;
        }

        int exploredCells = 0;
        for (const int8_t cellValue : msg->data) {
            if (static_cast<int>(cellValue) >= 0) {
                ++exploredCells;
            }
        }
        {
            std::lock_guard<std::mutex> lock(m_dataMutex);
            m_latestMap = msg;
            m_mapDirty = true;
        }

        const double exploredRatio = static_cast<double>(exploredCells) / static_cast<double>(width * height);
        if (onStatusUpdated) {
            onStatusUpdated(QStringLiteral("실시간 Cartographer 맵 수신 중 (%1 x %2, 탐색 %3%)")
                                .arg(width)
                                .arg(height)
                                .arg(exploredRatio * 100.0, 0, 'f', 1));
        }
    }

    void handleOdom(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
    {
        constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
        constexpr double kPreviewSpanMeters = 4.0;

        const auto &q = msg->pose.pose.orientation;
        const double sinyCosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosyCosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        const double yawRad = std::atan2(sinyCosp, cosyCosp);
        const qreal headingDegrees = static_cast<qreal>(90.0 - (yawRad * kRadToDeg));

        m_latestOdomX = msg->pose.pose.position.x;
        m_latestOdomY = msg->pose.pose.position.y;
        m_latestYawRad = yawRad;
        m_hasLatestOdom = true;
        ++m_odomMessageCount;
        m_lastOdomAt = std::chrono::steady_clock::now();
        m_lastOdomFrameId = QString::fromStdString(msg->header.frame_id);
        m_lastOdomChildFrameId = QString::fromStdString(msg->child_frame_id);

        if (!m_hasLoggedFirstOdom) {
            m_hasLoggedFirstOdom = true;
            RCLCPP_INFO(
                m_node->get_logger(),
                "[SLAM] first /odom received frame_id=%s child_frame_id=%s",
                msg->header.frame_id.c_str(),
                msg->child_frame_id.c_str());
        }

        double normalizedX = 0.5;
        double normalizedY = 0.5;

        if (m_mapReceived && m_mapResolution > 0.0 && m_sessionBounds.valid
            && m_sessionBounds.widthCells > 0 && m_sessionBounds.heightCells > 0) {
            double worldX = 0.0;
            double worldY = 0.0;
            if (tryLookupRobotPose(worldX, worldY)) {
                const double spanX = m_sessionBounds.maxX - m_sessionBounds.minX;
                const double spanY = m_sessionBounds.maxY - m_sessionBounds.minY;
                if (spanX > 1e-9 && spanY > 1e-9) {
                    normalizedX = (worldX - m_sessionBounds.minX) / spanX;
                    normalizedY = 1.0 - ((worldY - m_sessionBounds.minY) / spanY);
                    m_hasMapAnchor = true;
                    {
                        std::lock_guard<std::mutex> lock(m_dataMutex);
                        m_latestRobotPose.valid = true;
                        m_latestRobotPose.normalizedPos = QPointF(std::clamp(normalizedX, 0.0, 1.0),
                                                                  std::clamp(normalizedY, 0.0, 1.0));
                        m_latestRobotPose.headingDegrees = headingDegrees;
                    }
                } else {
                    m_hasMapAnchor = false;
                }
            } else {
                m_hasMapAnchor = false;
            }
        } else {
            if (!m_hasInitialOdom) {
                m_initialOdomX = m_latestOdomX;
                m_initialOdomY = m_latestOdomY;
                m_hasInitialOdom = true;
            }

            const double deltaX = m_latestOdomX - m_initialOdomX;
            const double deltaY = m_latestOdomY - m_initialOdomY;
            normalizedX = 0.5 + (deltaX / kPreviewSpanMeters);
            normalizedY = 0.5 - (deltaY / kPreviewSpanMeters);
            std::lock_guard<std::mutex> lock(m_dataMutex);
            m_latestRobotPose.valid = true;
            m_latestRobotPose.normalizedPos = QPointF(std::clamp(normalizedX, 0.0, 1.0),
                                                      std::clamp(normalizedY, 0.0, 1.0));
            m_latestRobotPose.headingDegrees = headingDegrees;
        }

        if (!m_mapReceived && onStatusUpdated) {
            onStatusUpdated(QStringLiteral("새 Cartographer 맵 수신 대기 중"));
        } else if (m_mapReceived && !m_hasMapAnchor && onStatusUpdated) {
            onStatusUpdated(QStringLiteral("Cartographer 맵 수신 중 · 로봇 기준점을 맞추는 중입니다."));
        }
    }

    void handleScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg)
    {
        (void)msg;
        m_hasScan = true;
        m_lastScanAt = std::chrono::steady_clock::now();
    }

    void handleTf(const tf2_msgs::msg::TFMessage::ConstSharedPtr &msg, bool isStatic)
    {
        if (msg->transforms.empty()) {
            return;
        }
        if (isStatic) {
            m_hasTfStatic = true;
            m_lastTfStaticAt = std::chrono::steady_clock::now();
        } else {
            m_hasTf = true;
            m_lastTfAt = std::chrono::steady_clock::now();
        }
    }

    rclcpp::Node::SharedPtr m_node;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmdVelPublisher;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr m_localizationLifecycleClient;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr m_navigationLifecycleClient;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr m_amclStateClient;
    // Qt 스레드(pauseNav2ForMapping/resumeNav2IfPaused)와 executor spin 스레드
    // (handleMap의 중복 publisher 판정)에서 동시에 읽고 쓰므로 atomic이어야 한다.
    std::atomic<bool> m_localizationPausedByMapping {false};
    std::atomic<bool> m_navigationPausedByMapping {false};
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_mapSubscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odomSubscription;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_scanSubscription;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr m_tfSubscription;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr m_tfStaticSubscription;
    std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
    std::thread m_spinThread;
    std::atomic<bool> m_running;
    std::mutex m_dataMutex;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr m_latestMap;
    bool m_mapDirty {false};
    RobotPoseSnapshot m_latestRobotPose;
    bool m_mapReceived;
    bool m_hasScan {false};
    bool m_hasTf {false};
    bool m_hasTfStatic {false};
    bool m_hasPublisherInfoLogged {false};
    bool m_hasLoggedFirstAcceptedMap {false};
    bool m_hasLoggedMapInfo {false};
    uint32_t m_lastLoggedWidth {0};
    uint32_t m_lastLoggedHeight {0};
    double m_lastLoggedResolution {0.0};
    double m_lastLoggedOriginX {0.0};
    double m_lastLoggedOriginY {0.0};
    size_t m_lastLoggedDataSize {0};
    size_t m_lastLoggedPublisherCount {0};
    int m_lastInvalidWidth {-1};
    int m_lastInvalidHeight {-1};
    size_t m_lastInvalidDataSize {0};
    int m_mapWidth {0};
    int m_mapHeight {0};
    double m_mapResolution {0.0};
    double m_mapOriginX {0.0};
    double m_mapOriginY {0.0};
    bool m_hasInitialOdom {false};
    double m_initialOdomX {0.0};
    double m_initialOdomY {0.0};
    bool m_hasLatestOdom {false};
    double m_latestOdomX {0.0};
    double m_latestOdomY {0.0};
    double m_latestYawRad {0.0};
    bool m_hasMapAnchor {false};
    bool m_hasLoggedFirstOdom {false};
    StableMapBounds m_sessionBounds;
    std::chrono::steady_clock::time_point m_lastScanAt {};
    std::chrono::steady_clock::time_point m_lastOdomAt {};
    std::chrono::steady_clock::time_point m_lastTfAt {};
    std::chrono::steady_clock::time_point m_lastTfStaticAt {};
    std::atomic<uint64_t> m_odomMessageCount {0};
    QString m_lastOdomFrameId;
    QString m_lastOdomChildFrameId;

public:
    bool hasReceivedRealMap() const
    {
        return m_mapReceived;
    }

    MapTopicSnapshot mapTopicSnapshot()
    {
        MapTopicSnapshot snapshot;
        snapshot.publisherNames = effectiveMapPublisherNames();
        snapshot.publisherCount = static_cast<size_t>(snapshot.publisherNames.size());

        std::lock_guard<std::mutex> lock(m_dataMutex);
        if (!m_latestMap || !m_mapReceived) {
            return snapshot;
        }

        const int width = static_cast<int>(m_latestMap->info.width);
        const int height = static_cast<int>(m_latestMap->info.height);
        const size_t expectedSize = (width > 0 && height > 0)
            ? static_cast<size_t>(width) * static_cast<size_t>(height)
            : 0;
        if (width <= 0 || height <= 0 || m_latestMap->info.resolution <= 0.0
            || m_latestMap->data.size() != expectedSize) {
            return snapshot;
        }

        snapshot.hasValidMap = true;
        snapshot.width = width;
        snapshot.height = height;
        snapshot.resolution = m_latestMap->info.resolution;
        snapshot.originX = m_latestMap->info.origin.position.x;
        snapshot.originY = m_latestMap->info.origin.position.y;
        snapshot.dataSize = m_latestMap->data.size();
        return snapshot;
    }

    // /amcl 노드 존재 여부로 Nav2 로컬라이제이션 스택(run.sh -> bringup.launch.py로
    // 뜬 AMCL)이 살아있는지 판단한다. AMCL과 Cartographer가 동시에 map->odom tf를
    // 발행하면 tf 충돌로 odom이 중복되므로, 매핑 시작 전에 이 상태를 감지해야 한다.
    // /amcl 노드는 PAUSE되어도(inactive) 그래프에서 사라지지 않으므로 단순
    // get_node_names() 존재 여부로는 "지금 실제로 map->odom을 publish 중인지"를
    // 판정할 수 없다(이전엔 그렇게 했다가, 매핑을 시작했다 GUI를 강제 종료해
    // amcl이 inactive로 멈춰 있는 상태에서도 계속 "Nav2 감지됨"으로 오판해
    // PAUSE를 재요청 -> lifecycle_manager가 이미 paused라 요청을 거부해
    // "Nav2 일시정지 실패"로 매핑이 막히는 문제가 있었다). GetState로 실제
    // PRIMARY_STATE_ACTIVE인지 직접 확인한다.
    bool isNav2LocalizationActive() const
    {
        if (!m_amclStateClient || !m_amclStateClient->service_is_ready()) {
            return false;
        }

        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto future = m_amclStateClient->async_send_request(request);
        if (future.wait_for(std::chrono::milliseconds(300)) != std::future_status::ready) {
            m_amclStateClient->remove_pending_request(future);
            return false;
        }
        return future.get()->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    }

    // AMCL(map_server 포함)과 Nav2 내비게이션 스택을 PAUSE시킨다(비blocking).
    // 로컬라이제이션 매니저(AMCL) PAUSE가 성공해야 map->odom tf 충돌이 실제로
    // 해소되므로 이것만 필수로 보고, 내비게이션 매니저는 best-effort로 같이
    // 멈춘다(실패해도 매핑은 진행). 실패 시 아무 것도 PAUSE된 채로 남기지
    // 않도록 부분 성공 상태를 플래그로 정확히 기록해서 resume 쪽이 그것만
    // 되돌리게 한다. onDone(false)면 로컬라이제이션 PAUSE 자체가 실패해 아무
    // 것도 PAUSE되지 않은 상태 — 호출자는 매핑을 시작하면 안 된다.
    void pauseNav2ForMappingAsync(std::function<void(bool)> onDone)
    {
        callNav2LifecycleManagerAsync(
            m_localizationLifecycleClient,
            nav2_msgs::srv::ManageLifecycleNodes::Request::PAUSE,
            [this, onDone](LifecycleCallResult localizationResult) {
                if (localizationResult != LifecycleCallResult::Success) {
                    m_localizationPausedByMapping = false;
                    onDone(false);
                    return;
                }
                m_localizationPausedByMapping = true;

                callNav2LifecycleManagerAsync(
                    m_navigationLifecycleClient,
                    nav2_msgs::srv::ManageLifecycleNodes::Request::PAUSE,
                    [this, onDone](LifecycleCallResult navigationResult) {
                        m_navigationPausedByMapping = (navigationResult == LifecycleCallResult::Success);
                        onDone(true);
                    });
            });
    }

    // pauseNav2ForMappingAsync()가 실제로 PAUSE시킨 매니저만 RESUME한다(비blocking,
    // 둘 다 필요하면 동시에 요청). 매핑을 시작한 적이 없거나 PAUSE가 애초에
    // 실패했던 경우엔 아무 요청도 보내지 않고 즉시 onDone(빈 목록)을 호출한다.
    // onDone은 RESUME에 실패한 매니저 이름 목록을 받는다(비어 있으면 전부
    // 성공했거나 애초에 PAUSE된 게 없었다는 뜻). 성공한 매니저만 플래그를
    // 지워서 실패한 쪽은 다음 호출 때 재시도되게 한다.
    // UI를 막지 않으므로 인터랙티브 경로(초기화 버튼 등)에서 쓴다 — GUI 종료
    // 직전처럼 "완료를 보장해야 하는" 경로는 resumeNav2IfPausedBlocking을 쓴다.
    void resumeNav2IfPausedAsync(std::function<void(QStringList)> onDone)
    {
        const bool needNavigation = m_navigationPausedByMapping.load();
        const bool needLocalization = m_localizationPausedByMapping.load();
        if (!needNavigation && !needLocalization) {
            onDone(QStringList());
            return;
        }

        auto failedManagers = std::make_shared<QStringList>();
        auto pending = std::make_shared<int>((needNavigation ? 1 : 0) + (needLocalization ? 1 : 0));
        auto finishOne = [pending, failedManagers, onDone]() {
            if (--(*pending) == 0) {
                onDone(*failedManagers);
            }
        };

        if (needNavigation) {
            callNav2LifecycleManagerAsync(
                m_navigationLifecycleClient,
                nav2_msgs::srv::ManageLifecycleNodes::Request::RESUME,
                [this, failedManagers, finishOne](LifecycleCallResult result) {
                    if (result == LifecycleCallResult::Success) {
                        m_navigationPausedByMapping = false;
                    } else {
                        failedManagers->append(QStringLiteral("navigation"));
                    }
                    finishOne();
                });
        }
        if (needLocalization) {
            callNav2LifecycleManagerAsync(
                m_localizationLifecycleClient,
                nav2_msgs::srv::ManageLifecycleNodes::Request::RESUME,
                [this, failedManagers, finishOne](LifecycleCallResult result) {
                    if (result == LifecycleCallResult::Success) {
                        m_localizationPausedByMapping = false;
                    } else {
                        failedManagers->append(QStringLiteral("localization"));
                    }
                    finishOne();
                });
        }
    }

    // resumeNav2IfPausedAsync()의 blocking 버전. GUI 종료(소멸자)에서만 쓴다 —
    // 이 시점 이후로는 노드/executor가 곧 파괴되어 재시도 기회가 없으므로,
    // 비동기로 던지고 끝내면 응답이 오기 전에 spin 스레드가 먼저 죽어 RESUME
    // 요청이 유실될 수 있다. navigation은 관리 노드가 7개(bt_navigator/
    // controller_server/planner_server/smoother_server/behavior_server/
    // waypoint_follower/velocity_smoother)라 localization(map_server+amcl
    // 2개)보다 오래 걸려 타임아웃을 더 넉넉히 준다.
    QStringList resumeNav2IfPausedBlocking(int navigationTimeoutMs = 20000, int localizationTimeoutMs = 8000)
    {
        QStringList failedManagers;
        if (m_navigationPausedByMapping) {
            const auto navigationResult =
                callNav2LifecycleManagerBlocking(m_navigationLifecycleClient,
                                                  nav2_msgs::srv::ManageLifecycleNodes::Request::RESUME,
                                                  navigationTimeoutMs);
            if (navigationResult == LifecycleCallResult::Success) {
                m_navigationPausedByMapping = false;
            } else {
                failedManagers.append(QStringLiteral("navigation"));
            }
        }
        if (m_localizationPausedByMapping) {
            const auto localizationResult =
                callNav2LifecycleManagerBlocking(m_localizationLifecycleClient,
                                                  nav2_msgs::srv::ManageLifecycleNodes::Request::RESUME,
                                                  localizationTimeoutMs);
            if (localizationResult == LifecycleCallResult::Success) {
                m_localizationPausedByMapping = false;
            } else {
                failedManagers.append(QStringLiteral("localization"));
            }
        }
        return failedManagers;
    }

private:
    void logMapPublishersOnce()
    {
        const auto publishers = m_node->get_publishers_info_by_topic("/map");
        if (!m_hasPublisherInfoLogged) {
            m_hasPublisherInfoLogged = true;
            if (publishers.empty()) {
                RCLCPP_WARN(m_node->get_logger(), "[MapSource] no /map publishers found");
            } else {
                for (const auto &info : publishers) {
                    RCLCPP_INFO(
                        m_node->get_logger(),
                        "[MapSource] /map publisher node=%s namespace=%s type=%s depth=%zu reliability=%d durability=%d",
                        info.node_name().c_str(),
                        info.node_namespace().c_str(),
                        info.topic_type().c_str(),
                        info.qos_profile().depth(),
                        static_cast<int>(info.qos_profile().reliability()),
                        static_cast<int>(info.qos_profile().durability()));
                }
            }
        }

        if (publishers.size() > 1) {
            RCLCPP_WARN(m_node->get_logger(), "[MapSource] multiple /map publishers detected: %zu", publishers.size());
        }
    }

    // PAUSE(deactivate)된 lifecycle 노드는 rclcpp 퍼블리셔 핸들이 여전히 살아있어
    // count_publishers()/get_publishers_info_by_topic()에 계속 잡힌다 — 실제로
    // 새 메시지를 발행하는지와 무관하게 DDS 그래프상 publisher로 카운트된다.
    // 우리가 매핑을 위해 직접 PAUSE시킨 /map_server는 이미 알고 있는 상태이므로
    // "중복 SLAM 실행" 판정에서 제외한다. 그 외 이름은 그대로 실제 충돌로 취급한다.
    QStringList effectiveMapPublisherNames() const
    {
        const auto publishers = m_node->get_publishers_info_by_topic("/map");
        QStringList names;
        names.reserve(static_cast<int>(publishers.size()));
        for (const auto &info : publishers) {
            names.append(
                QStringLiteral("%1%2")
                    .arg(QString::fromStdString(info.node_namespace()))
                    .arg(QString::fromStdString(info.node_name())));
        }
        if (m_localizationPausedByMapping.load()) {
            names.removeAll(QStringLiteral("/map_server"));
        }
        return names;
    }
};
