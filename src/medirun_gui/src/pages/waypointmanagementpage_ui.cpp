#include "waypointmanagementpage.h"

#include "../services/missioncontroller.h"
#include "../services/waypointrepository.h"
#include "../services/waypointfile.h"
#include "../theme/dashboardpalette.h"
#include "../widgets/badgelabel.h"

#include <QAbstractItemModel>
#include <QAbstractItemView>
#include <QComboBox>
#include <QCoreApplication>
#include <QDate>
#include <QDebug>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPoint>
#include <QPushButton>
#include <QSet>
#include <QSlider>
#include <QStackedWidget>
#include <QStyle>
#include <QThread>
#include <QVBoxLayout>
#include <QtMath>

#include <algorithm>

#include "waypointmanagementpage_helpers.h"

QWidget *WaypointManagementPage::buildHeader()
{
    auto *row = new QWidget;
    auto *layout = new QHBoxLayout(row);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(10);

    auto *titleCol = new QVBoxLayout;
    titleCol->setSpacing(2);
    titleCol->addWidget(makeLabel(QStringLiteral("웨이포인트 · 경로 관리"), QStringLiteral("panelTitle")));
    auto *subtitleLabel = makeLabel(
        QStringLiteral("호출 구역·경유점·홈 위치를 등록하고, 등록된 웨이포인트를 엮어 주행 경로를 만듭니다."),
        QStringLiteral("pageSubtitle"));
    subtitleLabel->setWordWrap(false);
    titleCol->addWidget(subtitleLabel);
    layout->addLayout(titleCol);
    layout->addStretch();

    m_nav2ConnectionBadge = makeBadgeLabel(QStringLiteral("Nav2 부팅 중"), QStringLiteral("amber"), row);
    layout->addWidget(m_nav2ConnectionBadge);

    m_navStatusLabel = makeLabel(QString(), QStringLiteral("metricCaption"));
    m_navStatusLabel->setWordWrap(false);
    layout->addWidget(m_navStatusLabel);

    m_setInitialPoseButton = new QPushButton(QStringLiteral("초기 위치 설정"), row);
    m_setInitialPoseButton->setObjectName(QStringLiteral("ghostChipButton"));
    m_setInitialPoseButton->setCheckable(true);
    m_setInitialPoseButton->setToolTip(
        QStringLiteral("RViz의 2D Pose Estimate와 동일 · 켠 뒤 지도를 클릭+드래그하면 그 위치·방향으로 AMCL 초기 위치를 설정합니다"));
    connect(m_setInitialPoseButton, &QPushButton::toggled, this, &WaypointManagementPage::handleSetInitialPoseToggled);
    layout->addWidget(m_setInitialPoseButton);

    m_speedButton = new QPushButton(QStringLiteral("주행 속도 100%"), row);
    m_speedButton->setObjectName(QStringLiteral("ghostChipButton"));
    m_speedButton->setToolTip(QStringLiteral("웨이포인트 주행 속도 제한 (Nav2 컨트롤러 최대 속도 대비 %)"));
    connect(m_speedButton, &QPushButton::clicked, this, &WaypointManagementPage::showSpeedPopup);
    layout->addWidget(m_speedButton);

    auto *navDivider = new QFrame(row);
    navDivider->setObjectName(QStringLiteral("sectionDivider"));
    navDivider->setFixedWidth(1);
    navDivider->setFixedHeight(22);
    layout->addWidget(navDivider);

    auto *clearMapButton = new QPushButton(QStringLiteral("지도 초기화"), row);
    clearMapButton->setObjectName(QStringLiteral("ghostChipButton"));
    connect(clearMapButton, &QPushButton::clicked, this, &WaypointManagementPage::handleClearMap);
    layout->addWidget(clearMapButton);

    auto *loadMapButton = new QPushButton(QStringLiteral("지도 불러오기"), row);
    loadMapButton->setObjectName(QStringLiteral("cyanChipButton"));
    connect(loadMapButton, &QPushButton::clicked, this, &WaypointManagementPage::handleLoadMap);
    layout->addWidget(loadMapButton);

    m_mapStatusLabel = makeLabel(QStringLiteral("불러온 지도 없음"), QStringLiteral("metricCaption"));
    layout->addWidget(m_mapStatusLabel);

    auto *headerDivider = new QFrame(row);
    headerDivider->setObjectName(QStringLiteral("sectionDivider"));
    headerDivider->setFixedWidth(1);
    headerDivider->setFixedHeight(22);
    layout->addWidget(headerDivider);

    auto *tabSwitcher = new QFrame(row);
    tabSwitcher->setObjectName(QStringLiteral("tabSwitcher"));
    auto *tabLayout = new QHBoxLayout(tabSwitcher);
    tabLayout->setContentsMargins(0, 0, 0, 0);
    tabLayout->setSpacing(6);

    m_waypointTabButton = new QPushButton(QStringLiteral("웨이포인트"), tabSwitcher);
    m_trajectoryTabButton = new QPushButton(QStringLiteral("경로 만들기"), tabSwitcher);
    for (QPushButton *button : {m_waypointTabButton, m_trajectoryTabButton}) {
        button->setObjectName(QStringLiteral("tabButton"));
        tabLayout->addWidget(button);
    }
    connect(m_waypointTabButton, &QPushButton::clicked, this, [this]() { selectTab(0); });
    connect(m_trajectoryTabButton, &QPushButton::clicked, this, [this]() { selectTab(1); });
    layout->addWidget(tabSwitcher);

    return row;
}

QWidget *WaypointManagementPage::buildWarningBanner()
{
    auto *panel = panelFrame(QStringLiteral("warningBanner"));
    auto *layout = new QHBoxLayout(panel);
    layout->setContentsMargins(12, 8, 12, 8);
    layout->setSpacing(8);

    auto *icon = makeLabel(QStringLiteral("⚠"));
    icon->setStyleSheet(QStringLiteral("color: %1; font-size: 13px;").arg(DashboardPalette::Amber.name()));
    layout->addWidget(icon);

    auto *text = makeLabel(QStringLiteral("모든 로봇이 대기 상태일 때만 편집 권장 · 저장한 파일은 dispatcher 재기동 후 반영됩니다"));
    text->setStyleSheet(QStringLiteral("color: %1; font-size: 11px;").arg(DashboardPalette::Amber.name()));
    layout->addWidget(text, 1);

    return panel;
}

QWidget *WaypointManagementPage::buildWaypointTab()
{
    auto *tab = new QWidget;
    auto *bodyRow = new QHBoxLayout(tab);
    bodyRow->setContentsMargins(0, 0, 0, 0);
    bodyRow->setSpacing(10);

    m_waypointMap = new WaypointMapWidget(tab);
    m_waypointMap->setMinimumHeight(250);
    m_waypointMap->setCaption(QStringLiteral("지도 클릭 → 웨이포인트 생성 · 드래그 → 방향 지정 · 마커 클릭 → 편집"));
    m_waypointMap->setLegend(QStringLiteral("⊙ 호출 구역   ○ 경유점   ⌂ 홈"));
    connect(m_waypointMap, &WaypointMapWidget::markerClicked, this, &WaypointManagementPage::selectWaypoint);
    connect(m_waypointMap, &WaypointMapWidget::waypointPickRequested, this, &WaypointManagementPage::handleMapPickRequested);
    bodyRow->addWidget(m_waypointMap, 7);

    auto *rightCol = new QVBoxLayout;
    rightCol->setContentsMargins(0, 0, 0, 0);
    rightCol->setSpacing(8);
    rightCol->addWidget(buildWaypointListPanel(), 1);
    rightCol->addWidget(buildWaypointEditPanel());

    auto *rightColWidget = new QWidget(tab);
    rightColWidget->setFixedWidth(230);
    rightColWidget->setLayout(rightCol);
    bodyRow->addWidget(rightColWidget);

    return tab;
}

QWidget *WaypointManagementPage::buildWaypointListPanel()
{
    auto *panel = panelFrame(QStringLiteral("contentPanel"));
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(12, 10, 12, 10);
    layout->setSpacing(6);

    auto *headerRow = new QHBoxLayout;
    m_waypointListCountLabel = makeLabel(QStringLiteral("웨이포인트 · 0개"), QStringLiteral("metricCaption"));
    headerRow->addWidget(m_waypointListCountLabel, 1);
    auto *newButton = new QPushButton(QStringLiteral("+ 새 웨이포인트"), panel);
    newButton->setObjectName(QStringLiteral("cyanChipButton"));
    connect(newButton, &QPushButton::clicked, this, &WaypointManagementPage::handleNewWaypointRequested);
    headerRow->addWidget(newButton);
    layout->addLayout(headerRow);

    m_waypointList = new QListWidget(panel);
    connect(m_waypointList, &QListWidget::itemClicked, this, [this](QListWidgetItem *item) {
        selectWaypoint(item->data(Qt::UserRole).toString());
    });
    layout->addWidget(m_waypointList, 1);

    return panel;
}

QWidget *WaypointManagementPage::buildWaypointEditPanel()
{
    auto *panel = panelFrame(QStringLiteral("contentPanel"));
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(14, 12, 14, 12);
    layout->setSpacing(8);

    m_editPanelTitle = makeLabel(QStringLiteral("편집"), QStringLiteral("panelTitle"));
    layout->addWidget(m_editPanelTitle);

    m_callZoneIdLabel = makeLabel(
        QStringLiteral("감지된 콜존 ID (실제 버튼을 눌러야 목록에 추가됩니다)"), QStringLiteral("fieldLabel"));
    layout->addWidget(m_callZoneIdLabel);
    m_callZoneIdCombo = new QComboBox(panel);
    m_callZoneIdCombo->addItem(QStringLiteral("선택 안 함 (자동 채번)"));
    connect(m_callZoneIdCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
        // 중복 콜존 ID 에러로 빨갛게 표시돼 있었다면, 다른 값을 고르는 순간 지운다.
        m_callZoneIdCombo->setStyleSheet(QString());
        m_waypointStatusLabel->setStyleSheet(QString());
    });
    layout->addWidget(m_callZoneIdCombo);

    layout->addWidget(makeLabel(QStringLiteral("이름"), QStringLiteral("fieldLabel")));
    m_nameEdit = new QLineEdit(panel);
    layout->addWidget(m_nameEdit);

    auto *coordRow = new QHBoxLayout;
    coordRow->setSpacing(6);

    auto *xCol = new QVBoxLayout;
    xCol->addWidget(makeLabel(QStringLiteral("X (m)"), QStringLiteral("fieldLabel")));
    m_xEdit = new QLineEdit(panel);
    m_xEdit->setObjectName(QStringLiteral("compactField"));
    xCol->addWidget(m_xEdit);
    coordRow->addLayout(xCol);

    auto *yCol = new QVBoxLayout;
    yCol->addWidget(makeLabel(QStringLiteral("Y (m)"), QStringLiteral("fieldLabel")));
    m_yEdit = new QLineEdit(panel);
    m_yEdit->setObjectName(QStringLiteral("compactField"));
    yCol->addWidget(m_yEdit);
    coordRow->addLayout(yCol);

    auto *thetaCol = new QVBoxLayout;
    thetaCol->addWidget(makeLabel(QStringLiteral("θ (°)"), QStringLiteral("fieldLabel")));
    m_thetaEdit = new QLineEdit(panel);
    m_thetaEdit->setObjectName(QStringLiteral("compactField"));
    thetaCol->addWidget(m_thetaEdit);
    coordRow->addLayout(thetaCol);

    layout->addLayout(coordRow);

    layout->addWidget(makeLabel(QStringLiteral("유형"), QStringLiteral("fieldLabel")));
    m_kindCombo = new QComboBox(panel);
    m_kindCombo->addItem(QStringLiteral("호출 구역"), static_cast<int>(WaypointMarkerKind::CallZone));
    m_kindCombo->addItem(QStringLiteral("경유점"), static_cast<int>(WaypointMarkerKind::Waypoint));
    m_kindCombo->addItem(QStringLiteral("홈"), static_cast<int>(WaypointMarkerKind::Home));
    layout->addWidget(m_kindCombo);
    connect(m_kindCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &WaypointManagementPage::updateCallZoneIdComboVisibility);

    auto *actionRow = new QHBoxLayout;
    m_saveWaypointButton = new QPushButton(QStringLiteral("저장"), panel);
    m_saveWaypointButton->setObjectName(QStringLiteral("primaryCtaButton"));
    connect(m_saveWaypointButton, &QPushButton::clicked, this, &WaypointManagementPage::handleSaveWaypoint);
    actionRow->addWidget(m_saveWaypointButton, 1);

    m_deleteWaypointButton = new QPushButton(QStringLiteral("삭제"), panel);
    m_deleteWaypointButton->setObjectName(QStringLiteral("dangerFilledButton"));
    m_deleteWaypointButton->setFixedWidth(56);
    connect(m_deleteWaypointButton, &QPushButton::clicked, this, &WaypointManagementPage::handleDeleteWaypoint);
    actionRow->addWidget(m_deleteWaypointButton);
    layout->addLayout(actionRow);

    auto *navRow = new QHBoxLayout;
    // "이 웨이포인트로 주행"은 이 사이드바 폭(230px)에서 "정지" 버튼과
    // 나눠 쓰면 텍스트가 잘려서(ellipsis) "주행 시작"으로 줄임 — 위에 이미
    // 선택된 웨이포인트 이름/좌표가 보이므로 어디로 가는지는 문맥상 명확하다.
    m_navigateButton = new QPushButton(QStringLiteral("주행 시작"), panel);
    m_navigateButton->setObjectName(QStringLiteral("cyanOutlineButton"));
    connect(m_navigateButton, &QPushButton::clicked, this, &WaypointManagementPage::handleNavigateToSelectedWaypoint);
    navRow->addWidget(m_navigateButton, 1);

    m_cancelNavButton = new QPushButton(QStringLiteral("정지"), panel);
    m_cancelNavButton->setObjectName(QStringLiteral("dangerFilledButton"));
    m_cancelNavButton->setFixedWidth(56);
    connect(m_cancelNavButton, &QPushButton::clicked, this, &WaypointManagementPage::handleCancelNavigation);
    navRow->addWidget(m_cancelNavButton);
    layout->addLayout(navRow);

    m_waypointStatusLabel = makeLabel(QString(), QStringLiteral("pageSubtitle"));
    layout->addWidget(m_waypointStatusLabel);

    return panel;
}

QWidget *WaypointManagementPage::buildTrajectoryTab()
{
    auto *tab = new QWidget;
    auto *outer = new QVBoxLayout(tab);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->setSpacing(10);

    auto *bodyRow = new QHBoxLayout;
    bodyRow->setSpacing(10);

    m_routeMap = new WaypointMapWidget(tab);
    m_routeMap->setMinimumHeight(250);
    m_routeMap->setCaption(QStringLiteral("경로에 담긴 웨이포인트는 번호·연결선으로 표시됩니다"));
    m_routeMap->setLegend(QStringLiteral("① 순서   ○ 미포함 웨이포인트"));
    bodyRow->addWidget(m_routeMap, 7);

    auto *rightCol = new QVBoxLayout;
    rightCol->setContentsMargins(0, 0, 0, 0);
    rightCol->setSpacing(8);
    rightCol->addWidget(buildRouteAddPanel());
    rightCol->addWidget(buildRouteOrderPanel(), 1);
    rightCol->addWidget(buildRouteNamePanel());

    auto *rightColWidget = new QWidget(tab);
    rightColWidget->setFixedWidth(230);
    rightColWidget->setLayout(rightCol);
    bodyRow->addWidget(rightColWidget);

    outer->addLayout(bodyRow, 1);
    outer->addWidget(buildSavedRoutesBar());

    return tab;
}

QWidget *WaypointManagementPage::buildRouteAddPanel()
{
    auto *panel = panelFrame(QStringLiteral("contentPanel"));
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(12, 10, 12, 10);
    layout->setSpacing(6);
    layout->addWidget(makeLabel(QStringLiteral("경로에 추가"), QStringLiteral("metricCaption")));

    auto *row = new QHBoxLayout;
    m_routeAddCombo = new QComboBox(panel);
    row->addWidget(m_routeAddCombo, 1);

    auto *addButton = new QPushButton(QStringLiteral("추가"), panel);
    addButton->setObjectName(QStringLiteral("primaryCtaButton"));
    addButton->setFixedWidth(56);
    connect(addButton, &QPushButton::clicked, this, &WaypointManagementPage::handleAddToRoute);
    row->addWidget(addButton);
    layout->addLayout(row);

    return panel;
}

QWidget *WaypointManagementPage::buildRouteOrderPanel()
{
    auto *panel = panelFrame(QStringLiteral("contentPanel"));
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(12, 10, 12, 10);
    layout->setSpacing(6);

    auto *headerRow = new QHBoxLayout;
    m_routeOrderCountLabel = makeLabel(QStringLiteral("경로 순서 · 0개"), QStringLiteral("metricCaption"));
    headerRow->addWidget(m_routeOrderCountLabel, 1);
    headerRow->addWidget(makeLabel(QStringLiteral("드래그로 순서 변경"), QStringLiteral("metricCaption")));
    layout->addLayout(headerRow);

    m_routeOrderList = new QListWidget(panel);
    m_routeOrderList->setDragDropMode(QAbstractItemView::InternalMove);
    m_routeOrderList->setDefaultDropAction(Qt::MoveAction);
    connect(m_routeOrderList->model(), &QAbstractItemModel::rowsMoved, this, &WaypointManagementPage::renumberRouteOrderItems);
    layout->addWidget(m_routeOrderList, 1);

    auto *buttonRow = new QHBoxLayout;
    auto *removeButton = new QPushButton(QStringLiteral("선택 제거"), panel);
    connect(removeButton, &QPushButton::clicked, this, &WaypointManagementPage::handleRemoveSelectedFromRoute);
    buttonRow->addWidget(removeButton);

    auto *clearButton = new QPushButton(QStringLiteral("전체 초기화"), panel);
    connect(clearButton, &QPushButton::clicked, this, &WaypointManagementPage::handleClearRoute);
    buttonRow->addWidget(clearButton);
    layout->addLayout(buttonRow);

    return panel;
}

QWidget *WaypointManagementPage::buildRouteNamePanel()
{
    auto *panel = panelFrame(QStringLiteral("contentPanel"));
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(12, 10, 12, 10);
    layout->setSpacing(6);
    layout->addWidget(makeLabel(QStringLiteral("경로 이름"), QStringLiteral("metricCaption")));

    m_routeNameEdit = new QLineEdit(panel);
    m_routeNameEdit->setPlaceholderText(QStringLiteral("예: patrol_a"));
    layout->addWidget(m_routeNameEdit);

    auto *saveButton = new QPushButton(QStringLiteral("Trajectory로 저장"), panel);
    saveButton->setObjectName(QStringLiteral("primaryCtaButton"));
    connect(saveButton, &QPushButton::clicked, this, &WaypointManagementPage::handleSaveTrajectory);
    layout->addWidget(saveButton);

    m_trajectoryStatusLabel = makeLabel(QString(), QStringLiteral("pageSubtitle"));
    layout->addWidget(m_trajectoryStatusLabel);

    return panel;
}

QWidget *WaypointManagementPage::buildSavedRoutesBar()
{
    auto *panel = panelFrame(QStringLiteral("contentPanel"));
    auto *layout = new QHBoxLayout(panel);
    layout->setContentsMargins(14, 10, 14, 10);
    layout->setSpacing(8);
    m_savedRoutesBar = panel;

    m_navigateRouteButton = new QPushButton(QStringLiteral("선택 경로 주행"), panel);
    m_navigateRouteButton->setObjectName(QStringLiteral("cyanOutlineButton"));
    connect(m_navigateRouteButton, &QPushButton::clicked, this, &WaypointManagementPage::handleNavigateSelectedRoute);

    auto *cancelButton = new QPushButton(QStringLiteral("정지"), panel);
    cancelButton->setObjectName(QStringLiteral("dangerFilledButton"));
    connect(cancelButton, &QPushButton::clicked, this, &WaypointManagementPage::handleCancelNavigation);

    layout->addWidget(m_navigateRouteButton);
    layout->addWidget(cancelButton);

    return panel;
}
