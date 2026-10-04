#include "livecontrolpage.h"

#include "../ros/navigationservice.h"
#include "../services/camerafeedprovider.h"
#include "../services/drivingrecordservice.h"
#include "../services/missioncontroller.h"
#include "../services/operationsoverviewservice.h"
#include "../services/robotservice.h"
#include "../services/sensortelemetryprovider.h"
#include "../services/telemetryprovider.h"
#include "../services/waypointrepository.h"
#include "../theme/dashboardpalette.h"
#include "../widgets/badgelabel.h"
#include "../widgets/mapviewwidget.h"
#include "../widgets/proportionalbar.h"

#include <QDate>
#include <QDir>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayoutItem>
#include <QMessageBox>
#include <QPixmap>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QStandardPaths>
#include <QStyle>
#include <QTimer>
#include <QUuid>
#include <QVBoxLayout>
#include <QtMath>

#include <algorithm>

#include "livecontrolpage_helpers.h"

void LiveControlPage::handleCameraFrameReady(const QImage &frame)
{
    m_lastCameraFrame = frame;
    if (m_cameraViewLabel == nullptr || frame.isNull()) {
        return;
    }
    const QPixmap pixmap = QPixmap::fromImage(frame).scaled(
        m_cameraViewLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    m_cameraViewLabel->setPixmap(pixmap);
}

void LiveControlPage::handleRecordingStateChanged(bool recording)
{
    if (m_saveRecordingButton != nullptr) {
        m_saveRecordingButton->setEnabled(recording);
    }
    if (m_pauseRecordingButton != nullptr) {
        m_pauseRecordingButton->setEnabled(recording);
        m_pauseRecordingButton->setText(QStringLiteral("⏸"));
        m_pauseRecordingButton->setToolTip(QStringLiteral("일시정지"));
        m_pauseRecordingButton->setObjectName(QStringLiteral("pauseChipButton"));
        m_pauseRecordingButton->style()->unpolish(m_pauseRecordingButton);
        m_pauseRecordingButton->style()->polish(m_pauseRecordingButton);
    }

    if (m_cameraTimelinePanel != nullptr) {
        m_cameraTimelinePanel->setVisible(recording);
    }
    if (recording) {
        m_recordingElapsedSeconds = 0;
        if (m_recordingElapsedLabel != nullptr) {
            m_recordingElapsedLabel->setText(QStringLiteral("00:00"));
        }
        if (m_recordingElapsedTimer != nullptr) {
            m_recordingElapsedTimer->start(1000);
        }
    } else if (m_recordingElapsedTimer != nullptr) {
        m_recordingElapsedTimer->stop();
    }
}

void LiveControlPage::handleRecordingPausedChanged(bool paused)
{
    if (m_pauseRecordingButton != nullptr) {
        m_pauseRecordingButton->setText(paused ? QStringLiteral("▶") : QStringLiteral("⏸"));
        m_pauseRecordingButton->setToolTip(paused ? QStringLiteral("재개") : QStringLiteral("일시정지"));
        m_pauseRecordingButton->setObjectName(paused ? QStringLiteral("resumeChipButton") : QStringLiteral("pauseChipButton"));
        m_pauseRecordingButton->style()->unpolish(m_pauseRecordingButton);
        m_pauseRecordingButton->style()->polish(m_pauseRecordingButton);
    }
    const bool recording = m_drivingRecordService != nullptr && m_drivingRecordService->isRecording();
    if (!recording || m_recordingElapsedTimer == nullptr) {
        return;
    }
    if (paused) {
        m_recordingElapsedTimer->stop();
    } else {
        m_recordingElapsedTimer->start(1000);
    }
}

void LiveControlPage::handleRecordingSaveFinished(bool ok, const QString &message)
{
    if (ok) {
        QMessageBox::information(
            this, QStringLiteral("영상 저장"), QStringLiteral("영상이 저장되었습니다.\n%1").arg(message));
    } else {
        QMessageBox::warning(
            this, QStringLiteral("영상 저장 실패"), QStringLiteral("영상 저장 중 문제가 발생했습니다.\n%1").arg(message));
    }
}

void LiveControlPage::tickRecordingElapsed()
{
    ++m_recordingElapsedSeconds;
    if (m_recordingElapsedLabel == nullptr) {
        return;
    }
    m_recordingElapsedLabel->setText(QStringLiteral("%1:%2")
                                          .arg(m_recordingElapsedSeconds / 60, 2, 10, QChar('0'))
                                          .arg(m_recordingElapsedSeconds % 60, 2, 10, QChar('0')));
}

void LiveControlPage::handleRecordButtonClicked()
{
    if (m_drivingRecordService != nullptr) {
        m_drivingRecordService->startRecording();
    }
}

void LiveControlPage::handleSaveRecordingButtonClicked()
{
    if (m_drivingRecordService == nullptr) {
        return;
    }
    const auto confirm = QMessageBox::question(
        this, QStringLiteral("영상 저장"),
        QStringLiteral("녹화를 종료하고 지금까지 촬영한 영상을 저장하시겠습니까?"));
    if (confirm != QMessageBox::Yes) {
        return;
    }
    m_drivingRecordService->stopRecording();
}

void LiveControlPage::handlePauseRecordingButtonClicked()
{
    if (m_drivingRecordService == nullptr) {
        return;
    }
    if (m_drivingRecordService->isPaused()) {
        m_drivingRecordService->resumeRecording();
    } else {
        m_drivingRecordService->pauseRecording();
    }
}

void LiveControlPage::handleSnapshotButtonClicked()
{
    if (m_lastCameraFrame.isNull() || m_operationsOverviewService == nullptr || m_waypointRepository == nullptr) {
        QMessageBox::warning(this, QStringLiteral("스냅샷 저장 실패"), QStringLiteral("카메라 영상이 없어 스냅샷을 저장할 수 없습니다."));
        return;
    }

    const QString snapshotDir = m_waypointRepository->workspaceRootPath() + QStringLiteral("/snapshots");
    QDir().mkpath(snapshotDir);
    const QString fileName = QStringLiteral("snapshot_%1_%2.png")
                                  .arg(QDate::currentDate().toString(QStringLiteral("yyyyMMdd")),
                                       QUuid::createUuid().toString(QUuid::WithoutBraces));
    const QString fullPath = snapshotDir + QStringLiteral("/") + fileName;
    // PNG는 무손실이라 quality는 픽셀 품질이 아니라 zlib 압축 강도를 뜻하지만,
    // 100(최소 압축)으로 저장 시간을 줄이고 의도를 명확히 한다.
    if (!m_lastCameraFrame.save(fullPath, "PNG", 100)) {
        QMessageBox::warning(this, QStringLiteral("스냅샷 저장 실패"), QStringLiteral("파일 저장 중 오류가 발생했습니다.\n%1").arg(fullPath));
        return;
    }

    const QString missionId = m_missionController == nullptr ? QString() : m_missionController->missionSnapshot().missionId;
    const QString entityId = missionId.isEmpty() ? QStringLiteral("none") : missionId;
    QString errorMessage;
    if (!m_operationsOverviewService->recordFile(
            QStringLiteral("mission"), entityId, QStringLiteral("snapshot"), fullPath, fileName, QStringLiteral("image/png"),
            m_session.username, errorMessage)) {
        QMessageBox::warning(
            this, QStringLiteral("스냅샷 저장"),
            QStringLiteral("파일은 저장됐지만 DB 기록에 실패했습니다.\n%1\n%2").arg(fullPath, errorMessage));
        return;
    }

    QMessageBox::information(this, QStringLiteral("스냅샷 저장"), QStringLiteral("스냅샷이 저장되었습니다.\n%1").arg(fullPath));
}
