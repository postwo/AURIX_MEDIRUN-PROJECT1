#include "slammappingpage_internal.h"

void SlamMappingPage::setManualDriveCommand(double linearX, double angularZ, const QString &statusMessage)
{
    if (m_mapBridge == nullptr) {
        qWarning() << "[TELEOP] cmd_vel publisher 없음";
        appendUiLog(QStringLiteral("[TELEOP][ERROR] cmd_vel publisher 없음"));
        return;
    }
    m_manualLinearX = linearX;
    m_manualAngularZ = angularZ;
    qInfo() << "[TELEOP] publishing cmd_vel:" << linearX << angularZ;
    m_mapBridge->publishVelocity(linearX, angularZ);
    if (!qFuzzyIsNull(m_manualLinearX) || !qFuzzyIsNull(m_manualAngularZ)) {
        m_manualDriveTimer->start();
    } else {
        m_manualDriveTimer->stop();
    }

    setStatusMessage(statusMessage);
}

void SlamMappingPage::applyManualKeyState()
{
    double linearX = 0.0;
    double angularZ = 0.0;

    if (m_upPressed == m_downPressed) {
        linearX = 0.0;
    } else if (m_upPressed) {
        linearX = kManualLinearSpeedMps;
    } else {
        linearX = -kManualLinearSpeedMps;
    }

    if (m_leftPressed == m_rightPressed) {
        angularZ = 0.0;
    } else if (m_leftPressed) {
        angularZ = kManualAngularSpeedRadPerSec;
    } else {
        angularZ = -kManualAngularSpeedRadPerSec;
    }

    if (qFuzzyIsNull(linearX) && qFuzzyIsNull(angularZ)) {
        stopManualDrive();
        return;
    }

    setManualDriveCommand(
        linearX,
        angularZ,
        QStringLiteral("수동 제어 · linear %1 m/s / angular %2 rad/s")
            .arg(linearX, 0, 'f', 2)
            .arg(angularZ, 0, 'f', 2));
}

void SlamMappingPage::stopManualDrive()
{
    m_manualDriveTimer->stop();
    m_manualLinearX = 0.0;
    m_manualAngularZ = 0.0;
    m_upPressed = false;
    m_downPressed = false;
    m_leftPressed = false;
    m_rightPressed = false;
    if (m_mapBridge != nullptr) {
        m_mapBridge->publishVelocity(0.0, 0.0);
    }
    qInfo() << "[TELEOP] stop command published";
    appendUiLog(QStringLiteral("[TELEOP] stop command published"));
}

bool SlamMappingPage::consumeManualKey(int key)
{
    switch (key) {
    case Qt::Key_Up:
    case Qt::Key_W:
        qInfo() << "[TELEOP] key pressed: W/Up";
        m_upPressed = true;
        m_downPressed = false;
        applyManualKeyState();
        return true;
    case Qt::Key_Down:
    case Qt::Key_S:
        qInfo() << "[TELEOP] key pressed: S/Down";
        m_downPressed = true;
        m_upPressed = false;
        applyManualKeyState();
        return true;
    case Qt::Key_Left:
    case Qt::Key_A:
        qInfo() << "[TELEOP] key pressed: A/Left";
        m_leftPressed = true;
        m_rightPressed = false;
        applyManualKeyState();
        return true;
    case Qt::Key_Right:
    case Qt::Key_D:
        qInfo() << "[TELEOP] key pressed: D/Right";
        m_rightPressed = true;
        m_leftPressed = false;
        applyManualKeyState();
        return true;
    case Qt::Key_Space:
        qInfo() << "[TELEOP] key pressed: Space";
        m_upPressed = false;
        m_downPressed = false;
        m_leftPressed = false;
        m_rightPressed = false;
        stopManualDrive();
        return true;
    default:
        return false;
    }
}

bool SlamMappingPage::releaseManualKey(int key)
{
    switch (key) {
    case Qt::Key_Up:
    case Qt::Key_W:
        return true;
    case Qt::Key_Down:
    case Qt::Key_S:
        return true;
    case Qt::Key_Left:
    case Qt::Key_A:
        m_leftPressed = false;
        applyManualKeyState();
        return true;
    case Qt::Key_Right:
    case Qt::Key_D:
        m_rightPressed = false;
        applyManualKeyState();
        return true;
    default:
        return false;
    }
}
