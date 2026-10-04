#include "initialcredentials.h"

#include <QDebug>
#include <QRandomGenerator>
#include <QtGlobal>

namespace {

constexpr int kMinPasswordLength = 8;
constexpr int kGeneratedPasswordLength = 16;

QString generatePassword()
{
    static const QString alphabet = QStringLiteral(
        "ABCDEFGHJKLMNPQRSTUVWXYZabcdefghijkmnpqrstuvwxyz23456789");
    QString password;
    password.reserve(kGeneratedPasswordLength);
    for (int i = 0; i < kGeneratedPasswordLength; ++i) {
        password += alphabet.at(QRandomGenerator::system()->bounded(alphabet.size()));
    }
    return password;
}

} // namespace

QString initialPasswordFromEnv(const char *envName, const QString &loginId)
{
    const QString fromEnv = qEnvironmentVariable(envName).trimmed();
    if (fromEnv.size() >= kMinPasswordLength) {
        return fromEnv;
    }
    if (!fromEnv.isEmpty()) {
        qWarning().noquote() << QStringLiteral("%1 값이 %2자 미만이라 무시하고 임시 비밀번호를 생성합니다.")
                                    .arg(QString::fromLatin1(envName))
                                    .arg(kMinPasswordLength);
    }

    const QString generated = generatePassword();
    qWarning().noquote()
        << QStringLiteral("[최초 설정] '%1' 계정의 임시 비밀번호: %2 (이 메시지는 한 번만 표시됩니다. 로그인 후 변경하세요.)")
               .arg(loginId, generated);
    return generated;
}
