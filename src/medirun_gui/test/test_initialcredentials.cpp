// 초기 계정 비밀번호 결정 시험: 소스에 고정된 기본 비밀번호가 없어야 한다.

#include "../src/services/initialcredentials.h"

#include <QtTest>

namespace {

QStringList g_messages;

void captureMessages(QtMsgType, const QMessageLogContext &, const QString &message)
{
    g_messages << message;
}

constexpr const char *kEnv = "AURIX_TEST_INITIAL_PASSWORD";

} // namespace

class InitialCredentialsTest : public QObject
{
    Q_OBJECT

private slots:
    void init();
    void cleanup();

    void usesEnvironmentPasswordWithoutLoggingIt();
    void trimsSurroundingWhitespace();
    void acceptsEightCharactersButNotSeven();
    void generatesPasswordWhenEnvironmentIsMissing();
    void generatedPasswordsDifferEveryTime();
    void generatedPasswordAvoidsAmbiguousCharacters();
    void generatedPasswordIsLoggedOnceWithAccountName();
    void neverFallsBackToKnownDefaults();
};

void InitialCredentialsTest::init()
{
    qunsetenv(kEnv);
    g_messages.clear();
    qInstallMessageHandler(captureMessages);
}

void InitialCredentialsTest::cleanup()
{
    qInstallMessageHandler(nullptr);
    qunsetenv(kEnv);
}

void InitialCredentialsTest::usesEnvironmentPasswordWithoutLoggingIt()
{
    qputenv(kEnv, "S3cretPassw0rd");

    const QString password = initialPasswordFromEnv(kEnv, QStringLiteral("admin"));

    QCOMPARE(password, QStringLiteral("S3cretPassw0rd"));
    QVERIFY(g_messages.isEmpty()); // 사용자가 정한 비밀번호는 로그에 남기지 않는다
}

void InitialCredentialsTest::trimsSurroundingWhitespace()
{
    qputenv(kEnv, "  S3cretPassw0rd \n");

    QCOMPARE(initialPasswordFromEnv(kEnv, QStringLiteral("admin")), QStringLiteral("S3cretPassw0rd"));
}

void InitialCredentialsTest::acceptsEightCharactersButNotSeven()
{
    qputenv(kEnv, "abcd1234");
    QCOMPARE(initialPasswordFromEnv(kEnv, QStringLiteral("admin")), QStringLiteral("abcd1234"));

    qputenv(kEnv, "abcd123");
    const QString password = initialPasswordFromEnv(kEnv, QStringLiteral("admin"));
    QVERIFY(password != QStringLiteral("abcd123"));
    QCOMPARE(password.size(), 16);
    // 너무 짧아 무시했다는 사실을 경고로 알린다.
    bool warned = false;
    for (const QString &message : std::as_const(g_messages)) {
        warned = warned || message.contains(QLatin1String(kEnv));
    }
    QVERIFY(warned);
}

void InitialCredentialsTest::generatesPasswordWhenEnvironmentIsMissing()
{
    const QString password = initialPasswordFromEnv(kEnv, QStringLiteral("admin"));

    QCOMPARE(password.size(), 16);
}

void InitialCredentialsTest::generatedPasswordsDifferEveryTime()
{
    QSet<QString> seen;
    for (int i = 0; i < 20; ++i) {
        seen.insert(initialPasswordFromEnv(kEnv, QStringLiteral("admin")));
    }
    QCOMPARE(seen.size(), 20);
}

void InitialCredentialsTest::generatedPasswordAvoidsAmbiguousCharacters()
{
    // 화면에서 읽어 입력하는 임시 비밀번호이므로 0/O, 1/l/I 처럼 헷갈리는 글자는 뺀다.
    const QString forbidden = QStringLiteral("0O1lI");
    for (int i = 0; i < 50; ++i) {
        const QString password = initialPasswordFromEnv(kEnv, QStringLiteral("admin"));
        for (const QChar &ch : password) {
            QVERIFY2(!forbidden.contains(ch) && ch.isLetterOrNumber(), qPrintable(password));
        }
    }
}

void InitialCredentialsTest::generatedPasswordIsLoggedOnceWithAccountName()
{
    const QString password = initialPasswordFromEnv(kEnv, QStringLiteral("operator"));

    QCOMPARE(g_messages.size(), 1);
    QVERIFY(g_messages.first().contains(QStringLiteral("operator")));
    QVERIFY(g_messages.first().contains(password));
}

void InitialCredentialsTest::neverFallsBackToKnownDefaults()
{
    const QStringList knownDefaults = {QStringLiteral("admin123"), QStringLiteral("user123"), QStringLiteral("admin"),
                                       QStringLiteral("password")};
    for (int i = 0; i < 20; ++i) {
        QVERIFY(!knownDefaults.contains(initialPasswordFromEnv(kEnv, QStringLiteral("admin"))));
    }
}

QTEST_GUILESS_MAIN(InitialCredentialsTest)
#include "test_initialcredentials.moc"
