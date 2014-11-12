#include "qtheaders.h"
#include "headers.h"
#include "classes.h"
#include "definitions.h"
#include "functions.h"

#include "mainwindow.h"
#include "processingthread.h"
#include "controllerthread.h"
#include "cargroupbox.h"
#include "carsettingsdialog.h"
#include "drawsettingsdialog.h"

// Initialize external declarations.
struct DrawThreadData drawThreadData;
struct ControllerThreadData controllerThreadData;
struct LapData lapData;
CRITICAL_SECTION csDrawThreadData, csControllerThreadData;
HANDLE hDrawThreadEvent;
HANDLE hControllerThreadEvent1, hControllerThreadEvent_signalsWritten, hControllerThreadEvent_signalsRead;
std::vector<float> gRef;
std::vector<float> vRef;
int gRefLen;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QCoreApplication::setOrganizationName("Uppsala University");
    QCoreApplication::setApplicationName("CARS");

    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, "settings");

    // Initialize critical sections.
    InitializeCriticalSection(&csDrawThreadData);
    InitializeCriticalSection(&csControllerThreadData);
    // Create events. Initial state for hControllerThreadEvent1 is not signaled.
    hDrawThreadEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
    hControllerThreadEvent1 = CreateEvent(NULL, FALSE, FALSE, NULL);
    hControllerThreadEvent_signalsWritten = CreateEvent(NULL, FALSE, TRUE, NULL);
    hControllerThreadEvent_signalsRead = CreateEvent(NULL, FALSE, FALSE, NULL);

    MainWindow w;
    w.show();

    return a.exec();
}
