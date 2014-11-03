#ifndef CONTROLLERTHREAD_H
#define CONTROLLERTHREAD_H

#include "qtheaders.h"
#include "classes.h"
#include "definitions.h"

class ControllerThread : public QThread
{
    Q_OBJECT
public:
    explicit ControllerThread(QObject *parent = 0);

    bool stop();

protected:
    void run();

private:
    QMutex m_doStopMutex;
    volatile bool m_doStop;
    QSettings m_settings;
    int m_numCars;
    std::vector<Controller*> m_controllers;
    std::vector<Controller*> m_backingSequence;
    std::vector<bool> m_isBacking;
    std::vector<int> m_stuckCounter;

    void loadControllerSettings(void);

signals:


};

#endif // CONTROLLERTHREAD_H
