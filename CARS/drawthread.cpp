#include "drawthread.h"

DrawThread::DrawThread(QObject *parent) :
    QThread(parent)
{
    m_doStop = false;
}

void DrawThread::run()
{
    m_doStopMutex.lock();
    if (m_doStop)
    {
        m_doStop = false;
        m_doStopMutex.unlock();
        break;
    }
    m_doStopMutex.unlock();

    // Draw.

}

void DrawThread::stop()
{
    QMutexLocker locker(&m_doStopMutex);
    doStop = true;
}
