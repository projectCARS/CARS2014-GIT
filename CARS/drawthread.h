#ifndef DRAWTHREAD_H
#define DRAWTHREAD_H

#include <QThread>

class DrawThread : public QThread
{
    Q_OBJECT
public:
    explicit DrawThread(QObject *parent = 0);

protected:
    void run();
    void stop();

private:
    QMutex m_doStopMutex;
    volatile bool m_doStop;

signals:

public slots:

};

#endif // DRAWTHREAD_H
